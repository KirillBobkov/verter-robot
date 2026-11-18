#!/usr/bin/env python3
"""
ROS2 Speech-to-Text node using GigaAM v3 CTC INT8 model.
Based on speech_to_text_node.py architecture.
"""

import os
import queue
import threading
import time
from typing import Optional
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import sounddevice as sd
import numpy as np
import onnxruntime as ort
import sherpa_onnx
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool


@dataclass
class SpeechToTextConfig:
    """Конфигурация Speech-to-Text распознавателя GigaAM v3 CTC"""

    MODEL_DIR: str = 'gigaam-v3-sherpa-onnx'
    MODEL_NAME: str = 'gigaam_v3_ctc_int8.onnx'
    TOKENS_NAME: str = 'gigaam_v3_ctc_tokens.txt'
    VAD_MODEL: str = 'silero_vad.onnx'

    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 512  # 32ms - нативный размер для Silero VAD
    CHANNELS: int = 1

    VAD_THRESHOLD: float = 0.7
    SILENCE_DURATION: float = 0.7  # Секунд тишины для конца фразы
    PRE_BUFFER_DURATION: float = 0.5  # Секунд предыстории

    NUM_THREADS: int = 4
    FEATURE_DIM: int = 64  # GigaAM (v2/v3) обучена с 64 mel bins (ONNX input: [B, 64, T])
    DECODING_METHOD: str = 'greedy_search'
    PROVIDER: str = 'cpu'


# Константы
VAD_INPUT_SIZE = SpeechToTextConfig.BLOCK_SIZE
VAD_STATE_SIZE = 64


class SpeechToTextGigaAMV3CTCNode(Node):
    """ROS2 узел для преобразования речи в текст через GigaAM v3 CTC INT8"""

    def __init__(self) -> None:
        super().__init__('speech_to_text_gigaam_v3_ctc_node')

        self.config = SpeechToTextConfig()

        # Рассчитываем параметры в чанках
        chunk_ms = self.config.BLOCK_SIZE / self.config.SAMPLE_RATE
        self.silence_chunks = int(self.config.SILENCE_DURATION / chunk_ms)
        self.pre_buffer_maxlen = int(self.config.PRE_BUFFER_DURATION / chunk_ms)

        # Флаг активности. По умолчанию выключен (dialog-only kiosk)
        self.is_active = False

        # Потокобезопасность
        self.shutdown_event = threading.Event()

        # Очередь аудио и буферы
        self.audio_queue = queue.Queue()
        self.pre_buffer = deque(maxlen=self.pre_buffer_maxlen)
        self.audio_buffer = []

        self.is_speaking = False
        self.silence_counter = 0

        # VAD состояния
        self.vad_h = np.zeros((2, 1, VAD_STATE_SIZE), dtype=np.float32)
        self.vad_c = np.zeros((2, 1, VAD_STATE_SIZE), dtype=np.float32)

        # Модели
        self.recognizer: Optional[sherpa_onnx.OfflineRecognizer] = None
        self.vad_model: Optional[ort.InferenceSession] = None

        # Инициализация
        self._setup_ros_communication()
        self._setup_audio_system()
        self._load_models()

        # Запуск обработки и захвата
        self.worker_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.worker_thread.start()

        self._start_audio_capture()
        self._log_startup_info()

    def _setup_ros_communication(self) -> None:
        self.recognized_text_pub = self.create_publisher(String, 'recognized_text', 10)
        # TRANSIENT_LOCAL для получения последнего speech_control при подключении
        speech_control_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(Bool, 'speech_control', self._handle_activation, speech_control_qos)
        self.create_subscription(Bool, 'tts_control', self._handle_activation, 10)

    def _log_startup_info(self) -> None:
        self.get_logger().info(
            f'🎤 SpeechToTextGigaAMV3CTCNode (GigaAM v3 CTC - sherpa-onnx, {self.config.PROVIDER.upper()})'
        )
        self.get_logger().info(f'   Block size: {self.config.BLOCK_SIZE} (32ms)')
        self.get_logger().info(f'   VAD Threshold: {self.config.VAD_THRESHOLD}')

    def _setup_audio_system(self) -> None:
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if any(name in device['name'] for name in ['ReSpeaker', 'ArrayUAC10']):
                if device['max_input_channels'] > 0:
                    self.audio_device = i
                    self.get_logger().info(f'✓ ReSpeaker: {device["name"]}')
                    return

        # Fallback
        self.audio_device = None
        for i in [1] + list(range(len(devices))):
            if i < len(devices) and devices[i]['max_input_channels'] > 0:
                self.audio_device = i
                self.get_logger().info(f'✓ Device {i}: {devices[i]["name"]}')
                return

    def _load_models(self) -> None:
        asr_path, tokens_path, vad_path = self._resolve_model_paths()
        if not all([asr_path, tokens_path, vad_path]):
            raise RuntimeError('Models not found')

        # ASR Recognizer (sherpa-onnx NeMo CTC: fbank + CMVN + CTC decode с расстановкой слов)
        try:
            self.recognizer = sherpa_onnx.OfflineRecognizer.from_nemo_ctc(
                model=asr_path,
                tokens=tokens_path,
                num_threads=self.config.NUM_THREADS,
                sample_rate=self.config.SAMPLE_RATE,
                feature_dim=self.config.FEATURE_DIM,
                decoding_method=self.config.DECODING_METHOD,
                provider=self.config.PROVIDER,
            )
            self.get_logger().info('✓ GigaAM v3 CTC (sherpa-onnx) loaded')
        except Exception as e:
            self.get_logger().error(f'ASR Load Error: {e}')
            raise

        # VAD Session (CPU usually best for small chunks latency)
        vad_opts = ort.SessionOptions()
        vad_opts.intra_op_num_threads = 1
        self.vad_model = ort.InferenceSession(vad_path, sess_options=vad_opts, providers=['CPUExecutionProvider'])
        self.get_logger().info('✓ VAD loaded')

    def _resolve_model_paths(self):
        try:
            share = os.environ.get('VERTER_ASSETS_DIR', os.path.expanduser('~/verter-assets'))
            base = os.path.join(share, self.config.MODEL_DIR)
            paths = [
                os.path.join(base, self.config.MODEL_NAME),
                os.path.join(base, self.config.TOKENS_NAME),
                os.path.join(share, self.config.VAD_MODEL),
            ]
            return tuple(paths) if all(os.path.exists(p) for p in paths) else (None, None, None)
        except:
            return None, None, None

    def _start_audio_capture(self) -> None:
        self.audio_thread = threading.Thread(target=self._audio_capture_loop, daemon=True)
        self.audio_thread.start()

    def _audio_capture_loop(self) -> None:
        try:
            with sd.InputStream(
                samplerate=self.config.SAMPLE_RATE,
                blocksize=self.config.BLOCK_SIZE,
                device=self.audio_device,
                dtype='float32',
                channels=self.config.CHANNELS,
                callback=self._audio_callback,
            ):
                self.shutdown_event.wait()
        except Exception as e:
            self.get_logger().error(f'Audio Error: {e}')

    def _audio_callback(self, indata, frames, time, status):
        if status:
            pass
        self.audio_queue.put(indata.copy())

    def _processing_loop(self) -> None:
        """Цикл обработки: VAD -> Буферизация -> Распознавание"""
        while not self.shutdown_event.is_set():
            try:
                chunk = self.audio_queue.get(timeout=1.0)
            except queue.Empty:
                continue

            if not self.is_active:
                self.audio_buffer = []
                self.pre_buffer.clear()
                self.is_speaking = False
                continue

            self._process_chunk_logic(chunk)

    def _process_chunk_logic(self, chunk: np.ndarray) -> None:
        flat = chunk.flatten()

        # VAD
        prob = self._vad_predict(flat)
        is_speech = prob > self.config.VAD_THRESHOLD

        if is_speech:
            if not self.is_speaking:
                self.is_speaking = True
                self.audio_buffer.extend(self.pre_buffer)
                self.pre_buffer.clear()

            self.audio_buffer.append(flat)
            self.silence_counter = 0

        elif self.is_speaking:
            self.audio_buffer.append(flat)
            self.silence_counter += 1

            if self.silence_counter >= self.silence_chunks:
                self._recognize_buffer()
                self.is_speaking = False
                self.silence_counter = 0
                self.audio_buffer = []
        else:
            self.pre_buffer.append(flat)

    def _vad_predict(self, chunk: np.ndarray) -> float:
        target = self.config.BLOCK_SIZE
        if len(chunk) != target:
            chunk = np.pad(chunk, (0, max(0, target - len(chunk))))[:target]

        inputs = {'x': chunk.reshape(1, -1), 'h': self.vad_h, 'c': self.vad_c}
        outs = self.vad_model.run(None, inputs)
        self.vad_h, self.vad_c = outs[1], outs[2]
        return float(outs[0][0][0])

    def _recognize_buffer(self) -> None:
        if not self.audio_buffer:
            return

        try:
            t_start = time.time()
            full_audio = np.concatenate(self.audio_buffer)

            # sherpa-onnx: принимает сырые float32 сэмплы, сам делает fbank + CMVN + CTC decode
            stream = self.recognizer.create_stream()
            stream.accept_waveform(self.config.SAMPLE_RATE, full_audio)
            self.recognizer.decode_stream(stream)
            text = stream.result.text.strip()

            duration = time.time() - t_start

            # Фильтр мусора: отсекаем однобуквенные «плевки» от ложных VAD-срабатываний
            if text and len(text) >= 2:
                self._publish_text(text, duration)

        except Exception as e:
            self.get_logger().error(f'Recog Error: {e}')

    def _publish_text(self, text: str, duration: float = 0.0):
        if not self.is_active:
            return
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().info(f'🎤 GigaAM v3 CTC ({duration:.3f}s): {text}')

    def _handle_activation(self, msg: Bool):
        # is_active — bool, чтение/запись атомарны под GIL; lock не нужен.
        self.is_active = msg.data
        status = 'ON' if self.is_active else 'OFF'
        self.get_logger().info(f'State: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechToTextGigaAMV3CTCNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
