#!/usr/bin/env python3
"""
ROS2 Speech-to-Text node using GigaAM v3 RNN-Transducer (with punctuation) model.

Модель: sherpa-onnx-nemo-transducer-punct-giga-am-v3-russian-2025-12-16
        (encoder.int8.onnx + decoder.onnx + joiner.onnx + tokens.txt)
В отличие от CTC-варианта, transducer с пунктуацией сам расставляет пробелы
и знаки препинания — ручной greedy-декодер не нужен.
Архитектура (VAD Silero → буферизация → распознавание → публикация в recognized_text)
повторяет speech_to_text_sherpa_node.py, но с диалоговым режимом v3-ноды
(is_active=False по умолчанию, TRANSIENT_LOCAL для speech_control).
"""

import os
import queue
import threading
import time
from collections import deque
from typing import Optional
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
    """Конфигурация Speech-to-Text распознавателя GigaAM v3 RNNT (punct)"""
    MODEL_DIR: str = 'sherpa-onnx-nemo-transducer-punct-giga-am-v3-russian-2025-12-16'
    ENCODER_NAME: str = 'encoder.int8.onnx'
    DECODER_NAME: str = 'decoder.onnx'
    JOINER_NAME: str = 'joiner.onnx'
    TOKENS_NAME: str = 'tokens.txt'
    VAD_MODEL: str = 'silero_vad.onnx'

    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 512  # 32ms @ 16kHz - нативный размер для Silero VAD
    CHANNELS: int = 1

    VAD_THRESHOLD: float = 0.7
    SILENCE_DURATION: float = 0.7  # Секунд тишины для конца фразы
    PRE_BUFFER_DURATION: float = 0.5  # Секунд предыстории (чтобы не глотать начало)

    NUM_THREADS: int = 4
    FEATURE_DIM: int = 64  # GigaAM (v2/v3) обучена с 64 mel bins (ONNX encoder input: [B, 64, T])
    DECODING_METHOD: str = "greedy_search"  # или "modified_beam_search"
    PROVIDER: str = "cpu"


# Константы
VAD_STATE_SIZE = 64


class SpeechToTextGigaAMV3RNNTNode(Node):
    """ROS2 узел для преобразования речи в текст через GigaAM v3 RNNT (punct)"""

    def __init__(self) -> None:
        super().__init__('speech_to_text_gigaam_v3_rnnt_node')

        self.config = SpeechToTextConfig()

        # Рассчитываем параметры в чанках
        chunk_ms = self.config.BLOCK_SIZE / self.config.SAMPLE_RATE
        self.silence_chunks = int(self.config.SILENCE_DURATION / chunk_ms)
        self.pre_buffer_maxlen = int(self.config.PRE_BUFFER_DURATION / chunk_ms)

        # Флаг активности. По умолчанию выключен (dialog-only kiosk: STT
        # включается только кнопкой «Начать» через speech_control=True).
        self.is_active = False

        # Потокобезопасность
        self.shutdown_event = threading.Event()

        # Очередь аудио и буферы
        self.audio_queue = queue.Queue()
        self.pre_buffer = deque(maxlen=self.pre_buffer_maxlen)
        self.audio_buffer = []

        self.is_speaking = False
        self.silence_counter = 0

        # VAD состояния (h, c)
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
        # TRANSIENT_LOCAL (latched): получаем последнее speech_control от
        # recognition_node при подключении — стартовое значение не теряется.
        speech_control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, 'speech_control', self._handle_activation, speech_control_qos)
        self.create_subscription(Bool, 'tts_control', self._handle_activation, 10)

    def _log_startup_info(self) -> None:
        self.get_logger().info(
            f"🎤 SpeechToTextGigaAMV3RNNTNode (GigaAM v3 RNNT punct - sherpa-onnx, {self.config.PROVIDER.upper()})"
        )
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE} ({self.config.BLOCK_SIZE / self.config.SAMPLE_RATE * 1000:.1f}ms)")
        self.get_logger().info(f"   VAD Threshold: {self.config.VAD_THRESHOLD}")

    def _setup_audio_system(self) -> None:
        self.audio_device = self._find_audio_device()

    def _find_audio_device(self) -> Optional[int]:
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if any(name in device['name'] for name in ['ReSpeaker', 'ArrayUAC10']):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ ReSpeaker: {device['name']}")
                    return i

        # Fallback
        for i in [1] + list(range(len(devices))):
            if i < len(devices) and devices[i]['max_input_channels'] > 0:
                self.get_logger().info(f"✓ Device {i}: {devices[i]['name']}")
                return i
        return None

    def _resolve_model_paths(self):
        try:
            share = get_package_share_directory('verter_admin')
            model_dir = os.path.join(share, self.config.MODEL_DIR)
            paths = {
                'encoder': os.path.join(model_dir, self.config.ENCODER_NAME),
                'decoder': os.path.join(model_dir, self.config.DECODER_NAME),
                'joiner': os.path.join(model_dir, self.config.JOINER_NAME),
                'tokens': os.path.join(model_dir, self.config.TOKENS_NAME),
                'vad': os.path.join(share, self.config.VAD_MODEL),
            }
            if not all(os.path.exists(p) for p in paths.values()):
                self.get_logger().error(f"Не найдены файлы моделей в {model_dir}")
                return None
            return paths
        except Exception as e:
            self.get_logger().error(f"Ошибка путей: {e}")
            return None

    def _load_models(self) -> None:
        paths = self._resolve_model_paths()
        if not paths:
            raise RuntimeError("Models not found")

        # ASR Recognizer (sherpa-onnx NeMo Transducer: fbank + CMVN + RNNT decode с пунктуацией)
        try:
            self.recognizer = sherpa_onnx.OfflineRecognizer.from_transducer(
                encoder=paths['encoder'],
                decoder=paths['decoder'],
                joiner=paths['joiner'],
                tokens=paths['tokens'],
                num_threads=self.config.NUM_THREADS,
                sample_rate=self.config.SAMPLE_RATE,
                feature_dim=self.config.FEATURE_DIM,
                decoding_method=self.config.DECODING_METHOD,
                provider=self.config.PROVIDER,
                model_type="nemo_transducer",
            )
            self.get_logger().info("✓ GigaAM v3 RNNT (sherpa-onnx) loaded")
        except Exception as e:
            self.get_logger().error(f"ASR Load Error: {e}")
            raise

        # VAD Session (CPU usually best for small chunks latency)
        vad_opts = ort.SessionOptions()
        vad_opts.intra_op_num_threads = 1
        vad_opts.inter_op_num_threads = 1
        vad_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.vad_model = ort.InferenceSession(
            paths['vad'],
            sess_options=vad_opts,
            providers=['CPUExecutionProvider'],
        )
        self.get_logger().info("✓ VAD loaded")

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
                callback=self._audio_callback
            ):
                self.shutdown_event.wait()
        except Exception as e:
            self.get_logger().error(f"Audio Error: {e}")

    def _audio_callback(self, indata, frames, time_info, status) -> None:
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

            try:
                self._process_chunk_logic(chunk)
            except Exception as e:
                self.get_logger().error(f"Ошибка в цикле обработки: {e}")

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

        inputs = {
            'x': chunk.reshape(1, -1),
            'h': self.vad_h,
            'c': self.vad_c
        }
        outs = self.vad_model.run(None, inputs)
        self.vad_h, self.vad_c = outs[1], outs[2]
        return float(outs[0][0][0])

    def _recognize_buffer(self) -> None:
        if not self.audio_buffer:
            return

        try:
            t_start = time.time()
            full_audio = np.concatenate(self.audio_buffer)

            # sherpa-onnx: принимает сырые float32 сэмплы, сам делает fbank + CMVN + RNNT decode
            stream = self.recognizer.create_stream()
            stream.accept_waveform(self.config.SAMPLE_RATE, full_audio)
            self.recognizer.decode_stream(stream)
            text = stream.result.text.strip()

            duration = time.time() - t_start

            # Фильтр мусора: отсекаем однобуквенные «плевки» от ложных VAD-срабатываний
            if text and len(text) >= 2:
                self._publish_text(text, duration)

        except Exception as e:
            self.get_logger().error(f"Recog Error: {e}")

    def _publish_text(self, text: str, duration: float = 0.0):
        if not self.is_active:
            return
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().info(f"🎤 GigaAM v3 RNNT ({duration:.3f}s): {text}")

    def _handle_activation(self, msg: Bool):
        # is_active — bool, чтение/запись атомарны под GIL; lock не нужен.
        self.is_active = msg.data
        status = "ON" if self.is_active else "OFF"
        self.get_logger().info(f"State: {status}")

    def _cleanup_resources(self) -> None:
        try:
            sd.stop()
        except Exception:
            pass

    def shutdown(self) -> None:
        self.shutdown_event.set()
        self._cleanup_resources()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechToTextGigaAMV3RNNTNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
