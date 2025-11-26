#!/usr/bin/env python3
import os
import queue
import threading
import time
from typing import Optional, Tuple, Dict, List
from collections import deque
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import kaldi_native_fbank as knf
import onnxruntime as ort
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool


@dataclass
class SpeechToTextConfig:
    """Конфигурация Speech-to-Text распознавателя"""
    MODEL_DIR: str = 'sherpa-onnx-nemo-ctc-giga-am-v2-russian-2025-04-19'
    MODEL_NAME: str = 'model.int8.onnx'
    TOKENS_NAME: str = 'tokens.txt'
    VAD_MODEL: str = 'silero_vad.onnx'
    
    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 512  # 32ms - нативный размер для Silero VAD
    CHANNELS: int = 1
    
    VAD_THRESHOLD: float = 0.5
    SILENCE_DURATION: float = 0.6  # Секунд тишины для конца фразы
    PRE_BUFFER_DURATION: float = 0.5  # Секунд предыстории
    
    NUM_THREADS: int = 6
    AUDIO_LATENCY: float = 0.2
    USE_CUDA: bool = True


# Константы
VAD_INPUT_SIZE = 512
FBANK_FEATURE_SIZE = 64
VAD_STATE_SIZE = 64


class SpeechToTextNode(Node):
    """ROS2 узел для преобразования речи в текст через Sherpa-ONNX CTC (Optimized)"""
    
    def __init__(self) -> None:
        super().__init__('speech_to_text_node')
        
        self.config = SpeechToTextConfig()
        
        # Рассчитываем параметры в чанках
        chunk_ms = self.config.BLOCK_SIZE / self.config.SAMPLE_RATE
        self.silence_chunks = int(self.config.SILENCE_DURATION / chunk_ms)
        self.pre_buffer_maxlen = int(self.config.PRE_BUFFER_DURATION / chunk_ms)
        
        # Флаг активности
        self.is_active = True
        self.active_lock = threading.Lock()
        
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
        self.asr_model: Optional[ort.InferenceSession] = None
        self.vad_model: Optional[ort.InferenceSession] = None
        self.id2token: Dict[int, str] = {}
        self.blank_id: int = 0
        self.max_token_id: int = 0
        
        # Кэши имен ONNX
        self.asr_input_names: List[str] = []
        self.asr_output_names: List[str] = []
        self.fbank_opts: Optional[knf.FbankOptions] = None
        
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
        self.create_subscription(Bool, 'speech_control', self._handle_activation, 10)
        self.create_subscription(Bool, 'tts_control', self._handle_activation, 10)
    
    def _log_startup_info(self) -> None:
        mode_str = "CUDA" if self.config.USE_CUDA else "CPU"
        self.get_logger().info(f"🎤 SpeechToTextNode (CTC Optimized - {mode_str})")
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE} (32ms)")
        self.get_logger().info(f"   VAD Threshold: {self.config.VAD_THRESHOLD}")
    
    def _setup_audio_system(self) -> None:
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if any(name in device['name'] for name in ['ReSpeaker', 'ArrayUAC10']):
                if device['max_input_channels'] > 0:
                    self.audio_device = i
                    self.get_logger().info(f"✓ ReSpeaker: {device['name']}")
                    return
        
        # Fallback
        self.audio_device = None
        for i in [1] + list(range(len(devices))):
             if i < len(devices) and devices[i]['max_input_channels'] > 0:
                self.audio_device = i
                self.get_logger().info(f"✓ Device {i}: {devices[i]['name']}")
                return
    
    def _load_models(self) -> None:
        asr_path, tokens_path, vad_path = self._resolve_model_paths()
        if not all([asr_path, tokens_path, vad_path]):
            raise RuntimeError("Models not found")

        # ASR Session Options
        sess_opts = ort.SessionOptions()
        sess_opts.intra_op_num_threads = self.config.NUM_THREADS
        sess_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        # Автоматическая проверка доступности CUDA
        available_providers = ort.get_available_providers()
        if self.config.USE_CUDA and 'CUDAExecutionProvider' in available_providers:
            providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            self.get_logger().info(f"Используется CUDA (найдено в {available_providers})")
        else:
            providers = ['CPUExecutionProvider']
            if self.config.USE_CUDA:
                self.get_logger().warn(f"CUDA не найдена или отключена. Доступно: {available_providers}. Использую CPU.")
        
        try:
            self.asr_model = ort.InferenceSession(asr_path, sess_options=sess_opts, providers=providers)
            self.get_logger().info(f"✓ ASR loaded ({self.asr_model.get_providers()[0]})")
        except Exception as e:
            self.get_logger().error(f"ASR Load Error: {e}")
            raise

        # VAD Session (CPU usually best for small chunks latency)
        vad_opts = ort.SessionOptions()
        vad_opts.intra_op_num_threads = 1
        self.vad_model = ort.InferenceSession(vad_path, sess_options=vad_opts, providers=['CPUExecutionProvider'])
        self.get_logger().info("✓ VAD loaded")

        # Tokens
        self.id2token = self._load_tokens(tokens_path)
        self.blank_id = len(self.id2token) - 1
        self.max_token_id = max(self.id2token.keys()) if self.id2token else 0

        # Metadata caches
        self.asr_input_names = [x.name for x in self.asr_model.get_inputs()]
        self.asr_output_names = [x.name for x in self.asr_model.get_outputs()]
        self.fbank_opts = self._create_fbank_options()

    def _resolve_model_paths(self):
        try:
            share = get_package_share_directory('verter_admin')
            base = os.path.join(share, self.config.MODEL_DIR)
            paths = [
                os.path.join(base, self.config.MODEL_NAME),
                os.path.join(base, self.config.TOKENS_NAME),
                os.path.join(share, self.config.VAD_MODEL)
            ]
            return tuple(paths) if all(os.path.exists(p) for p in paths) else (None, None, None)
        except:
            return None, None, None

    def _load_tokens(self, path: str) -> Dict[int, str]:
        mapping = {}
        with open(path, encoding="utf-8") as f:
            for line in f:
                parts = line.split()
                if len(parts) > 1:
                    mapping[int(parts[1])] = parts[0]
                elif len(parts) == 1:
                    mapping[int(parts[0])] = " "
        return mapping

    def _create_fbank_options(self) -> knf.FbankOptions:
        opts = knf.FbankOptions()
        opts.frame_opts.dither = 0
        opts.frame_opts.remove_dc_offset = False
        opts.frame_opts.window_type = "hann"
        opts.mel_opts.num_bins = FBANK_FEATURE_SIZE
        return opts

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

    def _audio_callback(self, indata, frames, time, status):
        if status: pass
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
        if len(chunk) != VAD_INPUT_SIZE:
            chunk = np.pad(chunk, (0, max(0, VAD_INPUT_SIZE - len(chunk))))[:VAD_INPUT_SIZE]
            
        inputs = {
            'x': chunk.reshape(1, -1),
            'h': self.vad_h,
            'c': self.vad_c
        }
        outs = self.vad_model.run(None, inputs)
        self.vad_h, self.vad_c = outs[1], outs[2]
        return float(outs[0][0][0])

    def _recognize_buffer(self) -> None:
        if not self.audio_buffer: return
        
        try:
            t_start = time.time()
            full_audio = np.concatenate(self.audio_buffer)
            
            # Fbank
            fbank = knf.OnlineFbank(self.fbank_opts)
            fbank.accept_waveform(self.config.SAMPLE_RATE, full_audio)
            
            if fbank.num_frames_ready == 0: return
            
            features = np.empty((fbank.num_frames_ready, FBANK_FEATURE_SIZE), dtype=np.float32)
            for i in range(fbank.num_frames_ready):
                features[i] = fbank.get_frame(i)
            
            # Inference
            text = self._run_ctc_inference(features)
            duration = time.time() - t_start
            
            if text:
                self._publish_text(text, duration)
                
        except Exception as e:
            self.get_logger().error(f"Recog Error: {e}")

    def _run_ctc_inference(self, features: np.ndarray) -> str:
        x = np.expand_dims(features.T, axis=0) # (1, Features, Time) ? Check model format
        # GigaAM / NeMo models usually expect [B, Feature, Time] or [B, Time, Feature]
        # The previous code had: x = np.expand_dims(features.T, axis=0) -> [1, 64, T]
        # Let's verify. NeMo usually is [B, D, T]. 
        
        x_lens = np.array([x.shape[-1]], dtype=np.int64)
        
        logits = self.asr_model.run(
            self.asr_output_names[:1],
            {self.asr_input_names[0]: x, self.asr_input_names[1]: x_lens}
        )[0] # [B, T, Vocab]
        
        # Greedy Decode
        ids = np.argmax(logits[0], axis=1)
        tokens = []
        prev = -1
        for tid in ids:
            if tid == self.blank_id or tid == prev:
                prev = tid
                continue
            if tid in self.id2token:
                tokens.append(self.id2token[tid])
            prev = tid
            
        return "".join(tokens).strip()

    def _publish_text(self, text: str, duration: float = 0.0):
        if not self.is_active: return
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().info(f"🎤 ({duration:.3f}s): {text}")

    def _handle_activation(self, msg: Bool):
        with self.active_lock:
            self.is_active = msg.data
        status = "ON" if msg.data else "OFF"
        self.get_logger().info(f"State: {status}")

    def shutdown(self):
        self.shutdown_event.set()
        sd.stop()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechToTextNode()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    except Exception as e: print(f"Error: {e}")
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
