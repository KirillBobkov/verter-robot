#!/usr/bin/env python3
"""ROS2 узел для распознавания речи с использованием sherpa-onnx библиотеки"""

import os
import queue
import threading
import time
from collections import deque
from typing import Optional, List
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
import sounddevice as sd
import numpy as np
import onnxruntime as ort
import sherpa_onnx
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool


@dataclass
class SpeechToTextSherpaConfig:
    """Конфигурация Speech-to-Text распознавателя с sherpa-onnx"""
    MODEL_DIR: str = 'sherpa-onnx-nemo-transducer-giga-am-v2-russian-2025-04-19'
    ENCODER_NAME: str = 'encoder.int8.onnx'
    DECODER_NAME: str = 'decoder.onnx'
    JOINER_NAME: str = 'joiner.onnx'
    TOKENS_NAME: str = 'tokens.txt'
    VAD_MODEL: str = 'silero_vad.onnx'
    
    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 512  # 32ms @ 16kHz - нативный размер для Silero VAD
    CHANNELS: int = 1
    
    VAD_THRESHOLD: float = 0.7 # пороговое значение для определения речи
    SILENCE_DURATION: float = 0.7  # Секунд тишины для определения конца фразы
    PRE_BUFFER_DURATION: float = 0.5  # Секунд предыстории (чтобы не глотать начало)
    
    NUM_THREADS: int = 6
    FEATURE_DIM: int = 64  # GigaAM v2 использует 64 mel bins
    DECODING_METHOD: str = "greedy_search"  # или "modified_beam_search"
    PROVIDER: str = "cpu"  # или "cuda" если доступна GPU

class SpeechToTextSherpaNode(Node):
    """ROS2 узел для преобразования речи в текст через sherpa-onnx библиотеку"""
    
    def __init__(self) -> None:
        super().__init__('speech_to_text_sherpa_node')
        
        # Конфигурация
        self.config = SpeechToTextSherpaConfig()
        
        # Рассчитываем параметры в чанках
        chunk_ms = self.config.BLOCK_SIZE / self.config.SAMPLE_RATE
        self.silence_chunks = int(self.config.SILENCE_DURATION / chunk_ms)
        self.pre_buffer_maxlen = int(self.config.PRE_BUFFER_DURATION / chunk_ms)
        
        # Флаг активности
        self.is_active = True
        self.active_lock = threading.Lock()
        
        # Очередь и буферы
        self.audio_queue = queue.Queue()
        self.pre_buffer = deque(maxlen=self.pre_buffer_maxlen)
        self.audio_buffer = []
        
        self.is_speaking = False
        self.silence_counter = 0
        
        # Потокобезопасность
        self.shutdown_event = threading.Event()
        
        # VAD состояния (h, c)
        self.vad_h = np.zeros((2, 1, 64), dtype=np.float32)
        self.vad_c = np.zeros((2, 1, 64), dtype=np.float32)
        
        # Инициализация
        self._setup_ros_communication()
        self._setup_audio_system()
        self._load_models()
        
        # Запуск worker-потока
        self.worker_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.worker_thread.start()
        
        # Запуск захвата аудио
        self._start_audio_capture()
        
        self.get_logger().info("🎤 SpeechToTextSherpaNode запущен (Optimized)")
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE} ({chunk_ms*1000:.1f}ms)")
        self.get_logger().info(f"   VAD Check Rate: {1/chunk_ms:.1f} Hz")
        self.get_logger().info(f"   Silence Trigger: {self.silence_chunks} chunks ({self.config.SILENCE_DURATION}s)")
    
    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 коммуникации"""
        self.recognized_text_pub = self.create_publisher(String, 'recognized_text', 10)
        self.create_subscription(Bool, 'speech_control', self._handle_activation, 10)
        self.create_subscription(Bool, 'tts_control', self._handle_activation, 10)
    
    def _setup_audio_system(self) -> None:
        """Настройка аудио системы"""
        self.audio_device = self._find_audio_device()
    
    def _find_audio_device(self) -> Optional[int]:
        """Поиск подходящего аудио устройства"""
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if any(name in device['name'] for name in ['ReSpeaker', 'ArrayUAC10']):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Найден ReSpeaker: {device['name']}")
                    return i
        
        fallback_devices = [1] + list(range(len(devices)))
        for i in fallback_devices:
            if i < len(devices) and devices[i]['max_input_channels'] > 0:
                self.get_logger().info(f"✓ Используется устройство {i}: {devices[i]['name']}")
                return i
        return None
    
    def _resolve_model_paths(self):
        """Определение путей к моделям"""
        try:
            package_share = get_package_share_directory('verter_admin')
            model_dir = os.path.join(package_share, self.config.MODEL_DIR)
            
            files = {
                'encoder': os.path.join(model_dir, self.config.ENCODER_NAME),
                'decoder': os.path.join(model_dir, self.config.DECODER_NAME),
                'joiner': os.path.join(model_dir, self.config.JOINER_NAME),
                'tokens': os.path.join(model_dir, self.config.TOKENS_NAME),
                'vad': os.path.join(package_share, self.config.VAD_MODEL)
            }
            
            if not all(os.path.exists(p) for p in files.values()):
                self.get_logger().error(f"Не найдены файлы моделей в {model_dir}")
                return None
            return files
        except Exception as e:
            self.get_logger().error(f"Ошибка путей: {e}")
            return None
            
    def _load_models(self) -> None:
        """Загрузка моделей"""
        paths = self._resolve_model_paths()
        if not paths:
            raise RuntimeError("Models not found")
            
        # Загрузка Recognizer
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
            self.get_logger().info("✓ Sherpa-onnx Transducer загружен")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки recognizer: {e}")
            raise

        # Загрузка VAD
        vad_opts = ort.SessionOptions()
        vad_opts.intra_op_num_threads = 1
        vad_opts.inter_op_num_threads = 1
        vad_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        self.vad_model = ort.InferenceSession(
            paths['vad'],
            sess_options=vad_opts,
            providers=["CPUExecutionProvider"],
        )
        self.get_logger().info("✓ VAD модель загружена")

    def _start_audio_capture(self) -> None:
        """Запуск захвата аудио"""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
    
    def _audio_capture_loop(self) -> None:
        """Цикл захвата аудио (Producer)"""
        try:
            with sd.InputStream(
                samplerate=self.config.SAMPLE_RATE,
                blocksize=self.config.BLOCK_SIZE,
                device=self.audio_device,
                dtype='float32',
                channels=self.config.CHANNELS,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Аудио поток активен")
                self.shutdown_event.wait()
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио потока: {e}")
            # Retry with latency if needed logic could be here
    
    def _audio_callback(self, indata, frames, time_info, status) -> None:
        """Callback (только кладет в очередь)"""
        if status:
            pass  # Можно логировать, но аккуратно
        self.audio_queue.put(indata.copy())

    def _processing_loop(self) -> None:
        """Основной цикл обработки (Consumer: VAD -> Recognition)"""
        while not self.shutdown_event.is_set():
            # 1. Получаем данные
            try:
                chunk = self.audio_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            
            # 2. Проверка активности
            if not self.is_active:
                # Если выключены - просто очищаем очередь
                self.audio_buffer = []
                self.pre_buffer.clear()
                self.is_speaking = False
                continue

            # 3. Обработка
            try:
                self._process_chunk_logic(chunk)
            except Exception as e:
                self.get_logger().error(f"Ошибка в цикле обработки: {e}")

    def _process_chunk_logic(self, chunk: np.ndarray) -> None:
        """Логика VAD и буферизации"""
        # Flatten для обработки
        flat_chunk = chunk.flatten()
        
        # VAD Inference
        prob = self._vad_predict(flat_chunk)
        is_speech = prob > self.config.VAD_THRESHOLD
        
        if is_speech:
            if not self.is_speaking:
                # Начало речи
                self.is_speaking = True
                # Переносим предысторию в основной буфер
                self.audio_buffer.extend(self.pre_buffer)
                self.pre_buffer.clear()
            
            self.audio_buffer.append(flat_chunk)
            self.silence_counter = 0
            
        elif self.is_speaking:
            # Речь была, но сейчас тишина -> ждем окончания фразы
            self.audio_buffer.append(flat_chunk)
            self.silence_counter += 1
            
            if self.silence_counter >= self.silence_chunks:
                # Конец фразы подтвержден
                self._recognize_buffer()
                
                # Сброс
                self.is_speaking = False
                self.silence_counter = 0
                self.audio_buffer = []
        else:
            # Просто шум/тишина -> в кольцевой буфер
            self.pre_buffer.append(flat_chunk)

    def _vad_predict(self, audio_chunk: np.ndarray) -> float:
        """VAD Inference (оптимизирован под BLOCK_SIZE=512)"""
        # Silero требует 512 сэмплов. Если BLOCK_SIZE=512, паддинг не нужен.
        # Если что-то пошло не так и размер другой - паддим.
        if len(audio_chunk) != 512:
            if len(audio_chunk) < 512:
                audio_chunk = np.pad(audio_chunk, (0, 512 - len(audio_chunk)))
            else:
                audio_chunk = audio_chunk[:512]
        
        inputs = {
            'x': audio_chunk.reshape(1, -1),
            'h': self.vad_h,
            'c': self.vad_c
        }
        
        outs = self.vad_model.run(None, inputs)
        self.vad_h, self.vad_c = outs[1], outs[2]
        return float(outs[0][0][0])

    def _recognize_buffer(self) -> None:
        """Распознавание накопленного буфера"""
        if not self.audio_buffer:
            return
            
        try:
            full_audio = np.concatenate(self.audio_buffer)
            
            # Распознавание
            t1 = time.time()
            stream = self.recognizer.create_stream()
            stream.accept_waveform(self.config.SAMPLE_RATE, full_audio)
            self.recognizer.decode_stream(stream)
            raw_text = stream.result.text
            
            dur = time.time() - t1
            
            # 2. ФИЛЬТР: Очистка и проверка
            text = raw_text.strip()
            
            # Игнорируем пустоту и слишком короткие "плевки" (менее 2 символов)
            if text and len(text) >= 2:
                self._publish_text(text)
                self.get_logger().info(f"⏱️  Inference: {dur*1000:.0f}ms | Text len: {len(text)}")
            else:
                if raw_text:
                    self.get_logger().debug(f"Отфильтрован мусор: '{raw_text}'")
                
        except Exception as e:
            self.get_logger().error(f"Ошибка распознавания: {e}")

    def _publish_text(self, text: str) -> None:
        if not self.is_active: 
            return
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().info(f"🎤 Распознано: '{text}'")

    def _handle_activation(self, msg: Bool) -> None:
        """Обработка включения/выключения"""
        with self.active_lock:
            self.is_active = msg.data
        
        status = "включен" if msg.data else "выключен"
        self.get_logger().info(f"📡 SpeechToText {status}")

    def _cleanup_resources(self) -> None:
        try:
            sd.stop()
        except:
            pass

    def shutdown(self) -> None:
        self.shutdown_event.set()
        self._cleanup_resources()

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechToTextSherpaNode()
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
