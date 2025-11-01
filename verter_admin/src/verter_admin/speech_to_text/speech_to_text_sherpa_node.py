#!/usr/bin/env python3
"""ROS2 узел для распознавания речи с использованием sherpa-onnx библиотеки"""

import os
import threading
import time
from typing import Optional
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
    BLOCK_SIZE: int = 1600  # 100ms чанки
    CHANNELS: int = 1
    VAD_THRESHOLD: float = 0.5
    SILENCE_CHUNKS: int = 6  # 0.6 сек паузы
    PRE_BUFFER_CHUNKS: int = 5  # 0.5 сек предыстории
    NUM_THREADS: int = 4
    FEATURE_DIM: int = 64  # GigaAM v2 использует 64 mel bins
    DECODING_METHOD: str = "greedy_search"  # или "modified_beam_search"
    PROVIDER: str = "cpu"  # или "cuda" если доступна GPU
    AUDIO_LATENCY: float = 0.2


class SpeechToTextSherpaNode(Node):
    """ROS2 узел для преобразования речи в текст через sherpa-onnx библиотеку"""
    
    def __init__(self) -> None:
        super().__init__('speech_to_text_sherpa_node')
        
        # Конфигурация
        self.config = SpeechToTextSherpaConfig()
        
        # Флаг активности (для остановки во время TTS)
        self.is_active = True
        self.active_lock = threading.Lock()
        
        # Потокобезопасность
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()
        
        # Буферы для VAD и накопления аудио
        self.audio_buffer = []
        self.pre_buffer = []
        self.is_speaking = False
        self.silence_counter = 0
        
        # VAD состояния
        self.vad_h = np.zeros((2, 1, 64), dtype=np.float32)
        self.vad_c = np.zeros((2, 1, 64), dtype=np.float32)
        
        # Инициализация
        self._setup_ros_communication()
        self._setup_audio_system()
        self._load_models()
        self._start_audio_capture()
        
        self.get_logger().info("🎤 SpeechToTextSherpaNode запущен (sherpa-onnx библиотека)")
        self.get_logger().info(f"   Sample rate: {self.config.SAMPLE_RATE}Hz")
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE} ({self.config.BLOCK_SIZE/self.config.SAMPLE_RATE*1000:.0f}ms)")
        self.get_logger().info(f"   VAD threshold: {self.config.VAD_THRESHOLD}")
        pause_ms = self.config.SILENCE_CHUNKS * self.config.BLOCK_SIZE / self.config.SAMPLE_RATE * 1000
        self.get_logger().info(f"   Pause trigger: {pause_ms:.0f}ms ⚡")
        self.get_logger().info(f"   Threads: {self.config.NUM_THREADS}")
        self.get_logger().info(f"   Decoding method: {self.config.DECODING_METHOD}")
        self.get_logger().info("   🔀 Асинхронное распознавание через sherpa-onnx")
    
    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 коммуникации"""
        # Publisher для распознанного текста
        self.recognized_text_pub = self.create_publisher(
            String, 
            'recognized_text', 
            10
        )
        
        # Subscribers для управления активностью
        self.create_subscription(
            Bool,
            'speech_control',
            self._handle_activation,
            10
        )
        self.create_subscription(
            Bool,
            'tts_control',
            self._handle_activation,
            10
        )
    
    def _setup_audio_system(self) -> None:
        """Настройка аудио системы"""
        self.audio_device = self._find_audio_device()
    
    def _find_audio_device(self) -> Optional[int]:
        """Поиск подходящего аудио устройства"""
        devices = sd.query_devices()
        
        # Приоритет: ReSpeaker → устройство 1 → первое доступное
        for i, device in enumerate(devices):
            if any(name in device['name'] for name in ['ReSpeaker', 'ArrayUAC10']):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Найден ReSpeaker: {device['name']}")
                    return i
        
        # Fallback стратегия
        fallback_devices = [1] + list(range(len(devices)))
        for i in fallback_devices:
            if i < len(devices) and devices[i]['max_input_channels'] > 0:
                self.get_logger().info(f"✓ Используется устройство {i}: {devices[i]['name']}")
                return i
        
        self.get_logger().warn("⚠️ Входные устройства не найдены, используется дефолт")
        return None
    
    def _resolve_model_paths(self) -> tuple[Optional[str], Optional[str], Optional[str], Optional[str], Optional[str]]:
        """Определение путей к моделям"""
        try:
            package_share = get_package_share_directory('verter_admin')
            model_dir = os.path.join(package_share, self.config.MODEL_DIR)
            
            encoder_path = os.path.join(model_dir, self.config.ENCODER_NAME)
            decoder_path = os.path.join(model_dir, self.config.DECODER_NAME)
            joiner_path = os.path.join(model_dir, self.config.JOINER_NAME)
            tokens_path = os.path.join(model_dir, self.config.TOKENS_NAME)
            vad_model_path = os.path.join(package_share, self.config.VAD_MODEL)
            
            # Проверяем существование
            if not all(os.path.exists(p) for p in [encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path]):
                self.get_logger().error(f"Не найдены файлы в {model_dir}")
                return None, None, None, None, None
            
            return encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска моделей: {e}")
            return None, None, None, None, None
    
    def _check_decoder_metadata(self, decoder_path: str) -> bool:
        """Проверка метаданных декодера"""
        try:
            decoder_session = ort.InferenceSession(
                decoder_path,
                providers=["CPUExecutionProvider"],
            )
            meta = decoder_session.get_modelmeta().custom_metadata_map
            self.get_logger().debug(f"Decoder metadata: {meta}")
            
            # Проверяем наличие vocab_size
            if "vocab_size" not in meta:
                self.get_logger().warn(
                    "⚠️  WARNING: decoder.onnx не содержит 'vocab_size' в метаданных\n"
                    "   Это может вызвать ошибку при загрузке в sherpa-onnx\n"
                    "   Попробуйте использовать другую версию модели или переэкспортировать"
                )
                return False
            return True
        except Exception as e:
            self.get_logger().warn(f"⚠️  Не удалось проверить метаданные декодера: {e}")
            return None
    
    def _load_models(self) -> None:
        """Загрузка моделей Transducer ASR (sherpa-onnx) и VAD"""
        # Определяем пути
        encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path = self._resolve_model_paths()
        
        if not all([encoder_path, decoder_path, joiner_path, tokens_path, vad_model_path]):
            raise RuntimeError("Не найдены файлы моделей sherpa-onnx Transducer")
        
        # Проверяем метаданные перед загрузкой
        self.get_logger().info("🔄 Проверка метаданных модели...")
        metadata_ok = self._check_decoder_metadata(decoder_path)
        if metadata_ok is False:
            self.get_logger().warn(
                "\n❌ Проблема с метаданными модели. Попробуйте:\n"
                "   1. Использовать модель, экспортированную специально для sherpa-onnx\n"
                "   2. Или использовать ручную реализацию (speech_to_text_transducer_node.py)\n"
                "\nПопытка загрузки все равно будет выполнена...\n"
            )
        
        # Загрузка распознавателя sherpa-onnx
        self.get_logger().info("🔄 Загрузка Transducer модели (sherpa-onnx)...")
        try:
            self.recognizer = sherpa_onnx.OfflineRecognizer.from_transducer(
                encoder=encoder_path,
                decoder=decoder_path,
                joiner=joiner_path,
                tokens=tokens_path,
                num_threads=self.config.NUM_THREADS,
                sample_rate=self.config.SAMPLE_RATE,
                feature_dim=self.config.FEATURE_DIM,
                decoding_method=self.config.DECODING_METHOD,
                provider=self.config.PROVIDER,
                model_type="nemo_transducer",  # Важно для NeMo моделей!
            )
            self.get_logger().info("✓ Transducer модель загружена (sherpa-onnx)")
        except Exception as e:
            error_msg = str(e)
            if "vocab_size" in error_msg.lower() or "metadata" in error_msg.lower():
                self.get_logger().error(
                    f"\n❌ Ошибка загрузки модели: {e}\n"
                    "\nЭта модель NeMo может быть несовместима с sherpa-onnx OfflineRecognizer\n"
                    "Рекомендуется использовать ручную реализацию из speech_to_text_transducer_node.py\n"
                    "или найти модель, экспортированную специально для sherpa-onnx"
                )
            raise
        
        # Загрузка VAD модели
        self.get_logger().info("🔄 Загрузка VAD модели...")
        vad_opts = ort.SessionOptions()
        vad_opts.intra_op_num_threads = 2
        vad_opts.inter_op_num_threads = 2
        vad_opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        
        self.vad_model = ort.InferenceSession(
            vad_model_path,
            sess_options=vad_opts,
            providers=["CPUExecutionProvider"],
        )
        self.get_logger().info("✓ VAD модель загружена")
    
    def _start_audio_capture(self) -> None:
        """Запуск захвата аудио в отдельном потоке"""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        self.get_logger().info("✓ Захват аудио запущен")
    
    def _audio_capture_loop(self) -> None:
        """Основной цикл захвата аудио"""
        try:
            # Пробуем без latency
            with sd.InputStream(
                samplerate=self.config.SAMPLE_RATE,
                blocksize=self.config.BLOCK_SIZE,
                device=self.audio_device,
                dtype='float32',
                channels=self.config.CHANNELS,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Аудио поток успешно запущен")
                self.shutdown_event.wait()
        except Exception as e:
            # Если не получилось - пробуем с latency
            self.get_logger().warn(f"Ошибка без latency: {e}")
            self.get_logger().info("Попытка с латентностью...")
            try:
                with sd.InputStream(
                    samplerate=self.config.SAMPLE_RATE,
                    blocksize=self.config.BLOCK_SIZE,
                    device=self.audio_device,
                    dtype='float32',
                    channels=self.config.CHANNELS,
                    latency=self.config.AUDIO_LATENCY,
                    callback=self._audio_callback
                ):
                    self.get_logger().info("✓ Аудио поток запущен с latency")
                    self.shutdown_event.wait()
            except Exception as e2:
                self.get_logger().error(f"Критическая ошибка аудио: {e2}")
    
    def _audio_callback(self, indata, frames, time_info, status) -> None:
        """Callback обработки аудио данных"""
        if self.shutdown_event.is_set():
            return
        
        # Проверяем активность
        with self.active_lock:
            if not self.is_active:
                return
        
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            audio_chunk = indata.flatten().copy()
            self._process_audio_chunk(audio_chunk)
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")
    
    def _process_audio_chunk(self, audio_chunk: np.ndarray) -> None:
        """Обработка аудио чанка с VAD"""
        # VAD - определение речи
        speech_prob = self._vad_predict(audio_chunk)
        is_speech = speech_prob > self.config.VAD_THRESHOLD
        
        if is_speech:
            if not self.is_speaking:
                self.is_speaking = True
                self.audio_buffer = list(self.pre_buffer)
                self.pre_buffer = []
            self.audio_buffer.append(audio_chunk)
            self.silence_counter = 0
            
        elif self.is_speaking:
            # Продолжаем накапливать после речи (для ловли конца фразы)
            self.audio_buffer.append(audio_chunk)
            self.silence_counter += 1
            
            if self.silence_counter >= self.config.SILENCE_CHUNKS:
                # Конец фразы - запускаем распознавание АСИНХРОННО
                # Копируем буфер чтобы не блокировать audio callback
                audio_to_recognize = list(self.audio_buffer)
                threading.Thread(
                    target=self._recognize_accumulated_audio,
                    args=(audio_to_recognize,),
                    daemon=True
                ).start()
                
                self.is_speaking = False
                self.silence_counter = 0
                self.audio_buffer = []
        else:
            # Нет речи - сохраняем в предбуфер
            self.pre_buffer.append(audio_chunk)
            if len(self.pre_buffer) > self.config.PRE_BUFFER_CHUNKS:
                self.pre_buffer.pop(0)
    
    def _vad_predict(self, audio_chunk: np.ndarray) -> float:
        """VAD предсказание вероятности речи"""
        # Silero VAD требует 512 сэмплов для 16kHz
        if len(audio_chunk) < 512:
            audio_chunk = np.pad(audio_chunk, (0, 512 - len(audio_chunk)))
        audio_chunk = audio_chunk[:512].astype(np.float32)
        
        ort_inputs = {
            'x': audio_chunk.reshape(1, -1),
            'h': self.vad_h,
            'c': self.vad_c
        }
        
        ort_outs = self.vad_model.run(None, ort_inputs)
        prob = ort_outs[0][0][0]
        self.vad_h = ort_outs[1]
        self.vad_c = ort_outs[2]
        
        return float(prob)
    
    def _recognize_accumulated_audio(self, audio_buffer: list) -> None:
        """Распознавание накопленного аудио (в отдельном потоке!)"""
        if not audio_buffer:
            return
        
        start_time = time.time()
        with self.recognition_lock:
            try:
                # Объединяем все чанки
                full_audio = np.concatenate(audio_buffer).astype(np.float32)
                
                # Добавляем padding в конец для лучшего распознавания
                tail_padding = np.zeros(int(self.config.SAMPLE_RATE * 0.3), dtype=np.float32)
                full_audio = np.concatenate([full_audio, tail_padding])
                
                # Распознавание через sherpa-onnx
                t1 = time.time()
                text = self._recognize_audio(full_audio)
                inference_time = time.time() - t1
                
                total_time = time.time() - start_time
                
                if text:
                    self._publish_recognized_text(text)
                    self.get_logger().info(
                        f"⏱️  Timing: inference={inference_time*1000:.0f}ms, "
                        f"total={total_time*1000:.0f}ms"
                    )
            except Exception as e:
                self.get_logger().error(f"Ошибка распознавания: {e}")
    
    def _recognize_audio(self, audio_data: np.ndarray) -> str:
        """Распознавание аудио с помощью sherpa-onnx"""
        # Создаем stream для распознавания
        stream = self.recognizer.create_stream()
        
        # Подаем аудио в stream
        stream.accept_waveform(self.config.SAMPLE_RATE, audio_data)
        
        # Распознаем
        self.recognizer.decode_stream(stream)
        
        # Получаем результат
        result = stream.result
        
        return result.text if result.text else ""
    
    def _publish_recognized_text(self, text: str) -> None:
        """Публикация распознанного текста"""
        # Двойная проверка активности
        with self.active_lock:
            if not self.is_active:
                self.get_logger().debug(f"🔇 Пропущено (деактивирован): '{text}'")
                return
        
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().info(f"🎤 Распознано: '{text}'")
    
    def _handle_activation(self, msg: Bool) -> None:
        """Обработка команды активации/деактивации"""
        with self.active_lock:
            self.is_active = msg.data
        
        # Сбрасываем буферы при деактивации
        if not msg.data:
            self.audio_buffer = []
            self.pre_buffer = []
            self.is_speaking = False
            self.silence_counter = 0
        
        status = "включен" if msg.data else "выключен (TTS активен)"
        self.get_logger().info(f"📡 SpeechToText {status}")
    
    def _cleanup_resources(self) -> None:
        """Очистка ресурсов"""
        with self.recognition_lock:
            self.recognizer = None
            self.vad_model = None
        
        try:
            sd.stop()
        except Exception:
            pass
    
    def shutdown(self) -> None:
        """Корректное завершение работы"""
        try:
            self.get_logger().info("🔄 Завершение работы SpeechToTextSherpaNode...")
            self.shutdown_event.set()
            self._cleanup_resources()
            self.get_logger().info("✅ SpeechToTextSherpaNode завершен")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения: {e}")


def main(args=None) -> None:
    """Главная функция запуска"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechToTextSherpaNode()
        print("🎤 SPEECH-TO-TEXT NODE (sherpa-onnx библиотека)")
        print("   Публикация в топик: recognized_text")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔄 Завершение по Ctrl+C...")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Система завершена")


if __name__ == '__main__':
    main()

