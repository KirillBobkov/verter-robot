#!/usr/bin/env python3

import os
import subprocess
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory
import sounddevice as sd

try:
    import torch
    import numpy as np
    from ru_normalizr import Normalizer, NormalizeOptions
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    torch = None
    np = None
    Normalizer = None
    NormalizeOptions = None


class SileroTTSNode(Node):
    
    def __init__(self):
        super().__init__('silero_tts_node')
        
        if not TORCH_AVAILABLE:
            self.get_logger().error("torch не установлен. Установите: pip install torch torchaudio")
            raise ImportError("torch не найден")
            
        # Оптимизация для CPU (Jetson/RPi)
        torch.set_num_threads(4)

        # Параметры ROS
        self.declare_parameter('speaker', 'eugene')
        self.declare_parameter('sample_rate', 24000)
        self.declare_parameter('model_version', 'v5_5_ru')  # v5_ru или v5_5_ru
        self.declare_parameter('audio_device', 'pulse')

        # Настройки Silero v5
        self.speaker = self.get_parameter('speaker').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().integer_value
        self.model_version = self.get_parameter('model_version').get_parameter_value().string_value
        self.audio_device = self.get_parameter('audio_device').get_parameter_value().string_value

        # Валидация диктора
        valid_speakers = ['aidar', 'baya', 'kseniya', 'xenia', 'eugene']
        if self.speaker not in valid_speakers:
            self.get_logger().warning(f"Неверный диктор: {self.speaker}. Доступные: {valid_speakers}. Используем 'xenia'")
            self.speaker = 'eugene'

        # Валидация версии модели
        valid_models = ['v5_ru', 'v5_5_ru']
        if self.model_version not in valid_models:
            self.get_logger().warning(f"Неверная версия модели: {self.model_version}. Используем 'v5_ru'")
            self.model_version = 'v5_ru'

        # Путь к локальной модели (в той же директории что и нода)
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech')
            self.model_path = os.path.join(tts_models_dir, f"{self.model_version}.pt")
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска модели: {e}")
            raise

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Файл модели не найден: {self.model_path}")
            raise FileNotFoundError("Модель Silero не найдена")
        
        self._setup_environment()
        self._initialize_tts()
        
        # ROS2 подписки
        self.subscription = self.create_subscription(String, 'text_to_speech', self.text_callback, 10)
        # Publisher для управления распознаванием
        self.tts_control_pub = self.create_publisher(Bool, 'tts_control', 10)
        
        # Управление конкуренцией
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None
        self._current_process = None  # Для возможности прерывания
        
        self.get_logger().info(f"Silero TTS v5_5 готов. Модель: {self.model_version}, Диктор: {self.speaker}, Sample rate: {self.sample_rate}")
    
    def _initialize_tts(self):
        """Загрузка локальной модели Silero"""
        try:
            self.get_logger().info(f"Загрузка локальной модели: {self.model_path}")
            start_load = time.time()

            # Загрузка локальной модели из .pt файла (как в примере с локальной моделью)
            importer = torch.package.PackageImporter(self.model_path)
            self.model = importer.load_pickle("tts_models", "model")

            load_time = time.time() - start_load
            self.get_logger().info(f"✓ Модель Silero загружена за {load_time:.2f} сек")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise

        # Инициализация ru-normalizr для контекстного склонения числительных (режим TTS)
        if Normalizer is None:
            self.get_logger().warning("ru-normalizr не установлен. Установите: pip install ru-normalizr")
            self.normalizer = None
        else:
            try:
                self.normalizer = Normalizer(NormalizeOptions.tts())
                self.get_logger().info("ru-normalizr инициализирован")
            except Exception as e:
                self.get_logger().error(f"Ошибка инициализации нормализатора: {e}")
                self.normalizer = None
    
    def _setup_environment(self):
        """Настройка окружения для аудио"""
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
    
    def text_callback(self, msg):
        """Обработка входящих сообщений с текстом"""
        text = msg.data.strip()
        if not text:
            return
        
        start_new = False
        with self._busy_lock:
            if self._busy:
                self._pending_text = text
            else:
                self._busy = True
                start_new = True

        if start_new:
            threading.Thread(target=self._speak_loop, args=(text,), daemon=True).start()

    def _normalize_text(self, text):
        """Нормализация текста для TTS с контекстным склонением (годы, даты, числа, аббревиатуры)"""
        if not text or self.normalizer is None:
            return text

        try:
            # ru-normalizr (режим TTS) обрабатывает: годы, даты, время, валюты, проценты,
            # сокращения, римские цифры, аббревиатуры, латиницу с контекстным склонением
            normalized = self.normalizer.normalize(text)
            if normalized != text:
                self.get_logger().info(f"Нормализация текста: \"{text[:50]}...\" → \"{normalized[:50]}...\"")
            return normalized
        except Exception as e:
            self.get_logger().warning(f"ru-normalizr ошибка: {e}, возвращаем оригинал")
            return text

    def _speak_loop(self, first_text):
        """Цикл обработки текстов с очередью"""
        current_text = first_text
        try:
            while current_text:
                try:
                    self._synthesize_and_play(current_text)
                except Exception as e:
                    self.get_logger().error(f"Ошибка синтеза: {e}")
                    self._activate_speech_recognition()

                with self._busy_lock:
                    current_text = self._pending_text
                    self._pending_text = None
        finally:
            with self._busy_lock:
                self._busy = False
    
    def _synthesize_and_play(self, text):
        """Синтез речи и воспроизведение через sounddevice (pysound)"""
        try:
            # СНАЧАЛА ОТКЛЮЧАЕМ МИКРОФОН
            self._deactivate_speech_recognition()

            # Нормализация текста (годы, даты, числа, аббревиатуры)
            normalized = self._normalize_text(text)
            if normalized != text:
                self.get_logger().info(f"Нормализация текста: \"{text[:50]}...\" → \"{normalized[:50]}...\"")
            text = normalized

            self.get_logger().info(f"Синтез текста: {text[:50]}..." if len(text) > 50 else f"Синтез текста: {text}")
            start_synth = time.time()
            
            # Оптимизация: torch.inference_mode() ускоряет и снижает потребление памяти
            with torch.inference_mode():
                audio = self.model.apply_tts(
                    text=text.strip(),
                    speaker=self.speaker,
                    sample_rate=self.sample_rate
                )
            
            synth_time = time.time() - start_synth
            self.get_logger().info(f"✓ Синтез выполнен за {synth_time:.2f} сек")
            
            # Проверяем, не нужно ли прервать (новое сообщение в очереди)
            with self._busy_lock:
                if self._pending_text:
                    self.get_logger().info("⏭ Пропускаю текущее сообщение, есть новое в очереди")
                    return
            
            audio_np = audio.numpy()
            
            # Выбор устройства вывода
            device = None
            if self.audio_device and self.audio_device not in ['pulse', 'default']:
                try:
                    device = int(self.audio_device)
                except ValueError:
                    device = self.audio_device

            # Воспроизведение через sounddevice (Direct stream)
            try:
                with sd.OutputStream(samplerate=self.sample_rate, device=device, channels=1, dtype='float32') as stream:
                    chunk_size = 8192
                    for i in range(0, len(audio_np), chunk_size):
                        # Проверяем прерывание
                        with self._busy_lock:
                            if self._pending_text:
                                break
                        
                        # Блокирующая запись чанка
                        stream.write(audio_np[i:i + chunk_size])
                        
                self.get_logger().info("✓ Аудио воспроизведено успешно")
                
            except Exception as e:
                self.get_logger().error(f"Ошибка sounddevice: {e}")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения: {e}")
        finally:
            # Очищаем ссылку на процесс (для совместимости)
            with self._busy_lock:
                self._current_process = None
            
            # ВКЛЮЧАЕМ МИКРОФОН ОБРАТНО С ЗАДЕРЖКОЙ (предотвращение эхо)
            time.sleep(0.1)
            self._activate_speech_recognition()
    
    
    def _deactivate_speech_recognition(self):
        """Отключить распознавание перед TTS"""
        msg = Bool()
        msg.data = False
        self.tts_control_pub.publish(msg)
        self.get_logger().info("🔇 Отключаю микрофон перед TTS")
    
    def _activate_speech_recognition(self):
        """Включить распознавание после TTS"""
        msg = Bool()
        msg.data = True
        self.tts_control_pub.publish(msg)
        self.get_logger().info("🎤 Включаю микрофон после TTS")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SileroTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

