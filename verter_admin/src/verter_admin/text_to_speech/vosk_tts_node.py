#!/usr/bin/env python3

import os
import threading
import time
import re
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

try:
    import sounddevice as sd
    import numpy as np
    from vosk_tts import Model, Synth
    from ru_normalizr import Normalizer, NormalizeOptions
    SOUNDDEVICE_AVAILABLE = True
except ImportError as e:
    SOUNDDEVICE_AVAILABLE = False


class VoskTTSNode(Node):

    def __init__(self):
        super().__init__('vosk_tts_node')

        if not SOUNDDEVICE_AVAILABLE:
            self.get_logger().error("Required packages not installed. Install: pip install vosk-tts sounddevice ru-normalizr numpy")
            raise ImportError("vosk-tts or dependencies not found")

        # Путь к модели vosk-tts
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech')
            self.model_path = os.path.join(tts_models_dir, "vosk-model-tts-ru-0.9-multi")
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска модели: {e}")
            raise

        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Модель не найдена: {self.model_path}")
            raise FileNotFoundError("Модель Vosk TTS не найдена")

        # Параметры
        self.declare_parameter('speaker_id', 4)
        self.speaker_id = self.get_parameter('speaker_id').get_parameter_value().integer_value

        # Нормализация текста всегда включена для vosk-tts (обязательная)
        # Vosk TTS не может правильно произносить текст без нормализации (годы, даты, числа, аббревиатуры)
        self.normalize_numbers = True

        self.declare_parameter('audio_device', 'pulse')
        self.audio_device = self.get_parameter('audio_device').get_parameter_value().string_value

        self.sample_rate = 22050  # vosk-tts использует 22050 Гц

        self._initialize_tts()

        # ROS2 подписки
        self.subscription = self.create_subscription(String, 'text_to_speech', self.text_callback, 10)
        self.tts_control_pub = self.create_publisher(Bool, 'tts_control', 10)

        # Управление конкуренцией
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None

        # Инициализация ru-normalizr для контекстного склонения числительных (режим TTS)
        self.normalizer = Normalizer(NormalizeOptions.tts())

        self.get_logger().info(f"Vosk TTS готов. Speaker ID: {self.speaker_id}, ru-normalizr + char filter enabled")

    def _initialize_tts(self):
        """Загрузка модели vosk-tts"""
        try:
            self.get_logger().info(f"Загрузка модели: {self.model_path}")
            start_load = time.time()

            self.model = Model(model_path=self.model_path)
            self.synth = Synth(self.model)

            load_time = time.time() - start_load
            self.get_logger().info(f"✓ Модель Vosk TTS загружена за {load_time:.2f} сек")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise

    def _filter_supported_chars(self, text):
        """Фильтрует текст, оставляя только символы, поддерживаемые моделью Vosk TTS"""
        if not text:
            return text

        # Поддерживаемые символы из config.json модели
        # ВНИМАНИЕ: Модель НЕ поддерживает латинские буквы, только кириллицу!
        # Русские буквы + пробел + знаки препинания: !'(),,-...:;?^_
        supported_pattern = r"[а-яА-ЯёЁ\s!'(),\-.:;?_^]"

        # Фильтруем: оставляем только поддерживаемые символы
        filtered = []
        for char in text:
            if re.match(supported_pattern, char):
                filtered.append(char)
            # Любой другой символ просто пропускаем (не добавляем)

        result = ''.join(filtered)
        # Очищаем множественные пробелы после фильтрации
        result = re.sub(r'\s+', ' ', result).strip()

        if result != text:
            self.get_logger().debug(f"Фильтрация символов: \"{text}\" → \"{result}\"")

        return result

    def _normalize_text(self, text):
        """Нормализация текста для TTS с контекстным склонением (годы, даты, числа, аббревиатуры)"""
        if not text or not self.normalize_numbers:
            return text

        try:
            # ru-normalizr (режим TTS) обрабатывает: годы, даты, время, валюты, проценты,
            # сокращения, римские цифры, аббревиатуры, латиницу с контекстным склонением
            normalized = self.normalizer.normalize(text)
            return normalized
        except Exception as e:
            self.get_logger().warning(f"ru-normalizr ошибка: {e}, возвращаем оригинал")
            return text

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
        """Синтез речи и воспроизведение через sounddevice"""
        try:
            # Отключаем микрофон перед TTS
            self._deactivate_speech_recognition()

            # Нормализация текста (годы, даты, числа, аббревиатуры)
            normalized = self._normalize_text(text)
            if normalized != text:
                self.get_logger().info(f"Нормализация текста: \"{text}\" → \"{normalized}\"")
            text = normalized

            # Фильтрация неподдерживаемых символов
            filtered = self._filter_supported_chars(text)
            if filtered != text:
                self.get_logger().info(f"Фильтрация символов: \"{text}\" → \"{filtered}\"")
            text = filtered

            self.get_logger().info(f"Синтез текста: {text[:50]}..." if len(text) > 50 else f"Синтез текста: {text}")
            start_synth = time.time()

            # Синтез (возвращает numpy.ndarray с dtype=int16)
            audio = self.synth.synth_audio(text, speaker_id=self.speaker_id)

            # Конвертация int16 → float32 для совместимости с OutputStream
            # vosk-tts возвращает int16 (диапазон -32768...32767)
            # OutputStream ожидает float32 (диапазон -1.0...1.0)
            audio = audio.astype(np.float32) / 32768.0

            synth_time = time.time() - start_synth
            duration = len(audio) / self.sample_rate
            rtf = synth_time / duration if duration > 0 else 0

            self.get_logger().info(f"✓ Синтез: {synth_time:.2f}s, Длительность: {duration:.2f}s, RTF: {rtf:.3f}")

            # Проверяем, не нужно ли прервать (новое сообщение в очереди)
            with self._busy_lock:
                if self._pending_text:
                    self.get_logger().info("⏭ Пропускаю текущее сообщение, есть новое в очереди")
                    return

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
                    for i in range(0, len(audio), chunk_size):
                        # Проверяем прерывание
                        with self._busy_lock:
                            if self._pending_text:
                                break

                        # Блокирующая запись чанка
                        stream.write(audio[i:i + chunk_size])

                self.get_logger().info("✓ Аудио воспроизведено успешно")

            except Exception as e:
                self.get_logger().error(f"Ошибка sounddevice: {e}")

        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения: {e}")
        finally:
            # Включаем микрофон обратно с задержкой
            time.sleep(0.1)
            self._activate_speech_recognition()

    def _deactivate_speech_recognition(self):
        """Отключить распознавание перед TTS"""
        msg = Bool()
        msg.data = False
        self.tts_control_pub.publish(msg)
        self.get_logger().debug("🔇 Отключаю микрофон перед TTS")

    def _activate_speech_recognition(self):
        """Включить распознавание после TTS"""
        msg = Bool()
        msg.data = True
        self.tts_control_pub.publish(msg)
        self.get_logger().debug("🎤 Включаю микрофон после TTS")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = VoskTTSNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
