#!/usr/bin/env python3
import json
import os
import re
import threading
from typing import Optional, Dict, Set, Tuple, Any
from enum import Enum, auto

import rclpy
from rclpy.node import Node
import sounddevice as sd
import vosk
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool

# === КОНСТАНТЫ ===
DEFAULT_MODEL_NAME = 'vosk-model-small-ru-0.22'
SAMPLE_RATE = 16000
BLOCK_SIZE = 6000
CHANNELS = 1

# Таймауты
NO_SPEECH_TIMEOUT = 10.0
DIALOG_TIMEOUT = 30.0
MIN_COMMAND_LENGTH = 5
MAX_TEXT_LENGTH_FOR_TRIGGER = 100
MAX_TEXT_LENGTH_FOR_COMMANDS = 50

# Звуковые файлы
SOUND_TRIGGER = 'trigger.wav'
SOUND_SUCCESS = 'success.wav'
SOUND_TIMEOUT = 'fail_timeout.wav'

class RecognitionState(Enum):
    """Состояния системы распознавания речи"""
    LISTENING_FOR_TRIGGER = auto()  # Ждем триггер
    CAPTURING_COMMAND = auto()      # Ждем команду после триггера  
    DIALOG_MODE = auto()            # Режим диалога
    PAUSED = auto()                 # Пауза (TTS говорит)

class SpeechRecognitionStateMachine:
    """Конечный автомат для управления состояниями распознавания"""
    
    def __init__(self):
        self.state = RecognitionState.LISTENING_FOR_TRIGGER
        self._previous_state = None
        
    def set_trigger_found(self):
        """Найден триггер - переход к захвату команды"""
        if self.state == RecognitionState.LISTENING_FOR_TRIGGER:
            self.state = RecognitionState.CAPTURING_COMMAND
            
    def start_dialog(self):
        """Запуск режима диалога"""
        self.state = RecognitionState.DIALOG_MODE
        
    def end_dialog(self):
        """Завершение диалога"""
        self.state = RecognitionState.LISTENING_FOR_TRIGGER
        
    def pause(self):
        """Пауза (TTS говорит)"""
        if self.state != RecognitionState.PAUSED:
            self._previous_state = self.state
            self.state = RecognitionState.PAUSED
        
    def resume(self):
        """Возобновление после паузы"""
        if self._previous_state:
            self.state = self._previous_state
            self._previous_state = None
        else:
            self.state = RecognitionState.LISTENING_FOR_TRIGGER

class SpeechRecognitionNode(Node):
    """ROS2 узел для распознавания речи с полным соответствием тест-кейсам"""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Потокобезопасность
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()
        
        # Инициализация компонентов
        self._setup_word_sets()
        self._setup_patterns()
        self._setup_state_variables()
        
        # Настройка ROS2
        self._setup_ros_communication()
        self._setup_parameters()
        self._setup_vosk()
        
        # Запуск системы
        self._start_audio_capture()
        self._start_reminder_timer()
        
        self._log_startup_info()

    def _setup_word_sets(self) -> None:
        """Инициализация наборов слов"""
        self.trigger_words_set: Set[str] = {
            'робот', 'робат', 'бертом', 'верстах', 'вертэр', 'вертер',
            'ветер', 'ертер', 'верте', 'вектор', 'ветера', 'мерсер',
            'вертера', 'вердер', 'лестер'
        }
        
        self.stop_words_set: Set[str] = {
            'хватит', 'спасибо', 'пока', 'досвидания', 'до свидания', 
            'конец', 'достаточно'
        }
        
        self.chassis_commands: Dict[str, str] = {
            'назад': 'CHASSIS:BACK', 
            'стоп': 'CHASSIS:STOP', 
            'остановись': 'CHASSIS:STOP',
            'влево': 'CHASSIS:LEFT',
            'налево': 'CHASSIS:LEFT',
            'вправо': 'CHASSIS:RIGHT',
            'направо': 'CHASSIS:RIGHT',
            'вперед': 'CHASSIS:FRONT',
            'прямо': 'CHASSIS:FRONT',
            'вперёд': 'CHASSIS:FRONT',
        }

    def _setup_patterns(self) -> None:
        """Создание регулярных выражений"""
        trigger_pattern = '|'.join(re.escape(word) for word in self.trigger_words_set)
        self.trigger_pattern = re.compile(trigger_pattern, re.IGNORECASE)
        
        stop_pattern = '|'.join(re.escape(word) for word in self.stop_words_set)
        self.stop_pattern = re.compile(stop_pattern, re.IGNORECASE)
        
        chassis_pattern = '|'.join(re.escape(word) for word in self.chassis_commands.keys())
        self.chassis_pattern = re.compile(chassis_pattern, re.IGNORECASE)

    def _setup_state_variables(self) -> None:
        """Инициализация переменных состояния"""
        self.state_machine = SpeechRecognitionStateMachine()
        self.no_speech_timeout = NO_SPEECH_TIMEOUT
        self.dialog_timeout = DIALOG_TIMEOUT
        self.reminder_interval = 180.0

    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 publishers и subscribers"""
        # Publishers
        self.ai_question_publisher = self.create_publisher(String, 'ai_question', 10)
        self.text_to_speech_publisher = self.create_publisher(String, 'text_to_speech', 10)
        self.sound_player_publisher = self.create_publisher(String, 'play', 10)
        self.dialog_control_publisher = self.create_publisher(String, 'dialog_control', 10)
        self.command_publisher = self.create_publisher(String, 'verter_commands', 10)
        
        # Subscribers
        self.create_subscription(
            Bool,
            'set_recognition_active',
            self._handle_recognition_control,
            10
        )

    def _log_startup_info(self) -> None:
        """Логирование информации при запуске"""
        self.get_logger().info("🎤 SpeechRecognitionNode запущен (ДИАЛОГОВЫЙ РЕЖИМ)")
        self.get_logger().info(f"   Тайм-аут команды: {self.no_speech_timeout}с")
        self.get_logger().info(f"   Тайм-аут диалога: {self.dialog_timeout}с")
        self.get_logger().info(f"   Интервал напоминаний: {self.reminder_interval}с")

    def _setup_parameters(self) -> None:
        """Настройка параметров"""
        self.model_name = DEFAULT_MODEL_NAME
        self.sample_rate = SAMPLE_RATE
        self.block_size = BLOCK_SIZE
        self.channels = CHANNELS
        self.device = self._find_audio_device()
        vosk.SetLogLevel(-1)

    def _find_audio_device(self) -> int:
        """Найти рабочее аудио устройство"""
        devices = sd.query_devices()
        
        # Поиск ReSpeaker
        for i, device in enumerate(devices):
            if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Найден ReSpeaker: {i} - {device['name']}")
                    return i
        
        # Fallback на устройство 1
        if len(devices) > 1 and devices[1]['max_input_channels'] > 0:
            self.get_logger().info(f"✓ Используется устройство 1: {devices[1]['name']}")
            return 1
        
        # Любое рабочее устройство
        for i, device in enumerate(devices):
            if device['max_input_channels'] > 0:
                self.get_logger().info(f"✓ Fallback на устройство {i}: {device['name']}")
                return i
        
        self.get_logger().warn("Входные устройства не найдены, используется дефолт")
        return None

    def _setup_vosk(self) -> None:
        """Инициализация Vosk"""
        model_path = self._resolve_model_path()
        if not model_path:
            raise RuntimeError(f"Не удалось найти модель Vosk: {self.model_name}")
            
        try:
            self.model = vosk.Model(model_path)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)  
            self.get_logger().info(f"✓ Vosk инициализирован: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Ошибка загрузки Vosk модели {model_path}: {e}")

    def _resolve_model_path(self) -> Optional[str]:
        """Определить путь к модели"""
        if os.path.isabs(self.model_name):
            model_path = self.model_name
        else:
            try:
                package_share = get_package_share_directory('verter_admin')
                model_path = os.path.join(package_share, self.model_name)
            except Exception:
                return None
        
        return model_path if os.path.isdir(model_path) else None

    def _start_audio_capture(self) -> None:
        """Запустить захват аудио"""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

    def _start_reminder_timer(self) -> None:
        """Запустить таймер напоминаний"""
        self.reminder_timer = self.create_timer(self.reminder_interval, self._send_reminder_message)
        self.get_logger().info(f"✓ Таймер напоминаний запущен ({self.reminder_interval}с)")

    def _audio_capture_loop(self) -> None:
        """Цикл захвата аудио"""
        try:
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=self.block_size,  
                device=self.device,
                dtype='int16',
                channels=self.channels,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Захват аудио запущен")
                self.shutdown_event.wait()
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио: {e}")

    def _audio_callback(self, indata, frames, time, status) -> None:
        """Callback обработки аудио - только финальные результаты Vosk"""
        if self.shutdown_event.is_set() or self.state_machine.state == RecognitionState.PAUSED:
            return
            
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            data = bytes(indata)
            with self.recognition_lock:
                if not hasattr(self, 'recognizer') or self.recognizer is None:
                    return
                    
                # Только финальные результаты для избежания дублирования
                if self.recognizer.AcceptWaveform(data):
                    self._process_vosk_result()
                        
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")

    def _process_vosk_result(self) -> None:
        """Обработка результата от Vosk"""
        try:
            result = json.loads(self.recognizer.Result())
            text = result.get('text', '').strip()
            
            if text:
                self._handle_recognized_text(text)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Ошибка парсинга JSON от Vosk: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка обработки результата Vosk: {e}")

    def _handle_recognized_text(self, text: str) -> None:
        """Единый диспетчер распознанного текста"""
        try:
            self.get_logger().debug(f"Распознано: '{text}'")
            
            # Игнорируем в состоянии PAUSED
            if self.state_machine.state == RecognitionState.PAUSED:
                return
            
            # Диспетчеризация по состояниям
            if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                self._process_trigger_state(text)
            else:
                self._process_active_state(text)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки текста: {e}")

    def _process_trigger_state(self, text: str) -> None:
        """Обработка текста в состоянии ожидания триггера"""
        trigger_found, command_after_trigger = self._check_trigger_and_command(text)
        
        if not trigger_found:
            return
            
        self.get_logger().info(f"✓ Триггер найден: '{text}'")
        
        if command_after_trigger:
            # КЕЙС 1.2, 1.3: Триггер + команда - сразу в диалог
            self._start_dialog_with_command(command_after_trigger)
        else:
            # КЕЙС 1.1: Только триггер - ждем команду
            self._play_sound(SOUND_TRIGGER)
            self.state_machine.set_trigger_found()
            self._start_command_capture()

    def _process_active_state(self, text: str) -> None:
        """Обработка текста в активных состояниях (CAPTURING_COMMAND, DIALOG_MODE)"""
        # Приоритет 1: Стоп-слова
        if self._check_stop_words(text):
            self.get_logger().info(f"✋ Стоп-слово найдено: '{text}'")
            self._handle_stop_word()
            return
        
        # Приоритет 2: Команды шасси
        chassis_command = self._check_chassis_words(text)
        if chassis_command:
            self.get_logger().info(f"🚗 Команда шасси: '{text}' -> {chassis_command}")
            self._handle_chassis_command(chassis_command)
            return
        
        # Приоритет 3: Обычная команда для AI
        self._handle_ai_command(text)

    def _handle_stop_word(self) -> None:
        """Обработка стоп-слова"""
        current_state = self.state_machine.state
        
        if current_state == RecognitionState.DIALOG_MODE:
            # КЕЙС 3.3: Завершение диалога с прощанием
            self._end_dialog(with_farewell=True)
        elif current_state == RecognitionState.CAPTURING_COMMAND:
            # КЕЙС 2.3: Стоп-слово после триггера
            self._send_farewell_message()
            self._reset_to_listening()
        else:
            # Резервный сброс
            self._reset_to_listening()

    def _handle_chassis_command(self, command: str) -> None:
        """Обработка команды шасси"""
        # Останавливаем таймер команды и переходим в диалог если нужно
        self._stop_command_timer()
        self._transition_to_dialog_if_needed()
        
        # Отправляем команду
        msg = String()
        msg.data = command
        self.command_publisher.publish(msg)
        self._play_sound(SOUND_SUCCESS)
        
        # В диалоге сбрасываем таймер
        if self.state_machine.state == RecognitionState.DIALOG_MODE:
            self._reset_dialog_timer()

    def _handle_ai_command(self, text: str) -> None:
        """Обработка команды для AI"""
        cleaned_text = text.strip()
        
        # Валидация команды
        if not cleaned_text or len(cleaned_text) < MIN_COMMAND_LENGTH:
            self.get_logger().debug(f"Команда проигнорирована: '{cleaned_text}'")
            return

        # Останавливаем таймер и переходим в диалог
        self._stop_command_timer()
        self._transition_to_dialog_if_needed()
        
        # Отправляем к AI
        self.get_logger().info(f"📤 Команда AI: '{cleaned_text}'")
        self._play_sound(SOUND_SUCCESS)
        self._send_to_ai(cleaned_text)

    def _start_dialog_with_command(self, command: str) -> None:
        """Запуск диалога с немедленной обработкой команды"""
        self._start_dialog()
        self._process_active_state(command)

    def _start_dialog(self) -> None:
        """Запуск режима диалога"""
        self.state_machine.start_dialog()
        self._reset_dialog_timer()
        
        # Уведомляем AI о начале диалога
        msg = String()
        msg.data = "start_dialog"
        self.dialog_control_publisher.publish(msg)
        
        self.get_logger().info("🗣️ Режим диалога запущен")

    def _end_dialog(self, with_farewell: bool = False) -> None:
        """Завершение режима диалога"""
        self._stop_all_timers()
        
        # Уведомляем AI о завершении диалога
        msg = String()
        msg.data = "end_dialog"
        self.dialog_control_publisher.publish(msg)
        
        if with_farewell:
            self._send_farewell_message()
        
        self.state_machine.end_dialog()
        self.get_logger().info("🔚 Диалог завершен")

    def _send_farewell_message(self) -> None:
        """Отправка прощального сообщения"""
        farewell_msg = String()
        farewell_msg.data = "рад был помочь"
        self.text_to_speech_publisher.publish(farewell_msg)

    def _send_to_ai(self, text: str) -> None:
        """Отправка команды к AI с переходом в PAUSED"""
        self._stop_all_timers()
        
        msg = String()
        msg.data = text
        self.ai_question_publisher.publish(msg)
        
        self.state_machine.pause()
        self.get_logger().info(f"📤 Отправлено в AI: '{text}' | ⏸️ Микрофон отключен")

    # === ТАЙМЕРЫ ===

    def _start_command_capture(self) -> None:
        """Запуск захвата команды с таймаутом"""
        self._stop_command_timer()
        if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
            self.no_speech_timer = self.create_timer(self.no_speech_timeout, self._no_speech_timeout)
            self.get_logger().info(f"⏱️ Таймер команды запущен ({self.no_speech_timeout}с)")

    def _reset_dialog_timer(self) -> None:
        """Сброс таймера диалога"""
        self._stop_dialog_timer()
        if self.state_machine.state == RecognitionState.DIALOG_MODE:
            self.dialog_timer = self.create_timer(self.dialog_timeout, self._dialog_timeout)
            self.get_logger().debug(f"⏱️ Таймер диалога сброшен ({self.dialog_timeout}с)")

    def _transition_to_dialog_if_needed(self) -> None:
        """Автоматический переход в диалог при обработке команды"""
        if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
            self._start_dialog()
            self.get_logger().debug("🔄 Автоматический переход: CAPTURING_COMMAND → DIALOG_MODE")

    def _stop_command_timer(self) -> None:
        """Остановка таймера команды"""
        if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
            self.destroy_timer(self.no_speech_timer)
            self.no_speech_timer = None

    def _stop_dialog_timer(self) -> None:
        """Остановка таймера диалога"""
        if hasattr(self, 'dialog_timer') and self.dialog_timer:
            self.destroy_timer(self.dialog_timer)
            self.dialog_timer = None

    def _stop_reminder_timer(self) -> None:
        """Остановка таймера напоминаний"""
        if hasattr(self, 'reminder_timer') and self.reminder_timer:
            self.destroy_timer(self.reminder_timer)
            self.reminder_timer = None

    def _stop_all_timers(self) -> None:
        """Остановка всех таймеров"""
        self._stop_command_timer()
        self._stop_dialog_timer()

    def _restart_reminder_timer(self, interval: float) -> None:
        """Перезапуск таймера напоминаний"""
        self._stop_reminder_timer()
        self.reminder_timer = self.create_timer(interval, self._send_reminder_message)

    def _no_speech_timeout(self) -> None:
        """Тайм-аут отсутствия речи после триггера"""
        self.get_logger().warn(f"⏰ Тайм-аут команды: {self.no_speech_timeout}с тишины")
        self._play_sound(SOUND_TIMEOUT)
        self._reset_to_listening()

    def _dialog_timeout(self) -> None:
        """Тайм-аут диалога"""
        self.get_logger().warn(f"⏰ Тайм-аут диалога: {self.dialog_timeout}с тишины")
        self._play_sound(SOUND_TIMEOUT)
        self._end_dialog()

    def _send_reminder_message(self) -> None:
        """Отправка напоминания пользователю"""
        try:
            if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                self.state_machine.pause()
                
                reminder_msg = String()
                reminder_msg.data = "Чтобы задать вопрос начните с ключевого слова Вертер, Или робот, и задайте свой вопрос."
                self.text_to_speech_publisher.publish(reminder_msg)
                
                self.get_logger().info("📢 Напоминание отправлено в TTS")
                self._restart_reminder_timer(self.reminder_interval)
            else:
                self._restart_reminder_timer(self.reminder_interval)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки напоминания: {e}")
            self.state_machine.resume()
            self._restart_reminder_timer(self.reminder_interval)

    # === ПОИСК КОМАНД ===

    def _check_trigger_and_command(self, text: str) -> Tuple[bool, Optional[str]]:
        """Проверка триггера и извлечение команды после него"""
        words = self._preprocess_text_for_checking(text, MAX_TEXT_LENGTH_FOR_TRIGGER)
        if not words:
            return False, None
            
        first_word = words[0]
        trigger_found = (first_word in self.trigger_words_set or 
                        self.trigger_pattern.search(first_word))
        
        if not trigger_found:
            return False, None
        
        command = ' '.join(words[1:]).strip() if len(words) > 1 else None
        return True, command if command else None

    def _check_stop_words(self, text: str) -> bool:
        """Проверка стоп-слов"""
        words = self._preprocess_text_for_checking(text, MAX_TEXT_LENGTH_FOR_COMMANDS)
        if not words:
            return False
        
        first_word = words[0]
        if first_word in self.stop_words_set or self.stop_pattern.search(text.lower()):
            return True
            
        return False

    def _check_chassis_words(self, text: str) -> Optional[str]:
        """Проверка команд шасси"""
        words = self._preprocess_text_for_checking(text, MAX_TEXT_LENGTH_FOR_COMMANDS)
        if not words:
            return None
        
        for word in words:
            if word in self.chassis_commands:
                return self.chassis_commands[word]
                
        match = self.chassis_pattern.search(text.lower())
        if match:
            matched_word = match.group()
            return self.chassis_commands.get(matched_word)
            
        return None

    def _preprocess_text_for_checking(self, text: str, max_length: int) -> Optional[list[str]]:
        """Предварительная обработка текста"""
        if not text or len(text) > max_length:
            return None
            
        words = text.lower().split()
        return words if words else None

    # === УПРАВЛЕНИЕ СОСТОЯНИЯМИ ===

    def _reset_to_listening(self) -> None:
        """Сброс в состояние прослушивания"""
        self.state_machine.end_dialog()
        self._stop_command_timer()
        
        with self.recognition_lock:
            try:
                self.recognizer.Reset()
            except Exception as e:
                self.get_logger().error(f"Ошибка сброса recognizer: {e}")
        
        self.get_logger().debug("🔄 Возврат в режим прослушивания")

    def _handle_recognition_control(self, msg: Bool) -> None:
        """Управление состоянием распознавания"""
        try:
            if msg.data:
                self._enable_microphone()
            else:
                self._disable_microphone()
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка управления распознаванием: {e}")
    
    def _enable_microphone(self) -> None:
        """Включение микрофона"""
        self._reset_vosk_recognizer()
        self.state_machine.resume()
        
        if self.state_machine.state == RecognitionState.DIALOG_MODE:
            self._reset_dialog_timer()
            self.get_logger().info("✓ Микрофон включен (диалог)")
        else:
            if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                self._restart_reminder_timer(self.reminder_interval)
            self.get_logger().info("✓ Микрофон включен")
    
    def _disable_microphone(self) -> None:
        """Отключение микрофона"""
        self.state_machine.pause()
        self._stop_all_timers()
        self._reset_vosk_recognizer()
        self.get_logger().info("🔇 Микрофон отключен")

    def _reset_vosk_recognizer(self) -> None:
        """Сброс Vosk recognizer"""
        try:
            with self.recognition_lock:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer.Reset()
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса Vosk recognizer: {e}")

    def _play_sound(self, sound_name: str) -> None:
        """Воспроизведение звука"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_player_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения звука {sound_name}: {e}")

    def _cleanup_resources(self) -> None:
        """Очистка ресурсов"""
        with self.recognition_lock:
            try:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer = None
            except Exception:
                pass
        
        try:
            sd.stop()
        except Exception:
            pass

    def shutdown(self) -> None:
        """Завершение работы"""
        try:
            self.get_logger().info("🔄 Завершение работы ноды...")
            self.shutdown_event.set()
            self._stop_all_timers()
            self._stop_reminder_timer()
            self._cleanup_resources()
            self.get_logger().info("✓ Завершение работы завершено")
        except Exception as e:
            self.get_logger().error(f"Ошибка при завершении: {e}")

def main(args=None) -> None:
    """Главная функция"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        print("🎤 VOSK РАСПОЗНАВАТЕЛЬ: Wake Word → Dialog Mode → Continuous AI")
        print("   Состояния: listening_for_trigger ↔ dialog_mode ↔ capturing_command")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🔄 Завершение по Ctrl+C...")
    except Exception as e:
        print(f"\n❌ Критическая ошибка: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✓ Завершено")

if __name__ == '__main__':
    main()