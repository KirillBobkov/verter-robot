#!/usr/bin/env python3
import json
import os
import re
import threading
from typing import Optional, Dict, Set, Tuple, Any, List
from enum import Enum, auto
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
import sounddevice as sd
import vosk
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool

# === КОНСТАНТЫ И КОНФИГУРАЦИЯ ===
@dataclass
class SpeechConfig:
    """Конфигурация распознавания речи"""
    MODEL_NAME: str = 'vosk-model-small-ru-0.22'
    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 6000
    CHANNELS: int = 1
    
    # Таймауты
    NO_SPEECH_TIMEOUT: float = 10.0
    DIALOG_TIMEOUT: float = 30.0
    REMINDER_INTERVAL: float = 180.0
    
    # Ограничения текста
    MIN_COMMAND_LENGTH: int = 5
    MAX_TEXT_LENGTH_FOR_TRIGGER: int = 100
    MAX_TEXT_LENGTH_FOR_COMMANDS: int = 50
    
    # Звуковые файлы
    SOUND_TRIGGER: str = 'trigger.wav'
    SOUND_SUCCESS: str = 'success.wav'
    SOUND_TIMEOUT: str = 'fail_timeout.wav'

class RecognitionState(Enum):
    """Состояния системы распознавания речи"""
    LISTENING_FOR_TRIGGER = auto()  # Ожидание триггера
    CAPTURING_COMMAND = auto()      # Захват команды после триггера  
    DIALOG_MODE = auto()            # Активный диалог
    PAUSED = auto()                 # Пауза (TTS говорит)

class CommandProcessor:
    """Обработчик команд с четкой стратегией приоритетов"""
    
    def __init__(self):
        self._setup_word_patterns()
    
    def _setup_word_patterns(self) -> None:
        """Инициализация паттернов команд"""
        self.trigger_words: Set[str] = {
            'робот', 'робат', 'бертом', 'верстах', 'вертэр', 'вертер',
            'ветер', 'ертер', 'верте', 'вектор', 'ветера', 'мерсер',
            'вертера', 'вердер', 'лестер'
        }
        
        self.stop_words: Set[str] = {
            'хватит', 'спасибо', 'пасибо', 'пока', 'досвидания', 'до свидания', 
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
        
        # Компиляция regex паттернов
        self.trigger_pattern = self._compile_pattern(self.trigger_words)
        self.stop_pattern = self._compile_pattern(self.stop_words)
        self.chassis_pattern = self._compile_pattern(self.chassis_commands.keys())
    
    def _compile_pattern(self, words: Set[str]) -> re.Pattern:
        """Компиляция regex паттерна из набора слов"""
        pattern = '|'.join(re.escape(word) for word in words)
        return re.compile(pattern, re.IGNORECASE)
    
    def extract_trigger_and_command(self, text: str, max_length: int) -> Tuple[bool, Optional[str]]:
        """Извлечение триггера и команды из текста"""
        if not self._is_valid_text(text, max_length):
            return False, None
            
        words = text.lower().split()
        if not words:
            return False, None
            
        first_word = words[0]
        is_trigger = (first_word in self.trigger_words or 
                     self.trigger_pattern.search(first_word))
        
        if not is_trigger:
            return False, None
        
        # Извлечение команды после триггера
        command = ' '.join(words[1:]).strip() if len(words) > 1 else None
        return True, command
    
    def is_stop_command(self, text: str, max_length: int) -> bool:
        """Проверка является ли текст стоп-командой"""
        if not self._is_valid_text(text, max_length):
            return False
            
        words = text.lower().split()
        if not words:
            return False
            
        first_word = words[0]
        return (first_word in self.stop_words or 
                self.stop_pattern.search(text.lower()))
    
    def get_chassis_command(self, text: str, max_length: int) -> Optional[str]:
        """Получение команды шасси из текста"""
        if not self._is_valid_text(text, max_length):
            return None
            
        words = text.lower().split()
        for word in words:
            if word in self.chassis_commands:
                return self.chassis_commands[word]
                
        match = self.chassis_pattern.search(text.lower())
        return self.chassis_commands.get(match.group()) if match else None
    
    def _is_valid_text(self, text: str, max_length: int) -> bool:
        """Проверка валидности текста"""
        return bool(text and len(text) <= max_length)

class StateManager:
    """Управление состояниями системы с чистыми переходами"""
    
    def __init__(self):
        self.state = RecognitionState.LISTENING_FOR_TRIGGER
        self._previous_state = None
    
    def on_trigger_detected(self) -> None:
        """Обработка обнаружения триггера"""
        if self.state == RecognitionState.LISTENING_FOR_TRIGGER:
            self.state = RecognitionState.CAPTURING_COMMAND
    
    def start_dialog(self) -> None:
        """Начало диалога"""
        self.state = RecognitionState.DIALOG_MODE
    
    def end_dialog(self) -> None:
        """Завершение диалога"""
        self.state = RecognitionState.LISTENING_FOR_TRIGGER
    
    def pause(self) -> None:
        """Пауза системы"""
        if self.state != RecognitionState.PAUSED:
            self._previous_state = self.state
            self.state = RecognitionState.PAUSED
    
    def resume(self) -> None:
        """Возобновление работы"""
        if self._previous_state:
            self.state = self._previous_state
            self._previous_state = None
        else:
            self.state = RecognitionState.LISTENING_FOR_TRIGGER
    
    def is_active_listening(self) -> bool:
        """Проверка активного прослушивания"""
        return self.state in [
            RecognitionState.CAPTURING_COMMAND, 
            RecognitionState.DIALOG_MODE
        ]
    
    def is_passive_listening(self) -> bool:
        """Проверка пассивного прослушивания"""
        return self.state == RecognitionState.LISTENING_FOR_TRIGGER

class TimerManager:
    """Централизованное управление таймерами"""
    
    def __init__(self, node: Node, config: SpeechConfig):
        self.node = node
        self.config = config
        self.timers: Dict[str, Any] = {}
    
    def start_timer(self, name: str, interval: float, callback) -> None:
        """Запуск таймера"""
        self.stop_timer(name)
        self.timers[name] = self.node.create_timer(interval, callback)
    
    def stop_timer(self, name: str) -> None:
        """Остановка таймера"""
        if name in self.timers and self.timers[name]:
            self.node.destroy_timer(self.timers[name])
            self.timers[name] = None
    
    def stop_all_timers(self) -> None:
        """Остановка всех таймеров"""
        for name in list(self.timers.keys()):
            self.stop_timer(name)

class SpeechRecognitionNode(Node):
    """Улучшенный ROS2 узел распознавания речи с чистотой кода"""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Конфигурация и компоненты
        self.config = SpeechConfig()
        self.command_processor = CommandProcessor()
        self.state_manager = StateManager()
        
        # Потокобезопасность
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()
        
        # Инициализация системы
        self._setup_ros_communication()
        self._setup_audio_system()
        self.timer_manager = TimerManager(self, self.config)
        
        # Запуск подсистем
        self._start_audio_capture()
        self._start_reminder_timer()
        
        self._log_system_startup()

    # === ИНИЦИАЛИЗАЦИЯ СИСТЕМЫ ===
    
    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 коммуникации"""
        # Publishers
        self.ai_question_pub = self.create_publisher(String, 'ai_question', 10)
        self.text_to_speech_pub = self.create_publisher(String, 'text_to_speech', 10)
        self.sound_player_pub = self.create_publisher(String, 'play', 10)
        self.dialog_control_pub = self.create_publisher(String, 'dialog_control', 10)
        self.command_pub = self.create_publisher(String, 'verter_commands', 10)
        
        # Subscribers
        self.create_subscription(Bool, 'set_recognition_active', 
                               self._handle_recognition_control, 10)

    def _setup_audio_system(self) -> None:
        """Настройка аудио системы и Vosk"""
        self.audio_device = self._find_audio_device()
        self.vosk_model = self._load_vosk_model()
        self.recognizer = vosk.KaldiRecognizer(self.vosk_model, self.config.SAMPLE_RATE)
        self.recognizer.SetWords(True)

    def _find_audio_device(self) -> int:
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

    def _load_vosk_model(self) -> vosk.Model:
        """Загрузка модели Vosk"""
        model_path = self._resolve_model_path()
        if not model_path:
            raise RuntimeError(f"Модель Vosk не найдена: {self.config.MODEL_NAME}")
        
        try:
            model = vosk.Model(model_path)
            self.get_logger().info(f"✓ Модель Vosk загружена: {model_path}")
            return model
        except Exception as e:
            raise RuntimeError(f"Ошибка загрузки модели Vosk: {e}")

    def _resolve_model_path(self) -> Optional[str]:
        """Определение пути к модели"""
        if os.path.isabs(self.config.MODEL_NAME):
            return self.config.MODEL_NAME
        
        try:
            package_share = get_package_share_directory('verter_admin')
            model_path = os.path.join(package_share, self.config.MODEL_NAME)
            return model_path if os.path.isdir(model_path) else None
        except Exception:
            return None

    def _log_system_startup(self) -> None:
        """Логирование информации о запуске"""
        self.get_logger().info("🎤 SpeechRecognitionNode запущен")
        self.get_logger().info(f"   Состояние: {self.state_manager.state.name}")
        self.get_logger().info(f"   Таймауты: команда={self.config.NO_SPEECH_TIMEOUT}с, диалог={self.config.DIALOG_TIMEOUT}с")

    # === АУДИО ОБРАБОТКА ===
    
    def _start_audio_capture(self) -> None:
        """Запуск захвата аудио в отдельном потоке"""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        self.get_logger().info("✓ Захват аудио запущен")

    def _audio_capture_loop(self) -> None:
        """Основной цикл захвата аудио"""
        try:
            with sd.RawInputStream(
                samplerate=self.config.SAMPLE_RATE,
                blocksize=self.config.BLOCK_SIZE,  
                device=self.audio_device,
                dtype='int16',
                channels=self.config.CHANNELS,
                callback=self._audio_callback
            ):
                self.shutdown_event.wait()
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио потока: {e}")

    def _audio_callback(self, indata, frames, time, status) -> None:
        """Callback обработки аудио данных"""
        if self.shutdown_event.is_set() or self.state_manager.state == RecognitionState.PAUSED:
            return
            
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            data = bytes(indata)
            with self.recognition_lock:
                if self.recognizer.AcceptWaveform(data):
                    self._process_final_recognition_result()
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")

    def _process_final_recognition_result(self) -> None:
        """Обработка финального результата распознавания"""
        try:
            result = json.loads(self.recognizer.Result())
            text = result.get('text', '').strip()
            
            if text:
                self._handle_recognized_speech(text)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Ошибка JSON от Vosk: {e}")
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки результата Vosk: {e}")

    # === ОСНОВНАЯ ЛОГИКА ОБРАБОТКИ ===
    
    def _handle_recognized_speech(self, text: str) -> None:
        """Главный диспетчер распознанной речи"""
        self.get_logger().debug(f"🎤 Распознано: '{text}'")
        
        if self.state_manager.state == RecognitionState.PAUSED:
            return  # Игнорируем в состоянии паузы
        
        # Маршрутизация по состояниям
        if self.state_manager.is_passive_listening():
            self._handle_passive_listening(text)
        else:
            self._handle_active_listening(text)

    def _handle_passive_listening(self, text: str) -> None:
        """Обработка речи в пассивном режиме (ожидание триггера)"""
        is_trigger, command = self.command_processor.extract_trigger_and_command(
            text, self.config.MAX_TEXT_LENGTH_FOR_TRIGGER
        )
        
        if not is_trigger:
            return
            
        self.get_logger().info(f"🔔 Триггер обнаружен: '{text}'")
        
        if command:
            # Триггер + команда: сразу начинаем диалог
            self._start_dialog_with_command(command)
        else:
            # Только триггер: переходим к захвату команды
            self._transition_to_command_capture()

    def _handle_active_listening(self, text: str) -> None:
        """Обработка речи в активном режиме (диалог/захват команды)"""
        # Приоритетная обработка по типам команд
        if self._process_stop_command(text):
            return
        if self._process_chassis_command(text):
            return
        if self._process_ai_command(text):
            return
        
        self.get_logger().debug(f"❓ Необработанная команда: '{text}'")

    def _process_stop_command(self, text: str) -> bool:
        """Обработка стоп-команды (высший приоритет)"""
        if not self.command_processor.is_stop_command(text, self.config.MAX_TEXT_LENGTH_FOR_COMMANDS):
            return False
            
        self.get_logger().info(f"🛑 Стоп-команда: '{text}'")
        self._execute_stop_sequence()
        return True

    def _process_chassis_command(self, text: str) -> bool:
        """Обработка команды шасси (средний приоритет)"""
        chassis_command = self.command_processor.get_chassis_command(
            text, self.config.MAX_TEXT_LENGTH_FOR_COMMANDS
        )
        
        if not chassis_command:
            return False
            
        self.get_logger().info(f"🚗 Команда шасси: '{text}' -> {chassis_command}")
        self._execute_chassis_command(chassis_command)
        return True

    def _process_ai_command(self, text: str) -> bool:
        """Обработка команды для AI (низший приоритет)"""
        cleaned_text = text.strip()
        
        # Валидация команды
        if (not cleaned_text or 
            len(cleaned_text) < self.config.MIN_COMMAND_LENGTH or
            len(cleaned_text) > self.config.MAX_TEXT_LENGTH_FOR_COMMANDS):
            return False

        self.get_logger().info(f"🤖 Команда AI: '{cleaned_text}'")
        self._execute_ai_command(cleaned_text)
        return True

    # === ВЫПОЛНЕНИЕ КОМАНД ===
    
    def _execute_stop_sequence(self) -> None:
        """Выполнение последовательности остановки"""
        current_state = self.state_manager.state
        
        if current_state == RecognitionState.DIALOG_MODE:
            self._end_dialog(with_farewell=True)
        elif current_state == RecognitionState.CAPTURING_COMMAND:
            self._send_farewell_message()
            self._reset_to_listening_state()
        else:
            self._reset_to_listening_state()

    def _execute_chassis_command(self, command: str) -> None:
        """Выполнение команды шасси"""
        self.timer_manager.stop_timer('command_timeout')
        self._transition_to_dialog_if_needed()
        
        # Отправка команды
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self._play_sound(self.config.SOUND_SUCCESS)
        
        # Сброс таймера диалога если в диалоге
        if self.state_manager.state == RecognitionState.DIALOG_MODE:
            self._reset_dialog_timer()

    def _execute_ai_command(self, command: str) -> None:
        """Выполнение команды для AI"""
        self.timer_manager.stop_timer('command_timeout')
        self._transition_to_dialog_if_needed()
        
        # Отправка к AI
        self._play_sound(self.config.SOUND_SUCCESS)
        self._send_to_ai_processor(command)

    def _send_to_ai_processor(self, text: str) -> None:
        """Отправка команды в AI процессор"""
        self.timer_manager.stop_all_timers()
        
        msg = String()
        msg.data = text
        self.ai_question_pub.publish(msg)
        
        self.state_manager.pause()
        self.get_logger().info(f"📤 Отправлено в AI: '{text}'")

    # === УПРАВЛЕНИЕ СОСТОЯНИЯМИ И ПЕРЕХОДАМИ ===
    
    def _transition_to_command_capture(self) -> None:
        """Переход в состояние захвата команды"""
        self.state_manager.on_trigger_detected()
        self._play_sound(self.config.SOUND_TRIGGER)
        self._start_command_timeout_timer()

    def _start_dialog_with_command(self, command: str) -> None:
        """Начало диалога с немедленным выполнением команды"""
        self._start_dialog_mode()
        self._handle_active_listening(command)  # Обрабатываем команду в контексте диалога

    def _start_dialog_mode(self) -> None:
        """Запуск режима диалога"""
        self.state_manager.start_dialog()
        self._reset_dialog_timer()
        
        # Уведомление AI о начале диалога
        msg = String()
        msg.data = "start_dialog"
        self.dialog_control_pub.publish(msg)
        
        self.get_logger().info("💬 Режим диалога активирован")

    def _end_dialog(self, with_farewell: bool = False) -> None:
        """Завершение режима диалога"""
        self.timer_manager.stop_all_timers()
        
        # Уведомление AI о завершении диалога
        msg = String()
        msg.data = "end_dialog"
        self.dialog_control_pub.publish(msg)
        
        if with_farewell:
            self._send_farewell_message()
        
        self.state_manager.end_dialog()
        self.get_logger().info("🔚 Диалог завершен")

    def _transition_to_dialog_if_needed(self) -> None:
        """Автоматический переход в диалог при обработке команды"""
        if self.state_manager.state == RecognitionState.CAPTURING_COMMAND:
            self._start_dialog_mode()
            self.get_logger().debug("🔄 Автопереход в режим диалога")

    def _reset_to_listening_state(self) -> None:
        """Сброс в состояние пассивного прослушивания"""
        self.state_manager.end_dialog()
        self.timer_manager.stop_timer('command_timeout')
        
        # Сброс распознавателя
        with self.recognition_lock:
            try:
                self.recognizer.Reset()
            except Exception as e:
                self.get_logger().error(f"Ошибка сброса распознавателя: {e}")
        
        self.get_logger().debug("🔄 Возврат в режим прослушивания")

    def _send_farewell_message(self) -> None:
        """Отправка прощального сообщения"""
        msg = String()
        msg.data = "рад был помочь"
        self.text_to_speech_pub.publish(msg)

    # === УПРАВЛЕНИЕ ТАЙМЕРАМИ ===
    
    def _start_reminder_timer(self) -> None:
        """Запуск таймера напоминаний"""
        self.timer_manager.start_timer(
            'reminder', 
            self.config.REMINDER_INTERVAL, 
            self._send_user_reminder
        )
        self.get_logger().info("⏰ Таймер напоминаний запущен")

    def _start_command_timeout_timer(self) -> None:
        """Запуск таймера таймаута команды"""
        if self.state_manager.state == RecognitionState.CAPTURING_COMMAND:
            self.timer_manager.start_timer(
                'command_timeout',
                self.config.NO_SPEECH_TIMEOUT,
                self._handle_command_timeout
            )

    def _reset_dialog_timer(self) -> None:
        """Сброс таймера диалога"""
        if self.state_manager.state == RecognitionState.DIALOG_MODE:
            self.timer_manager.start_timer(
                'dialog_timeout',
                self.config.DIALOG_TIMEOUT,
                self._handle_dialog_timeout
            )

    def _handle_command_timeout(self) -> None:
        """Обработка таймаута команды"""
        self.get_logger().warn(f"⏰ Таймаут команды ({self.config.NO_SPEECH_TIMEOUT}с)")
        self._play_sound(self.config.SOUND_TIMEOUT)
        self._reset_to_listening_state()

    def _handle_dialog_timeout(self) -> None:
        """Обработка таймаута диалога"""
        self.get_logger().warn(f"⏰ Таймаут диалога ({self.config.DIALOG_TIMEOUT}с)")
        self._play_sound(self.config.SOUND_TIMEOUT)
        self._end_dialog()

    def _send_user_reminder(self) -> None:
        """Отправка напоминания пользователю"""
        try:
            if self.state_manager.is_passive_listening():
                self.state_manager.pause()
                
                reminder_msg = String()
                reminder_msg.data = "Чтобы задать вопрос начните с ключевого слова Вертер, Или робот, и задайте свой вопрос."
                self.text_to_speech_pub.publish(reminder_msg)
                
                self.get_logger().info("🔔 Напоминание отправлено")
                self.timer_manager.start_timer('reminder', self.config.REMINDER_INTERVAL, self._send_user_reminder)
            else:
                self.timer_manager.start_timer('reminder', self.config.REMINDER_INTERVAL, self._send_user_reminder)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка напоминания: {e}")
            self.state_manager.resume()
            self.timer_manager.start_timer('reminder', self.config.REMINDER_INTERVAL, self._send_user_reminder)

    # === ВНЕШНЕЕ УПРАВЛЕНИЕ ===
    
    def _handle_recognition_control(self, msg: Bool) -> None:
        """Обработка внешнего управления распознаванием"""
        try:
            if msg.data:
                self._activate_recognition()
            else:
                self._deactivate_recognition()
        except Exception as e:
            self.get_logger().error(f"Ошибка управления распознаванием: {e}")
    
    def _activate_recognition(self) -> None:
        """Активация распознавания"""
        self._reset_vosk_recognizer()
        self.state_manager.resume()
        
        if self.state_manager.state == RecognitionState.DIALOG_MODE:
            self._reset_dialog_timer()
            self.get_logger().info("✅ Распознавание активировано (диалог)")
        else:
            if self.state_manager.is_passive_listening():
                self.timer_manager.start_timer('reminder', self.config.REMINDER_INTERVAL, self._send_user_reminder)
            self.get_logger().info("✅ Распознавание активировано")
    
    def _deactivate_recognition(self) -> None:
        """Деактивация распознавания"""
        self.state_manager.pause()
        self.timer_manager.stop_all_timers()
        self._reset_vosk_recognizer()
        self.get_logger().info("⏸️ Распознавание деактивировано")

    def _reset_vosk_recognizer(self) -> None:
        """Сброс распознавателя Vosk"""
        try:
            with self.recognition_lock:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer.Reset()
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса Vosk: {e}")

    # === ВСПОМОГАТЕЛЬНЫЕ МЕТОДЫ ===
    
    def _play_sound(self, sound_name: str) -> None:
        """Воспроизведение звука"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_player_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения {sound_name}: {e}")

    def _cleanup_resources(self) -> None:
        """Очистка ресурсов"""
        with self.recognition_lock:
            self.recognizer = None
        
        try:
            sd.stop()
        except Exception:
            pass

    def shutdown(self) -> None:
        """Корректное завершение работы"""
        try:
            self.get_logger().info("🔄 Завершение работы...")
            self.shutdown_event.set()
            self.timer_manager.stop_all_timers()
            self._cleanup_resources()
            self.get_logger().info("✅ Завершение работы завершено")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения: {e}")

def main(args=None) -> None:
    """Главная функция запуска"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        print("🎤 УЛУЧШЕННЫЙ РАСПОЗНАВАТЕЛЬ РЕЧИ")
        print("   Архитектура: CommandProcessor + StateManager + TimerManager")
        print("   Состояния: LISTENING → CAPTURING → DIALOG → PAUSED")
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
        print("✅ Система завершена")

if __name__ == '__main__':
    main()