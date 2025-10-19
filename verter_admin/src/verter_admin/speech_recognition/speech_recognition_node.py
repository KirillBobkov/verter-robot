#!/usr/bin/env python3
import json
import os
import re
import threading
from typing import Optional, Dict, Set, Tuple, Any
from enum import Enum, auto
from functools import wraps

import rclpy
from rclpy.node import Node
import sounddevice as sd
import vosk
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool

# === КОНСТАНТЫ ===
DEFAULT_MODEL_NAME = 'vosk-model-small-ru-0.22'
SAMPLE_RATE = 16000
BLOCK_SIZE = 6000  # Уменьшенный размер блока для быстрого отклика
CHANNELS = 1

RESPEAKER_VENDOR_ID = 0x2886
RESPEAKER_PRODUCT_ID = 0x0018

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
        self._previous_state: Optional[RecognitionState] = None
        
    def set_trigger_found(self):
        """Найден триггер"""
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
        if self._previous_state is not None:
            self.state = self._previous_state
            self._previous_state = None
        else:
            self.state = RecognitionState.LISTENING_FOR_TRIGGER

    def reset_to_listening(self, with_farewell: bool = False) -> bool:
        """Сброс в состояние прослушивания с опциональным прощанием"""
        if self.state == RecognitionState.DIALOG_MODE:
            return False  # Не сбрасываем, если в диалоге
        self.end_dialog()
        return True

class TimerManager:
    """Менеджер таймеров для упрощения создания, остановки и перезапуска"""
    
    def __init__(self, node: Node):
        self.node = node
        self.timers: Dict[str, rclpy.timer.Timer] = {}
    
    def create(self, name: str, interval: float, callback: callable) -> None:
        self.stop(name)
        self.timers[name] = self.node.create_timer(interval, callback)
        self.node.get_logger().debug(f"⏱️ Таймер {name} создан ({interval}с)")
    
    def stop(self, name: str) -> None:
        if name in self.timers:
            self.node.destroy_timer(self.timers[name])
            del self.timers[name]
            self.node.get_logger().debug(f"⏱️ Таймер {name} остановлен")
    
    def stop_all(self) -> None:
        for name in list(self.timers.keys()):
            self.stop(name)
    
    def restart(self, name: str, interval: float, callback: callable) -> None:
        self.stop(name)
        self.create(name, interval, callback)

def with_state_lock(func):
    """Декоратор для thread-safe доступа к state_machine"""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        with self.state_lock:
            return func(self, *args, **kwargs)
    return wrapper

class SpeechRecognitionNode(Node):
    """Простой ROS2 узел для распознавания речи - Vosk сам решает когда отдавать результат."""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Переменные для прямого стриминга
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()  # Thread-safety для Vosk
        self.state_lock = threading.Lock()  # Thread-safety для state_machine
        
        # Инициализация компонентов
        self._setup_word_sets()
        self._setup_patterns()
        self._setup_state_variables()
        
        # Настройка ROS2 коммуникации
        self._setup_ros_communication()
        
        # Настройка системы
        self._setup_parameters()
        self._setup_vosk()
        
        # Запуск потоков
        self._start_audio_capture()
        self._start_reminder_timer()
        
        # Логирование информации о запуске
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
        """Создание регулярных выражений для поиска"""
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
        self.reminder_interval = 180.0  # Интервал напоминаний (3 минуты)
        self.timer_manager = TimerManager(self)

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
        self.get_logger().info(f"   Размер блока: {BLOCK_SIZE} байт")
        self.get_logger().info(f"   Логика: Триггер → Диалог → 20с тайм-аут")
        self.get_logger().info(f"   Тайм-аут молчания: {self.no_speech_timeout}с после триггера")
        self.get_logger().info(f"   Тайм-аут диалога: {self.dialog_timeout}с")
        self.get_logger().info(f"   Интервал напоминаний: {self.reminder_interval}с (3 минуты)")
        self.get_logger().info(f"   Триггерные слова: {len(self.trigger_words_set)} шт.")
        self.get_logger().info(f"   Стоп-слова: {len(self.stop_words_set)} шт. {list(self.stop_words_set)}")
        self.get_logger().info(f"   Команды шасси: {len(self.chassis_commands)} шт. {list(self.chassis_commands.keys())}")

    def _setup_parameters(self) -> None:
        """Настройка параметров."""
        self.model_name = DEFAULT_MODEL_NAME
        self.sample_rate = SAMPLE_RATE
        self.block_size = BLOCK_SIZE
        self.channels = CHANNELS
        self.device = self._find_respeaker_device()
        vosk.SetLogLevel(-1)

    def _find_respeaker_device(self) -> Optional[int]:
        """Найти устройство ReSpeaker или fallback на рабочий микрофон."""
        devices = sd.query_devices()
        
        # Сначала ищем ReSpeaker
        for i, device in enumerate(devices):
            if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"✓ Найден ReSpeaker: {i} - {device['name']}")
                    return i
        
        # Проверяем устройство 1 (как было раньше)
        if len(devices) > 1 and devices[1]['max_input_channels'] > 0:
            self.get_logger().info(f"✓ Используется устройство 1: {devices[1]['name']}")
            return 1
        
        # Fallback: ищем любое рабочее входное устройство
        for i, device in enumerate(devices):
            if device['max_input_channels'] > 0:
                self.get_logger().info(f"✓ Fallback на устройство {i}: {device['name']}")
                return i
        
        # Последний вариант - None (дефолт sounddevice)
        self.get_logger().warn("Входные устройства не найдены, используется дефолт")
        return None

    def _setup_vosk(self) -> None:
        """Инициализация Vosk с оптимизацией для скорости."""
        model_path = self._resolve_model_path()
        if not model_path:
            error_msg = f"Не удалось найти модель Vosk по пути: {self.model_name}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
            
        try:
            self.model = vosk.Model(model_path)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)  
            self.get_logger().info(f"✓ Vosk инициализирован: {model_path}")
        except Exception as e:
            error_msg = f"Ошибка загрузки Vosk модели {model_path}: {e}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)

    def _resolve_model_path(self) -> Optional[str]:
        """Определить путь к модели."""
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
        """Запустить захват аудио."""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

    def _start_reminder_timer(self) -> None:
        """Запустить таймер напоминаний"""
        self.timer_manager.create('reminder', self.reminder_interval, self._send_reminder_message)
        self.get_logger().info(f"✓ Таймер напоминаний запущен ({self.reminder_interval}с)")

    def _audio_capture_loop(self) -> None:
        """Цикл захвата аудио."""
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
        except sd.PortAudioError as e:
            self.get_logger().error(f"Ошибка аудио: {e}")

    def _audio_callback(self, indata, frames, time, status) -> None:
        """Простой callback - Vosk решает всё сам"""
        if self.shutdown_event.is_set():
            return
        
        with self.state_lock:
            if self.state_machine.state == RecognitionState.PAUSED:
                return  # Полностью игнорируем аудио
            
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            data = bytes(indata)
            with self.recognition_lock:
                if not hasattr(self, 'recognizer') or self.recognizer is None:
                    return
                    
                if self.recognizer.AcceptWaveform(data):
                    self._process_vosk_result()
                        
        except sd.PortAudioError as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")

    def _handle_recognized_text(self, text: str) -> None:
        """Обработка распознанного текста - единый диспетчер"""
        try:
            self.get_logger().debug(f"Распознано: '{text}'")
            
            with self.state_lock:
                if self.state_machine.state == RecognitionState.PAUSED:
                    self.get_logger().debug(f"Распознавание неактивно, игнорируем: '{text}'")
                    return
                
                cmd_type, value = self._classify_text(text)
                
                if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                    if cmd_type == 'trigger':
                        self._handle_trigger_search(text, value)
                elif self.state_machine.state in [RecognitionState.CAPTURING_COMMAND, RecognitionState.DIALOG_MODE]:
                    self._handle_active_command(cmd_type, value, text)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки текста: {e}")
    
    def _handle_active_command(self, cmd_type: str, value: Optional[str], original_text: str) -> None:
        """Обработка команд в активном состоянии (диалог/захват)"""
        handlers = {
            'stop': self._handle_stop_word,
            'chassis': lambda: self._send_chassis_command(value),  # type: ignore
            'ai': lambda: self._handle_command_input(original_text)
        }
        handler = handlers.get(cmd_type)
        if handler:
            handler()
    
    def _handle_trigger_search(self, text: str, command_after_trigger: Optional[str]) -> None:
        """Обработка поиска триггера"""
        self.get_logger().info(f"✓ Триггер найден: '{text}'")
        
        if command_after_trigger:
            # Есть команда после триггера - сразу в DIALOG_MODE
            self._start_dialog()
            cmd_type, value = self._classify_text(command_after_trigger)
            self._handle_active_command(cmd_type, value, command_after_trigger)
        else:
            # Только триггер - переходим в CAPTURING_COMMAND
            self._play_sound(SOUND_TRIGGER)
            with self.state_lock:
                self.state_machine.set_trigger_found()
            self._start_command_capture()
    
    def _handle_command_input(self, text: str) -> None:
        """Обработка ввода команды"""
        cleaned = text.strip()
        if not cleaned:  # Проверка на пустоту
            self.get_logger().debug(f"Команда проигнорирована как пустая: '{cleaned}'")
            return
        
        # Проверка длины команды
        cleaned_length = len(cleaned)
        if cleaned_length < MIN_COMMAND_LENGTH:
            self.get_logger().debug(f"Команда проигнорирована как слишком короткая: '{cleaned}' ({cleaned_length} < {MIN_COMMAND_LENGTH})")
            return  # Не останавливаем таймер - пусть сработает тайм-аут

        # Команда валидна - останавливаем таймер и переходим в диалог при необходимости
        self.timer_manager.stop('no_speech')
        with self.state_lock:
            if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
                self._start_dialog()
                self.get_logger().debug("🔄 Автоматический переход: CAPTURING_COMMAND → DIALOG_MODE")
        self._process_command(cleaned)
    
    def _handle_stop_word(self) -> None:
        """Обработка стоп-слова - завершение диалога с прощальным сообщением"""
        try:
            self.get_logger().info("✋ Обработка стоп-слова")
            
            with self.state_lock:
                current_state = self.state_machine.state
            
            if current_state == RecognitionState.DIALOG_MODE:
                self._end_dialog(with_farewell=True)
                return
            
            if current_state == RecognitionState.CAPTURING_COMMAND:
                farewell_msg = String()
                farewell_msg.data = "рад был помочь"
                self.text_to_speech_publisher.publish(farewell_msg)
                self._reset_to_listening()
                self.get_logger().info("🔚 Стоп-слово после триггера - прощание")
                return
            
            self._reset_to_listening()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки стоп-слова: {e}")
            self._reset_to_listening()

    def _process_command(self, command: str) -> None:
        """Обработка команды - общая логика для всех источников"""
        try:
            self.get_logger().info(f"📤 Команда: '{command}' - отправка в AI")          
            self._play_sound(SOUND_SUCCESS)
            self._send_to_ai(command)
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки команды: {e}")
            with self.state_lock:
                if self.state_machine.state == RecognitionState.DIALOG_MODE:
                    self._end_dialog()
                else:
                    self._reset_to_listening()

    def _send_reminder_message(self) -> None:
        """Отправить напоминание пользователю о том, как использовать систему"""
        try:
            with self.state_lock:
                if self.state_machine.state != RecognitionState.LISTENING_FOR_TRIGGER:
                    self.get_logger().debug(f"⏸️ Напоминание пропущено - состояние: {self.state_machine.state}")
                    self.timer_manager.restart('reminder', self.reminder_interval, self._send_reminder_message)
                    return
                
                self.state_machine.pause()
            
            self.get_logger().info("🔔 Отправка напоминания пользователю")
            
            reminder_msg = String()
            reminder_msg.data = "Чтобы задать вопрос начните с ключевого слова Вертер, Или робот, и задайте свой вопрос."
            self.text_to_speech_publisher.publish(reminder_msg)
            
            self.get_logger().info("📢 Напоминание отправлено в TTS - микрофон отключен")
            
            self.timer_manager.restart('reminder', self.reminder_interval, self._send_reminder_message)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки напоминания: {e}")
            with self.state_lock:
                self.state_machine.resume()
            self.timer_manager.restart('reminder', self.reminder_interval, self._send_reminder_message)
    
    def _start_dialog(self) -> None:
        """Запустить режим диалога"""
        try:
            with self.state_lock:
                self.state_machine.start_dialog()
            
            msg = String()
            msg.data = "start_dialog"
            self.dialog_control_publisher.publish(msg)
            
            self.get_logger().info("🗣️ Режим диалога запущен")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска диалога: {e}")

    def _end_dialog(self, with_farewell: bool = False) -> None:
        """Завершить режим диалога"""
        self.get_logger().info("🔄 ВХОД в _end_dialog()")
        try:
            self.get_logger().info("🔄 Начинаем завершение диалога...")
            self.timer_manager.stop_all()
            
            msg = String()
            msg.data = "end_dialog"
            self.dialog_control_publisher.publish(msg)
            
            with self.state_lock:
                self.state_machine.end_dialog()

            if with_farewell:
                farewell_msg = String()
                farewell_msg.data = "рад был помочь"
                self.text_to_speech_publisher.publish(farewell_msg)
                self.get_logger().info("🔚 Диалог завершен с прощанием - TTS включит микрофон автоматически")
            else:
                self._reset_to_listening()
                self.get_logger().info("🔚 Диалог завершен - ожидание окончания TTS для возврата к триггерам")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения диалога: {e}")
            self._reset_to_listening()
    
    def _reset_dialog_timer(self) -> None:
        """Сбросить таймер диалога (только если микрофон активен)"""
        try:
            self.timer_manager.stop('dialog')
            with self.state_lock:
                if self.state_machine.state == RecognitionState.DIALOG_MODE:
                    self.timer_manager.create('dialog', self.dialog_timeout, self._dialog_timeout)
                    self.get_logger().debug(f"⏱️ Таймер диалога сброшен ({self.dialog_timeout}с)")
                else:
                    self.get_logger().debug("⏸️ Таймер диалога не запускается (не в диалоге)")
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса таймера диалога: {e}")
    
    def _dialog_timeout(self) -> None:
        """Тайм-аут диалога - завершаем диалог"""
        try:
            self.get_logger().warn(f"⏰ Тайм-аут диалога: {self.dialog_timeout}с тишины - завершение диалога")
            self._end_dialog()
            self._play_sound(SOUND_TIMEOUT)
        except Exception as e:
            self.get_logger().error(f"Ошибка тайм-аута диалога: {e}")
            self._end_dialog()
            self._play_sound(SOUND_TIMEOUT)

    def _start_command_capture(self) -> None:
        """Начать захват команды - ждем первую фразу от Vosk"""
        try:
            with self.state_lock:
                current_state = self.state_machine.state
            
            if current_state == RecognitionState.CAPTURING_COMMAND:
                self.timer_manager.restart('no_speech', self.no_speech_timeout, self._no_speech_timeout)
                self.get_logger().info(f"🎤 Ожидание команды от Vosk на произнесение ({self.no_speech_timeout}с)")
            else:
                self.get_logger().info(f"🎤 Ожидание команды от Vosk в диалоге (таймер диалога {self.dialog_timeout}с)")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска захвата: {e}")
            with self.state_lock:
                if self.state_machine.state == RecognitionState.DIALOG_MODE:
                    self._end_dialog()
                else:
                    self._reset_to_listening()

    def _no_speech_timeout(self) -> None:
        """Тайм-аут отсутствия речи - отменяем команду"""
        try:
            self.get_logger().warn(f"⏰ Тайм-аут: {self.no_speech_timeout}с тишины - отмена команды")
            self._play_sound(SOUND_TIMEOUT)
            with self.state_lock:
                if self.state_machine.state == RecognitionState.DIALOG_MODE:
                    self._end_dialog()
                else:
                    self._reset_to_listening()
        except Exception as e:
            self.get_logger().error(f"Ошибка тайм-аута отсутствия речи: {e}")
            with self.state_lock:
                if self.state_machine.state == RecognitionState.DIALOG_MODE:
                    self._end_dialog()
                else:
                    self._reset_to_listening()

    def _classify_text(self, text: str) -> Tuple[str, Optional[str]]:
        """Классификация текста: trigger, stop, chassis, ai или invalid"""
        if not text or len(text) > MAX_TEXT_LENGTH_FOR_COMMANDS:
            return 'invalid', None
            
        words = text.lower().split()
        if not words:
            return 'invalid', None
        
        first_word = words[0]
        
        # Trigger
        if first_word in self.trigger_words_set or self.trigger_pattern.search(first_word):
            command = ' '.join(words[1:]).strip() if len(words) > 1 else None
            return 'trigger', command
        
        # Stop
        if first_word in self.stop_words_set or self.stop_pattern.search(text.lower()):
            return 'stop', None
        
        # Chassis
        for word in words:
            if word in self.chassis_commands:
                return 'chassis', self.chassis_commands[word]
        match = self.chassis_pattern.search(text.lower())
        if match:
            matched_word = match.group()
            if matched_word in self.chassis_commands:
                return 'chassis', self.chassis_commands[matched_word]
        
        # AI
        return 'ai', text

    def _reset_to_listening(self) -> None:
        """Сброс в состояние прослушивания (только если не в диалоге)"""
        try:
            with self.state_lock:
                if self.state_machine.reset_to_listening():
                    self.get_logger().info("✓ Состояние сброшено в LISTENING_FOR_TRIGGER")
            
            self.timer_manager.stop('no_speech')
            
            try:
                with self.recognition_lock:
                    if hasattr(self, 'recognizer') and self.recognizer:
                        self.recognizer.Reset()
                        self.get_logger().debug("🔄 Vosk recognizer сброшен")
            except Exception as e:
                self.get_logger().error(f"Ошибка сброса Vosk recognizer: {e}")
            
            if self.state_machine.state == RecognitionState.DIALOG_MODE:
                self.get_logger().debug("🔄 Возврат в режим диалог")
            else:
                self.get_logger().debug("🔄 Возврат в режим прослушивание")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса состояния: {e}")

    def _send_to_ai(self, text: str) -> None:
        """Отправка команды к AI - переходим в PAUSED до ответа AI"""
        try:
            self.timer_manager.stop_all()
            
            msg = String()
            msg.data = text
            self.ai_question_publisher.publish(msg)
            
            with self.state_lock:
                self.state_machine.pause()
            
            if self.state_machine.state == RecognitionState.DIALOG_MODE:
                self.get_logger().info(f"📤 Отправлено в AI: '{text}' (диалог) | ⏸️ Микрофон отключен до ответа AI")
            else:
                self.get_logger().info(f"📤 Отправлено в AI: '{text}' | ⏸️ Микрофон отключен до ответа AI")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки к AI: {e}")
            with self.state_lock:
                self.state_machine.resume()
            if self.state_machine.state == RecognitionState.DIALOG_MODE:
                self._end_dialog()
            else:
                self._reset_to_listening()

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
        """Включение микрофона и возобновление распознавания"""
        try:
            with self.recognition_lock:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer.Reset()
                    self.get_logger().debug("🔄 Vosk recognizer сброшен")
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса Vosk recognizer: {e}")
        
        with self.state_lock:
            self.state_machine.resume()
        
        if self.state_machine.state == RecognitionState.DIALOG_MODE:
            self._start_command_capture()
            self._reset_dialog_timer()
            self.get_logger().info("✓ Микрофон включен (диалог) - буферы очищены")
        else:
            self._reset_to_listening()
            if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                self.timer_manager.restart('reminder', self.reminder_interval, self._send_reminder_message)
            self.get_logger().info("✓ Микрофон включен - буферы очищены")
    
    def _disable_microphone(self) -> None:
        """Отключение микрофона - переход в состояние PAUSED"""
        with self.state_lock:
            self.state_machine.pause()
        self.timer_manager.stop_all()
        
        try:
            with self.recognition_lock:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer.Reset()
                    self.get_logger().debug("🔄 Vosk recognizer сброшен")
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса Vosk recognizer: {e}")
        
        self.get_logger().info("🔇 Микрофон отключен - буферы очищены")
    
    def _cleanup_resources(self) -> None:
        """Безопасная очистка всех ресурсов при завершении"""
        with self.recognition_lock:
            try:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer = None
            except Exception as e:
                self.get_logger().warn(f"Ошибка очистки recognizer: {e}")
        
        try:
            sd.stop()
        except sd.PortAudioError as e:
            self.get_logger().warn(f"Ошибка остановки аудио: {e}")
    
    def _send_chassis_command(self, command: str) -> None:
        """Отправить команду шасси"""
        try:
            self.timer_manager.stop('no_speech')
            with self.state_lock:
                if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
                    self._start_dialog()
                    self.get_logger().debug("🔄 Автоматический переход: CAPTURING_COMMAND → DIALOG_MODE")
            
            with self.state_lock:
                if self.state_machine.state == RecognitionState.DIALOG_MODE:
                    self._reset_dialog_timer()
            
            msg = String()
            msg.data = command
            self.command_publisher.publish(msg)
            self.get_logger().info(f"🚗 Отправлена команда шасси: {command}")
            
            self._play_sound(SOUND_SUCCESS)
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки команды шасси: {e}")

    def _process_vosk_result(self) -> None:
        """Обработка результата от Vosk"""
        try:
            with self.recognition_lock:
                result = json.loads(self.recognizer.Result())
            text = result.get('text', '').strip()
            
            if text:
                self._handle_recognized_text(text)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"❌ Ошибка парсинга JSON от Vosk: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Ошибка обработки результата Vosk: {e}")
    
    def _play_sound(self, sound_name: str) -> None:
        """Воспроизвести звук через sound_player_node"""
        try:
            msg = String()
            msg.data = sound_name
            self.sound_player_publisher.publish(msg)
            self.get_logger().info(f"🔊 Воспроизводится звук: {sound_name}")
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения звука {sound_name}: {e}")

    def shutdown(self) -> None:
        """Завершение работы с очисткой ресурсов"""
        try:
            self.get_logger().info("🔄 Завершение работы ноды...")      
            self.shutdown_event.set()     
            self.timer_manager.stop_all()         
            self._cleanup_resources()
            self.get_logger().info("✓ Завершение работы завершено")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка при завершении: {e}")

def main(args=None) -> None:
    """Главная функция с улучшенной обработкой ошибок"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
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