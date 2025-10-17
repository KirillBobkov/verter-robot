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
        if hasattr(self, '_previous_state'):
            self.state = self._previous_state
        else:
            self.state = RecognitionState.LISTENING_FOR_TRIGGER

class SpeechRecognitionNode(Node):
    """Простой ROS2 узел для распознавания речи - Vosk сам решает когда отдавать результат."""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Переменные для прямого стриминга
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()  # Thread-safety для Vosk
        
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


    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 publishers и subscribers"""
        # Publishers
        self.ai_question_publisher = self.create_publisher(String, 'ai_question', 10)
        self.response_publisher = self.create_publisher(String, 'ai_response', 10)
        self.sound_player_publisher = self.create_publisher(String, '/play', 10)
        self.dialog_control_publisher = self.create_publisher(String, 'dialog_control', 10)
        self.command_publisher = self.create_publisher(String, '/verter_commands', 10)
        
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

    def _find_respeaker_device(self) -> int:
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
        self.reminder_timer = self.create_timer(self.reminder_interval, self._send_reminder_message)
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
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио: {e}")


    def _audio_callback(self, indata, frames, time, status) -> None:
        """Простой callback - Vosk решает всё сам"""
        if self.shutdown_event.is_set():
            return
        
        # КРИТИЧЕСКАЯ ПРОВЕРКА: блокируем микрофон в состоянии PAUSED
        if self.state_machine.state == RecognitionState.PAUSED:
            return  # Полностью игнорируем аудио
            
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            data = bytes(indata)
            with self.recognition_lock:
                # Проверяем, что recognizer еще валиден
                if not hasattr(self, 'recognizer') or self.recognizer is None:
                    return
                    
                # Подаем аудио в Vosk
                if self.recognizer.AcceptWaveform(data):
                    # Vosk говорит что фраза готова
                    self._process_vosk_result()
                        
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")

    def _handle_recognized_text(self, text: str) -> None:
        """Обработка распознанного текста - единый диспетчер"""
        try:
            self.get_logger().debug(f"Распознано: '{text}'")
            
            if self.state_machine.state == RecognitionState.PAUSED:
                self.get_logger().debug(f"Распознавание неактивно, игнорируем: '{text}'")
                return
            
            # Единый диспетчер команд по состояниям
            if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                self._handle_trigger_search(text)
            elif self.state_machine.state in [RecognitionState.CAPTURING_COMMAND, RecognitionState.DIALOG_MODE]:
                self._handle_active_command(text)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки текста: {e}")
    
    def _handle_active_command(self, text: str) -> None:
        """Обработка команд в активном состоянии (диалог/захват)"""
        # Приоритет 1: Стоп-слова
        if self._check_stop_words(text):
            self.get_logger().info(f"✋ Стоп-слово найдено: '{text}' - завершение диалога")
            self._handle_stop_word()
            return
        
        # Приоритет 2: Команды шасси
        chassis_command = self._check_chassis_words(text)
        if chassis_command:
            self.get_logger().info(f"🚗 Команда шасси найдена: '{text}' -> {chassis_command}")
            self._send_chassis_command(chassis_command)
            return
        
        # Приоритет 3: Обычная команда для AI
        self._handle_command_input(text)

    def _handle_trigger_search(self, text: str) -> None:
        """Обработка поиска триггера"""
        trigger_found, command_after_trigger = self._check_trigger_and_command(text)
        
        if not trigger_found:
            return
            
        self.get_logger().info(f"✓ Триггер найден: '{text}'")
        
        if command_after_trigger:
            # Есть команда после триггера - сразу в DIALOG_MODE
            # КЕЙС 1.2, 1.3: LISTENING_FOR_TRIGGER → DIALOG_MODE
            self._start_dialog()
            self._handle_active_command(command_after_trigger)
        else:
            # Только триггер - переходим в CAPTURING_COMMAND
            # КЕЙС 1.1: LISTENING_FOR_TRIGGER → CAPTURING_COMMAND
            self._play_sound(SOUND_TRIGGER)
            self.state_machine.set_trigger_found()
            self._start_command_capture()
    
    def _handle_command_input(self, text: str) -> None:
        """Обработка ввода команды"""
        cleaned = text.strip()
        if not cleaned:  # Проверка на пустоту
            self.get_logger().debug(f"Команда проигнорирована как пустая: '{cleaned}'")
            return
        
        # Проверка длины команды (КЕЙС 6.1)
        if len(cleaned) < MIN_COMMAND_LENGTH:
            self.get_logger().debug(f"Команда проигнорирована как слишком короткая: '{cleaned}' ({len(cleaned)} < {MIN_COMMAND_LENGTH})")
            return  # Не останавливаем таймер - пусть сработает тайм-аут

        # Команда валидна - останавливаем таймер и переходим в диалог при необходимости
        self._stop_command_timer()
        self._transition_to_dialog_if_needed()
        self._process_command(cleaned)
    
    def _handle_stop_word(self) -> None:
        """Обработка стоп-слова - завершение диалога с прощальным сообщением"""
        try:
            self.get_logger().info("✋ Обработка стоп-слова")
            
            # КЕЙС 3.3: завершаем активный диалог с прощанием
            if self.state_machine.state == RecognitionState.DIALOG_MODE:
                self._end_dialog(with_farewell=True)
                return
            
            # КЕЙС 2.3: стоп-слово после триггера (в CAPTURING_COMMAND) - прощание
            if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
                self._send_farewell_message()
                self._reset_to_listening()
                self.get_logger().info("🔚 Стоп-слово после триггера - прощание")
                return
            
            # В остальных случаях просто сбрасываем
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
            self._handle_command_error()
    
    def _handle_command_error(self) -> None:
        """Обработка ошибки команды"""
        if self.state_machine.state == RecognitionState.DIALOG_MODE:
            self._end_dialog()
        else:
            self._reset_to_listening()

    def _stop_timer(self, timer_name: str) -> None:
        """Универсальная остановка таймера"""
        timer = getattr(self, timer_name, None)
        if timer:
            self.destroy_timer(timer)
            setattr(self, timer_name, None)
    
    def _stop_command_timer(self) -> None:
        """Остановить тайм-аут команды"""
        self._stop_timer('no_speech_timer')
    
    def _transition_to_dialog_if_needed(self) -> None:
        """Автоматический переход CAPTURING_COMMAND → DIALOG_MODE при обработке команды"""
        if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
            self._start_dialog()
            self.get_logger().debug("🔄 Автоматический переход: CAPTURING_COMMAND → DIALOG_MODE")
    
    def _send_farewell_message(self) -> None:
        """Отправить прощальное сообщение в TTS"""
        farewell_msg = String()
        farewell_msg.data = "рад был помочь"
        self.response_publisher.publish(farewell_msg)
    
    def _send_reminder_message(self) -> None:
        """Отправить напоминание пользователю о том, как использовать систему"""
        try:
            # Проверяем, что мы в состоянии LISTENING_FOR_TRIGGER и не в паузе
            if (self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER):
                self.get_logger().info("🔔 Отправка напоминания пользователю")
                
                # Ставим микрофон на паузу
                self.state_machine.pause()
                
                # Отправляем напоминание в TTS
                reminder_msg = String()
                reminder_msg.data = "Чтобы задать вопрос начните с ключевого слова Вертер, Или робот, и задайте свой вопрос."
                self.response_publisher.publish(reminder_msg)
                
                self.get_logger().info("📢 Напоминание отправлено в TTS - микрофон отключен")
                
                # Перезапускаем таймер на 3 минуты
                self._restart_reminder_timer(self.reminder_interval)
            else:
                self.get_logger().debug(f"⏸️ Напоминание пропущено - состояние: {self.state_machine.state}")
                # Перезапускаем таймер на 3 минуты
                self._restart_reminder_timer(self.reminder_interval)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки напоминания: {e}")
            # В случае ошибки восстанавливаем состояние и перезапускаем на 3 минуты
            self.state_machine.resume()
            self._restart_reminder_timer(self.reminder_interval)
    
    def _start_dialog(self) -> None:
        """Запустить режим диалога"""
        try:
            self.state_machine.start_dialog()
            
            # Сообщаем AI о начале диалога
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
            # Останавливаем все таймеры
            self.get_logger().info("🔄 Останавливаем таймеры...")
            self._stop_all_timers()
            
            # Сообщаем AI о завершении диалога
            msg = String()
            msg.data = "end_dialog"
            self.dialog_control_publisher.publish(msg)
            
            # Сбрасываем состояние
            self.state_machine.end_dialog()

            if with_farewell:
                # Отправляем прощальное сообщение в ai_response для TTS
                self._send_farewell_message()
                self.get_logger().info("🔚 Диалог завершен с прощанием - TTS включит микрофон автоматически")
            else:
                # Обычное завершение
                self._reset_to_listening()
                self.get_logger().info("🔚 Диалог завершен - ожидание окончания TTS для возврата к триггерам")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения диалога: {e}")
            # В случае ошибки сбрасываем состояние
            self._reset_to_listening()
    
    def _reset_dialog_timer(self) -> None:
        """Сбросить таймер диалога (только если микрофон активен)"""
        try:
            self._stop_dialog_timer()
            if self.state_machine.state == RecognitionState.DIALOG_MODE:
                self.dialog_timer = self.create_timer(self.dialog_timeout, self._dialog_timeout)
                self.get_logger().debug(f"⏱️ Таймер диалога сброшен ({self.dialog_timeout}с)")
            else:
                self.get_logger().debug("⏸️ Таймер диалога не запускается (не в диалоге)")
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса таймера диалога: {e}")
    
    def _stop_dialog_timer(self) -> None:
        """Остановить таймер диалога"""
        self._stop_timer('dialog_timer')
    
    def _stop_reminder_timer(self) -> None:
        """Остановить таймер напоминаний"""
        self._stop_timer('reminder_timer')
    
    def _restart_reminder_timer(self, interval: float) -> None:
        """Перезапустить таймер напоминаний с новым интервалом"""
        self._stop_reminder_timer()
        self.reminder_timer = self.create_timer(interval, self._send_reminder_message)
        self.get_logger().debug(f"🔄 Таймер напоминаний перезапущен с интервалом {interval}с")
    
    def _stop_all_timers(self) -> None:
        """Остановить все таймеры (команды, диалога и напоминаний)"""
        self._stop_command_timer()
        self._stop_dialog_timer()
        self._stop_reminder_timer()
    
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
            if self.state_machine.state == RecognitionState.CAPTURING_COMMAND:
                # Запускаем тайм-аут "нет речи" только в CAPTURING_COMMAND
                self._stop_command_timer()
                self.no_speech_timer = self.create_timer(self.no_speech_timeout, self._no_speech_timeout)
                context = f"произнесение ({self.no_speech_timeout}с)"
            else:
                # В диалоге полагаемся только на 30-секундный таймер диалога
                context = f"диалоге (таймер диалога {self.dialog_timeout}с)"
                
            self.get_logger().info(f"🎤 Ожидание команды от Vosk на {context}")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска захвата: {e}")
            self._handle_command_error()


    def _no_speech_timeout(self) -> None:
        """Тайм-аут отсутствия речи - отменяем команду"""
        try:
            self.get_logger().warn(f"⏰ Тайм-аут: {self.no_speech_timeout}с тишины - отмена команды")
            self._play_sound(SOUND_TIMEOUT)
            self._handle_command_error()
        except Exception as e:
            self.get_logger().error(f"Ошибка тайм-аута отсутствия речи: {e}")
            self._handle_command_error()

    def _preprocess_text_for_checking(self, text: str, max_length: int) -> Optional[list[str]]:
        """Предварительная обработка текста для проверки команд"""
        if not self._is_text_valid_for_processing(text, max_length):
            return None
            
        words = text.lower().split()
        return words if words else None

    def _check_trigger_and_command(self, text: str) -> Tuple[bool, Optional[str]]:
        """Проверка триггера и извлечение команды после него"""
        words = self._preprocess_text_for_checking(text, MAX_TEXT_LENGTH_FOR_TRIGGER)
        if not words:
            return False, None
            
        # Проверяем первое слово на триггер
        first_word = words[0]
        trigger_found = (first_word in self.trigger_words_set or 
                        self.trigger_pattern.search(first_word))
        
        if not trigger_found:
            return False, None
        
        # Извлекаем команду после триггера (если есть)
        command = ' '.join(words[1:]).strip() if len(words) > 1 else None
        return True, command if command else None

    def _check_stop_words(self, text: str) -> bool:
        """Проверка стоп-слов для завершения диалога"""
        words = self._preprocess_text_for_checking(text, MAX_TEXT_LENGTH_FOR_COMMANDS)
        if not words:
            return False
        
        # Проверяем первое слово или полную фразу на стоп-слова
        first_word = words[0]
        if first_word in self.stop_words_set or self.stop_pattern.search(text.lower()):
            return True
            
        return False

    def _check_chassis_words(self, text: str) -> Optional[str]:
        """Проверка команд для шасси и возврат соответствующей команды"""
        words = self._preprocess_text_for_checking(text, MAX_TEXT_LENGTH_FOR_COMMANDS)
        if not words:
            return None
        
        # Проверяем каждое слово на команды шасси
        for word in words:
            if word in self.chassis_commands:
                return self.chassis_commands[word]
                
        # Также проверяем через регулярное выражение
        match = self.chassis_pattern.search(text.lower())
        if match:
            matched_word = match.group()
            return self.chassis_commands.get(matched_word)
            
        return None

    def _reset_to_listening(self) -> None:
        """Сброс в состояние прослушивания (только если не в диалоге)"""
        try:
            self.get_logger().info(f"🔄 _reset_to_listening: state={self.state_machine.state}")
            if self.state_machine.state != RecognitionState.DIALOG_MODE:
                self.state_machine.end_dialog()  # Возвращает в LISTENING_FOR_TRIGGER
                self.get_logger().info("✓ Состояние сброшено в LISTENING_FOR_TRIGGER")
            
            # Останавливаем таймер команды
            self._stop_command_timer()
            
            # Сбрасываем recognizer для чистого старта
            with self.recognition_lock:
                try:
                    self.recognizer.Reset()
                except Exception as e:
                    self.get_logger().error(f"Ошибка сброса recognizer: {e}")
            
            mode = "диалог" if self.state_machine.state == RecognitionState.DIALOG_MODE else "прослушивание"
            self.get_logger().debug(f"🔄 Возврат в режим {mode}")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса состояния: {e}")

    def _send_to_ai(self, text: str) -> None:
        """Отправка команды к AI - переходим в PAUSED до ответа AI"""
        try:
            # Останавливаем все таймеры
            self._stop_all_timers()
            
            # Отправляем к AI
            msg = String()
            msg.data = text
            self.ai_question_publisher.publish(msg)
            
            # Сохраняем информацию о диалоге до паузы
            mode_info = "(диалог)" if self.state_machine.state == RecognitionState.DIALOG_MODE else ""
            
            # Переходим в PAUSED - микрофон отключается
            self.state_machine.pause()
            self.get_logger().info(f"📤 Отправлено в AI: '{text}' {mode_info} | ⏸️ Микрофон отключен до ответа AI")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки к AI: {e}")
            # В случае ошибки восстанавливаем состояние
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
        # Сбрасываем Vosk recognizer для очистки буферов
        self._reset_vosk_recognizer()
        
        self.state_machine.resume()
        
        if self.state_machine.state == RecognitionState.DIALOG_MODE:
            self._start_command_capture()
            self._reset_dialog_timer()
            self.get_logger().info("✓ Микрофон включен (диалог) - буферы очищены")
        else:
            self._reset_to_listening()
            # Перезапускаем таймер напоминаний только если в LISTENING_FOR_TRIGGER
            if self.state_machine.state == RecognitionState.LISTENING_FOR_TRIGGER:
                self._restart_reminder_timer(self.reminder_interval)
            self.get_logger().info("✓ Микрофон включен - буферы очищены")
    
    def _disable_microphone(self) -> None:
        """Отключение микрофона - переход в состояние PAUSED"""
        self.state_machine.pause()
        self._stop_all_timers()
        
        # Сбрасываем Vosk recognizer для очистки буферов
        self._reset_vosk_recognizer()
        
        self.get_logger().info("🔇 Микрофон отключен - буферы очищены")
    
    def _cleanup_resources(self) -> None:
        """Безопасная очистка всех ресурсов при завершении"""
        # Очищаем recognizer
        with self.recognition_lock:
            try:
                if hasattr(self, 'recognizer') and self.recognizer:
                    # Не вызываем Reset() при завершении - это может вызвать crash
                    self.recognizer = None
            except Exception as e:
                self.get_logger().warn(f"Ошибка очистки recognizer: {e}")
        
        # Останавливаем аудио
        try:
            sd.stop()
        except Exception as e:
            self.get_logger().warn(f"Ошибка остановки аудио: {e}")
        

    def _reset_vosk_recognizer(self) -> None:
        """Сброс Vosk recognizer для очистки внутренних буферов"""
        try:
            with self.recognition_lock:
                if hasattr(self, 'recognizer') and self.recognizer:
                    self.recognizer.Reset()
                    self.get_logger().debug("🔄 Vosk recognizer сброшен")
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса Vosk recognizer: {e}")

    # === МЕТОДЫ-ХЕЛПЕРЫ ===
    
    def _is_text_valid_for_processing(self, text: str, max_length: int) -> bool:
        """Проверка валидности текста для обработки"""
        return bool(text and len(text) <= max_length)
    
    def _send_chassis_command(self, command: str) -> None:
        """Отправить команду шасси"""
        try:
            # Останавливаем таймеры (команда принята) и переходим в диалог при необходимости
            self._stop_command_timer()
            self._transition_to_dialog_if_needed()
            
            # Если в диалоге - сбрасываем таймер диалога
            if self.state_machine.state == RecognitionState.DIALOG_MODE:
                self._reset_dialog_timer()
            
            msg = String()
            msg.data = command
            self.command_publisher.publish(msg)
            self.get_logger().info(f"🚗 Отправлена команда шасси: {command}")
            
            # Воспроизводим звук подтверждения
            self._play_sound(SOUND_SUCCESS)
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки команды шасси: {e}")

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
            
            # Устанавливаем событие завершения
            self.shutdown_event.set()
            
            # Останавливаем таймеры
            self._stop_all_timers()
            
            # Безопасно очищаем ресурсы
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
        print("🎤 VOSK-BASED РАСПОЗНАВАТЕЛЬ: Wake Word → Dialog Mode → Continuous AI")
        print("   Состояния: listening_for_trigger ↔ dialog_mode ↔ capturing_command")
        print("   Режимы: Одиночные команды / Диалог с историей (20с тайм-аут)")
        print("   Vosk полностью контролирует сегментацию фраз")
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
