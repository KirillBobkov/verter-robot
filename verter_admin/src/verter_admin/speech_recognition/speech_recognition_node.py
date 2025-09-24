#!/usr/bin/env python3
import json
import os
import re
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
import sounddevice as sd
import usb.core
import usb.util
import vosk
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool

from .tuning import Tuning

# Константы для максимальной скорости
DEFAULT_MODEL_NAME = 'vosk-model-small-ru-0.22'
SAMPLE_RATE = 16000
BLOCK_SIZE = 4000  # Уменьшенный размер блока для быстрого отклика
CHANNELS = 1

RESPEAKER_VENDOR_ID = 0x2886
RESPEAKER_PRODUCT_ID = 0x0018

class SpeechRecognitionNode(Node):
    """Простой ROS2 узел для распознавания речи - Vosk сам решает когда отдавать результат."""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Переменные для прямого стриминга
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()  # Thread-safety для Vosk
        
        # Триггерные слова и состояние
        # Используем set для O(1) поиска
        self.trigger_words_set = {
            'робот', 'робат', 'бертом', 'верстах', 'вертэр', 'вертер',
            'ветер', 'ертер', 'верте', 'вектор', 'ветера', 'мерсер',
            'вертера', 'вердер', 'лестер'
        }
        
        # Стоп-слова для завершения диалога
        self.stop_words_set = {
            'стоп', 'хватит', 'спасибо', 'пока', 'досвидания', 'до свидания', 
            'завершить', 'закончить', 'конец', 'всё', 'все', 'достаточно'
        }
        
            # Создаем регулярные выражения для поиска триггерных и стоп-слов
        trigger_pattern = '|'.join(re.escape(word) for word in self.trigger_words_set)
        self.trigger_pattern = re.compile(trigger_pattern, re.IGNORECASE)
        
        stop_pattern = '|'.join(re.escape(word) for word in self.stop_words_set)
        self.stop_pattern = re.compile(stop_pattern, re.IGNORECASE)

        # Состояние системы
        self.listening_for_trigger = True  # Ищем триггер
        self.capturing_command = False     # Записываем команду
        self.dialog_mode = False           # Режим диалога
        self.no_speech_timeout = 10.0      # Тайм-аут тишины после триггера
        self.dialog_timeout = 20.0         # Тайм-аут диалога (20 секунд)
        
        # Глобальная блокировка микрофона во время TTS
        self.microphone_enabled = True
        
        
        # Publisher для отправки к AI
        self.ai_question_publisher = self.create_publisher(String, 'ai_question', 10)
        
        # Publisher для отправки ответов в TTS
        self.response_publisher = self.create_publisher(String, 'ai_response', 10)
        
        # Publisher для воспроизведения звуков
        self.sound_player_publisher = self.create_publisher(String, '/play', 10)
        
        # Publisher для управления диалогом
        self.dialog_control_publisher = self.create_publisher(String, 'dialog_control', 10)
        
        # Publisher для команд verter_commands
        self.command_publisher = self.create_publisher(String, '/verter_commands', 10)
        
        # Subscriber для управления паузами
        self.create_subscription(
            Bool,
            'set_recognition_active',
            self._handle_recognition_control,
            10
        )
        
        # Настройка
        self._setup_parameters()
        self._setup_vosk()
        self._setup_doa()
        self._start_audio_capture()
        
        # Запуск простого алгоритма
        self.get_logger().info("🎤 SpeechRecognitionNode запущен (ДИАЛОГОВЫЙ РЕЖИМ)")
        self.get_logger().info(f"   Размер блока: {BLOCK_SIZE} байт")
        self.get_logger().info(f"   Логика: Триггер → Диалог → 20с тайм-аут")
        self.get_logger().info(f"   Тайм-аут молчания: {self.no_speech_timeout}с после триггера")
        self.get_logger().info(f"   Тайм-аут диалога: {self.dialog_timeout}с")
        self.get_logger().info(f"   Триггерные слова: {len(self.trigger_words_set)} шт.")
        self.get_logger().info(f"   Стоп-слова: {len(self.stop_words_set)} шт. {list(self.stop_words_set)}")
        self.get_logger().info(f"   DOA: {'включен' if self.doa_enabled else 'отключен'} (только при capturing)")

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
            raise RuntimeError("Не удалось найти модель")
            
        try:
            self.model = vosk.Model(model_path)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)  
            self.get_logger().info(f"Vosk оптимизирован для скорости: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Ошибка загрузки Vosk: {e}")

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

    def _setup_doa(self) -> None:
        """Инициализация DOA."""
        try:
            self.doa_dev = usb.core.find(idVendor=RESPEAKER_VENDOR_ID, idProduct=RESPEAKER_PRODUCT_ID)
            if self.doa_dev is None:
                self.doa_enabled = False
            else:
                self.mic_tuning = Tuning(self.doa_dev)
                self.doa_enabled = True
                self.get_logger().info("DOA инициализирован")
        except Exception as e:
            self.get_logger().error(f"Ошибка DOA: {e}")
            self.doa_enabled = False


    def _start_audio_capture(self) -> None:
        """Запустить захват аудио."""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

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
        
        # КРИТИЧЕСКАЯ ПРОВЕРКА: блокируем микрофон во время TTS
        if not self.microphone_enabled:
            return  # Полностью игнорируем аудио
            
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        try:
            data = bytes(indata)
            
            # Получаем DOA угол в момент capturing_command прямо в audio callback
            if self.capturing_command:
                self._handle_doa()
            
            with self.recognition_lock:
                # Проверяем, что recognizer еще валиден
                if not hasattr(self, 'recognizer') or self.recognizer is None:
                    return
                    
                # Подаем аудио в Vosk
                if self.recognizer.AcceptWaveform(data):
                    # Vosk говорит что фраза готова
                    try:
                        result = json.loads(self.recognizer.Result())
                        text = result.get('text', '').strip()
                        
                        if text:
                            self._handle_recognized_text(text)
                    except json.JSONDecodeError as e:
                        self.get_logger().error(f"Ошибка парсинга JSON от Vosk: {e}")
                    except Exception as e:
                        self.get_logger().error(f"Ошибка обработки результата Vosk: {e}")
                        
        except Exception as e:
            self.get_logger().error(f"Ошибка аудио callback: {e}")

    def _handle_recognized_text(self, text: str) -> None:
        """Обработка распознанного текста - режим диалога"""
        try:
            self.get_logger().debug(f"Распознано: '{text}'")
            
            # Проверяем, что распознавание активно
            if not self._is_recognition_active():
                self.get_logger().debug(f"Распознавание неактивно, игнорируем: '{text}'")
                return
            
            # Проверка стоп-слов (только в режиме диалога или захвата команды)
            if (self.dialog_mode or self.capturing_command) and self._check_stop_words(text):
                self.get_logger().info(f"✋ Стоп-слово найдено: '{text}' - завершение диалога")
                self._handle_stop_word()
                return
            
            # Поиск триггера (только если не в диалоге)
            if self.listening_for_trigger and not self.dialog_mode:
                self._handle_trigger_search(text)
                return
            
            # Обработка команды в любом режиме
            if self.capturing_command:
                self._handle_command_input(text)
                
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки текста: {e}")
    
    def _is_recognition_active(self) -> bool:
        """Проверка активности распознавания"""
        return self.listening_for_trigger or self.capturing_command
    
    def _handle_trigger_search(self, text: str) -> None:
        """Обработка поиска триггера"""
        trigger_found, command_after_trigger = self._check_trigger_and_command(text)
        
        if not trigger_found:
            return
            
        self.get_logger().info(f"✓ Триггер найден: '{text}' - запуск диалога")
        self._play_sound('trigger.wav')
        self._start_dialog()
        
        if command_after_trigger:
            # Есть команда после триггера - сразу отправляем в AI
            self._process_command(command_after_trigger, "Команда с триггером")
        else:
            # Только триггер - переходим к захвату команды
            self._start_command_capture()
    
    def _handle_command_input(self, text: str) -> None:
        """Обработка ввода команды"""
        self._stop_command_timer()
        
        if self.dialog_mode:
            self._process_command(text, "Команда в диалоге")
        else:
            self._process_command(text, "Первая команда")
    
    def _handle_stop_word(self) -> None:
        """Обработка стоп-слова - завершение диалога с прощальным сообщением"""
        try:
            self.get_logger().info("✋ Обработка стоп-слова - отправка прощального сообщения")
            
            # Завершаем диалог стандартным способом с прощанием
            if self.dialog_mode:
                self._end_dialog(with_farewell=True)
            else:
                # Если не в диалоге, просто сбрасываем состояние
                self._reset_to_listening()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки стоп-слова: {e}")
            # В случае ошибки принудительно завершаем диалог
            if self.dialog_mode:
                self._end_dialog()
            else:
                self._reset_to_listening()

    def _process_command(self, command: str, source: str) -> None:
        """Обработка команды - общая логика для всех источников"""
        try:
            self.get_logger().info(f"📤 {source}: '{command}' - отправка в AI")
            
            self._play_sound('success.wav')
            
            # Таймер диалога теперь управляется через микрофон в _handle_recognition_control
            
            self._send_to_ai(command)
            
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки команды: {e}")
            self._handle_command_error()
    
    
    def _handle_command_error(self) -> None:
        """Обработка ошибки команды"""
        if self.dialog_mode:
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
    
    def _start_dialog(self) -> None:
        """Запустить режим диалога"""
        try:
            self.dialog_mode = True
            self.listening_for_trigger = False
            
            # Сообщаем AI о начале диалога
            msg = String()
            msg.data = "start_dialog"
            self.dialog_control_publisher.publish(msg)
            
            self.get_logger().info("🗣️ Режим диалога запущен")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска диалога: {e}")
    

    def _end_dialog(self, with_farewell: bool = False) -> None:
        """Завершить режим диалога
        
        Args:
            with_farewell: Если True, отправляет прощальное сообщение в ai_response 
                         вместо обычного сброса к прослушиванию
        """
        self.get_logger().info("🔄 ВХОД в _end_dialog()")
        try:
            self.get_logger().info("🔄 Начинаем завершение диалога...")
            # Останавливаем все таймеры
            self.get_logger().info("🔄 Останавливаем таймеры...")
            self._stop_command_timer()
            self._stop_dialog_timer()
            
            # Сообщаем AI о завершении диалога
            msg = String()
            msg.data = "end_dialog"
            self.dialog_control_publisher.publish(msg)
            
            # Сбрасываем состояние
            self.dialog_mode = False

            if with_farewell:
                # Отправляем прощальное сообщение в ai_response для TTS
                farewell_msg = String()
                farewell_msg.data = "рад был помочь"
                self.response_publisher.publish(farewell_msg)
                self.get_logger().info("🔚 Диалог завершен с прощанием - TTS включит микрофон автоматически")
            else:
                # Обычное завершение
                self._reset_to_listening()
                self.get_logger().info("🔚 Диалог завершен - ожидание окончания TTS для возврата к триггерам")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения диалога: {e}")
            # В случае ошибки тоже включаем микрофон
            self.microphone_enabled = True
            self._reset_to_listening()
    
    def _reset_dialog_timer(self) -> None:
        """Сбросить таймер диалога (только если микрофон активен)"""
        try:
            self._stop_dialog_timer()
            if self.dialog_mode and self.microphone_enabled:
                self.dialog_timer = self.create_timer(self.dialog_timeout, self._dialog_timeout)
                self.get_logger().debug(f"⏱️ Таймер диалога сброшен ({self.dialog_timeout}с)")
            else:
                self.get_logger().debug("⏸️ Таймер диалога не запускается (микрофон отключен)")
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса таймера диалога: {e}")
    
    def _stop_dialog_timer(self) -> None:
        """Остановить таймер диалога"""
        self._stop_timer('dialog_timer')
    
    def _dialog_timeout(self) -> None:
        """Тайм-аут диалога - завершаем диалог"""
        try:
            self.get_logger().warn(f"⏰ Тайм-аут диалога: {self.dialog_timeout}с тишины - завершение диалога")
            self._end_dialog()
            self._play_sound('fail_timeout.wav')
        except Exception as e:
            self.get_logger().error(f"Ошибка тайм-аута диалога: {e}")
            self._end_dialog()
            self._play_sound('fail_timeout.wav')

    def _start_command_capture(self) -> None:
        """Начать захват команды - ждем первую фразу от Vosk"""
        try:
            self.listening_for_trigger = False
            self.capturing_command = True
            
            if not self.dialog_mode:
                # Запускаем тайм-аут "нет речи" только ВНЕ диалога
                self._stop_command_timer()
                self.no_speech_timer = self.create_timer(self.no_speech_timeout, self._no_speech_timeout)
                context = f"произнесение ({self.no_speech_timeout}с)"
            else:
                # В диалоге полагаемся только на 30-секундный таймер диалога
                # 10-секундный таймер команды НЕ нужен!
                context = f"диалоге (таймер диалога {self.dialog_timeout}с)"
                
            self.get_logger().info(f"🎤 Ожидание команды от Vosk на {context}")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска захвата: {e}")
            self._handle_command_error()


    def _no_speech_timeout(self) -> None:
        """Тайм-аут отсутствия речи - отменяем команду"""
        try:
            self.get_logger().warn(f"⏰ Тайм-аут: {self.no_speech_timeout}с тишины - отмена команды")
            self._play_sound('fail_timeout.wav')
            self._handle_command_error()
        except Exception as e:
            self.get_logger().error(f"Ошибка тайм-аута отсутствия речи: {e}")
            self._handle_command_error()

    def _check_trigger_and_command(self, text: str) -> tuple[bool, Optional[str]]:
        """Проверка триггера и извлечение команды после него"""
        if not text or len(text) > 100:
            return False, None
            
        words = text.lower().split()
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
        if not text or len(text) > 50:  # Ограничиваем длину для проверки стоп-слов
            return False
            
        words = text.lower().split()
        if not words:
            return False
        
        # Проверяем первое слово или полную фразу на стоп-слова
        first_word = words[0]
        if first_word in self.stop_words_set or self.stop_pattern.search(text.lower()):
            return True
            
        return False

    def _reset_to_listening(self) -> None:
        """Сброс в состояние прослушивания (только если не в диалоге)"""
        try:
            self.get_logger().info(f"🔄 _reset_to_listening: dialog_mode={self.dialog_mode}, microphone_enabled={self.microphone_enabled}")
            if not self.dialog_mode:
                self.listening_for_trigger = True
                self.get_logger().info("✓ listening_for_trigger установлен в True")
            self.capturing_command = False
            
            # Останавливаем таймер команды
            self._stop_command_timer()
            
            # Сбрасываем recognizer для чистого старта
            with self.recognition_lock:
                try:
                    self.recognizer.Reset()
                except Exception as e:
                    self.get_logger().error(f"Ошибка сброса recognizer: {e}")
            
            mode = "диалог" if self.dialog_mode else "прослушивание"
            self.get_logger().debug(f"🔄 Возврат в режим {mode}")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса состояния: {e}")

    def _send_to_ai(self, text: str) -> None:
        """Отправка команды к AI с переходом в паузу"""
        try:
            # Останавливаем таймеры
            self._stop_command_timer()
            
            # Отправляем к AI
            msg = String()
            msg.data = text
            self.ai_question_publisher.publish(msg)
            
            # Переходим в паузу и ждем команды на возобновление от AI
            self.listening_for_trigger = False
            self.capturing_command = False
            
            mode_info = "(диалог)" if self.dialog_mode else ""
            self.get_logger().info(f"📤 Отправлено в AI: '{text}' {mode_info} | ⏸️ Ожидание ответа...")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки к AI: {e}")
            if self.dialog_mode:
                self._end_dialog()
            else:
                self._reset_to_listening()

    def _handle_recognition_control(self, msg: Bool) -> None:
        """Управление состоянием распознавания"""
        try:
            if msg.data:
                # Возобновление работы - включаем микрофон
                self.microphone_enabled = True
                
                if self.dialog_mode:
                    # В диалоге - продолжаем диалог и ЗАПУСКАЕМ таймер диалога
                    self._start_command_capture()
                    self._reset_dialog_timer()  # Запускаем таймер диалога
                    self.get_logger().info("✓ Микрофон включен, распознавание возобновлено (диалог)")
                else:
                    # Не в диалоге - возврат к прослушиванию
                    self._reset_to_listening()
                    self.get_logger().info("✓ Микрофон включен, распознавание возобновлено")
            else:
                # Пауза - ОТКЛЮЧАЕМ МИКРОФОН и ОСТАНАВЛИВАЕМ таймер диалога
                self.microphone_enabled = False
                self.listening_for_trigger = False
                self.capturing_command = False
                self._stop_command_timer()
                self._stop_dialog_timer()  # Останавливаем таймер диалога
                self.get_logger().info("🔇 Микрофон отключен, распознавание приостановлено")
                
        except Exception as e:
            self.get_logger().error(f"Ошибка управления распознаванием: {e}")

    def _handle_doa(self) -> None:
        """Обработка DOA - получение угла и отправка команды голове."""
        if not self.doa_enabled:
            return
            
        try:
            doa_angle = self.mic_tuning.direction
            if doa_angle is not None:
                self.get_logger().info(f"🎯 DOA угол: {doa_angle}°")
                self._send_eye_command_by_doa(doa_angle)
        except Exception as e:
            self.get_logger().error(f"Ошибка получения DOA: {e}")


    def _send_eye_command_by_doa(self, doa_angle: int) -> None:
        """Отправить команду голове на основе DOA угла"""
        try:
            if 10 <= doa_angle <= 150:
                command = "HEAD:CENTER"
            elif (270 <= doa_angle <= 360) or (0 <= doa_angle < 10):
                command = "HEAD:RIGHT"
            elif 150 < doa_angle < 270:
                command = "HEAD:LEFT"
            else:
                self.get_logger().warn(f"Некорректный DOA угол: {doa_angle}°")
                return
            
            msg = String()
            msg.data = command
            self.command_publisher.publish(msg)
            self.get_logger().info(f"Отправлена команда голове: {command} (DOA: {doa_angle}°)")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки команды голове: {e}")


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
            self._stop_command_timer()
            self._stop_dialog_timer()
            
            # Безопасно очищаем recognizer
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
            
            # Освобождаем USB ресурсы
            if hasattr(self, 'doa_dev') and self.doa_dev:
                try:
                    usb.util.dispose_resources(self.doa_dev)
                except Exception as e:
                    self.get_logger().warn(f"Ошибка освобождения USB: {e}")
            
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
