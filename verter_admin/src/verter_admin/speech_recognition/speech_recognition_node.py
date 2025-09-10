#!/usr/bin/env python3
import json
import os
import threading
import time
from typing import Optional

import numpy as np
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
        self.trigger_words = ['робот', 'робат', 'бертом', 'верстах', 'вертэр', 'вертер', 'ветер', 'ертер', 'верте', 'вектор', 'ветера', 'мерсер', 'вертера', 'вердер', 'лестер']
        
        # Простое состояние системы
        self.listening_for_trigger = True  # Ищем триггер
        self.capturing_command = False     # Записываем команду
        self.no_speech_timeout = 10.0  # Тайм-аут если 5 секунд тишины после триггера
        
        
        # Publisher для отправки к AI
        self.ai_question_publisher = self.create_publisher(String, 'ai_question', 10)
        
        # Publisher для воспроизведения звуков
        self.sound_player_publisher = self.create_publisher(String, '/play', 10)
        
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
        self.get_logger().info("🎤 SpeechRecognitionNode запущен (ДОВЕРЯЕМ VOSK)")
        self.get_logger().info(f"   Размер блока: {BLOCK_SIZE} байт")
        self.get_logger().info(f"   Логика: Триггер → Vosk решает фразу → СРАЗУ в AI")
        self.get_logger().info(f"   Тайм-аут молчания: {self.no_speech_timeout}с после триггера")
        self.get_logger().info(f"   Триггерные слова: {len(self.trigger_words)} шт.")

    def _setup_parameters(self) -> None:
        """Настройка параметров."""
        self.model_name = DEFAULT_MODEL_NAME
        self.sample_rate = SAMPLE_RATE
        self.block_size = BLOCK_SIZE
        self.channels = CHANNELS
        self.device = self._find_respeaker_device()
        vosk.SetLogLevel(-1)

    def _find_respeaker_device(self) -> int:
        """Найти устройство ReSpeaker."""
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
                self.get_logger().info(f"✓ Найден ReSpeaker: {i} - {device['name']}")
                return i
        self.get_logger().info("ReSpeaker не найден, используется устройство по умолчанию")
        return 1

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
        """Обработка распознанного текста - доверяем Vosk полностью"""
        try:
            self.get_logger().debug(f"Распознано: '{text}'")
            
            if self.listening_for_trigger:
                # Ищем триггерное слово
                if self._is_valid_trigger(text):
                    self.get_logger().info(f"✓ Триггер найден: '{text}'")
                    self._play_sound('trigger.wav')
                    self._start_command_capture()
                    
            elif self.capturing_command:
                # Vosk решил что фраза готова - сразу отправляем в AI!
                self.get_logger().info(f"📤 Команда от Vosk: '{text}' - отправка в AI")
                
                # Останавливаем тайм-аут "нет речи"
                if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
                    self.destroy_timer(self.no_speech_timer)
                    self.no_speech_timer = None
                
                # Получаем DOA направление
                doa_angle = self._get_doa_angle()
                if doa_angle is not None:
                    self.get_logger().info(f"DOA направление: {doa_angle}°")
                
                # Воспроизводим звук успеха
                self._play_sound('success.wav')
                # Сразу отправляем в AI
                self._send_to_ai(text)
            
                
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки текста: {e}")

    def _start_command_capture(self) -> None:
        """Начать захват команды - ждем первую фразу от Vosk"""
        try:
            self.listening_for_trigger = False
            self.capturing_command = True
            
            # Запускаем тайм-аут "нет речи" - если 5 секунд тишины, отменяем команду
            if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
                self.destroy_timer(self.no_speech_timer)
            self.no_speech_timer = self.create_timer(self.no_speech_timeout, self._no_speech_timeout)
            
            self.get_logger().info("🎤 Ожидание команды от Vosk (5с на произнесение)")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка запуска захвата: {e}")
            self._reset_to_listening()


    def _no_speech_timeout(self) -> None:
        """Тайм-аут отсутствия речи - отменяем команду"""
        try:
            self.get_logger().warn(f"⏰ Тайм-аут: {self.no_speech_timeout}с тишины после триггера - отмена команды")
            self._play_sound('fail_timeout.wav')
            self._reset_to_listening()
        except Exception as e:
            self.get_logger().error(f"Ошибка тайм-аута отсутствия речи: {e}")
            self._reset_to_listening()


    def _is_valid_trigger(self, text: str) -> bool:
        """Улучшенная проверка wake word с защитой от ложных срабатываний"""
        text_lower = text.lower()
        words = text_lower.split()
        
        # Игнорируем слишком длинные фразы (вероятно случайный разговор)
        if len(words) > 8:
            return False
        
        # Ищем триггерные слова
        for trigger in self.trigger_words:
            if trigger in text_lower:
                # Дополнительная проверка: триггер в начале фразы или фраза короткая
                trigger_position = text_lower.find(trigger)
                if trigger_position <= 10 or len(words) <= 3:  # В начале или короткая фраза
                    return True
        
        return False

    def _reset_to_listening(self) -> None:
        """Сброс в состояние прослушивания"""
        try:
            self.listening_for_trigger = True
            self.capturing_command = False
            
            # Останавливаем таймер "нет речи"
            if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
                self.destroy_timer(self.no_speech_timer)
                self.no_speech_timer = None
            
            # Сбрасываем recognizer для чистого старта - только в главном потоке
            with self.recognition_lock:
                try:
                    self.recognizer.Reset()
                except Exception as e:
                    self.get_logger().error(f"Ошибка сброса recognizer: {e}")
            
            self.get_logger().debug("🔄 Возврат в режим прослушивания")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка сброса состояния: {e}")

    def _send_to_ai(self, text: str) -> None:
        """Отправка команды к AI с переходом в паузу"""
        try:
            # Останавливаем все таймеры
            if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
                self.destroy_timer(self.no_speech_timer)
                self.no_speech_timer = None
            
            # Отправляем к AI
            msg = String()
            msg.data = text
            self.ai_question_publisher.publish(msg)
            
            # Переходим в паузу и ждем команды на возобновление от AI
            self.listening_for_trigger = False
            self.capturing_command = False
            self.get_logger().info(f"📤 Отправлено в AI: '{text}' | ⏸️ Ожидание ответа...")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки к AI: {e}")
            self._reset_to_listening()

    def _handle_recognition_control(self, msg: Bool) -> None:
        """Управление состоянием распознавания"""
        try:
            if msg.data:
                # Возобновление работы - возврат к прослушиванию
                self._reset_to_listening()
                self.get_logger().info("✓ Распознавание возобновлено")
            else:
                # Пауза - останавливаем все операции
                self.listening_for_trigger = False
                self.capturing_command = False
                
                # Останавливаем все таймеры
                if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
                    self.destroy_timer(self.no_speech_timer)
                    self.no_speech_timer = None
                
                self.get_logger().info("⏸️ Распознавание приостановлено")
                
        except Exception as e:
            self.get_logger().error(f"Ошибка управления распознаванием: {e}")

    def _get_doa_angle(self) -> Optional[int]:
        """Получить текущий DOA угол."""
        if not self.doa_enabled:
            return None
        try:
            return self.mic_tuning.direction
        except Exception as e:
            self.get_logger().error(f"Ошибка DOA: {e}")
            return None

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
            
            # Останавливаем все таймеры
            if hasattr(self, 'no_speech_timer') and self.no_speech_timer:
                self.destroy_timer(self.no_speech_timer)
                self.no_speech_timer = None
            
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
        print("🎤 VOSK-BASED РАСПОЗНАВАТЕЛЬ: Wake Word → Vosk Command → DIRECT AI")
        print("   Состояния: listening_for_trigger ↔ capturing_command")
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
