#!/usr/bin/env python3
import json
import os
import queue
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
from std_msgs.msg import String

from .tuning import Tuning

# Константы
DEFAULT_MODEL_NAME = 'vosk-model-small-ru-0.22'
SAMPLE_RATE = 16000
BLOCK_SIZE = 3200
CHANNELS = 1
AUDIO_QUEUE_SIZE = 50
RESULTS_QUEUE_SIZE = 10
PROCESS_TIMER_PERIOD = 0.01  # 100Hz
DOA_TIMER_PERIOD = 3.0  # 0.33Hz
AUDIO_TIMEOUT = 0.1

RESPEAKER_VENDOR_ID = 0x2886
RESPEAKER_PRODUCT_ID = 0x0018

# Константы для детекции завершения фраз
SILENCE_TIMEOUT = 2.0  # Секунды тишины для завершения фразы


class SpeechRecognitionNode(Node):
    """ROS2 узел для распознавания речи с использованием модели Vosk."""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Очереди для межпоточного взаимодействия
        self.audio_queue = queue.Queue(maxsize=AUDIO_QUEUE_SIZE)
        self.results_queue = queue.Queue(maxsize=RESULTS_QUEUE_SIZE)
        self.shutdown_event = threading.Event()
        
        # Переменные для детекции завершения фраз
        self.current_phrase = ""
        self.last_speech_time = time.time()
        self.is_paused = False
        self.phrase_lock = threading.Lock()
                # Создание publisher для отправки вопросов AI
        self.ai_question_publisher = self.create_publisher(String, 'ai_question', 10)
        
        # Создание subscriber для получения ответов AI
        self.ai_response_subscriber = self.create_subscription(
            String,
            'ai_response',
            self._handle_ai_response_topic,
            10
        )
        
        # Настройка компонентов
        self._setup_parameters()
        self._setup_vosk()
        self._setup_doa()
        self._start_audio_capture()
        
        # Запуск обработки
        self._start_speech_processing()
        self.publish_timer = self.create_timer(PROCESS_TIMER_PERIOD, self._publish_results)
        self.doa_timer = self.create_timer(DOA_TIMER_PERIOD, self._process_doa)
        
        self.get_logger().info("SpeechRecognitionNode инициализирован успешно")

    def _find_respeaker_device(self) -> int:
        """Найти устройство ReSpeaker."""
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
                self.get_logger().info(f"✓ Найден ReSpeaker: устройство {i} - {device['name']}")
                return i
        
        # Резервный вариант - устройство по умолчанию
        self.get_logger().info("ReSpeaker не найден, используется устройство по умолчанию")
        return 1

    def _setup_parameters(self) -> None:
        """Настройка локальных параметров."""
        self.model_name = DEFAULT_MODEL_NAME
        self.sample_rate = SAMPLE_RATE
        self.block_size = BLOCK_SIZE
        self.channels = CHANNELS
        self.device = self._find_respeaker_device()
        vosk.SetLogLevel(-1)  # Отключить логи Vosk

    def _setup_vosk(self) -> None:
        """Инициализация модели и распознавателя Vosk."""
        model_path = self._resolve_model_path()
        if not model_path:
            raise RuntimeError("Не удалось определить путь к модели")
            
        try:
            self.model = vosk.Model(model_path)
            self.recognizer = vosk.KaldiRecognizer(self.model, self.sample_rate)
            self.recognizer.SetWords(True)
            self.get_logger().info(f"Модель Vosk загружена из: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Ошибка загрузки модели Vosk: {e}")

    def _resolve_model_path(self) -> Optional[str]:
        """Определить полный путь к модели Vosk."""
        if os.path.isabs(self.model_name):
            model_path = self.model_name
        else:
            try:
                package_share = get_package_share_directory('verter_admin')
                model_path = os.path.join(package_share, self.model_name)
            except Exception as e:
                self.get_logger().error(f"Пакет 'verter_admin' не найден: {e}")
                return None
        
        if not os.path.isdir(model_path):
            self.get_logger().error(f"Директория модели не найдена: {model_path}")
            return None
            
        return model_path

    def _setup_doa(self) -> None:
        """Инициализация определения направления прихода звука (DOA)."""
        try:
            # Настройка USB для ReSpeaker
            self.doa_dev = usb.core.find(idVendor=RESPEAKER_VENDOR_ID, idProduct=RESPEAKER_PRODUCT_ID)
            if self.doa_dev is None:
                self.get_logger().error("ReSpeaker не найден для DOA")
                self.doa_enabled = False
            else:
                self.mic_tuning = Tuning(self.doa_dev)
                self.doa_enabled = True
                self.get_logger().info("ReSpeaker для DOA инициализирован успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка инициализации DOA: {e}")
            self.doa_enabled = False

    def _start_audio_capture(self) -> None:
        """Запустить поток захвата аудио."""
        self.audio_thread = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()

    def _start_speech_processing(self) -> None:
        """Запустить поток обработки речи."""
        self.speech_thread = threading.Thread(target=self._speech_processing_loop)
        self.speech_thread.daemon = True
        self.speech_thread.start()

    def _audio_capture_loop(self) -> None:
        """Основной цикл захвата аудио."""
        try:
            device_info = sd.query_devices(self.device, 'input') if self.device is not None else None
            device_name = device_info['name'] if device_info else 'default'
            self.get_logger().info(f"Используется аудио устройство: {device_name}")
            
            with sd.RawInputStream(
                samplerate=self.sample_rate,
                blocksize=self.block_size,  
                device=self.device,
                dtype='int16',
                channels=self.channels,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Захват аудио запущен успешно")
                self.shutdown_event.wait()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка захвата аудио: {e}")

    def _audio_callback(self, indata, frames, time, status) -> None:
        """Коллбэк ввода аудио."""
        if status:
            self.get_logger().warn(f"Статус аудио: {status}")
        
        # Не добавляем аудио в очередь если система на паузе или завершается
        if not self.shutdown_event.is_set() and not self.is_paused:
            try:
                self.audio_queue.put_nowait(bytes(indata))
            except queue.Full:
                pass  # Пропускаем если очередь переполнена

    def _speech_processing_loop(self) -> None:
        """Основной цикл обработки речи, выполняющийся в отдельном потоке."""
        while not self.shutdown_event.is_set():
            try:
                # Проверяем, не находимся ли мы в паузе
                current_time = time.time()
                if self.is_paused:
                    time.sleep(0.1)
                    continue
                
                # Блокирующее получение с таймаутом
                data = self.audio_queue.get(timeout=AUDIO_TIMEOUT)
                
                # Обрабатываем только если не в паузе
                if not self.is_paused:
                    if self.recognizer.AcceptWaveform(data):
                        result = json.loads(self.recognizer.Result())
                        text = result.get('text', '').strip()
                        if text:
                            self._handle_recognized_text(text)
                    else:
                        # Частичный результат - обновляем время последней речи
                        partial_result = json.loads(self.recognizer.PartialResult())
                        partial_text = partial_result.get('partial', '').strip()
                        if partial_text:
                            self.last_speech_time = current_time
                            with self.phrase_lock:
                                self.current_phrase = partial_text
                        
                        # Проверяем тишину для завершения фразы
                        self._check_phrase_completion(current_time)
                            
            except queue.Empty:
                # Проверяем тишину даже при пустой очереди
                self._check_phrase_completion(time.time())
                continue
            except Exception as e:
                self.get_logger().error(f"Ошибка обработки речи: {e}")

    def _handle_recognized_text(self, text: str) -> None:
        """Обработать распознанный текст."""
        with self.phrase_lock:
            self.current_phrase = text
        
        # Отправляем результат в очередь публикации
        try:
            self.results_queue.put_nowait(text)
        except queue.Full:
            pass  # Пропускаем, если очередь переполнена
            
        self.last_speech_time = time.time()

    def _check_phrase_completion(self, current_time: float) -> None:
        """Проверить завершение фразы по тишине."""
        if (current_time - self.last_speech_time) > SILENCE_TIMEOUT:
            with self.phrase_lock:
                if self.current_phrase and not self.is_paused:
                    # Фраза завершена - отправляем к AI
                    completed_phrase = self.current_phrase
                    self.get_logger().info(f"Фраза завершена: '{completed_phrase}'")
                    
                    # Начинаем паузу и очищаем состояние
                    self.is_paused = True
                    self.current_phrase = ""
                    
                    # Очищаем аудио очередь чтобы не накапливались старые данные
                    self._clear_queue(self.audio_queue)
                    
                    self.get_logger().info("Распознавание приостановлено - запрос к AI")
                    
                    # Публикуем вопрос в топик ai_question
                    msg = String()
                    msg.data = completed_phrase
                    self.ai_question_publisher.publish(msg)
    
    def _handle_ai_response_topic(self, msg: String) -> None:
        """Обработчик ответов AI из топика ai_response."""
        response = msg.data
        self._on_ai_response(response)
    
    def _on_ai_response(self, response: str) -> None:
        """Callback функция для получения ответа от AI."""
        # self.get_logger().info(f"Ответ AI: {response}")
        
        # Сбрасываем состояние речи и возобновляем распознавание
        with self.phrase_lock:
            self.current_phrase = ""
            self.last_speech_time = time.time()
            self.is_paused = False
        
        self.get_logger().info("Возобновление распознавания речи")

    def _publish_results(self) -> None:
        """Опубликовать результаты распознавания из очереди (вызывается таймером ROS2)."""
        try:
            while not self.results_queue.empty():
                text = self.results_queue.get_nowait()
                self.get_logger().info(f"Распознано: {text}")
        except queue.Empty:
            pass

    def _process_doa(self) -> None:
        """Обработать и залогировать угол DOA."""
        if not hasattr(self, 'doa_enabled') or not self.doa_enabled:
            return
        
        try:
            doa_angle = self.mic_tuning.direction
            self.get_logger().info(f"Угол DOA: {doa_angle} градусов")
        except Exception as e:
            self.get_logger().error(f"Ошибка получения DOA: {e}")

    def shutdown(self) -> None:
        """Корректно завершить работу узла."""
        self.shutdown_event.set()
        
        # Остановить аудио
        try:
            sd.stop()
        except:
            pass
        
        # Очистить USB для DOA
        if hasattr(self, 'doa_dev') and self.doa_dev:
            usb.util.dispose_resources(self.doa_dev)
        
        # Ожидать завершения потоков
        if hasattr(self, 'audio_thread'):
            self.audio_thread.join(timeout=1.0)
        if hasattr(self, 'speech_thread'):
            self.speech_thread.join(timeout=1.0)
        
        # Очистить очереди
        self._clear_queue(self.audio_queue)
        self._clear_queue(self.results_queue)
        
    def _clear_queue(self, q: queue.Queue) -> None:
        """Очистить все элементы из очереди."""
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                break

def main(args=None) -> None:
    """Основная функция для запуска узла распознавания речи."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        print("Узел распознавания речи и DOA запущен. Говорите...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()