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
from std_msgs.msg import String, Bool

from .tuning import Tuning

# Константы для максимальной скорости
DEFAULT_MODEL_NAME = 'vosk-model-small-ru-0.22'
SAMPLE_RATE = 16000
BLOCK_SIZE = 3200   # 50мс - минимальная латентность
CHANNELS = 1
RESULT_QUEUE_SIZE = 3  # Мини-очередь только для результатов
DOA_TIMER_PERIOD = 5.0  # Реже DOA для экономии CPU

RESPEAKER_VENDOR_ID = 0x2886
RESPEAKER_PRODUCT_ID = 0x0018

class SpeechRecognitionNode(Node):
    """Простой ROS2 узел для распознавания речи - Vosk сам решает когда отдавать результат."""
    
    def __init__(self) -> None:
        super().__init__('speech_recognition_node')
        
        # Переменные для максимальной скорости
        self.shutdown_event = threading.Event()
        self.is_paused = False
        self.recognition_lock = threading.Lock()  # Для thread-safety в callback
        
        # Быстрая очередь только для результатов - не блокирует audio callback
        self.result_queue = queue.Queue(maxsize=RESULT_QUEUE_SIZE)
        
        # Publisher для отправки к AI
        self.ai_question_publisher = self.create_publisher(String, 'ai_question', 10)
        
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
        
        # Запуск быстрой отправки результатов + DOA
        self._start_result_publisher()
        self.doa_timer = self.create_timer(DOA_TIMER_PERIOD, self._process_doa)
        
        self.get_logger().info("SpeechRecognitionNode запущен (простая версия)")

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
            
            # Оптимизации для максимальной скорости
            self.recognizer.SetWords(False)  # Отключаем детализацию слов
            # self.recognizer.SetMaxAlternatives(1)  # Одна альтернатива
            
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
        """Прямой стрим в Vosk - максимально быстро!"""
        if self.shutdown_event.is_set() or self.is_paused:
            return
            
        if status:
            self.get_logger().warn(f"Аудио статус: {status}")
        
        # Прямая обработка в callback - никаких очередей!
        with self.recognition_lock:
            try:
                data = bytes(indata)
                
                # Стримим напрямую в Vosk
                if self.recognizer.AcceptWaveform(data):
                    # Vosk решил что фраза готова!
                    result = json.loads(self.recognizer.Result())
                    text = result.get('text', '').strip()
                    
                    if text:
                        self.get_logger().info(f"Быстрый стрим - фраза: '{text}'")
                        self._queue_result_fast(text)
                        
            except Exception as e:
                self.get_logger().error(f"Ошибка в прямом стриме: {e}")


    def _queue_result_fast(self, text: str) -> None:
        """Молниеносное добавление результата - не блокирует audio callback."""
        try:
            # Паузим сразу
            self.is_paused = True
            
            # Молниеносно кладём в очередь
            self.result_queue.put_nowait(text)
        except queue.Full:
            # Если переполнено - выталкиваем старый результат
            try:
                self.result_queue.get_nowait()
                self.result_queue.put_nowait(text)
            except queue.Empty:
                pass
    
    def _start_result_publisher(self) -> None:
        """Запустить быстрый publisher для результатов."""
        self.publisher_thread = threading.Thread(target=self._result_publisher_loop)
        self.publisher_thread.daemon = True
        self.publisher_thread.start()
    
    def _result_publisher_loop(self) -> None:
        """Молниеносная отправка результатов к AI."""
        while not self.shutdown_event.is_set():
            try:
                # Молниеносно получаем результат
                text = self.result_queue.get(timeout=0.1)
                
                self.get_logger().info(f"Молниеносная отправка к AI: {text}")
                
                # Отправляем к AI
                msg = String()
                msg.data = text
                self.ai_question_publisher.publish(msg)
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Ошибка отправки к AI: {e}")

    def _handle_recognition_control(self, msg: Bool) -> None:
        """Управление паузами."""
        if msg.data:
            self.is_paused = False
            self.get_logger().info("Распознавание возобновлено")
        else:
            self.is_paused = True
            self.get_logger().info("Распознавание приостановлено")

    def _process_doa(self) -> None:
        """DOA обработка."""
        if not self.doa_enabled:
            return
        try:
            doa_angle = self.mic_tuning.direction
            self.get_logger().info(f"DOA: {doa_angle}°")
        except Exception as e:
            self.get_logger().error(f"Ошибка DOA: {e}")

    def shutdown(self) -> None:
        """Завершение работы."""
        self.shutdown_event.set()
        
        try:
            sd.stop()
        except:
            pass
        
        if hasattr(self, 'doa_dev') and self.doa_dev:
            usb.util.dispose_resources(self.doa_dev)

def main(args=None) -> None:
    """Главная функция."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        print("МОЛНИЕНОСНЫЙ СТРИМ: 50мс блоки, мини очередь результатов!")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nЗавершение...")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
