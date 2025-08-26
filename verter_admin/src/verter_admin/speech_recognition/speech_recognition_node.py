#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import vosk
import sounddevice as sd
import queue
import json
import os
import threading
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory


class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Инициализация базовых атрибутов
        self.audio_queue_ = queue.Queue(maxsize=50)
        self.shutdown_event_ = threading.Event()
        
        # Настройка компонентов
        self._setup_parameters()
        self._setup_vosk()
        self._start_audio_capture()
        
        # Запуск обработки
        self.process_timer = self.create_timer(0.05, self.process_audio)
        self.get_logger().info("SpeechRecognitionNode инициализирован успешно")

    def _find_audio_device(self):
        """Автоматический поиск аудио устройства"""
        try:
            devices = sd.query_devices()
            
            # Ищем любое доступное устройство ввода
            for i, device in enumerate(devices):
                if device['max_input_channels'] > 0:
                    self.get_logger().info(f"Найдено аудиоустройство: device {i} - {device['name']}")
                    try:
                        sd.check_input_settings(device=i, samplerate=16000, channels=1)
                        self.get_logger().info(f"✓ Устройство {i} поддерживает требуемые параметры")
                        return i
                    except Exception as e:
                        self.get_logger().warn(f"Устройство {i} не поддерживает параметры: {e}")
                        continue
            
            # Fallback на системное устройство по умолчанию
            self.get_logger().warn("Используется устройство по умолчанию")
            return None
            
        except Exception as e:
            self.get_logger().error(f"Ошибка поиска устройства: {e}")
            return None

    def _find_respeaker_device(self):
        """Поиск ReSpeaker устройства"""
        devices = sd.query_devices()
        for i, device in enumerate(devices):
            if 'ReSpeaker' in device['name'] or 'ArrayUAC10' in device['name']:
                self.get_logger().info(f"✓ Найден ReSpeaker: device {i} - {device['name']}")
                return i
        
        # Fallback на hw:3,0
        self.get_logger().info("Используется hw:3,0 для ReSpeaker")
        return 3


    def _setup_parameters(self):
        """Настройка локальных параметров"""
        self.model_name_ = 'vosk-model-small-ru-0.22'
        self.samplerate_ = 16000
        self.blocksize_ = 3200
        self.channels_ = 1
        self.device_ = self._find_respeaker_device()
        vosk.SetLogLevel(-1)  # Отключить логи Vosk

    def _setup_vosk(self):
        """Инициализация модели и распознавателя Vosk"""
        model_path = self._resolve_model_path()
        if not model_path:
            raise RuntimeError("Failed to resolve model path")
            
        try:
            self.model_ = vosk.Model(model_path)
            self.recognizer_ = vosk.KaldiRecognizer(self.model_, self.samplerate_)
            self.recognizer_.SetWords(True)
            self.get_logger().info(f"Модель Vosk загружена из: {model_path}")
        except Exception as e:
            raise RuntimeError(f"Ошибка загрузки модели Vosk: {e}")

    def _resolve_model_path(self):
        """Определение полного пути к модели Vosk"""
        if os.path.isabs(self.model_name_):
            model_path = self.model_name_
        else:
            try:
                package_share = get_package_share_directory('verter_admin')
                model_path = os.path.join(package_share, self.model_name_)
            except Exception as e:
                self.get_logger().error(f"Пакет 'verter_admin' не найден: {e}")
                return None
        
        if not os.path.isdir(model_path):
            self.get_logger().error(f"Директория модели не найдена: {model_path}")
            return None
            
        return model_path

    def _start_audio_capture(self):
        """Запуск потока захвата аудио"""
        self.audio_thread_ = threading.Thread(target=self._audio_capture_loop)
        self.audio_thread_.daemon = True
        self.audio_thread_.start()

    def _audio_capture_loop(self):
        """Основной цикл захвата аудио"""
        try:
            device_info = sd.query_devices(self.device_, 'input') if self.device_ is not None else None
            device_name = device_info['name'] if device_info else 'default'
            self.get_logger().info(f"Используется аудиоустройство: {device_name}")
            
            with sd.RawInputStream(
                samplerate=self.samplerate_,
                blocksize=self.blocksize_,  
                device=self.device_,
                dtype='int16',
                channels=self.channels_,
                callback=self._audio_callback
            ):
                self.get_logger().info("✓ Захват аудио запущен успешно")
                self.shutdown_event_.wait()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка захвата аудио: {e}")


    def _audio_callback(self, indata, frames, time, status):
        """Callback без отладки"""
        if status:
            self.get_logger().warn(f"Статус аудио: {status}")
        
        if not self.shutdown_event_.is_set():
            self.audio_queue_.put(bytes(indata))

    def process_audio(self):
        """Батчевая обработка без блокировки"""
        if self.shutdown_event_.is_set():
            return
        
        # Обработать все данные за раз
        processed = 0
        while not self.audio_queue_.empty() and processed < 10:  # Лимит на batch
            try:
                data = self.audio_queue_.get_nowait()
                if self.recognizer_.AcceptWaveform(data):
                    result = json.loads(self.recognizer_.Result())
                    text = result.get('text', '').strip()
                    if text:
                        self.get_logger().info(f"Распознано: {text}")
                processed += 1
            except queue.Empty:
                break
            except Exception as e:
                self.get_logger().error(f"Ошибка: {e}")
                break

    def shutdown(self):
        """Завершение работы узла"""
        self.shutdown_event_.set()
        
        # Остановка аудио
        try:
            sd.stop()
        except:
            pass
        
        # Ожидание завершения потока
        if hasattr(self, 'audio_thread_'):
            self.audio_thread_.join(timeout=1.0)
        
        # Очистка очереди
        while not self.audio_queue_.empty():
            try:
                self.audio_queue_.get_nowait()
            except:
                break


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechRecognitionNode()
        print("Узел распознавания речи запущен. Говорите...")
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