#!/usr/bin/env python3
import json
import os
import threading
from typing import Optional
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
import sounddevice as sd
import vosk
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool


@dataclass
class SpeechToTextConfig:
    """Конфигурация Speech-to-Text распознавателя"""
    MODEL_NAME: str = 'vosk-model-small-ru-0.22'
    SAMPLE_RATE: int = 16000
    BLOCK_SIZE: int = 6000
    CHANNELS: int = 1


class SpeechToTextNode(Node):
    """ROS2 узел для преобразования речи в текст через Vosk"""
    
    def __init__(self) -> None:
        super().__init__('speech_to_text_node')
        
        # Конфигурация
        self.config = SpeechToTextConfig()
        
        # Флаг активности (для остановки во время TTS)
        self.is_active = True
        self.active_lock = threading.Lock()
        
        # Потокобезопасность
        self.shutdown_event = threading.Event()
        self.recognition_lock = threading.Lock()
        
        # Инициализация
        self._setup_ros_communication()
        self._setup_audio_system()
        self._start_audio_capture()
        
        self.get_logger().info("🎤 SpeechToTextNode запущен")
        self.get_logger().info(f"   Sample rate: {self.config.SAMPLE_RATE}Hz")
        self.get_logger().info(f"   Block size: {self.config.BLOCK_SIZE}")
    
    def _setup_ros_communication(self) -> None:
        """Настройка ROS2 коммуникации"""
        # Publisher для распознанного текста
        self.recognized_text_pub = self.create_publisher(
            String, 
            'recognized_text', 
            10
        )
        
        # Subscribers для управления активностью
        # Слушаем команды от recognition_node
        self.create_subscription(
            Bool,
            'speech_control',
            self._handle_activation,
            10
        )
        # Слушаем команды от text_to_speech_node
        self.create_subscription(
            Bool,
            'tts_control',
            self._handle_activation,
            10
        )
    
    def _setup_audio_system(self) -> None:
        """Настройка аудио системы и Vosk"""
        self.audio_device = self._find_audio_device()
        self.vosk_model = self._load_vosk_model()
        self.recognizer = vosk.KaldiRecognizer(
            self.vosk_model, 
            self.config.SAMPLE_RATE
        )
        self.recognizer.SetWords(True)
    
    def _find_audio_device(self) -> Optional[int]:
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
        if self.shutdown_event.is_set():
            return
        
        # Проверяем активность - не обрабатываем аудио если деактивированы
        with self.active_lock:
            if not self.is_active:
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
                self._publish_recognized_text(text)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Ошибка JSON от Vosk: {e}")
        except Exception as e:
            self.get_logger().error(f"Ошибка обработки результата Vosk: {e}")
    
    def _publish_recognized_text(self, text: str) -> None:
        """Публикация распознанного текста"""
        # Двойная проверка активности перед публикацией
        with self.active_lock:
            if not self.is_active:
                self.get_logger().debug(f"🔇 Пропущено (деактивирован): '{text}'")
                return
        
        msg = String()
        msg.data = text
        self.recognized_text_pub.publish(msg)
        self.get_logger().debug(f"🎤 Распознано: '{text}'")
    
    def _handle_activation(self, msg: Bool) -> None:
        """Обработка команды активации/деактивации"""
        with self.active_lock:
            self.is_active = msg.data
        
        status = "включен" if msg.data else "выключен (TTS активен)"
        self.get_logger().info(f"📡 SpeechToText {status}")
    
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
            self.get_logger().info("🔄 Завершение работы SpeechToTextNode...")
            self.shutdown_event.set()
            self._cleanup_resources()
            self.get_logger().info("✅ SpeechToTextNode завершен")
        except Exception as e:
            self.get_logger().error(f"Ошибка завершения: {e}")


def main(args=None) -> None:
    """Главная функция запуска"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = SpeechToTextNode()
        print("🎤 SPEECH-TO-TEXT NODE")
        print("   Публикация в топик: recognized_text")
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

