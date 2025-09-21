#!/usr/bin/env python3

import os
import subprocess
import threading
import io
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

try:
    from piper import PiperVoice
    import wave
except ImportError:
    PiperVoice = None
    wave = None

class TextToSpeechNode(Node):
    
    def __init__(self):
        super().__init__('text_to_speech_node')
        
        if PiperVoice is None or wave is None:
            self.get_logger().error("piper-tts не установлен. Установите: pip install piper-tts")
            raise ImportError("piper-tts не найден")
        
        # Найти модель Piper
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech_node')
            self.model_path = os.path.join(tts_models_dir, "ru_RU-ruslan-medium.onnx")
        except Exception as e:
            self.get_logger().error(f"Модель не найдена: {e}")
            raise
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Файл модели не найден: {self.model_path}")
            raise FileNotFoundError("Модель Piper не найдена")
        
        self.audio_device = "pulse"
        self._setup_environment()
        self._initialize_tts()
        
        # ROS2 подписки
        self.subscription = self.create_subscription(String, 'ai_response', self.text_callback, 10)
        self.recognition_publisher = self.create_publisher(Bool, 'set_recognition_active', 10)
        
        # Управление конкуренцией
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None
        
        # Отслеживание текущего aplay процесса
        self.current_aplay_pid = None

        self.get_logger().info(f"Piper TTS готов. Модель: {self.model_path}")
    
    def _initialize_tts(self):
        # Загружаем модель через PiperVoice API
        try:
            self.voice = PiperVoice.load(self.model_path)
            self.get_logger().info(f"Piper модель загружена: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise
    
    def _setup_environment(self):
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
    
    def text_callback(self, msg):
        text = msg.data.strip()
        if not text:
            return
        
        start_new = False
        with self._busy_lock:
            if self._busy:
                self._pending_text = text
            else:
                self._busy = True
                start_new = True

        if start_new:
            threading.Thread(target=self._speak_loop, args=(text,), daemon=True).start()
    
    def _speak_loop(self, first_text):
        current_text = first_text
        try:
            while current_text:
                try:
                    self._synthesize_and_play(current_text)
                except Exception as e:
                    self.get_logger().error(f"Ошибка синтеза: {e}")
                    self._activate_speech_recognition()

                with self._busy_lock:
                    current_text = self._pending_text
                    self._pending_text = None
        finally:
            with self._busy_lock:
                self._busy = False
    
    def _synthesize_and_play(self, text):
        try:
            # СНАЧАЛА ОТКЛЮЧАЕМ МИКРОФОН
            self._deactivate_speech_recognition()
            
            # Используем streaming API с оптимизированными параметрами
            process = subprocess.Popen(
                ["aplay", "-D", self.audio_device, "-q", "-f", "S16_LE", "-r", "22050", "-c", "1", "--buffer-size=8192"],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=self.env
            )
            
            # Буферизация для более эффективной передачи данных
            buffer = bytearray()
            buffer_threshold = 4096  # 4KB буфер
            
            # Синтез речи с оптимизированной буферизацией
            for chunk in self.voice.synthesize(text.strip()):
                if process.poll() is not None:
                    break
                    
                buffer.extend(chunk.audio_int16_bytes)
                
                # Отправляем данные пакетами, а не по одному chunk
                if len(buffer) >= buffer_threshold:
                    try:
                        process.stdin.write(buffer)
                        buffer.clear()
                    except BrokenPipeError:
                        break
            
            # Отправляем остатки буфера
            if buffer:
                try:
                    process.stdin.write(buffer)
                except BrokenPipeError:
                    pass
            
            process.stdin.close()
            process.wait()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения: {e}")
        finally:
            # ВКЛЮЧАЕМ МИКРОФОН ОБРАТНО
            self._activate_speech_recognition()
    
    
    def _deactivate_speech_recognition(self):
        """Отключить распознавание перед TTS"""
        msg = Bool()
        msg.data = False
        self.recognition_publisher.publish(msg)
        self.get_logger().info("🔇 Отключаю микрофон перед TTS")
    
    def _activate_speech_recognition(self):
        """Включить распознавание после TTS"""
        msg = Bool()
        msg.data = True
        self.recognition_publisher.publish(msg)
        self.get_logger().info("🎤 Включаю микрофон после TTS")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TextToSpeechNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
