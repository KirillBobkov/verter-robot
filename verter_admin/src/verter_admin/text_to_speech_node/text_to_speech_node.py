#!/usr/bin/env python3

import os
import subprocess
import threading
import time
from typing import Optional, Any
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
    """Узел для синтеза речи с использованием Piper TTS."""
    
    # Константы
    AUDIO_DEVICE = "pulse"
    MODEL_FILENAME = "ru_RU-ruslan-medium.onnx"
    BUFFER_THRESHOLD = 4096  # 4KB буфер
    AUDIO_SAMPLE_RATE = "22050"
    AUDIO_CHANNELS = "1"
    AUDIO_FORMAT = "S16_LE"
    AUDIO_BUFFER_SIZE = "8192"
    MICROPHONE_DELAY = 0.7  # Задержка для предотвращения эхо
    QUEUE_SIZE = 10
    
    def __init__(self) -> None:
        super().__init__('text_to_speech_node')
        
        if PiperVoice is None or wave is None:
            self.get_logger().error("piper-tts не установлен. Установите: pip install piper-tts")
            raise ImportError("piper-tts не найден")
        
        # Найти модель Piper
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech_node')
            self.model_path = os.path.join(tts_models_dir, self.MODEL_FILENAME)
        except Exception as e:
            self.get_logger().error(f"Модель не найдена: {e}")
            raise
        
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Файл модели не найден: {self.model_path}")
            raise FileNotFoundError("Модель Piper не найдена")
        self._setup_environment()
        self._initialize_tts()
        
        # ROS2 подписки
        self.subscription = self.create_subscription(
            String, 'text_to_speech', self.text_callback, self.QUEUE_SIZE
        )
        self.recognition_publisher = self.create_publisher(
            Bool, 'set_recognition_active', self.QUEUE_SIZE
        )
        
        # Управление конкуренцией
        self._busy_lock = threading.Lock()
        self._busy: bool = False
        self._pending_text: Optional[str] = None
        
        # Отслеживание текущего aplay процесса
        self.current_aplay_pid: Optional[int] = None

        self.get_logger().info(f"Piper TTS готов. Модель: {self.model_path}")
    
    def _initialize_tts(self) -> None:
        """Инициализация TTS модели."""
        try:
            self.voice = PiperVoice.load(self.model_path)
            self.get_logger().info(f"Piper модель загружена: {self.model_path}")
        except Exception as e:
            self.get_logger().error(f"Ошибка загрузки модели: {e}")
            raise
    
    def _setup_environment(self) -> None:
        """Настройка переменных окружения."""
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
    
    def text_callback(self, msg: String) -> None:
        """Обработчик входящих сообщений с текстом для синтеза."""
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
    
    def _speak_loop(self, first_text: str) -> None:
        """Основной цикл синтеза речи."""
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
    
    def _synthesize_and_play(self, text: str) -> None:
        """Синтез и воспроизведение речи."""
        try:
            # СНАЧАЛА ОТКЛЮЧАЕМ МИКРОФОН
            self._deactivate_speech_recognition()
            
            # Используем streaming API с оптимизированными параметрами
            process = subprocess.Popen(
                [
                    "aplay", "-D", self.AUDIO_DEVICE, "-q", "-f", self.AUDIO_FORMAT,
                    "-r", self.AUDIO_SAMPLE_RATE, "-c", self.AUDIO_CHANNELS,
                    f"--buffer-size={self.AUDIO_BUFFER_SIZE}"
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,  # Захватываем stderr для отладки
                env=self.env
            )
            
            # Буферизация для более эффективной передачи данных
            buffer = bytearray()
            
            # Синтез речи с оптимизированной буферизацией
            for chunk in self.voice.synthesize(text.strip()):
                if process.poll() is not None:
                    break
                    
                buffer.extend(chunk.audio_int16_bytes)
                
                # Отправляем данные пакетами, а не по одному chunk
                if len(buffer) >= self.BUFFER_THRESHOLD:
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
            
            # Ждем завершения и проверяем ошибки
            return_code = process.wait()
            if return_code != 0:
                stderr_output = process.stderr.read().decode() if process.stderr else "no stderr"
                self.get_logger().error(f"aplay завершился с кодом {return_code}, stderr: {stderr_output}")
            else:
                self.get_logger().info("✓ Аудио воспроизведено успешно")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения: {e}")
        finally:
            # ВКЛЮЧАЕМ МИКРОФОН ОБРАТНО С ЗАДЕРЖКОЙ (предотвращение эхо)
            time.sleep(self.MICROPHONE_DELAY)  # Задержка для затухания звука в колонках
            self._activate_speech_recognition()
    
    
    def _deactivate_speech_recognition(self) -> None:
        """Отключить распознавание перед TTS."""
        msg = Bool()
        msg.data = False
        self.recognition_publisher.publish(msg)
        self.get_logger().info("🔇 Отключаю микрофон перед TTS")
    
    def _activate_speech_recognition(self) -> None:
        """Включить распознавание после TTS."""
        msg = Bool()
        msg.data = True
        self.recognition_publisher.publish(msg)
        self.get_logger().info("🎤 Включаю микрофон после TTS")

def main(args: Optional[Any] = None) -> None:
    """Главная функция для запуска узла."""
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
