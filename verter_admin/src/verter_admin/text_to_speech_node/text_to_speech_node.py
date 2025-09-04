#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class TextToSpeechNode(Node):
    """ROS2 узел для преобразования текста в речь используя Piper TTS."""
    
    def __init__(self):
        super().__init__('text_to_speech_node')
        
        # Путь к модели Piper TTS через ROS2 package system
        try:
            package_share = get_package_share_directory('verter_admin')
            tts_models_dir = os.path.join(package_share, 'text_to_speech_node')
            self.model_path = os.path.join(tts_models_dir, "ru_RU-ruslan-medium.onnx")
            self.config_path = os.path.join(tts_models_dir, "ru_RU-ruslan-medium.onnx.json")
        except Exception as e:
            self.get_logger().warning(f"Не удалось найти пакет verter_admin: {e}")
        
        # Проверка наличия модели
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"Модель TTS не найдена: {self.model_path}")
            raise FileNotFoundError(f"Модель TTS не найдена: {self.model_path}")
        
        if not os.path.exists(self.config_path):
            self.get_logger().error(f"Конфиг TTS не найден: {self.config_path}")
            raise FileNotFoundError(f"Конфиг TTS не найден: {self.config_path}")
        
        # Настройки аудио
        self.audio_device = "pulse"  # Используем pulse audio
        
        # Создание subscriber на топик ai_response
        self.subscription = self.create_subscription(
            String,
            'ai_response',
            self.text_callback,
            10
        )
        
        self.get_logger().info("TextToSpeechNode инициализирован успешно")
        self.get_logger().info(f"Модель TTS: {self.model_path}")
    
    def text_callback(self, msg):
        """Callback для обработки текста из топика ai_response."""
        text = msg.data.strip()
        
        if not text:
            self.get_logger().warning("Получен пустой текст")
            return
        
        self.get_logger().info(f"Получен текст для озвучки: {text}")
        
        try:
            self._synthesize_and_play(text)
        except Exception as e:
            self.get_logger().error(f"Ошибка синтеза речи: {e}")
    
    def _synthesize_and_play(self, text):
        """Прямой синтез речи и воспроизведение без файлов."""
        try:
            # Настройка окружения для WSL
            env = os.environ.copy()
            if "WSL_DISTRO_NAME" in env:
                env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
            
            self.get_logger().info("Начинается синтез и воспроизведение речи")
            
            # piper: text -> raw audio (stdout)
            piper_process = subprocess.Popen(
                [
                    "piper",
                    "-m", self.model_path,
                    "-c", self.config_path,
                    "--output-raw",
                    "-t", text
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                env=env
            )
            
            # aplay: raw audio (stdin) -> speakers
            aplay_process = subprocess.Popen(
                [
                    "aplay",
                    "-D", self.audio_device,
                    "-q",
                    "-r", "22050",
                    "-f", "S16_LE",
                    "-t", "raw",
                    "-c", "1"
                ],
                stdin=piper_process.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env
            )
            
            # Закрываем stdout piper'а для корректной работы pipe
            piper_process.stdout.close()
            
            # Ждем завершения воспроизведения
            aplay_process.wait()
            piper_process.wait()
            
            if piper_process.returncode == 0 and aplay_process.returncode == 0:
                self.get_logger().info("Синтез и воспроизведение речи завершены успешно")
            else:
                self.get_logger().error(f"Ошибка процессов: piper={piper_process.returncode}, aplay={aplay_process.returncode}")
            
        except Exception as e:
            self.get_logger().error(f"Ошибка синтеза речи: {e}")
    
    def shutdown(self):
        """Завершение работы узла."""
        self.get_logger().info("Завершение работы TextToSpeechNode")


def main(args=None):
    """Основная функция для запуска узла TTS."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = TextToSpeechNode()
        print("TextToSpeech Node запущен. Ожидание текста для озвучки...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nЗавершение работы...")
    except Exception as e:
        print(f"Ошибка: {e}")
    finally:
        if node:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
