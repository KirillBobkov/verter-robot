#!/usr/bin/env python3

import os
import subprocess
import threading
import time
from typing import List, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

# Константы для аудио/инструментов
PIPER_EXEC = "piper"
APLAY_EXEC = "aplay"
SAMPLE_RATE = "22050"
AUDIO_FORMAT = "S16_LE"
CHANNELS = "1"

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
        
        # Инициализация для оптимизации
        self._setup_environment()
        self._preload_model()
        
        # Создание subscriber на топик ai_response
        self.subscription = self.create_subscription(
            String,
            'ai_response',
            self.text_callback,
            10
        )
        
        # Создание publisher для активации распознавания речи
        self.recognition_control_publisher = self.create_publisher(
            Bool,
            'set_recognition_active',
            10
        )
        
        # Простое управление конкуренцией синтеза: один поток за раз, "последнее слово выигрывает"
        self._busy_lock = threading.Lock()
        self._busy = False
        self._pending_text = None

        self.get_logger().info("TextToSpeechNode инициализирован успешно")
        self.get_logger().info(f"Модель TTS: {self.model_path}")
    
    def _setup_environment(self) -> None:
        """Подготовка переменных окружения один раз при старте."""
        # Базовое окружение
        self.env = os.environ.copy()
        # Поддержка WSLg для PulseAudio
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
            self.get_logger().info("Настроено окружение для WSL")
        # Гарантия UTF-8 для stdin Piper
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")
    
    def _preload_model(self) -> None:
        """Предзагрузка модели Piper для ускорения последующих вызовов."""
        try:
            self.get_logger().info("Начинается предзагрузка модели TTS...")
            start_time = time.time()
            
            # Запускаем piper с минимальным текстом для инициализации модели
            preload_process = subprocess.run([
                PIPER_EXEC,
                "-m", self.model_path,
                "-c", self.config_path,
                "--output-raw",
                "-t", "тест"
            ], 
            capture_output=True, 
            timeout=15,
            env=self.env
            )
            
            load_time = time.time() - start_time
            
            if preload_process.returncode == 0:
                self.get_logger().info(f"Модель TTS предзагружена за {load_time:.2f}с")
            else:
                self.get_logger().warning(f"Предзагрузка модели завершилась с кодом: {preload_process.returncode}")
                
        except subprocess.TimeoutExpired:
            self.get_logger().warning("Превышено время ожидания предзагрузки модели")
        except Exception as e:
            self.get_logger().warning(f"Не удалось предзагрузить модель: {e}")
            # Не критично, продолжаем работу
    
    def text_callback(self, msg: String) -> None:
        """Callback для обработки текста из топика ai_response."""
        text = msg.data.strip()
        
        if not text:
            self.get_logger().warning("Получен пустой текст")
            return
        
        self.get_logger().info(f"Получен текст для озвучки: {text}")
        
        # Если уже озвучиваем — запомним только последнее сообщение
        start_new = False
        with self._busy_lock:
            if self._busy:
                self._pending_text = text
                self.get_logger().info("TTS занят, обновлен отложенный текст")
            else:
                self._busy = True
                start_new = True

        if start_new:
            synthesis_thread = threading.Thread(
                target=self._speak_loop,
                args=(text,),
                daemon=True
            )
            synthesis_thread.start()
    
    def _speak_loop(self, first_text: str) -> None:
        """Последовательно озвучивает первый и (если есть) последний отложенный текст."""
        current_text = first_text
        try:
            while current_text:
                try:
                    self._synthesize_and_play(current_text)
                except Exception as e:
                    self.get_logger().error(f"Ошибка синтеза речи: {e}")
                    self._activate_speech_recognition()

                # Проверяем, не появился ли новый текст пока говорили
                with self._busy_lock:
                    if self._pending_text:
                        current_text = self._pending_text
                        self._pending_text = None
                    else:
                        current_text = None
        finally:
            with self._busy_lock:
                self._busy = False
    
    def _synthesize_and_play(self, text: str) -> None:
        """Оптимизированный синтез речи и воспроизведение (stdin → Piper → raw → aplay)."""
        try:
            start_ts = time.time()
            self.get_logger().info("Начинается синтез и воспроизведение речи")
            
            # piper: text -> raw audio (stdout) - используем предварительно настроенное окружение
            piper_process = None
            aplay_process = None
            piper_process = subprocess.Popen(
                self._build_piper_cmd(text),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                env=self.env
            )
            
            # aplay: raw audio (stdin) -> speakers
            aplay_process = subprocess.Popen(
                self._build_aplay_cmd(),
                stdin=piper_process.stdout,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=self.env
            )
            
            # Подаем текст в stdin Piper (UTF-8) и закрываем stdin
            try:
                if piper_process.stdin is not None:
                    piper_process.stdin.write((text.strip() + "\n").encode("utf-8"))
                    piper_process.stdin.close()
            except Exception:
                pass

            # Закрываем stdout piper'а для корректной работы pipe
            piper_process.stdout.close()
            
            # Ждем завершения воспроизведения
            aplay_process.wait()
            piper_process.wait()
            
            if piper_process.returncode == 0 and aplay_process.returncode == 0:
                duration = time.time() - start_ts
                self.get_logger().info(f"Синтез и воспроизведение речи завершены успешно за {duration:.2f}с")
                # Отправляем сигнал для возобновления распознавания речи
                self._activate_speech_recognition()
            else:
                self.get_logger().error(f"Ошибка процессов: piper={piper_process.returncode}, aplay={aplay_process.returncode}")
                # В случае ошибки тоже активируем распознавание
                self._activate_speech_recognition()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка синтеза речи: {e}")
            # В случае исключения тоже активируем распознавание
            self._activate_speech_recognition()
        finally:
            # Жесткая очистка процессов на случай редких зависаний
            for proc in (aplay_process, piper_process):
                try:
                    if proc is not None and proc.poll() is None:
                        proc.terminate()
                        try:
                            proc.wait(timeout=1)
                        except Exception:
                            proc.kill()
                except Exception:
                    pass
    
    def _activate_speech_recognition(self) -> None:
        """Отправить сигнал для активации распознавания речи."""
        try:
            msg = Bool()
            msg.data = True
            self.recognition_control_publisher.publish(msg)
            self.get_logger().info("Отправлен сигнал активации распознавания речи")
        except Exception as e:
            self.get_logger().error(f"Ошибка отправки сигнала активации: {e}")
    
    def _build_piper_cmd(self, text: str) -> List[str]:
        """Сформировать команду запуска Piper для генерации raw-аудио через stdout (текст пойдет в stdin)."""
        return [
            PIPER_EXEC,
            "-m", self.model_path,
            "-c", self.config_path,
            "--output-raw",
        ]

    def _build_aplay_cmd(self) -> List[str]:
        """Сформировать команду запуска aplay для воспроизведения raw-аудио."""
        return [
            APLAY_EXEC,
            "-D", self.audio_device,
            "-q",
            "-r", SAMPLE_RATE,
            "-f", AUDIO_FORMAT,
            "-c", CHANNELS,
            "-t", "raw",
        ]
    
    def shutdown(self) -> None:
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
