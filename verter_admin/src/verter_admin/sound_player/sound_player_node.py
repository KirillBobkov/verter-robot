#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import threading
from ament_index_python.packages import get_package_share_directory

class SoundPlayerNode(Node):
    def __init__(self):
        super().__init__('sound_player_node')
        
        # Аудиоустройство для воспроизведения
        if "WSL_DISTRO_NAME" in os.environ:
            self.audio_device = "pulse"  # WSL - используем PulseAudio
        else:
            self.audio_device = "pulse"  # Используем PulseAudio
        
        # Процессы воспроизведения для остановки
        self.current_ffmpeg = None
        self.current_aplay = None
        
        # Счетчик для чередования шуток
        self.joke_counter = 0
        
        # Создание подписчика на топик play
        self.subscription = self.create_subscription(
            String,
            'play',
            self.sound_command_callback,
            10
        )
        
        
        # Получение пути к папке со звуковыми файлами
        self._setup_sound_directory()
        
        # Настройка окружения для аудио
        self._setup_environment()
        
        self.get_logger().info("SoundPlayerNode инициализирован успешно")

    def _setup_sound_directory(self):
        """Настройка директории со звуковыми файлами"""
        try:
            package_share = get_package_share_directory('verter_admin')
            self.sound_directory = os.path.join(package_share, 'sounds')
            
            # Создание директории если не существует
            if not os.path.exists(self.sound_directory):
                os.makedirs(self.sound_directory)
                self.get_logger().info(f"Создана директория звуков: {self.sound_directory}")
            else:
                self.get_logger().info(f"Используется директория звуков: {self.sound_directory}")
                
        except Exception as e:
            self.get_logger().error(f"Ошибка настройки директории звуков: {e}")
            self.sound_directory = None

    def _setup_environment(self):
        """Настройка окружения для аудио"""
        self.env = os.environ.copy()
        if "WSL_DISTRO_NAME" in os.environ:
            self.env["PULSE_SERVER"] = "unix:/mnt/wslg/PulseServer"
        self.env.setdefault("LC_ALL", "C.UTF-8")
        self.env.setdefault("LANG", "C.UTF-8")

    def sound_command_callback(self, msg):
        """Callback для обработки команд воспроизведения звука"""
        command = msg.data.strip()
        self.get_logger().info(f"Получена команда звука: {command}")
        
        # Специальные команды
        if command == 'stop_sound':
            self._stop_sound()
            self._kill_all_audio_processes()
        else:
            # Обычные звуковые команды - добавляем .wav если нет расширения
            if not any(command.endswith(ext) for ext in ['.mp3', '.wav', '.ogg']):
                filename = f'{command}.wav'
            else:
                filename = command
            self._play_sound(filename)

    def _play_sound(self, filename):
        """Воспроизведение звукового файла - wav напрямую, mp3 через ffmpeg"""
        if not self.sound_directory:
            self.get_logger().error("Директория звуков недоступна")
            return
            
        sound_path = os.path.join(self.sound_directory, filename)
        
        if not os.path.exists(sound_path):
            self.get_logger().error(f"Звуковой файл не найден: {sound_path}")
            return
            
        try:
            # Остановка предыдущего воспроизведения если есть
            self._stop_sound()
            
            if filename.endswith('.wav'):
                # Прямое воспроизведение wav файла - БЫСТРО!
                self.current_aplay = subprocess.Popen(
                    ['aplay', '-D', self.audio_device, '-q', sound_path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                    env=self.env
                )
                self.current_ffmpeg = None  # ffmpeg не нужен
                
            else:
                # Конвертация mp3/ogg через ffmpeg
                self.current_ffmpeg = subprocess.Popen(
                    [
                        'ffmpeg', '-i', sound_path,
                        '-f', 'wav', 'pipe:1'
                    ],
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
                )

                # aplay: wav (stdin) -> динамики
                self.current_aplay = subprocess.Popen(
                    ['aplay', '-D', self.audio_device, '-q', '-f', 'S16_LE', '-r', '22050', '-c', '1', '--buffer-size=8192'],
                    stdin=self.current_ffmpeg.stdout, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    env=self.env
                )

            self.get_logger().info(f"Воспроизводится звук: {filename}")
            
            # Запуск асинхронного ожидания завершения
            threading.Thread(target=self._wait_for_completion, daemon=True).start()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения звука {filename}: {e}")
            self.current_ffmpeg = None
            self.current_aplay = None

    def _stop_sound(self):
        """Остановка текущего воспроизведения звука"""
        try:
            if self.current_aplay and self.current_aplay.poll() is None:
                self.current_aplay.kill()
                try:
                    self.current_aplay.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
                self.get_logger().info("Процесс aplay остановлен")
                
            if self.current_ffmpeg and self.current_ffmpeg.poll() is None:
                self.current_ffmpeg.kill()
                try:
                    self.current_ffmpeg.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    pass
                self.get_logger().info("Процесс ffmpeg остановлен")
                
            self.current_ffmpeg = None
            self.current_aplay = None
            
        except Exception as e:
            self.get_logger().error(f"Ошибка остановки звука: {e}")

    def _kill_all_audio_processes(self):
        """Принудительная остановка всех аудио процессов"""
        try:
            # Убиваем все процессы aplay
            subprocess.run(['pkill', '-f', 'aplay'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # Убиваем все процессы ffmpeg
            subprocess.run(['pkill', '-f', 'ffmpeg'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("Все аудио процессы остановлены")
            
            # Очищаем ссылки на процессы
            self.current_ffmpeg = None
            self.current_aplay = None
                
        except Exception as e:
            self.get_logger().error(f"Ошибка остановки аудио процессов: {e}")

    def _wait_for_completion(self):
        """Асинхронное ожидание завершения воспроизведения"""
        try:
            if self.current_aplay:
                self.current_aplay.wait()
                if self.current_aplay.returncode != 0:
                    stderr = self.current_aplay.stderr.read().decode() if self.current_aplay.stderr else ''
                    self.get_logger().error(f"aplay завершился с кодом {self.current_aplay.returncode}: {stderr}")
                
            # Очистка процессов после завершения
            if self.current_aplay and self.current_aplay.poll() is not None:
                self.current_ffmpeg = None
                self.current_aplay = None
                self.get_logger().info("Воспроизведение звука завершено")
                
        except Exception as e:
            self.get_logger().error(f"Ошибка ожидания завершения: {e}")


    def _play_joke(self):
        """Воспроизведение шуток с чередованием"""
        # Увеличиваем счетчик и сбрасываем после 3
        self.joke_counter = (self.joke_counter % 3) + 1
        
        # Выбираем файл
        joke_filename = f'joke{self.joke_counter}.mp3'
        
        self.get_logger().info(f"Воспроизводится шутка {self.joke_counter}")
        self._play_sound(joke_filename)

    def shutdown(self):
        """Завершение работы узла"""
        self._stop_sound()
        self.get_logger().info("Завершение работы SoundPlayerNode")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SoundPlayerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard interrupt received")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
