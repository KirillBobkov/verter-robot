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
        self.audio_device = "plughw:CARD=Headphones,DEV=0"
        
        # Процессы воспроизведения для остановки
        self.current_ffmpeg = None
        self.current_aplay = None
        
        # Счетчик для чередования шуток
        self.joke_counter = 0
        
        # Создание подписчика на топик /verter_say
        self.subscription = self.create_subscription(
            String,
            '/verter_say',
            self.sound_command_callback,
            10
        )
        
        # Создание publisher для сигнализации о состоянии воспроизведения
        self.audio_status_publisher = self.create_publisher(
            String,
            '/audio_in_progress',
            10
        )
        
        # Получение пути к папке со звуковыми файлами
        self._setup_sound_directory()
        
        self.get_logger().info("SoundPlayerNode инициализирован успешно")

    def _setup_sound_directory(self):
        """Настройка директории со звуковыми файлами"""
        try:
            package_share = get_package_share_directory('verter')
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

    def sound_command_callback(self, msg):
        """Callback для обработки команд воспроизведения звука"""
        command = msg.data.strip()
        self.get_logger().info(f"Получена команда звука: {command}")
        
        # Специальные команды
        if command == 'stop_sound':
            self._stop_sound()
            self._kill_all_audio_processes()
        elif command == 'joke':
            self._play_joke()
        else:
            # Обычные звуковые команды - добавляем .mp3 если нет
            filename = command if command.endswith('.mp3') else f'{command}.mp3'
            self._play_sound(filename)

    def _play_sound(self, filename):
        """Воспроизведение звукового файла через ffmpeg+aplay"""
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
            
            # Сигнализируем о начале воспроизведения
            status_msg = String()
            status_msg.data = 'True'
            self.audio_status_publisher.publish(status_msg)
            self.get_logger().info("Воспроизведение звука началось - распознавание речи отключено")
            
            # ffmpeg: mp3 -> wav (stdout)
            self.current_ffmpeg = subprocess.Popen(
                [
                    'ffmpeg', '-i', sound_path,
                    '-f', 'wav', 'pipe:1'
                ],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )

            # aplay: wav (stdin) -> динамики
            self.current_aplay = subprocess.Popen(
                ['aplay', '-D', self.audio_device],
                stdin=self.current_ffmpeg.stdout, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )

            self.get_logger().info(f"Воспроизводится звук: {filename}")
            
            # Запуск асинхронного ожидания завершения
            threading.Thread(target=self._wait_for_completion, daemon=True).start()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка воспроизведения звука {filename}: {e}")
            self.current_ffmpeg = None
            self.current_aplay = None
            # Сигнализируем об окончании при ошибке
            self._signal_audio_finished()

    def _stop_sound(self):
        """Остановка текущего воспроизведения звука"""
        was_playing = self.current_aplay is not None or self.current_ffmpeg is not None
        
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
            
            # Сигнализируем об окончании если что-то воспроизводилось
            if was_playing:
                self._signal_audio_finished()
            
        except Exception as e:
            self.get_logger().error(f"Ошибка остановки звука: {e}")
            # Сигнализируем об окончании при ошибке
            if was_playing:
                self._signal_audio_finished()

    def _kill_all_audio_processes(self):
        """Принудительная остановка всех аудио процессов"""
        was_playing = self.current_aplay is not None or self.current_ffmpeg is not None
        
        try:
            # Убиваем все процессы aplay
            subprocess.run(['pkill', '-f', 'aplay'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # Убиваем все процессы ffmpeg
            subprocess.run(['pkill', '-f', 'ffmpeg'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.get_logger().info("Все аудио процессы остановлены")
            
            # Очищаем ссылки на процессы
            self.current_ffmpeg = None
            self.current_aplay = None
            
            # Сигнализируем об окончании если что-то воспроизводилось
            if was_playing:
                self._signal_audio_finished()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка остановки аудио процессов: {e}")
            # Сигнализируем об окончании при ошибке
            if was_playing:
                self._signal_audio_finished()

    def _wait_for_completion(self):
        """Асинхронное ожидание завершения воспроизведения"""
        try:
            if self.current_aplay:
                self.current_aplay.wait()
                
            # Очистка процессов после завершения
            if self.current_aplay and self.current_aplay.poll() is not None:
                self.current_ffmpeg = None
                self.current_aplay = None
                self.get_logger().info("Воспроизведение звука завершено")
                # Сигнализируем об окончании воспроизведения
                self._signal_audio_finished()
                
        except Exception as e:
            self.get_logger().error(f"Ошибка ожидания завершения: {e}")
            # Сигнализируем об окончании при ошибке
            self._signal_audio_finished()

    def _signal_audio_finished(self):
        """Сигнализация об окончании воспроизведения"""
        status_msg = String()
        status_msg.data = 'False'
        self.audio_status_publisher.publish(status_msg)
        self.get_logger().info("Воспроизведение звука завершено - распознавание речи включено")

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