import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time
import subprocess
import threading
from typing import Optional, List


class ChassisNode(Node):
    # Константы конфигурации
    BAUD_RATE = 9600
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.1

    # Пороги для определения движения
    LINEAR_THRESHOLD = 0.01    # 1 см/с
    ANGULAR_THRESHOLD = 0.01   # ~0.57 градуса/с

    def __init__(self):
        super().__init__('chassis_node')

        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None

        # Lock для thread-safe доступа к serial
        self.serial_lock = threading.Lock()

        # Переменные для потока чтения энкодеров
        self.reading_thread: Optional[threading.Thread] = None
        self.stop_thread = False

        self._setup_subscribers()
        self._setup_publishers()

        if self._connect_to_arduino():
            self._start_encoder_reading()
            self.get_logger().info('Chassis нода инициализирована и готова к работе')
        else:
            self.get_logger().warn('Chassis не подключен при инициализации')

    def _setup_subscribers(self):
        """Настройка подписчиков."""
        # Подписчик на команды от голосового управления
        self.command_subscriber = self.create_subscription(
            String, 'verter_commands', self.command_callback, 10
        )
        self.get_logger().info('Подписчик для verter_commands создан')

        # Подписчик на команды скорости от Nav2
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.get_logger().info('Подписчик для /cmd_vel создан')

    def _setup_publishers(self):
        """Настройка публикаторов."""
        # Publisher для данных энкодеров
        self.encoder_publisher = self.create_publisher(
            Int64MultiArray, '/wheel_encoders', 10
        )
        self.get_logger().info('Публикатор для /wheel_encoders создан')

    def _start_encoder_reading(self):
        """Запуск потока чтения данных с энкодеров."""
        self.stop_thread = False
        self.reading_thread = threading.Thread(
            target=self._encoder_reading_loop, daemon=True
        )
        self.reading_thread.start()
        self.get_logger().info('Поток чтения энкодеров запущен')

    def _encoder_reading_loop(self):
        """Цикл чтения данных энкодеров из Serial."""
        while not self.stop_thread:
            try:
                with self.serial_lock:
                    if self._is_chassis_connected() and self.arduino_serial.in_waiting > 0:
                        line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith('ENC:'):
                            self._publish_encoder_data(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Ошибка чтения Serial: {e}')
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().debug(f'Исключение в потоке энкодеров: {e}')
            time.sleep(0.01)

    def _publish_encoder_data(self, line: str):
        """Парсинг и публикация данных энкодеров."""
        try:
            parts = line[4:].split(':')  # Skip 'ENC:'
            if len(parts) == 3:
                msg = Int64MultiArray()
                msg.data = [int(parts[0]), int(parts[1]), int(parts[2])]
                self.encoder_publisher.publish(msg)
        except ValueError as e:
            self.get_logger().debug(f'Ошибка парсинга данных энкодеров: {e}')

    # Симлинк для Arduino Chassis (настраивается через udev)
    SYMLINK_PATH = '/dev/arduino_chassis'

    def _find_arduino_port(self) -> List[str]:
        """Поиск порта Arduino Chassis."""
        import os

        # Сначала проверяем симлинк (рекомендуемый способ)
        if os.path.exists(self.SYMLINK_PATH):
            self.get_logger().info(f'Найден симлинк {self.SYMLINK_PATH}')
            return [self.SYMLINK_PATH]

        # Fallback: поиск по devpath (для обратной совместимости)
        self.get_logger().info('Симлинк не найден, поиск устройства по devpath=1.1')
        available_ports = []
        ports = serial.tools.list_ports.comports()

        for port in ports:
            self.get_logger().info(f'Проверяю порт: {port.device}')

            try:
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device],
                    capture_output=True,
                    text=True
                )

                if "devpath==\"1.1\"" in result.stdout or "ATTRS{devpath}==\"1.1\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.1 на порту {port.device}')
                    available_ports.append(port.device)
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')

        if not available_ports:
            self.get_logger().warn(f'Настройте udev правила для {self.SYMLINK_PATH} (см. документацию)')

        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino."""
        available_ports = self._find_arduino_port()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.1 для Chassis')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Chassis (devpath=1.1)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port
            
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Chassis на порту {port} (devpath=1.1)')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Chassis на порту {port}: {e}')
            return False

    def _is_chassis_connected(self) -> bool:
        """Проверка подключения Chassis."""
        return self.arduino_serial and self.arduino_serial.is_open

    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands (голосовое управление)."""
        command = msg.data
        self.get_logger().info(f'Получена голосовая команда: "{command}"')
        self._send_to_arduino(command)

    def cmd_vel_callback(self, msg: Twist):
        """
        Обработчик команд скорости из топика /cmd_vel (от Nav2).

        Конвертирует команды Twist в формат verter_commands и отправляет на Arduino.
        """
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Конвертируем в команду
        command = self._twist_to_command(linear_vel, angular_vel)

        # Отправляем на Arduino
        self._send_to_arduino(command)
        self.get_logger().debug(
            f'Команда Nav2: vx={linear_vel:.2f}, vth={angular_vel:.2f} → {command}'
        )

    def _twist_to_command(self, linear_vel: float, angular_vel: float) -> str:
        """
        Конвертирует команды скорости (Twist) в команды для Arduino.

        Args:
            linear_vel: Линейная скорость (м/с)
            angular_vel: Угловая скорость (рад/с)

        Returns:
            Строка команды для Arduino в формате CHASSIS:VEL:linear:angular
        """
        # Проверяем пороги для полной остановки
        if abs(linear_vel) < self.LINEAR_THRESHOLD and abs(angular_vel) < self.ANGULAR_THRESHOLD:
            return "CHASSIS:STOP"

        # Velocity mode: отправляем линейную и угловую скорость напрямую
        return f"CHASSIS:VEL:{linear_vel:.3f}:{angular_vel:.3f}"

    def _send_to_arduino(self, command: str):
        """Отправка команды на Arduino."""
        if not self._is_chassis_connected():
            self.get_logger().warn('Chassis не подключен')
            return

        try:
            with self.serial_lock:
                self.arduino_serial.write(f"{command}\n".encode('utf-8'))
                self.arduino_serial.flush()  # Принудительная отправка буфера
            self.get_logger().info(f'Команда отправлена: "{command}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка отправки: {e}')
            # Попытка переподключения
            self._reconnect_chassis()
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка отправки: {e}')

    def _reconnect_chassis(self):
        """Переподключение к Chassis."""
        self.get_logger().info('Попытка переподключения к Chassis...')
        self._close_chassis_connection()
        
        if self._connect_to_arduino():
            self.get_logger().info('Успешно переподключен к Chassis')
        else:
            self.get_logger().warn('Не удалось переподключиться к Chassis')

    def _close_chassis_connection(self):
        """Закрытие соединения с Chassis."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()

    def destroy_node(self):
        """Корректное завершение работы узла."""
        self.get_logger().info('Завершение работы Chassis ноды')

        # Останавливаем поток чтения энкодеров
        self.stop_thread = True
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=2.0)
            self.get_logger().info('Поток чтения энкодеров остановлен')

        self._close_chassis_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    chassis_node = ChassisNode()
    
    try:
        rclpy.spin(chassis_node)
    except KeyboardInterrupt:
        chassis_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        chassis_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
