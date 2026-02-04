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
    """
    Chassis Node - управление шасси робота через Arduino.

    Архитектура:
    - Основной поток ROS2: обрабатывает callbacks, отправляет команды
    - Daemon thread: читает данные энкодеров
    """

    # Константы конфигурации
    BAUD_RATE = 115200  # Синхронизировано с Arduino
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.1  # Достаточный таймаут для readline

    # Пороги для определения движения
    LINEAR_THRESHOLD = 0.01    # 1 см/с
    ANGULAR_THRESHOLD = 0.01   # ~0.57 градуса/с

    # Throttling: минимальный интервал между командами (секунды)
    # Arduino PID работает на 50мс, teleop на 50мс - отправляем не чаще 30мс
    MIN_CMD_INTERVAL = 0.03

    # Симлинк для Arduino Chassis (настраивается через udev)
    SYMLINK_PATH = '/dev/arduino_chassis'

    def __init__(self):
        super().__init__('chassis_node')

        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None

        # Lock для serial операций (чтение в потоке, запись из callbacks)
        self.serial_lock = threading.Lock()

        # Поток чтения энкодеров
        self.reading_thread: Optional[threading.Thread] = None
        self.stop_thread = False

        # Throttling: время последней отправленной команды
        self.last_cmd_time = 0.0
        self.last_command = ""

        self._setup_subscribers()
        self._setup_publishers()

        if self._connect_to_arduino():
            self._start_encoder_reading()
            self.get_logger().info('Chassis node ready (115200 baud)')
        else:
            self.get_logger().warn('Chassis not connected at init')

    def _setup_subscribers(self):
        """Настройка подписчиков."""
        self.command_subscriber = self.create_subscription(
            String, 'verter_commands', self.command_callback, 10
        )

        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )

    def _setup_publishers(self):
        """Настройка публикаторов."""
        self.encoder_publisher = self.create_publisher(
            Int64MultiArray, '/wheel_encoders', 10
        )

    def _start_encoder_reading(self):
        """Запуск потока чтения данных с энкодеров."""
        self.stop_thread = False
        self.reading_thread = threading.Thread(
            target=self._encoder_reading_loop, daemon=True
        )
        self.reading_thread.start()

    def _encoder_reading_loop(self):
        """Цикл чтения данных энкодеров из Serial (отдельный поток)."""
        while not self.stop_thread:
            try:
                if not self._is_chassis_connected():
                    time.sleep(0.1)
                    continue

                with self.serial_lock:
                    if self.arduino_serial.in_waiting > 0:
                        line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                        if line.startswith('ENC:'):
                            self._publish_encoder_data(line)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}', throttle_duration_sec=5.0)
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().debug(f'Encoder thread exception: {e}')

            time.sleep(0.005)  # 200 Hz polling

    def _publish_encoder_data(self, line: str):
        """Парсинг и публикация данных энкодеров."""
        try:
            parts = line[4:].split(':')  # Skip 'ENC:'
            if len(parts) == 3:
                msg = Int64MultiArray()
                msg.data = [int(parts[0]), int(parts[1]), int(parts[2])]
                self.encoder_publisher.publish(msg)
        except ValueError:
            pass

    def _find_arduino_port(self) -> List[str]:
        """Поиск порта Arduino Chassis."""
        import os

        # Проверяем симлинк
        if os.path.exists(self.SYMLINK_PATH):
            self.get_logger().info(f'Found symlink {self.SYMLINK_PATH}')
            return [self.SYMLINK_PATH]

        # Ищем по devpath
        self.get_logger().info('Symlink not found, searching by devpath=1.1')
        available_ports = []
        ports = serial.tools.list_ports.comports()

        for port in ports:
            try:
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device],
                    capture_output=True, text=True
                )
                if 'devpath=="1.1"' in result.stdout or 'ATTRS{devpath}=="1.1"' in result.stdout:
                    self.get_logger().info(f'Found device at {port.device}')
                    available_ports.append(port.device)
            except Exception as e:
                self.get_logger().error(f'Error checking {port.device}: {e}')

        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino."""
        available_ports = self._find_arduino_port()

        if not available_ports:
            self.get_logger().error('No chassis device found')
            return False

        port = available_ports[0]
        self.get_logger().info(f'Connecting to {port}')

        try:
            self.arduino_serial = serial.Serial(
                port,
                self.BAUD_RATE,
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port

            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Connected to {port}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return False

    def _is_chassis_connected(self) -> bool:
        """Проверка подключения Chassis."""
        return self.arduino_serial and self.arduino_serial.is_open

    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands."""
        command = msg.data
        self.get_logger().info(f'Voice command: "{command}"')
        self._send_to_arduino(command)

    def cmd_vel_callback(self, msg: Twist):
        """Обработчик команд скорости из топика /cmd_vel."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        command = self._twist_to_command(linear_vel, angular_vel)
        current_time = time.time()

        # STOP команды отправляем всегда (безопасность)
        is_stop = (command == "CHASSIS:STOP")

        if not is_stop:
            # Throttling: ограничиваем частоту
            if current_time - self.last_cmd_time < self.MIN_CMD_INTERVAL:
                return

            # Дедупликация: не отправляем идентичные команды
            if command == self.last_command:
                return

        self._send_to_arduino(command)
        self.last_cmd_time = current_time
        self.last_command = command

    def _twist_to_command(self, linear_vel: float, angular_vel: float) -> str:
        """Конвертирует Twist в команду для Arduino."""
        if abs(linear_vel) < self.LINEAR_THRESHOLD and abs(angular_vel) < self.ANGULAR_THRESHOLD:
            return "CHASSIS:STOP"
        return f"CHASSIS:VEL:{linear_vel:.3f}:{angular_vel:.3f}"

    def _send_to_arduino(self, command: str):
        """Отправка команды на Arduino (синхронная)."""
        if not self._is_chassis_connected():
            self.get_logger().warn('Chassis not connected', throttle_duration_sec=5.0)
            return

        try:
            with self.serial_lock:
                self.arduino_serial.write(f"{command}\n".encode('utf-8'))
                self.arduino_serial.flush()  # Гарантируем отправку
        except serial.SerialException as e:
            self.get_logger().error(f'Send error: {e}')
            self._reconnect_chassis()

    def _reconnect_chassis(self):
        """Переподключение к Chassis."""
        self.get_logger().info('Reconnecting to chassis...')
        self._close_chassis_connection()

        if self._connect_to_arduino():
            self.get_logger().info('Reconnected successfully')
        else:
            self.get_logger().warn('Reconnection failed')

    def _close_chassis_connection(self):
        """Закрытие соединения с Chassis."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()

    def destroy_node(self):
        """Корректное завершение работы узла."""
        self.get_logger().info('Shutting down chassis node')

        # Останавливаем робота
        self._send_to_arduino("CHASSIS:STOP")

        # Останавливаем поток чтения энкодеров
        self.stop_thread = True
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=2.0)

        self._close_chassis_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    chassis_node = ChassisNode()

    try:
        rclpy.spin(chassis_node)
    except KeyboardInterrupt:
        chassis_node.get_logger().info('Interrupted (Ctrl+C)')
    finally:
        chassis_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
