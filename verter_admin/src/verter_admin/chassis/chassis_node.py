import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int64MultiArray
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time
import subprocess
from collections import deque
from typing import Optional, List


class ChassisNode(Node):
    """
    Chassis Node - управление шасси робота через Arduino.

    Архитектура: один поток для всего serial I/O (без lock contention).
    Команды складываются в очередь и обрабатываются в едином цикле
    вместе с чтением данных энкодеров.
    """

    # Константы конфигурации
    BAUD_RATE = 115200  # Синхронизировано с Arduino
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.001  # Минимальный таймаут для неблокирующего чтения

    # Пороги для определения движения
    LINEAR_THRESHOLD = 0.01    # 1 см/с
    ANGULAR_THRESHOLD = 0.01   # ~0.57 градуса/с

    # Частота обработки serial (Гц)
    SERIAL_LOOP_RATE = 50.0  # 50 Гц = каждые 20мс

    # Минимальное изменение скорости для отправки новой команды
    # (дедупликация для экономии bandwidth)
    LINEAR_CHANGE_THRESHOLD = 0.02   # 2 см/с
    ANGULAR_CHANGE_THRESHOLD = 0.05  # ~3 градуса/с

    # Минимальный интервал между командами (мс)
    MIN_CMD_INTERVAL = 0.1  # 100мс = макс 10 команд/сек

    # Симлинк для Arduino Chassis (настраивается через udev)
    SYMLINK_PATH = '/dev/arduino_chassis'

    def __init__(self):
        super().__init__('chassis_node')

        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None

        # Очередь команд для отправки (вместо прямой отправки)
        self.command_queue: deque = deque(maxlen=10)

        # Буфер для накопления данных serial
        self.serial_buffer = ""

        # Для дедупликации и throttling
        self.last_sent_command = ""
        self.last_cmd_time = 0.0
        self.last_linear_vel = 0.0
        self.last_angular_vel = 0.0

        self._setup_subscribers()
        self._setup_publishers()

        if self._connect_to_arduino():
            self._start_serial_loop()
            self.get_logger().info('Chassis нода инициализирована и готова к работе')
        else:
            self.get_logger().warn('Chassis не подключен при инициализации')

    def _setup_subscribers(self):
        """Настройка подписчиков."""
        self.command_subscriber = self.create_subscription(
            String, 'verter_commands', self.command_callback, 10
        )
        self.get_logger().info('Подписчик для verter_commands создан')

        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.get_logger().info('Подписчик для /cmd_vel создан')

    def _setup_publishers(self):
        """Настройка публикаторов."""
        self.encoder_publisher = self.create_publisher(
            Int64MultiArray, '/wheel_encoders', 10
        )
        self.get_logger().info('Публикатор для /wheel_encoders создан')

    def _start_serial_loop(self):
        """Запуск единого цикла обработки serial (таймер ROS2)."""
        timer_period = 1.0 / self.SERIAL_LOOP_RATE
        self.serial_timer = self.create_timer(timer_period, self._serial_loop_callback)
        self.get_logger().info(f'Serial loop запущен с частотой {self.SERIAL_LOOP_RATE} Гц')

    def _serial_loop_callback(self):
        """
        Единый цикл обработки serial - и чтение, и запись.
        Вызывается таймером ROS2, без отдельного потока.
        """
        if not self._is_chassis_connected():
            return

        # 1. Отправляем команду из очереди (если есть)
        if self.command_queue:
            command = self.command_queue.popleft()
            self._write_to_serial(command)

        # 2. Читаем данные с serial (если есть)
        self._read_from_serial()

    def _write_to_serial(self, command: str):
        """Запись команды в serial (без lock, вызывается только из serial_loop)."""
        try:
            self.arduino_serial.write(f"{command}\n".encode('utf-8'))
            self.get_logger().debug(f'Отправлено: "{command}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка записи serial: {e}')
            self._reconnect_chassis()

    def _read_from_serial(self):
        """Чтение данных из serial (без lock, вызывается только из serial_loop)."""
        try:
            # Читаем все доступные данные
            if self.arduino_serial.in_waiting > 0:
                data = self.arduino_serial.read(self.arduino_serial.in_waiting)
                self.serial_buffer += data.decode('utf-8', errors='ignore')

                # Обрабатываем полные строки
                while '\n' in self.serial_buffer:
                    line, self.serial_buffer = self.serial_buffer.split('\n', 1)
                    line = line.strip()
                    if line.startswith('ENC:'):
                        self._publish_encoder_data(line)

        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка чтения serial: {e}')
            self._reconnect_chassis()

    def _publish_encoder_data(self, line: str):
        """Парсинг и публикация данных энкодеров."""
        try:
            parts = line[4:].split(':')  # Skip 'ENC:'
            if len(parts) == 3:
                msg = Int64MultiArray()
                msg.data = [int(parts[0]), int(parts[1]), int(parts[2])]
                self.encoder_publisher.publish(msg)
        except ValueError as e:
            self.get_logger().debug(f'Ошибка парсинга энкодеров: {e}')

    def _find_arduino_port(self) -> List[str]:
        """Поиск порта Arduino Chassis."""
        import os

        if os.path.exists(self.SYMLINK_PATH):
            self.get_logger().info(f'Найден симлинк {self.SYMLINK_PATH}')
            return [self.SYMLINK_PATH]

        self.get_logger().info('Симлинк не найден, поиск устройства по devpath=1.1')
        available_ports = []
        ports = serial.tools.list_ports.comports()

        for port in ports:
            self.get_logger().info(f'Проверяю порт: {port.device}')
            try:
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device],
                    capture_output=True, text=True
                )
                if "devpath==\"1.1\"" in result.stdout or "ATTRS{devpath}==\"1.1\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.1 на порту {port.device}')
                    available_ports.append(port.device)
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')

        if not available_ports:
            self.get_logger().warn(f'Настройте udev правила для {self.SYMLINK_PATH}')

        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino."""
        available_ports = self._find_arduino_port()

        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.1 для Chassis')
            return False

        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Chassis')

        try:
            self.arduino_serial = serial.Serial(
                port,
                self.BAUD_RATE,
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port

            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Chassis на порту {port}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Chassis: {e}')
            return False

    def _is_chassis_connected(self) -> bool:
        """Проверка подключения Chassis."""
        return self.arduino_serial and self.arduino_serial.is_open

    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands."""
        command = msg.data
        self.get_logger().info(f'Голосовая команда: "{command}"')
        self.command_queue.append(command)

    def cmd_vel_callback(self, msg: Twist):
        """Обработчик команд скорости из топика /cmd_vel."""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        current_time = time.time()

        command = self._twist_to_command(linear_vel, angular_vel)

        # STOP команды отправляем всегда (безопасность)
        is_stop = (command == "CHASSIS:STOP")

        if not is_stop:
            # Throttling: ограничиваем частоту команд
            if current_time - self.last_cmd_time < self.MIN_CMD_INTERVAL:
                return

            # Дедупликация: игнорируем мелкие изменения скорости
            linear_change = abs(linear_vel - self.last_linear_vel)
            angular_change = abs(angular_vel - self.last_angular_vel)

            if (linear_change < self.LINEAR_CHANGE_THRESHOLD and
                angular_change < self.ANGULAR_CHANGE_THRESHOLD):
                return

        self.command_queue.append(command)
        self.last_sent_command = command
        self.last_cmd_time = current_time
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel

    def _twist_to_command(self, linear_vel: float, angular_vel: float) -> str:
        """Конвертирует Twist в команду для Arduino."""
        if abs(linear_vel) < self.LINEAR_THRESHOLD and abs(angular_vel) < self.ANGULAR_THRESHOLD:
            return "CHASSIS:STOP"
        return f"CHASSIS:VEL:{linear_vel:.3f}:{angular_vel:.3f}"

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
