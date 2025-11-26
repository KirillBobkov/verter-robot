import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports
import time
import subprocess
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

        self._setup_subscribers()

        if self._connect_to_arduino():
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

    def _find_arduino_port(self) -> List[str]:
        """Автоматический поиск портов Arduino по devpath."""
        available_ports = []
        
        # Ищем устройство с конкретным devpath для рук
        self.get_logger().info('Поиск устройства с devpath=2.3 для Chassis')
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            self.get_logger().info(f'Проверяю порт: {port.device}')
            
            # Проверяем devpath с помощью udevadm
            try:
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device], 
                    capture_output=True, 
                    text=True
                )
                
                if "devpath==\"2.3\"" in result.stdout or "ATTRS{devpath}==\"2.3\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=2.3 на порту {port.device}')
                    available_ports.append(port.device)
                else:
                    self.get_logger().info(f'Порт {port.device} не соответствует devpath=2.3')
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')
        
        if not available_ports:
            self.get_logger().warn('Не найдено устройств с devpath=2.3')
        
        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino."""
        available_ports = self._find_arduino_port()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=2.3 для Chassis')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Chassis (devpath=2.3)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port
            
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Chassis на порту {port} (devpath=2.3)')
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
            Строка команды для Arduino (CHASSIS:FRONT, CHASSIS:STOP и т.д.)
        """
        # Определяем команду на основе скоростей
        if abs(linear_vel) < self.LINEAR_THRESHOLD and abs(angular_vel) < self.ANGULAR_THRESHOLD:
            # Робот не двигается
            return "CHASSIS:STOP"

        elif abs(angular_vel) > abs(linear_vel):
            # Преобладает поворот
            if angular_vel > 0:
                return f"CHASSIS:LEFT:{abs(angular_vel):.2f}"   # Поворот налево
            else:
                return f"CHASSIS:RIGHT:{abs(angular_vel):.2f}"  # Поворот направо

        elif linear_vel > 0:
            # Движение вперед
            return f"CHASSIS:FRONT:{linear_vel:.2f}"
        elif linear_vel < 0:
            # Движение назад
            return f"CHASSIS:BACK:{abs(linear_vel):.2f}"

        else:
            # На всякий случай - стоп
            return "CHASSIS:STOP"

    def _send_to_arduino(self, command: str):
        """Отправка команды на Arduino."""
        if not self._is_chassis_connected():
            self.get_logger().warn('Chassis не подключен')
            return
            
        try:
            self.arduino_serial.write(f"{command}\n".encode('utf-8'))
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
