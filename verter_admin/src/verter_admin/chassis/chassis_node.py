import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
        self.command_subscriber = self.create_subscription(
            String, 'verter_commands', self.command_callback, 10
        )
        self.get_logger().info('Подписчик для verter_commands создан')

    def _find_arduino_port(self) -> List[str]:
        """Автоматический поиск портов Arduino по devpath."""
        available_ports = []
        
        # Ищем устройство с конкретным devpath для рук
        self.get_logger().info('Поиск устройства с devpath=1.2 для Chassis')
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
                
                if "devpath==\"1.2\"" in result.stdout or "ATTRS{devpath}==\"1.2\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.2 на порту {port.device}')
                    available_ports.append(port.device)
                else:
                    self.get_logger().info(f'Порт {port.device} не соответствует devpath=1.2')
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')
        
        if not available_ports:
            self.get_logger().warn('Не найдено устройств с devpath=1.2')
        
        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino."""
        available_ports = self._find_arduino_port()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.2 для Chassis')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Chassis (devpath=1.2)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port
            
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Chassis на порту {port} (devpath=1.2)')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Chassis на порту {port}: {e}')
            return False

    def _is_chassis_connected(self) -> bool:
        """Проверка подключения Chassis."""
        return self.arduino_serial and self.arduino_serial.is_open

    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands."""
        command = msg.data
        self.get_logger().info(f'Получена команда: "{command}"')
        self._send_to_arduino(command)

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
