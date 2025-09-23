import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
import time
import subprocess
from typing import Optional, List


class ArduinoNode(Node):
    # Константы конфигурации
    BAUD_RATE = 9600
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.1
    
    def __init__(self):
        super().__init__('arduino_node')
        
        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None
        
        self._setup_subscribers()
        
        if self._connect_to_arduino():
            self.get_logger().info('Arduino нода инициализирована и готова к работе')
        else:
            self.get_logger().warn('Arduino не подключен при инициализации')

    def _setup_subscribers(self):
        """Настройка подписчиков."""
        self.command_subscriber = self.create_subscription(
            String, '/verter_commands', self.command_callback, 10
        )
        self.get_logger().info('Подписчик для /verter_commands создан')

    def _find_arduino_port(self) -> List[str]:
        """Автоматический поиск портов Arduino по devpath."""
        available_ports = []
        
        # Ищем устройство с конкретным devpath для рук
        self.get_logger().info('Поиск устройства с devpath=1.2 для Arduino')
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
            self.get_logger().error('Не найдено устройств с devpath=1.2 для Arduino')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Arduino (devpath=1.2)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port
            
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Arduino на порту {port} (devpath=1.2)')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Arduino на порту {port}: {e}')
            return False

    def _is_arduino_connected(self) -> bool:
        """Проверка подключения Arduino."""
        return self.arduino_serial and self.arduino_serial.is_open

    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands."""
        command = msg.data
        self.get_logger().info(f'Получена команда: "{command}"')
        self._send_to_arduino(command)

    def _send_to_arduino(self, command: str):
        """Отправка команды на Arduino."""
        if not self._is_arduino_connected():
            self.get_logger().warn('Arduino не подключен')
            return
            
        try:
            self.arduino_serial.write(f"{command}\n".encode('utf-8'))
            self.get_logger().info(f'Команда отправлена: "{command}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка отправки: {e}')
            # Попытка переподключения
            self._reconnect_arduino()
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка отправки: {e}')

    def _reconnect_arduino(self):
        """Переподключение к Arduino."""
        self.get_logger().info('Попытка переподключения к Arduino...')
        self._close_arduino_connection()
        
        if self._connect_to_arduino():
            self.get_logger().info('Успешно переподключен к Arduino')
        else:
            self.get_logger().warn('Не удалось переподключиться к Arduino')

    def _close_arduino_connection(self):
        """Закрытие соединения с Arduino."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()

    def destroy_node(self):
        """Корректное завершение работы узла."""
        self.get_logger().info('Завершение работы Arduino ноды')
        self._close_arduino_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    arduino_node = ArduinoNode()
    
    try:
        rclpy.spin(arduino_node)
    except KeyboardInterrupt:
        arduino_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        arduino_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
