import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String, Bool
import serial
import serial.tools.list_ports
import time
import glob
from typing import Optional, List, Tuple

class ArduinoMovingNavigationNode(Node):
    def __init__(self):
        super().__init__('arduino_moving_navigation_node')
        
        self.baud_rate = 9600
        self.arduino_serial = None
        self.serial_buffer = ""
        self.arduino_port = None
        self.device_id = None
        
        # Известные идентификаторы Arduino
        self.KNOWN_DEVICES = {
            "ARDUINO_CHASSIS": "Шасси и навигация"
        }
        
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_serial_connection()
        self._setup_timer()
        
        if self.device_id:
            self.get_logger().info(f'Узел arduino_moving_navigation инициализирован и готов к работе. '
                                   f'Устройство: {self.KNOWN_DEVICES.get(self.device_id, "Неизвестное")}')
        else:
            self.get_logger().info('Узел arduino_moving_navigation инициализирован, но устройство не идентифицировано.')
    
    def _setup_publishers(self):
        """Инициализация издателей."""
        self.distance_publisher = self.create_publisher(Float32, '/distance_h04', 10)
        self.get_logger().info('Издатель для /distance_h04 создан.')
        
        self.status_publisher = self.create_publisher(String, '/arduino_status', 10)
        self.get_logger().info('Издатель для /arduino_status создан.')
    
    def _setup_subscribers(self):
        """Инициализация подписчиков."""
        self.command_subscriber = self.create_subscription(
            String, '/verter_commands', self.command_callback, 10
        )
        self.get_logger().info('Подписчик для /verter_commands создан.')
    
    def _find_arduino_ports(self) -> List[str]:
        """Поиск доступных портов Arduino по devpath."""
        available_ports = []
        
        # Ищем устройство с конкретным devpath для шасси
        self.get_logger().info('Поиск устройства с devpath=1.4 для контроллера шасси')
        ports = serial.tools.list_ports.comports()
        
        for port in ports:
            self.get_logger().info(f'Проверяю порт: {port.device}')
            
            # Проверяем devpath с помощью udevadm
            try:
                import subprocess
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device], 
                    capture_output=True, 
                    text=True
                )
                
                if "devpath==\"1.4\"" in result.stdout or "ATTRS{devpath}==\"1.4\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.4 на порту {port.device}')
                    available_ports.append(port.device)
                else:
                    self.get_logger().info(f'Порт {port.device} не соответствует devpath=1.4')
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')
        
        if not available_ports:
            self.get_logger().warn('Не найдено устройств с devpath=1.4')
        
        return available_ports
    
    def _setup_serial_connection(self):
        """Установка соединения с Arduino по devpath."""
        self.get_logger().info('Начинаю процесс поиска и подключения к Arduino')
        available_ports = self._find_arduino_ports()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.4')
            return
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для шасси (devpath=1.4)')
        
        try:
            arduino_serial = serial.Serial(port, self.baud_rate, timeout=0.1)
            time.sleep(1.0)  # Даем время на инициализацию
            
            self.arduino_serial = arduino_serial
            self.arduino_port = port
            self.device_id = "ARDUINO_CHASSIS"  # Устанавливаем идентификатор напрямую
            self.get_logger().info(f'Arduino "{self.KNOWN_DEVICES[self.device_id]}" успешно подключен на порту {port} (devpath=1.4)')
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f'Не удалось подключиться к {port}: {e}')
    
    def _setup_timer(self):
        """Создание таймера для чтения данных."""
        self.serial_read_timer = self.create_timer(0.1, self.serial_read_callback)
    
    def serial_read_callback(self):
        """Чтение данных с Arduino и публикация в соответствующие топики."""
        if not self._is_serial_ready():
            return
        
        try:
            raw_data = self._read_serial_data()
            if raw_data:
                self._process_serial_data(raw_data)
        except serial.SerialException as e:
            self._handle_serial_error(e)
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка при чтении с Arduino: {e}')
    
    def _is_serial_ready(self):
        """Проверка готовности serial соединения."""
        return self.arduino_serial and self.arduino_serial.is_open
    
    def _read_serial_data(self):
        """Чтение данных из serial порта."""
        if self.arduino_serial.in_waiting > 0:
            return self.arduino_serial.read(self.arduino_serial.in_waiting).decode('utf-8', errors='replace')
        return None
    
    def _process_serial_data(self, raw_data):
        """Обработка полученных данных."""
        self.serial_buffer += raw_data
        
        while '\n' in self.serial_buffer:
            line, self.serial_buffer = self.serial_buffer.split('\n', 1)
            line = line.strip()
            
            if line:
                self._process_line(line)
    
    def _process_line(self, line):
        """Обработка одной строки данных от Arduino."""
        parts = line.split(':', 1)
        if len(parts) < 2:
            self.get_logger().warn(f'Некорректный формат сообщения от Arduino: "{line}"')
            return
        
        prefix, data = parts[0], parts[1]
        
        handlers = {
            "STATUS": self._handle_status_data,
            "ERROR": self._handle_error_data,
            "ACK": self._handle_ack_data
        }
        
        handler = handlers.get(prefix)
        if handler:
            handler(data)
        else:
            self.get_logger().debug(f'Неизвестный тип сообщения от Arduino: {prefix}, данные: {data}')
    
    def _handle_status_data(self, data):
        """Обработка статусов от Arduino."""
        status_parts = data.split(':', 1)
        status_msg = String()
        
        device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
        
        if len(status_parts) == 2:
            code, description = status_parts
            status_msg.data = f"[{device_name}][{code}] {description}"
            self.get_logger().info(f'Статус Arduino [{device_name}]: [{code}] {description}')
        else:
            status_msg.data = f"[{device_name}] {data}"
            self.get_logger().info(f'Статус Arduino [{device_name}]: {data}')
        
        self.status_publisher.publish(status_msg)
    
    def _handle_error_data(self, data):
        """Обработка сообщений об ошибках от Arduino."""
        error_parts = data.split(':', 1)
        device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
        
        if len(error_parts) == 2:
            code, description = error_parts
            self.get_logger().error(f'Ошибка Arduino [{device_name}]: [{code}] {description}')
    
    def _handle_ack_data(self, data):
        """Обработка подтверждений выполнения команд."""
        device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
        self.get_logger().info(f'Подтверждение от Arduino [{device_name}]: {data}')
    
    def _handle_serial_error(self, error):
        """Обработка ошибок serial соединения."""
        device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
        self.get_logger().error(f'Ошибка последовательного порта при чтении [{device_name}]: {error}')
        
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
        
        # Попытка переподключения
        self.get_logger().info('Попытка переподключения к Arduino...')
        self._setup_serial_connection()
        
        if not self.arduino_serial:
            if self.serial_read_timer:
                self.serial_read_timer.cancel()
    
    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands и отправка на Arduino."""
        command = msg.data
        self.get_logger().info(f'Получена команда для Arduino: "{command}"')
        
        if not self._validate_command(command):
            return
        
        self._send_command_to_arduino(command)
    
    def _validate_command(self, command):
        """Валидация команды."""
        parts = command.split(':')
        
        if len(parts) < 2:
            self.get_logger().warn(f'Неверный формат команды: "{command}"')
            return False
        
        component = parts[0]
        
        # Проверяем, что команда предназначена для шасси или глаз
        if component != 'CHASSIS' and component != 'EYES':
            self.get_logger().info(f'Команда "{command}" не предназначена для шасси или глаз и будет проигнорирована этим узлом.')
            return False
        
        return True
    
    def _send_command_to_arduino(self, command):
        """Отправка команды на Arduino."""
        if not self._is_serial_ready():
            self.get_logger().warn('Arduino не подключен или порт не открыт. Команда не отправлена.')
            return
        
        try:
            self.arduino_serial.write(f"{command}\n".encode('utf-8'))
            device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
            self.get_logger().info(f'Команда отправлена на Arduino [{device_name}]: "{command}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка при отправке команды на Arduino: {e}')
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка при отправке команды: {e}')
    
    def destroy_node(self):
        """Корректное завершение работы узла."""
        device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
        self.get_logger().info(f'Завершение работы узла arduino_moving_navigation [{device_name}].')
        
        if hasattr(self, 'serial_read_timer') and self.serial_read_timer:
            self.serial_read_timer.cancel()
        
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
            self.get_logger().info('Последовательный порт Arduino закрыт.')
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    arduino_moving_navigation_node = ArduinoMovingNavigationNode()
    
    try:
        rclpy.spin(arduino_moving_navigation_node)
    except KeyboardInterrupt:
        arduino_moving_navigation_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        arduino_moving_navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 