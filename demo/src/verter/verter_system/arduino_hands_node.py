import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
import time
import glob
from typing import Optional, Tuple, Dict, List

class ArduinoHandsNode(Node):
    # Константы конфигурации
    BAUD_RATE = 9600
    CONNECTION_TIMEOUT = 2.0
    READ_TIMER_PERIOD = 0.1
    SERIAL_TIMEOUT = 0.1
    
    # Известные идентификаторы Arduino
    KNOWN_DEVICES = {
        "ARDUINO_HANDS": "Руки"
    }
    
    def __init__(self):
        super().__init__('arduino_hands_node')
        
        self.arduino_serial: Optional[serial.Serial] = None
        self.serial_read_timer = None
        self.current_port = None
        self.device_id = None
        
        self._setup_publishers_and_subscribers()
        
        if self._connect_to_arduino():
            self._start_serial_reading()
            self.get_logger().info(f'Узел arduino_hands инициализирован и готов к работе. Устройство: {self.KNOWN_DEVICES.get(self.device_id, "Неизвестное")}')

    def _setup_publishers_and_subscribers(self):
        """Настройка подписчиков и издателей."""
        self.command_subscriber = self.create_subscription(
            String, '/verter_commands', self.command_callback, 10
        )
        self.get_logger().info('Подписчик для /verter_commands создан.')
        
        self.log_publisher = self.create_publisher(String, '/arduino_logs', 10)
        self.get_logger().info('Издатель для /arduino_logs создан.')
        
        self.command_publisher = self.create_publisher(String, '/verter_commands', 10)
        self.get_logger().info('Издатель для /verter_commands создан.')

    def _find_arduino_port(self) -> List[str]:
        """Автоматический поиск портов Arduino по devpath."""
        available_ports = []
        
        # Ищем устройство с конкретным devpath для рук
        self.get_logger().info('Поиск устройства с devpath=1.1 для контроллера рук')
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
                
                if "devpath==\"1.1\"" in result.stdout or "ATTRS{devpath}==\"1.1\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.1 на порту {port.device}')
                    available_ports.append(port.device)
                else:
                    self.get_logger().info(f'Порт {port.device} не соответствует devpath=1.1')
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')
        
        if not available_ports:
            self.get_logger().warn('Не найдено устройств с devpath=1.1')
        
        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino для рук."""
        available_ports = self._find_arduino_port()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.1 для рук')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для управления руками (devpath=1.1)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port
            
            # Проверяем, что это действительно устройство для рук
            if not self._verify_hands_device():
                self.get_logger().error(f'Устройство на порту {port} не соответствует критериям для контроллера рук')
                self.arduino_serial.close()
                return False
            
            self.device_id = "ARDUINO_HANDS"  # Устанавливаем идентификатор напрямую
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(
                f'Успешно подключено к Arduino "{self.KNOWN_DEVICES[self.device_id]}" на порту {port} (devpath=1.1)'
            )
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Arduino на порту {port}: {e}')
            return False
            
    def _verify_hands_device(self) -> bool:
        """Проверяет, что подключенное устройство действительно контроллер рук."""
        if not self.arduino_serial or not self.arduino_serial.is_open:
            return False
            
        # Проверяем devpath устройства
        try:
            import subprocess
            result = subprocess.run(
                ['udevadm', 'info', '-a', '-n', self.current_port], 
                capture_output=True, 
                text=True
            )
            
            if "devpath==\"1.1\"" not in result.stdout and "ATTRS{devpath}==\"1.1\"" not in result.stdout:
                self.get_logger().error(f'Устройство на порту {self.current_port} не имеет devpath=1.1')
                return False
                
            self.get_logger().info(f'Устройство на порту {self.current_port} подтверждено по devpath=1.1')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Ошибка при проверке devpath: {e}')
            return False

    def _start_serial_reading(self):
        """Запуск чтения данных с Arduino."""
        self.serial_read_timer = self.create_timer(
            self.READ_TIMER_PERIOD, 
            self._read_from_arduino
        )

    def _read_from_arduino(self):
        """Чтение данных с Arduino и публикация в топик."""
        if not self._is_arduino_connected():
            return

        try:
            data = self._get_arduino_data()
            if data:
                self._publish_arduino_log(data)
                self._process_arduino_response(data)
        except serial.SerialException as e:
            self._handle_serial_error(e)
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка при чтении: {e}')

    def _is_arduino_connected(self) -> bool:
        """Проверка подключения Arduino."""
        return self.arduino_serial and self.arduino_serial.is_open

    def _get_arduino_data(self) -> str:
        """Получение данных с Arduino."""
        if self.arduino_serial.in_waiting > 0:
            return self.arduino_serial.read(
                self.arduino_serial.in_waiting
            ).decode('utf-8', errors='replace')
        return ""

    def _publish_arduino_log(self, data: str):
        """Публикация лога от Arduino."""
        device_name = self.KNOWN_DEVICES.get(self.device_id, "Неизвестное устройство")
        self.get_logger().info(f'Arduino [{device_name}]: {data}')
        log_msg = String()
        log_msg.data = f"[{device_name}] {data}"
        self.log_publisher.publish(log_msg)

    def _handle_serial_error(self, error: serial.SerialException):
        """Обработка ошибок последовательного порта."""
        self.get_logger().error(f'Ошибка последовательного порта: {error}')
        self._close_arduino_connection()
        
        # Попытка переподключения с поиском нового порта
        self.get_logger().info('Попытка переподключения к Arduino...')
        if self._connect_to_arduino():
            self._start_serial_reading()
            self.get_logger().info(f'Успешно переподключен к Arduino "{self.KNOWN_DEVICES[self.device_id]}"')
        else:
            self.get_logger().warn('Не удалось переподключиться к Arduino')

    def _close_arduino_connection(self):
        """Закрытие соединения с Arduino."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()
        if self.serial_read_timer:
            self.serial_read_timer.cancel()
            self.serial_read_timer = None

    def command_callback(self, msg):
        """Обработчик команд из топика /verter_commands."""
        command = msg.data
        self.get_logger().info(f'Получена команда: "{command}"')

        if not self._validate_command(command):
            return
            
        self._send_to_arduino(command)


    def _validate_command(self, command: str) -> bool:
        """Валидация команды для рук."""
        parts = command.split(':')
        
        if len(parts) < 2:
            self.get_logger().warn(f'Неверный формат команды: "{command}"')
            return False
            
        component = parts[0]
        
        # Проверяем что команда предназначена для рук
        if not component.startswith('ARM_'):
            self.get_logger().info(f'Команда "{command}" не предназначена для рук и будет проигнорирована этим узлом.')
            return False
        
        return True

    def _send_to_arduino(self, command: str):
        """Отправка команды на Arduino."""
        if not self._is_arduino_connected():
            self.get_logger().warn('Arduino не подключен.')
            return
            
        try:
            self.arduino_serial.write(f"{command}\n".encode('utf-8'))
            self.get_logger().info(f'Команда отправлена: "{command}"')
        except serial.SerialException as e:
            self.get_logger().error(f'Ошибка отправки: {e}')
        except Exception as e:
            self.get_logger().error(f'Неожиданная ошибка отправки: {e}')

    def destroy_node(self):
        """Корректное завершение работы узла."""
        self.get_logger().info('Завершение работы узла arduino_hands.')
        self._close_arduino_connection()
        super().destroy_node()

    def _process_arduino_response(self, data: str):
        """Обрабатывает ответы от Arduino и перенаправляет команды в соответствующие топики."""
        lines = data.strip().split('\n')
        for line in lines:
            line = line.strip()
            if line == "EYES:CENTER":
                self.get_logger().info('Получена команда EYES:CENTER от Arduino, перенаправляю в /verter_commands')
                cmd_msg = String()
                cmd_msg.data = line
                self.command_publisher.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    arduino_hands_node = ArduinoHandsNode()
    
    try:
        rclpy.spin(arduino_hands_node)
    except KeyboardInterrupt:
        arduino_hands_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        arduino_hands_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 