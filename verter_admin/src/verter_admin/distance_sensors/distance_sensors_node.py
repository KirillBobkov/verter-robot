import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import serial.tools.list_ports
import time
import subprocess
import threading
from typing import Optional, List


class DistanceSensorsNode(Node):
    # Константы конфигурации
    BAUD_RATE = 9600
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.1
    CRITICAL_DISTANCE = 50.0  # 30 см
    NUM_SENSORS = 7
    
    def __init__(self):
        super().__init__('distance_sensors_node')
        
        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None
        self.sensor_data = [0.0] * self.NUM_SENSORS
        self.reading_thread = None
        self.stop_thread = False
        self.last_command = None  # Запоминаем последнюю отправленную команду
        
        self._setup_publisher()
        
        if self._connect_to_arduino():
            self.get_logger().info('Distance Sensors нода инициализирована и готова к работе')
            self._start_sensor_reading()
        else:
            self.get_logger().warn('Arduino Mega не подключен при инициализации')

    def _setup_publisher(self):
        """Настройка издателя для команд."""
        self.command_publisher = self.create_publisher(
            String, 'verter_commands', 10
        )
        self.get_logger().info('Издатель для verter_commands создан')

    def _find_arduino_mega_port(self) -> List[str]:
        """Автоматический поиск портов Arduino Mega по devpath."""
        available_ports = []
        
        # Ищем устройство с конкретным devpath для Arduino Mega
        self.get_logger().info('Поиск устройства с devpath=1.4 для Arduino Mega')
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

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino Mega."""
        available_ports = self._find_arduino_mega_port()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.4 для Arduino Mega')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Arduino Mega (devpath=1.4)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port
            
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Arduino Mega на порту {port} (devpath=1.4)')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Arduino Mega на порту {port}: {e}')
            return False

    def _is_arduino_connected(self) -> bool:
        """Проверка подключения Arduino Mega."""
        return self.arduino_serial and self.arduino_serial.is_open

    def _start_sensor_reading(self):
        """Запуск потока для чтения данных с датчиков."""
        self.stop_thread = False
        self.reading_thread = threading.Thread(target=self._sensor_reading_loop)
        self.reading_thread.daemon = True
        self.reading_thread.start()
        self.get_logger().info('Поток чтения датчиков запущен')

    def _sensor_reading_loop(self):
        """Основной цикл чтения данных с датчиков."""
        while not self.stop_thread:
            try:
                if not self._is_arduino_connected():
                    self.get_logger().warn('Arduino Mega не подключен, попытка переподключения...')
                    self._reconnect_arduino()
                    time.sleep(1.0)
                    continue
                
                # Читаем данные с serial порта
                if self.arduino_serial.in_waiting > 0:
                    line = self.arduino_serial.readline().decode('utf-8').strip()
                    self._process_sensor_data(line)
                
                time.sleep(0.01)  # Короткая пауза чтобы не нагружать CPU
                
            except Exception as e:
                self.get_logger().error(f'Ошибка в цикле чтения датчиков: {e}')
                time.sleep(1.0)

    def _process_sensor_data(self, data_line: str):
        """Обработка данных с датчиков."""
        try:
            # Ожидаем формат: "DISTANCE:10:23:24:23:23:22:22"
            if not data_line.startswith('DISTANCE:'):
                return
            
            # Парсим данные датчиков
            parts = data_line.split(':')
            if len(parts) != 8:  # DISTANCE + 7 датчиков
                self.get_logger().warn(f'Неверный формат данных: {len(parts)-1} датчиков, ожидается 7')
                return
            
            critical_distance_detected = False
            
            for i in range(1, 8):  # Пропускаем "DISTANCE"
                try:
                    distance = float(parts[i])
                    
                    # 999 означает ошибку или вне диапазона
                    if distance == 999:
                        self.sensor_data[i-1] = 999.0
                        continue
                    
                    self.sensor_data[i-1] = distance
                    
                    # Проверяем критическое расстояние только для датчиков 2-6 (индексы 1-5)
                    # Датчики 1 и 7 - боковые, их не учитываем
                    if 1 <= i <= 5 and distance < self.CRITICAL_DISTANCE:
                        critical_distance_detected = True
                        
                except ValueError:
                    self.get_logger().error(f'Ошибка парсинга расстояния для датчика {i}: {parts[i]}')
            
            # Определяем команду на основе состояния датчиков
            if critical_distance_detected:
                # Логируем СТОП с данными датчиков
                sensor_values = ', '.join([f'S{i+1}:{self.sensor_data[i]:.0f}' for i in range(self.NUM_SENSORS)])
                self.get_logger().warn(f'СТОП: {sensor_values}')
                self._send_command("CHASSIS:STOP")
            else:
                # Препятствий нет - ничего не делаем
                pass
         
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки данных датчиков: {e}')

    def _send_command(self, command: str):
        """Отправка команды."""
        try:
            msg = String()
            msg.data = command
            self.command_publisher.publish(msg)
            self.last_command = command
            self.get_logger().info(f'Отправлена команда: {command}')
        except Exception as e:
            self.get_logger().error(f'Ошибка отправки команды {command}: {e}')

    def _reconnect_arduino(self):
        """Переподключение к Arduino Mega."""
        self.get_logger().info('Попытка переподключения к Arduino Mega...')
        self._close_arduino_connection()
        
        if self._connect_to_arduino():
            self.get_logger().info('Успешно переподключен к Arduino Mega')
        else:
            self.get_logger().warn('Не удалось переподключиться к Arduino Mega')

    def _close_arduino_connection(self):
        """Закрытие соединения с Arduino Mega."""
        if self.arduino_serial and self.arduino_serial.is_open:
            self.arduino_serial.close()

    def destroy_node(self):
        """Корректное завершение работы узла."""
        self.get_logger().info('Завершение работы Distance Sensors ноды')
        self.stop_thread = True
        
        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=2.0)
        
        self._close_arduino_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    distance_sensors_node = DistanceSensorsNode()
    
    try:
        rclpy.spin(distance_sensors_node)
    except KeyboardInterrupt:
        distance_sensors_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        distance_sensors_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
