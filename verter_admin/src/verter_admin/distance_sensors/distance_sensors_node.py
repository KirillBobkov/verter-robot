import rclpy
from rclpy.node import Node
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
    
    def __init__(self):
        super().__init__('distance_sensors_node')
        
        self.arduino_serial: Optional[serial.Serial] = None
        self.reading_thread = None
        self.stop_thread = False
        
        if self._connect_to_arduino():
            self.get_logger().info('Distance Sensors нода инициализирована и готова к работе')
            self._start_sensor_reading()
        else:
            self.get_logger().warn('Arduino Mega не подключен при инициализации')


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
                    outputs = self._process_sensor_data(line)
                    if outputs:
                        self.get_logger().info(' | '.join(outputs))
                
                time.sleep(1)  # Короткая пауза чтобы не нагружать CPU
                
            except Exception as e:
                self.get_logger().error(f'Ошибка в цикле чтения датчиков: {e}')
                time.sleep(1.0)

    def _process_sensor_data(self, data_line: str) -> list[str]:
        """Парсер строки: возвращает отдельные строки для логов или пустой список.
        Поддерживаемые форматы:
          - DISTANCE:...;ANGLES:angle_x:angle_y;GYRO:gx:gy:gz;COMPAS:deg → ["DISTANCE:...", "ANGLES:angle_x:angle_y", "GYRO:gx:gy:gz", "COMPAS:deg"]
          - COMPASS:... (инфо-лог) и прочее → []
        """
        line = (data_line or '').strip()
        if not line:
            return []

        # Одноразовый инфо-лог от Arduino
        if line.startswith('COMPASS:'):
            return []

        # Совмещённая строка DISTANCE;ANGLES;GYRO;COMPAS
        if not line.startswith('DISTANCE:'):
            return []

        # Разделяем по точкам с запятой
        parts = line.split(';')
        if len(parts) != 4:
            return []

        # DISTANCE часть
        if not parts[0].startswith('DISTANCE:'):
            return []
        distance_part = parts[0][len('DISTANCE:'):]
        if not distance_part or any(not seg.isdigit() for seg in distance_part.split(':')):
            return []

        # ANGLES часть (2 значения: angle_x:angle_y в градусах)
        # Yaw (рыскание) определяется компасом
        if not parts[1].startswith('ANGLES:'):
            return []
        angles_part = parts[1][len('ANGLES:'):]
        angles_fields = angles_part.split(':')
        if len(angles_fields) != 2:
            return []
        try:
            angle_x, angle_y = int(angles_fields[0]), int(angles_fields[1])
        except ValueError:
            return []

        # GYRO часть (3 значения: gx:gy:gz в градусах/сек)
        if not parts[2].startswith('GYRO:'):
            return []
        gyro_part = parts[2][len('GYRO:'):]
        gyro_fields = gyro_part.split(':')
        if len(gyro_fields) != 3:
            return []
        try:
            gx, gy, gz = int(gyro_fields[0]), int(gyro_fields[1]), int(gyro_fields[2])
        except ValueError:
            return []

        # COMPAS часть
        if not parts[3].startswith('COMPAS:'):
            return []
        compass_val = parts[3][len('COMPAS:'):].strip()
        if not compass_val.isdigit():
            return []

        return [
            f'DISTANCE:{distance_part}',
            f'ANGLES:{angle_x}:{angle_y}',
            f'GYRO:{gx}:{gy}:{gz}',
            f'COMPAS:{compass_val}',
        ]

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
