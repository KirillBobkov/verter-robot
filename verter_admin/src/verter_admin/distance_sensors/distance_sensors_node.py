import math
import subprocess
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import String

import serial
import serial.tools.list_ports


class DistanceSensorsNode(Node):
    # Константы конфигурации
    BAUD_RATE = 9600
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.1
    MIN_RANGE = 0.02  # 2 см
    MAX_RANGE = 4.5   # 4.5 м (с запасом от ~4 м HC-SR04)
    FIELD_OF_VIEW = math.radians(30.0)  # ~30°
    INVALID_DISTANCE_VALUE = 999
    SENSOR_ORDER = [
        'front_center',
        'front_left_inner',
        'front_left_outer',
        'front_right_inner',
        'front_right_outer',
        'left',
        'right',
    ]
    SENSOR_FRAMES = {
        'front_center': 'sensor_front_center',
        'front_left_inner': 'sensor_front_left_inner',
        'front_left_outer': 'sensor_front_left_outer',
        'front_right_inner': 'sensor_front_right_inner',
        'front_right_outer': 'sensor_front_right_outer',
        'left': 'sensor_left',
        'right': 'sensor_right',
    }
    IMU_FRAME_ID = 'imu_link'
    IMU_ORIENTATION_COVARIANCE = [
        0.05, 0.0, 0.0,
        0.0, 0.05, 0.0,
        0.0, 0.0, 0.2,
    ]
    IMU_ORIENTATION_UNKNOWN = [
        -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ]
    IMU_ANGULAR_VELOCITY_COVARIANCE = [
        0.02, 0.0, 0.0,
        0.0, 0.02, 0.0,
        0.0, 0.0, 0.05,
    ]
    IMU_LINEAR_ACCELERATION_COVARIANCE = [
        -1.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
    ]
    
    def __init__(self):
        super().__init__('distance_sensors_node')
        
        self.arduino_serial: Optional[serial.Serial] = None
        self.reading_thread = None
        self.stop_thread = False
        self._last_valid_yaw: Optional[float] = None
        
        self._setup_publisher()
        self.imu_publisher = self.create_publisher(Imu, '/verter/imu/data', 10)

        if self._connect_to_arduino():
            self.get_logger().info('Distance Sensors нода инициализирована и готова к работе')
            self._start_sensor_reading()
        else:
            self.get_logger().warn('Arduino Mega не подключен при инициализации')

    def _setup_publisher(self):
        """Настройка издателей для команд и Range сообщений."""
        # Publisher для команд
        self.command_publisher = self.create_publisher(
            String, 'verter_commands', 10
        )
        self.get_logger().info('Издатель для verter_commands создан')

        # Publishers для Range сообщений (7 HC-SR04 датчиков)
        self.range_publishers = {
            'front_center': self.create_publisher(Range, '/verter/distance_sensors/front_center', 10),
            'front_left_inner': self.create_publisher(Range, '/verter/distance_sensors/front_left_inner', 10),
            'front_left_outer': self.create_publisher(Range, '/verter/distance_sensors/front_left_outer', 10),
            'front_right_inner': self.create_publisher(Range, '/verter/distance_sensors/front_right_inner', 10),
            'front_right_outer': self.create_publisher(Range, '/verter/distance_sensors/front_right_outer', 10),
            'left': self.create_publisher(Range, '/verter/distance_sensors/left', 10),
            'right': self.create_publisher(Range, '/verter/distance_sensors/right', 10),
        }
        self.get_logger().info('Range publishers созданы для 7 ультразвуковых датчиков')

    # Симлинк для Arduino Sensors (настраивается через udev)
    SYMLINK_PATH = '/dev/arduino_sensors'

    def _find_arduino_mega_port(self) -> List[str]:
        """Поиск порта Arduino Mega (датчики)."""
        import os

        # Сначала проверяем симлинк (рекомендуемый способ)
        if os.path.exists(self.SYMLINK_PATH):
            self.get_logger().info(f'Найден симлинк {self.SYMLINK_PATH}')
            return [self.SYMLINK_PATH]

        # Fallback: поиск по devpath (для обратной совместимости)
        self.get_logger().info('Симлинк не найден, поиск устройства по devpath=1.2')
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

                if "devpath==\"1.2\"" in result.stdout or "ATTRS{devpath}==\"1.2\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.2 на порту {port.device}')
                    available_ports.append(port.device)
            except Exception as e:
                self.get_logger().error(f'Ошибка при проверке devpath для {port.device}: {e}')

        if not available_ports:
            self.get_logger().warn(f'Настройте udev правила для {self.SYMLINK_PATH} (см. документацию)')

        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino Mega."""
        available_ports = self._find_arduino_mega_port()
        
        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.2 для Arduino Mega')
            return False
        
        # Берем первый найденный порт
        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Arduino Mega (devpath=1.2)')
        
        try:
            self.arduino_serial = serial.Serial(
                port, 
                self.BAUD_RATE, 
                timeout=self.SERIAL_TIMEOUT
            )
            
            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Arduino Mega на порту {port} (devpath=1.2)')
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
                    raw_line = self.arduino_serial.readline().decode('utf-8', errors='ignore').strip()
                    packet = self._parse_sensor_data(raw_line)
                    if packet:
                        self._publish_range_messages(packet['distances'])
                        self._publish_imu(packet)
                        self.get_logger().debug(' | '.join(packet['log_parts']))
                
                time.sleep(1)  # Короткая пауза чтобы не нагружать CPU
                
            except Exception as e:
                self.get_logger().error(f'Ошибка в цикле чтения датчиков: {e}')
                time.sleep(1.0)

    def _parse_sensor_data(self, data_line: str) -> Optional[Dict[str, object]]:
        """Парсер строки с Arduino, возвращает структуру данных или None."""
        line = (data_line or '').strip()
        if not line:
            return None

        # Одноразовый инфо-лог от Arduino
        if line.startswith('COMPASS:'):
            return None

        if not line.startswith('DISTANCE:'):
            return None

        parts = line.split(';')
        if len(parts) != 4:
            return None

        distances = self._parse_distances(parts[0])
        if distances is None:
            return None

        angles = self._parse_angles(parts[1])
        if angles is None:
            return None

        gyro = self._parse_gyro(parts[2])
        if gyro is None:
            return None

        yaw = self._parse_compass(parts[3])

        return {
            'distances': distances,
            'roll': math.radians(angles[0]),
            'pitch': math.radians(angles[1]),
            'gyro': tuple(math.radians(g) for g in gyro),
            'yaw': yaw,
            'log_parts': [
                parts[0],
                parts[1],
                parts[2],
                parts[3],
            ],
        }

    def _parse_distances(self, distance_segment: str) -> Optional[Dict[str, Optional[float]]]:
        if not distance_segment.startswith('DISTANCE:'):
            return None

        distance_values = distance_segment[len('DISTANCE:'):].split(':')
        if len(distance_values) != len(self.SENSOR_ORDER):
            return None

        parsed: Dict[str, Optional[float]] = {}
        for sensor_name, raw_value in zip(self.SENSOR_ORDER, distance_values):
            raw_value = raw_value.strip()
            try:
                centimeters = int(raw_value)
            except ValueError:
                return None

            if centimeters <= 0 or centimeters >= self.INVALID_DISTANCE_VALUE:
                parsed[sensor_name] = None
                continue

            meters = centimeters / 100.0
            parsed[sensor_name] = meters

        return parsed

    @staticmethod
    def _parse_angles(angle_segment: str) -> Optional[tuple[int, int]]:
        if not angle_segment.startswith('ANGLES:'):
            return None

        fields = angle_segment[len('ANGLES:'):].split(':')
        if len(fields) != 2:
            return None

        try:
            return int(fields[0]), int(fields[1])
        except ValueError:
            return None

    @staticmethod
    def _parse_gyro(gyro_segment: str) -> Optional[tuple[int, int, int]]:
        if not gyro_segment.startswith('GYRO:'):
            return None

        fields = gyro_segment[len('GYRO:'):].split(':')
        if len(fields) != 3:
            return None

        try:
            return int(fields[0]), int(fields[1]), int(fields[2])
        except ValueError:
            return None

    def _parse_compass(self, compass_segment: str) -> Optional[float]:
        if not compass_segment.startswith('COMPAS:'):
            return None

        value = compass_segment[len('COMPAS:'):].strip()
        try:
            heading_deg = int(value)
        except ValueError:
            return None

        if heading_deg >= self.INVALID_DISTANCE_VALUE or heading_deg < 0:
            return None

        yaw_rad = math.radians(heading_deg)
        # Нормализуем в диапазон [-pi, pi]
        return (yaw_rad + math.pi) % (2 * math.pi) - math.pi

    def _publish_range_messages(self, distances: Dict[str, Optional[float]]):
        now = self.get_clock().now().to_msg()
        for sensor_name, publisher in self.range_publishers.items():
            if publisher is None:
                continue

            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = self.SENSOR_FRAMES.get(sensor_name, 'base_link')
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.FIELD_OF_VIEW
            msg.min_range = self.MIN_RANGE
            msg.max_range = self.MAX_RANGE

            value = distances.get(sensor_name)
            if value is None or value < self.MIN_RANGE or value > self.MAX_RANGE:
                msg.range = float('inf')
            else:
                msg.range = value

            publisher.publish(msg)

    def _publish_imu(self, packet: Dict[str, object]):
        if self.imu_publisher is None:
            return

        now = self.get_clock().now().to_msg()
        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.IMU_FRAME_ID

        yaw = packet['yaw']
        if yaw is not None:
            self._last_valid_yaw = yaw
        else:
            yaw = self._last_valid_yaw

        if yaw is not None:
            qx, qy, qz, qw = self._euler_to_quaternion(packet['roll'], packet['pitch'], yaw)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw
            imu_msg.orientation_covariance = list(self.IMU_ORIENTATION_COVARIANCE)
        else:
            imu_msg.orientation_covariance = list(self.IMU_ORIENTATION_UNKNOWN)

        gx, gy, gz = packet['gyro']
        imu_msg.angular_velocity.x = gx
        imu_msg.angular_velocity.y = gy
        imu_msg.angular_velocity.z = gz
        imu_msg.angular_velocity_covariance = list(self.IMU_ANGULAR_VELOCITY_COVARIANCE)

        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        imu_msg.linear_acceleration_covariance = list(self.IMU_LINEAR_ACCELERATION_COVARIANCE)

        self.imu_publisher.publish(imu_msg)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        """Преобразование углов Эйлера в кватернион."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * cp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

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
