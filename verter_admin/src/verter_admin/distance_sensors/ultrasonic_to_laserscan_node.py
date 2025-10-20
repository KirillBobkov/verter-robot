#!/usr/bin/env python3
"""
===================================================================================
ULTRASONIC TO LASERSCAN CONVERTER NODE
===================================================================================

Назначение:
-----------
Конвертирует данные с 7 ультразвуковых датчиков в формат LaserScan для Nav2.

Входные данные:
---------------
- Читает данные напрямую с Arduino (как distance_sensors_node)
- Формат: "DISTANCE:d1:d2:d3:d4:d5:d6:d7"

Выходные данные:
----------------
- /scan (sensor_msgs/LaserScan) - для Nav2 costmap
- /ultrasonic/ranges (sensor_msgs/LaserScan) - дополнительный топик для визуализации

Расположение датчиков (из URDF):
---------------------------------
Sensor 1 (левый боковой):     90° (1.57 rad)
Sensor 2 (левый передний):    45° (0.785 rad)
Sensor 3 (левый внутренний):  25° (0.436 rad)
Sensor 4 (центральный):        0° (0.0 rad)
Sensor 5 (правый внутренний): -25° (-0.436 rad)
Sensor 6 (правый передний):   -45° (-0.785 rad)
Sensor 7 (правый боковой):    -90° (-1.57 rad)

LaserScan параметры:
--------------------
- angle_min: -π/2 (-90°, правый боковой)
- angle_max: +π/2 (+90°, левый боковой)
- angle_increment: рассчитывается автоматически
- range_min: 0.02 м (2 см, минимум для HC-SR04)
- range_max: 4.0 м (400 см, максимум для HC-SR04)

===================================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import serial
import serial.tools.list_ports
import time
import subprocess
import threading
from typing import Optional, List
import math


class UltrasonicToLaserScanNode(Node):
    # Константы конфигурации
    BAUD_RATE = 9600
    CONNECTION_TIMEOUT = 2.0
    SERIAL_TIMEOUT = 0.1
    NUM_SENSORS = 7

    # Параметры LaserScan
    RANGE_MIN = 0.02  # 2 см минимум
    RANGE_MAX = 4.0   # 4 метра максимум

    # Углы датчиков в радианах (от центра робота, против часовой стрелки)
    # Порядок: [левый_боковой, левый_передний, левый_внутренний, центр,
    #           правый_внутренний, правый_передний, правый_боковой]
    SENSOR_ANGLES = [
        math.pi / 2,      # 90° - Sensor 1 (левый боковой)
        math.pi / 4,      # 45° - Sensor 2 (левый передний)
        math.radians(25), # 25° - Sensor 3 (левый внутренний)
        0.0,              #  0° - Sensor 4 (центральный)
        -math.radians(25),# -25° - Sensor 5 (правый внутренний)
        -math.pi / 4,     # -45° - Sensor 6 (правый передний)
        -math.pi / 2,     # -90° - Sensor 7 (правый боковой)
    ]

    def __init__(self):
        super().__init__('ultrasonic_to_laserscan_node')

        self.arduino_serial: Optional[serial.Serial] = None
        self.current_port = None
        self.sensor_data = [0.0] * self.NUM_SENSORS
        self.reading_thread = None
        self.stop_thread = False

        # Создаем издателя для LaserScan
        self._setup_publishers()

        # Подключаемся к Arduino
        if self._connect_to_arduino():
            self.get_logger().info('Ultrasonic to LaserScan нода инициализирована')
            self._start_sensor_reading()
        else:
            self.get_logger().warn('Arduino не подключен при инициализации')

    def _setup_publishers(self):
        """Настройка издателей."""
        # Основной топик для Nav2
        self.scan_publisher = self.create_publisher(
            LaserScan, '/scan', 10
        )

        # Дополнительный топик для отладки/визуализации
        self.ultrasonic_publisher = self.create_publisher(
            LaserScan, '/ultrasonic/ranges', 10
        )

        self.get_logger().info('Издатели для /scan и /ultrasonic/ranges созданы')

    def _find_arduino_mega_port(self) -> List[str]:
        """Автоматический поиск портов Arduino Mega по devpath."""
        available_ports = []

        self.get_logger().info('Поиск устройства с devpath=1.4 для Arduino Mega')
        ports = serial.tools.list_ports.comports()

        for port in ports:
            try:
                result = subprocess.run(
                    ['udevadm', 'info', '-a', '-n', port.device],
                    capture_output=True,
                    text=True
                )

                if "devpath==\"1.4\"" in result.stdout or "ATTRS{devpath}==\"1.4\"" in result.stdout:
                    self.get_logger().info(f'Найдено устройство с devpath=1.4 на порту {port.device}')
                    available_ports.append(port.device)
            except Exception as e:
                self.get_logger().debug(f'Ошибка при проверке devpath для {port.device}: {e}')

        if not available_ports:
            self.get_logger().warn('Не найдено устройств с devpath=1.4')

        return available_ports

    def _connect_to_arduino(self) -> bool:
        """Установка соединения с Arduino Mega."""
        available_ports = self._find_arduino_mega_port()

        if not available_ports:
            self.get_logger().error('Не найдено устройств с devpath=1.4 для Arduino Mega')
            return False

        port = available_ports[0]
        self.get_logger().info(f'Подключаюсь к порту {port} для Arduino Mega')

        try:
            self.arduino_serial = serial.Serial(
                port,
                self.BAUD_RATE,
                timeout=self.SERIAL_TIMEOUT
            )
            self.current_port = port

            time.sleep(self.CONNECTION_TIMEOUT)
            self.get_logger().info(f'Успешно подключено к Arduino Mega на порту {port}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Не удалось подключиться к Arduino Mega: {e}')
            return False

    def _is_arduino_connected(self) -> bool:
        """Проверка подключения Arduino."""
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
                    self.get_logger().warn('Arduino не подключен, попытка переподключения...')
                    self._reconnect_arduino()
                    time.sleep(1.0)
                    continue

                # Читаем данные с serial порта
                if self.arduino_serial.in_waiting > 0:
                    line = self.arduino_serial.readline().decode('utf-8').strip()
                    self._process_sensor_data(line)

                time.sleep(0.01)  # 100 Hz опрос

            except Exception as e:
                self.get_logger().error(f'Ошибка в цикле чтения датчиков: {e}')
                time.sleep(1.0)

    def _process_sensor_data(self, data_line: str):
        """Обработка данных с датчиков и публикация LaserScan."""
        try:
            # Ожидаем формат: "DISTANCE:10:23:24:23:23:22:22"
            if not data_line.startswith('DISTANCE:'):
                return

            # Парсим данные датчиков
            parts = data_line.split(':')
            if len(parts) != 8:  # DISTANCE + 7 датчиков
                self.get_logger().warn(f'Неверный формат данных: {len(parts)-1} датчиков')
                return

            # Обновляем данные датчиков
            for i in range(1, 8):
                try:
                    distance_cm = float(parts[i])

                    # Конвертируем см в метры
                    if distance_cm == 999:
                        # Ошибка или вне диапазона - ставим max range
                        self.sensor_data[i-1] = self.RANGE_MAX
                    else:
                        self.sensor_data[i-1] = distance_cm / 100.0  # см -> м

                except ValueError:
                    self.get_logger().error(f'Ошибка парсинга датчика {i}: {parts[i]}')
                    self.sensor_data[i-1] = self.RANGE_MAX

            # Публикуем LaserScan
            self._publish_laserscan()

        except Exception as e:
            self.get_logger().error(f'Ошибка обработки данных датчиков: {e}')

    def _publish_laserscan(self):
        """Публикация данных в формате LaserScan."""
        try:
            # Создаем сообщение LaserScan
            scan_msg = LaserScan()

            # Заполняем header
            scan_msg.header = Header()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'base_link'

            # Углы сканирования
            scan_msg.angle_min = -math.pi / 2  # -90° (правый боковой)
            scan_msg.angle_max = math.pi / 2   # +90° (левый боковой)

            # Создаем массив углов и расстояний
            # Сортируем датчики по углам (от -90° до +90°)
            sorted_indices = sorted(range(self.NUM_SENSORS),
                                  key=lambda i: self.SENSOR_ANGLES[i])

            # Количество лучей = количество датчиков
            num_readings = self.NUM_SENSORS
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (num_readings - 1)

            # Временные параметры
            scan_msg.time_increment = 0.0  # Все измерения одновременные
            scan_msg.scan_time = 0.1       # 10 Hz обновление

            # Диапазоны измерений
            scan_msg.range_min = self.RANGE_MIN
            scan_msg.range_max = self.RANGE_MAX

            # Заполняем массив расстояний
            # Создаем полный массив для интерполяции между датчиками
            # Для Nav2 лучше иметь больше лучей
            num_interpolated_readings = 360  # 1 градус разрешение
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (num_interpolated_readings - 1)

            ranges = []
            for i in range(num_interpolated_readings):
                angle = scan_msg.angle_min + i * scan_msg.angle_increment

                # Находим ближайший датчик к этому углу
                closest_sensor_idx = min(range(self.NUM_SENSORS),
                                       key=lambda idx: abs(self.SENSOR_ANGLES[idx] - angle))

                # Берем значение от ближайшего датчика
                # Если угол сильно отличается (>15°), ставим max range
                angle_diff = abs(self.SENSOR_ANGLES[closest_sensor_idx] - angle)
                if angle_diff > math.radians(15):
                    ranges.append(self.RANGE_MAX)
                else:
                    ranges.append(self.sensor_data[closest_sensor_idx])

            scan_msg.ranges = ranges
            scan_msg.intensities = []  # Ультразвуковые датчики не дают интенсивность

            # Публикуем в оба топика
            self.scan_publisher.publish(scan_msg)
            self.ultrasonic_publisher.publish(scan_msg)

            # Логируем каждые 2 секунды
            if not hasattr(self, '_last_log_time'):
                self._last_log_time = 0.0

            current_time = time.time()
            if current_time - self._last_log_time > 2.0:
                sensor_values = ', '.join([f'{d*100:.0f}' for d in self.sensor_data])
                self.get_logger().info(f'LaserScan опубликован: [{sensor_values}] см')
                self._last_log_time = current_time

        except Exception as e:
            self.get_logger().error(f'Ошибка публикации LaserScan: {e}')

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
        self.get_logger().info('Завершение работы Ultrasonic to LaserScan ноды')
        self.stop_thread = True

        if self.reading_thread and self.reading_thread.is_alive():
            self.reading_thread.join(timeout=2.0)

        self._close_arduino_connection()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicToLaserScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
