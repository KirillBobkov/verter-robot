#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
PointCloud2 to LaserScan Converter Node (версия с нуля)
================================================================================
Назначение:
------------
Эта нода преобразует сообщения типа sensor_msgs/PointCloud2, получаемые
от 3D-сенсора (например, ToF-камеры), в сообщения sensor_msgs/LaserScan,
которые используются системами 2D-картографирования (SLAM) и навигации (Nav2).

Принцип работы:
----------------
1.  **Подписка на PointCloud2**: Нода слушает топик с "облаком точек".
    Каждая точка в облаке имеет координаты (x, y, z).

2.  **Фильтрация по высоте**: Чтобы создать 2D-карту, нам нужны только
    препятствия на уровне робота. Точки, соответствующие полу или потолку,
    отбрасываются. Для этого задается диапазон высот [min_height, max_height].

3.  **Проекция на 2D-плоскость**: Оставшиеся 3D-точки проецируются на
    горизонтальную плоскость. Для каждой точки вычисляется её расстояние
    до сенсора и угол в этой плоскости.

4.  **Формирование LaserScan**: Сообщение LaserScan представляет собой массив
    "лучей", где для каждого угла в заданном диапазоне (например, от -90° до +90°)
    хранится расстояние до ближайшего препятствия.

5.  **Публикация**: Готовое сообщение LaserScan публикуется в топик /scan,
    который слушает SLAM Toolbox.

Системы координат (Frames):
---------------------------
- **Вход**: PointCloud2 обычно публикуется в "оптическом кадре" камеры
  (например, `camera_tof_optical`), где:
    - Ось Z направлена вперёд (из линзы)
    - Ось X направлена вправо
    - Ось Y направлена вниз

- **Выход**: LaserScan будет публиковаться в том же кадре (`frame_id`), что и
  входящее облако точек. Это самый надёжный подход, так как система TF (Transform)
  в ROS 2 сама позаботится о правильном расположении этого скана в пространстве
  относительно робота. SLAM Toolbox корректно обработает такой скан.
================================================================================
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from sensor_msgs_py import point_cloud2
import numpy as np
import math


class PointCloudToLaserScanNode(Node):
    """
    Класс ноды для конвертации PointCloud2 в LaserScan.
    """
    def __init__(self):
        """
        Конструктор ноды.
        Инициализирует параметры, подписчиков и издателей.
        """
        super().__init__('pointcloud_to_laserscan_node')

        # --- Объявление параметров ---
        # Эти параметры можно будет изменять из launch-файла
        self.declare_parameter('input_topic', '/verter/camera_tof/points')
        self.declare_parameter('output_topic', '/scan')
        self.declare_parameter('min_height', -0.3)  # Мин. высота точки от сенсора (м)
        self.declare_parameter('max_height', 0.5)   # Макс. высота точки от сенсора (м)
        self.declare_parameter('angle_min', -math.pi / 2)  # -90 градусов
        self.declare_parameter('angle_max', math.pi / 2)   # +90 градусов
        self.declare_parameter('angle_increment', math.radians(1.0))  # Шаг в 1 градус
        self.declare_parameter('range_min', 0.1)   # Мин. дальность (м)
        self.declare_parameter('range_max', 5.0)   # Макс. дальность (м)
        self.declare_parameter('scan_time', 0.1)   # Время сканирования (для 10 Гц)

        # --- Получение значений параметров ---
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scan_time = self.get_parameter('scan_time').value

        # Рассчитываем количество лучей в скане. +1 нужен, чтобы включить обе границы диапазона.
        self.num_readings = int(round((self.angle_max - self.angle_min) / self.angle_increment)) + 1

        # --- Настройка QoS (Quality of Service) ---
        # Для сенсорных данных от симуляции часто лучше использовать BEST_EFFORT
        qos_input = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        # Для SLAM требуется RELIABLE
        qos_output = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE, # SLAM не нужны старые сканы при подключении
            depth=10
        )

        # --- Создание подписчика и издателя ---
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_input
        )
        self.scan_publisher = self.create_publisher(LaserScan, self.output_topic, qos_output)

        self.get_logger().info(f"Нода запущена. Слушает '{self.input_topic}', публикует в '{self.output_topic}'")
        self.get_logger().info(f"Параметры скана: {self.num_readings} лучей, диапазон [{math.degrees(self.angle_min):.1f}°, {math.degrees(self.angle_max):.1f}°]")
        self.get_logger().info(f"Фильтр по высоте: [{self.min_height:.2f} м, {self.max_height:.2f} м]")

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        """
        Callback-функция, вызываемая при получении нового сообщения PointCloud2.
        """
        try:
            # --- Инициализация LaserScan сообщения ---
            scan_msg = LaserScan()
            scan_msg.header = cloud_msg.header  # Используем тот же frame_id и timestamp
            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = self.angle_increment
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = self.scan_time
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max

            # Инициализируем массив расстояний значением "бесконечность" (или max_range)
            ranges = np.full(self.num_readings, self.range_max, dtype=np.float32)

            # --- Итерация по точкам облака ---
            # Читаем точки из облака, пропуская NaN значения
            points_generator = point_cloud2.read_points(
                cloud_msg, field_names=("x", "y", "z"), skip_nans=True
            )

            for point in points_generator:
                x, y, z = point

                # --- Фильтрация по высоте ---
                # В оптическом кадре ось Y направлена вниз
                if y < self.min_height or y > self.max_height:
                    continue

                # --- Проекция на 2D и вычисления ---
                # Расстояние до точки в 2D-плоскости камеры (XZ)
                distance = math.hypot(x, z)

                # Фильтрация по дальности
                if distance < self.range_min or distance > self.range_max:
                    continue

                # Угол точки в плоскости камеры
                # atan2(x, z) потому что Z - "вперёд", а X - "вправо"
                angle = math.atan2(x, z)

                # Проверка, попадает ли угол в наш диапазон сканирования
                if angle < self.angle_min or angle > self.angle_max:
                    continue

                # --- Запись в массив ranges ---
                # Находим индекс луча, которому соответствует точка
                index = int((angle - self.angle_min) / self.angle_increment)

                # Если для этого луча уже есть точка, выбираем ту, что ближе
                if distance < ranges[index]:
                    ranges[index] = distance

            # --- Публикация результата ---
            scan_msg.ranges = ranges.tolist()
            self.scan_publisher.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f"Ошибка при обработке PointCloud: {e}")


def main(args=None):
    """
    Главная функция для запуска ноды.
    """
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Нода остановлена пользователем")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
