#!/usr/bin/env python3
"""
Конвертер Float32MultiArray → 7x sensor_msgs/Range.

Подписывается на /ultrasonic/distances (Float32MultiArray от ESP32)
и публикует 7 отдельных Range сообщений для каждого датчика
с правильными frame_id из URDF.

Далее range_to_laserscan.py конвертирует Range → LaserScan для Nav2.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range


# Порядок датчиков соответствует массиву из ESP32 (пины)
# Реальное расположение определено экспериментально
SENSORS = [
    {'name': 'front_left_inner',  'frame_id': 'sensor_front_left_inner'},   # [0] G16/G34 — не работает, TBD
    {'name': 'front_right_outer', 'frame_id': 'sensor_front_right_outer'},  # [1] G17/G35
    {'name': 'front_right_inner', 'frame_id': 'sensor_front_right_inner'},  # [2] G18/G32
    {'name': 'front_center',      'frame_id': 'sensor_front_center'},       # [3] G19/G33
    {'name': 'right',             'frame_id': 'sensor_right'},              # [4] G23/G25 — не работает, TBD
    {'name': 'front_left_outer',  'frame_id': 'sensor_front_left_outer'},   # [5] G26/G27
    {'name': 'left',              'frame_id': 'sensor_left'},               # [6] G14/G12
]

FIELD_OF_VIEW = math.radians(30.0)  # HC-SR04 FOV ~30°
MIN_RANGE = 0.02   # 2 см
MAX_RANGE = 2.5    # 2.5 м (соответствует таймауту ESP32)
INVALID_VALUE = -1.0


class RangeConverterNode(Node):
    def __init__(self):
        super().__init__('range_converter_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Подписка на массив расстояний от ESP32
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ultrasonic/distances',
            self.distances_callback,
            qos,
        )

        # 7 паблишеров Range (по одному на датчик)
        self.range_pubs = []
        for sensor in SENSORS:
            pub = self.create_publisher(
                Range,
                f'/verter/distance_sensors/{sensor["name"]}',
                qos,
            )
            self.range_pubs.append(pub)

        self.get_logger().info(
            f'Range converter: /ultrasonic/distances → {len(SENSORS)} Range topics'
        )

    def distances_callback(self, msg: Float32MultiArray):
        if len(msg.data) != len(SENSORS):
            self.get_logger().warn(
                f'Expected {len(SENSORS)} values, got {len(msg.data)}'
            )
            return

        now = self.get_clock().now().to_msg()

        for i, sensor in enumerate(SENSORS):
            range_msg = Range()
            range_msg.header.stamp = now
            range_msg.header.frame_id = sensor['frame_id']
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = FIELD_OF_VIEW
            range_msg.min_range = MIN_RANGE
            range_msg.max_range = MAX_RANGE

            distance = msg.data[i]
            if distance < 0 or distance < MIN_RANGE or distance > MAX_RANGE:
                range_msg.range = float('inf')
            else:
                range_msg.range = distance

            self.range_pubs[i].publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RangeConverterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
