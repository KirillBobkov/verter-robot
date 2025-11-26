#!/usr/bin/env python3
"""
Range Sensors to LaserScan Converter

Converts multiple sensor_msgs/Range messages from ultrasonic sensors
into a single sensor_msgs/LaserScan message for costmap integration.

This allows Nav2's costmap to detect low obstacles (below lidar height)
using the 7 HC-SR04 ultrasonic sensors mounted at 5cm height.

Subscribed Topics:
    /verter/distance_sensors/front_center (sensor_msgs/Range)
    /verter/distance_sensors/front_left_inner (sensor_msgs/Range)
    /verter/distance_sensors/front_left_outer (sensor_msgs/Range)
    /verter/distance_sensors/front_right_inner (sensor_msgs/Range)
    /verter/distance_sensors/front_right_outer (sensor_msgs/Range)
    /verter/distance_sensors/left (sensor_msgs/Range)
    /verter/distance_sensors/right (sensor_msgs/Range)

Published Topics:
    /ultrasonic/ranges (sensor_msgs/LaserScan)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Range, LaserScan
import math


class RangeToLaserScan(Node):
    """Converts Range messages to a combined LaserScan message"""

    def __init__(self):
        super().__init__('range_to_laserscan')

        # QoS profile for reliable sensor data delivery
        # RELIABLE ensures no data loss from HC-SR04 sensors
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Sensor configuration based on URDF
        # Angles in radians relative to robot base_link
        self.sensors = {
            'front_center': {'angle': 0.0, 'range': float('inf')},
            'front_left_inner': {'angle': 0.0995, 'range': float('inf')},  # ~5.7°
            'front_left_outer': {'angle': 0.2304, 'range': float('inf')},  # ~13.2°
            'front_right_inner': {'angle': -0.0995, 'range': float('inf')},  # ~-5.7°
            'front_right_outer': {'angle': -0.2304, 'range': float('inf')},  # ~-13.2°
            'left': {'angle': 1.3265, 'range': float('inf')},  # ~76°
            'right': {'angle': -1.3265, 'range': float('inf')},  # ~-76°
        }

        # Create subscribers for each sensor with reliable QoS
        self.create_subscription(
            Range, '/verter/distance_sensors/front_center',
            lambda msg: self.range_callback(msg, 'front_center'), qos_profile)
        self.create_subscription(
            Range, '/verter/distance_sensors/front_left_inner',
            lambda msg: self.range_callback(msg, 'front_left_inner'), qos_profile)
        self.create_subscription(
            Range, '/verter/distance_sensors/front_left_outer',
            lambda msg: self.range_callback(msg, 'front_left_outer'), qos_profile)
        self.create_subscription(
            Range, '/verter/distance_sensors/front_right_inner',
            lambda msg: self.range_callback(msg, 'front_right_inner'), qos_profile)
        self.create_subscription(
            Range, '/verter/distance_sensors/front_right_outer',
            lambda msg: self.range_callback(msg, 'front_right_outer'), qos_profile)
        self.create_subscription(
            Range, '/verter/distance_sensors/left',
            lambda msg: self.range_callback(msg, 'left'), qos_profile)
        self.create_subscription(
            Range, '/verter/distance_sensors/right',
            lambda msg: self.range_callback(msg, 'right'), qos_profile)

        # Publisher for combined LaserScan
        self.scan_pub = self.create_publisher(LaserScan, '/ultrasonic/ranges', 10)

        # Publish LaserScan at 10Hz
        self.create_timer(0.1, self.publish_scan)

        # LaserScan parameters
        self.min_angle = -1.5708  # -90° (right side)
        self.max_angle = 1.5708   # +90° (left side)
        self.angle_increment = 0.0175  # ~1° resolution
        self.num_readings = int((self.max_angle - self.min_angle) / self.angle_increment)

        # Range limits (HC-SR04 specs)
        self.min_range = 0.02  # 2cm
        self.max_range = 4.0   # 4m

        self.get_logger().info('Range to LaserScan converter started')
        self.get_logger().info(f'Publishing combined scan with {self.num_readings} readings')

    def range_callback(self, msg, sensor_name):
        """Update sensor reading"""
        if sensor_name in self.sensors:
            # Store valid range or infinity if out of bounds
            if msg.range >= msg.min_range and msg.range <= msg.max_range:
                self.sensors[sensor_name]['range'] = msg.range
            else:
                self.sensors[sensor_name]['range'] = float('inf')

    def publish_scan(self):
        """Publish combined LaserScan message"""
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = 'base_link'

        scan.angle_min = self.min_angle
        scan.angle_max = self.max_angle
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1  # 10Hz

        scan.range_min = self.min_range
        scan.range_max = self.max_range

        # Initialize all ranges to infinity (no obstacle)
        ranges = [float('inf')] * self.num_readings

        # Fill in sensor readings at their respective angles
        for sensor_name, sensor_data in self.sensors.items():
            angle = sensor_data['angle']
            range_value = sensor_data['range']

            # Find closest index for this angle
            if self.min_angle <= angle <= self.max_angle:
                index = int((angle - self.min_angle) / self.angle_increment)
                if 0 <= index < self.num_readings:
                    # Use minimum range if multiple sensors overlap
                    ranges[index] = min(ranges[index], range_value)

        scan.ranges = ranges
        scan.intensities = []  # Not used

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = RangeToLaserScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
