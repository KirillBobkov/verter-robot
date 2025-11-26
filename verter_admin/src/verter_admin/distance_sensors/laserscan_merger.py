#!/usr/bin/env python3
"""
LaserScan Merger for Ultrasonic Sensors (Gazebo Simulation)

Merges multiple sensor_msgs/LaserScan messages from Gazebo ultrasonic sensors
into a single sensor_msgs/LaserScan message for costmap integration.

This is specifically for Gazebo simulation where sensors publish LaserScan.
For real robot with Range sensors, use range_to_laserscan.py instead.

Subscribed Topics:
    /verter/distance_sensors/front_center (sensor_msgs/LaserScan)
    /verter/distance_sensors/front_left_inner (sensor_msgs/LaserScan)
    /verter/distance_sensors/front_left_outer (sensor_msgs/LaserScan)
    /verter/distance_sensors/front_right_inner (sensor_msgs/LaserScan)
    /verter/distance_sensors/front_right_outer (sensor_msgs/LaserScan)
    /verter/distance_sensors/left (sensor_msgs/LaserScan)
    /verter/distance_sensors/right (sensor_msgs/LaserScan)

Published Topics:
    /ultrasonic/ranges (sensor_msgs/LaserScan)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import math


class LaserScanMerger(Node):
    """Merges multiple LaserScan messages into a single combined scan"""

    def __init__(self):
        super().__init__('laserscan_merger')

        # QoS profile compatible with Gazebo sensors
        # Gazebo publishes with RELIABLE, so we must subscribe with RELIABLE
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Sensor configuration based on URDF - angles in radians relative to base_link
        # Each sensor will contribute its scan data at its respective angle
        self.sensors = {
            'front_center': {'angle': 0.0, 'data': None},
            'front_left_inner': {'angle': 0.0995, 'data': None},  # ~5.7°
            'front_left_outer': {'angle': 0.2304, 'data': None},  # ~13.2°
            'front_right_inner': {'angle': -0.0995, 'data': None},  # ~-5.7°
            'front_right_outer': {'angle': -0.2304, 'data': None},  # ~-13.2°
            'left': {'angle': 1.3265, 'data': None},  # ~76°
            'right': {'angle': -1.3265, 'data': None},  # ~-76°
        }

        # Create subscribers for each sensor with compatible QoS
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/front_center',
            self.front_center_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/front_left_inner',
            self.front_left_inner_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/front_left_outer',
            self.front_left_outer_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/front_right_inner',
            self.front_right_inner_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/front_right_outer',
            self.front_right_outer_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/left',
            self.left_callback, qos_profile)
        self.create_subscription(
            LaserScan, '/verter/distance_sensors/right',
            self.right_callback, qos_profile)

        # Publisher for combined LaserScan
        self.scan_pub = self.create_publisher(LaserScan, '/ultrasonic/ranges', 10)

        # Publish combined LaserScan at 10Hz
        self.create_timer(0.1, self.publish_scan)

        # LaserScan parameters for output
        self.min_angle = -1.5708  # -90° (right side)
        self.max_angle = 1.5708   # +90° (left side)
        self.angle_increment = 0.0175  # ~1° resolution
        self.num_readings = int((self.max_angle - self.min_angle) / self.angle_increment)

        # Range limits (HC-SR04 specs)
        self.min_range = 0.02  # 2cm
        self.max_range = 4.0   # 4m

        self.get_logger().info('LaserScan Merger started (Gazebo simulation mode)')
        self.get_logger().info(f'Publishing combined scan with {self.num_readings} readings')

    # Individual callbacks for each sensor
    def front_center_callback(self, msg):
        self.sensors['front_center']['data'] = msg

    def front_left_inner_callback(self, msg):
        self.sensors['front_left_inner']['data'] = msg

    def front_left_outer_callback(self, msg):
        self.sensors['front_left_outer']['data'] = msg

    def front_right_inner_callback(self, msg):
        self.sensors['front_right_inner']['data'] = msg

    def front_right_outer_callback(self, msg):
        self.sensors['front_right_outer']['data'] = msg

    def left_callback(self, msg):
        self.sensors['left']['data'] = msg

    def right_callback(self, msg):
        self.sensors['right']['data'] = msg

    def publish_scan(self):
        """Merge all sensor scans and publish combined LaserScan message"""
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

        # Merge data from all sensors
        for sensor_name, sensor_info in self.sensors.items():
            if sensor_info['data'] is None:
                continue  # No data from this sensor yet

            sensor_scan = sensor_info['data']
            sensor_angle = sensor_info['angle']

            # Process each range in the sensor's LaserScan
            for i, range_val in enumerate(sensor_scan.ranges):
                # Calculate the actual angle of this reading in the sensor's frame
                reading_angle_in_sensor = sensor_scan.angle_min + i * sensor_scan.angle_increment

                # Transform to base_link frame (add sensor mounting angle)
                reading_angle_in_base = sensor_angle + reading_angle_in_sensor

                # Check if this angle is within our output range
                if self.min_angle <= reading_angle_in_base <= self.max_angle:
                    # Find the corresponding index in output scan
                    output_index = int((reading_angle_in_base - self.min_angle) / self.angle_increment)

                    if 0 <= output_index < self.num_readings:
                        # Only use valid ranges
                        if sensor_scan.range_min <= range_val <= sensor_scan.range_max:
                            # Use minimum range if multiple sensors overlap
                            ranges[output_index] = min(ranges[output_index], range_val)

        scan.ranges = ranges
        scan.intensities = []  # Not used

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanMerger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
