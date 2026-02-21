#!/usr/bin/env python3
"""Initial in-place rotation to scan surroundings with front-only lidar."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class InitialScanSpinNode(Node):
    """Publishes angular velocity for a fixed angle, then stops and exits."""

    def __init__(self):
        super().__init__('initial_scan_spin_node')

        self.declare_parameter('cmd_topic', '/teleop_keyboard/cmd_vel')
        self.declare_parameter('angular_speed', 0.2)
        self.declare_parameter('spin_angle_rad', math.pi)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.spin_angle_rad = abs(float(self.get_parameter('spin_angle_rad').value))
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        if self.angular_speed == 0.0:
            self.angular_speed = 0.2

        self.duration_sec = self.spin_angle_rad / abs(self.angular_speed)
        self.start_time = self.get_clock().now()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.spin_twist = Twist()
        self.spin_twist.angular.z = self.angular_speed
        self.stop_twist = Twist()

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)
        self.get_logger().info(
            f'Initial scan spin started: angle={self.spin_angle_rad:.2f} rad, '
            f'speed={self.angular_speed:.2f} rad/s, duration={self.duration_sec:.1f} s'
        )

    def _tick(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.duration_sec:
            self.cmd_pub.publish(self.spin_twist)
            return

        self.cmd_pub.publish(self.stop_twist)
        self.get_logger().info('Initial scan spin finished')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = InitialScanSpinNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()

