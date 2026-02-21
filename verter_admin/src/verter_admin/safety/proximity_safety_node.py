#!/usr/bin/env python3
"""Emergency stop publisher based on near obstacle distance."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ProximitySafetyNode(Node):
    """Publishes zero Twist to /safety/cmd_vel when obstacle is too close."""

    def __init__(self):
        super().__init__('proximity_safety_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic/ranges')
        self.declare_parameter('cmd_topic', '/safety/cmd_vel')
        self.declare_parameter('stop_distance', 0.20)
        self.declare_parameter('resume_distance', 0.25)
        self.declare_parameter('sensor_timeout_sec', 0.8)
        self.declare_parameter('control_rate_hz', 20.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.ultrasonic_topic = self.get_parameter('ultrasonic_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        if self.resume_distance <= self.stop_distance:
            self.resume_distance = self.stop_distance + 0.05
            self.get_logger().warn(
                'resume_distance must be greater than stop_distance; '
                f'using {self.resume_distance:.2f} m'
            )

        self.scan_min_range = math.inf
        self.scan_last_stamp = None
        self.ultrasonic_min_range = math.inf
        self.ultrasonic_last_stamp = None
        self.safety_active = False

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            self.ultrasonic_topic,
            self._ultrasonic_callback,
            qos_profile_sensor_data,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.stop_twist = Twist()
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.get_logger().info(
            'proximity_safety_node started: '
            f'stop<{self.stop_distance:.2f}m, resume>{self.resume_distance:.2f}m'
        )

    @staticmethod
    def _min_valid_range(msg: LaserScan) -> float:
        valid_ranges = [
            distance
            for distance in msg.ranges
            if math.isfinite(distance) and msg.range_min <= distance <= msg.range_max
        ]
        if not valid_ranges:
            return math.inf
        return min(valid_ranges)

    def _scan_callback(self, msg: LaserScan):
        self.scan_min_range = self._min_valid_range(msg)
        self.scan_last_stamp = self.get_clock().now()

    def _ultrasonic_callback(self, msg: LaserScan):
        self.ultrasonic_min_range = self._min_valid_range(msg)
        self.ultrasonic_last_stamp = self.get_clock().now()

    def _get_recent_min(self, min_range: float, stamp):
        if stamp is None:
            return math.inf
        if (self.get_clock().now() - stamp) > Duration(seconds=self.sensor_timeout_sec):
            return math.inf
        return min_range

    def _control_loop(self):
        scan_min = self._get_recent_min(self.scan_min_range, self.scan_last_stamp)
        ultrasonic_min = self._get_recent_min(
            self.ultrasonic_min_range, self.ultrasonic_last_stamp
        )
        nearest = min(scan_min, ultrasonic_min)

        if self.safety_active:
            if nearest >= self.resume_distance:
                self.safety_active = False
                self.get_logger().info(
                    f'Obstacle cleared: nearest={nearest:.3f} m, releasing stop'
                )
        else:
            if nearest <= self.stop_distance:
                self.safety_active = True
                self.get_logger().warn(
                    f'Obstacle too close: nearest={nearest:.3f} m, publishing stop'
                )

        if self.safety_active:
            self.cmd_pub.publish(self.stop_twist)


def main(args=None):
    rclpy.init(args=args)
    node = ProximitySafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

