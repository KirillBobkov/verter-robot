#!/usr/bin/env python3
"""Periodic scan spin assist for front-only lidar exploration."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node


class ExplorationSpinAssistNode(Node):
    """Triggers short in-place spin when robot is idle for too long."""

    def __init__(self):
        super().__init__('exploration_spin_assist_node')

        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_topic', '/teleop_keyboard/cmd_vel')
        self.declare_parameter('safety_topic', '/safety/cmd_vel')
        self.declare_parameter('idle_speed_threshold', 0.03)
        self.declare_parameter('idle_duration_sec', 6.0)
        self.declare_parameter('spin_speed', 0.2)
        self.declare_parameter('spin_angle_rad', math.pi)
        self.declare_parameter('spin_duration_sec', 0.0)
        self.declare_parameter('cooldown_sec', 20.0)
        self.declare_parameter('safety_holdoff_sec', 1.0)
        self.declare_parameter('min_translation_after_spin_m', 0.12)
        self.declare_parameter('control_rate_hz', 20.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.safety_topic = self.get_parameter('safety_topic').value
        self.idle_speed_threshold = float(self.get_parameter('idle_speed_threshold').value)
        self.idle_duration_sec = float(self.get_parameter('idle_duration_sec').value)
        self.spin_speed = float(self.get_parameter('spin_speed').value)
        self.spin_angle_rad = abs(float(self.get_parameter('spin_angle_rad').value))
        spin_duration_param = float(self.get_parameter('spin_duration_sec').value)
        if abs(self.spin_speed) < 1e-6:
            self.spin_speed = 0.2
        if spin_duration_param > 0.0:
            self.spin_duration_sec = spin_duration_param
        else:
            self.spin_duration_sec = self.spin_angle_rad / abs(self.spin_speed)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)
        self.safety_holdoff_sec = float(self.get_parameter('safety_holdoff_sec').value)
        self.min_translation_after_spin_m = float(
            self.get_parameter('min_translation_after_spin_m').value
        )
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
        self.create_subscription(Twist, self.safety_topic, self._safety_cb, 10)

        self.last_motion_time = self.get_clock().now()
        self.last_spin_end_time = self.get_clock().now() - Duration(seconds=9999.0)
        self.last_safety_time = None
        self.current_speed = 0.0
        self.current_x = None
        self.current_y = None
        self.spin_active = False
        self.spin_end_time = None
        self.waiting_translation_after_spin = False
        self.spin_end_x = None
        self.spin_end_y = None

        self.spin_cmd = Twist()
        self.spin_cmd.angular.z = self.spin_speed
        self.stop_cmd = Twist()

        self.create_timer(1.0 / self.control_rate_hz, self._tick)
        self.get_logger().info(
            'exploration_spin_assist_node started: '
            f'idle={self.idle_duration_sec:.1f}s, spin={self.spin_duration_sec:.1f}s '
            f'(~{self.spin_angle_rad:.2f} rad), '
            f'min_translation_after_spin={self.min_translation_after_spin_m:.2f}m'
        )

    def _odom_cb(self, msg: Odometry):
        self.current_x = float(msg.pose.pose.position.x)
        self.current_y = float(msg.pose.pose.position.y)

        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        speed = (v.x ** 2 + v.y ** 2) ** 0.5 + abs(w.z) * 0.5
        self.current_speed = speed
        if speed > self.idle_speed_threshold:
            self.last_motion_time = self.get_clock().now()

        if self.waiting_translation_after_spin and self.spin_end_x is not None:
            dx = self.current_x - self.spin_end_x
            dy = self.current_y - self.spin_end_y
            if (dx * dx + dy * dy) ** 0.5 >= self.min_translation_after_spin_m:
                self.waiting_translation_after_spin = False
                self.get_logger().info(
                    'Translation after spin detected, spin-assist re-enabled'
                )

    def _safety_cb(self, _msg: Twist):
        self.last_safety_time = self.get_clock().now()

    def _safety_is_active(self) -> bool:
        if self.last_safety_time is None:
            return False
        return (self.get_clock().now() - self.last_safety_time) < Duration(
            seconds=self.safety_holdoff_sec
        )

    def _start_spin(self):
        now = self.get_clock().now()
        self.spin_active = True
        self.spin_end_time = now + Duration(seconds=self.spin_duration_sec)
        self.get_logger().warn('Idle detected, starting scan-assist spin')

    def _stop_spin(self):
        self.spin_active = False
        self.spin_end_time = None
        self.last_spin_end_time = self.get_clock().now()
        if self.current_x is not None:
            self.spin_end_x = self.current_x
            self.spin_end_y = self.current_y
            self.waiting_translation_after_spin = True
        self.cmd_pub.publish(self.stop_cmd)
        self.get_logger().info('Scan-assist spin finished')

    def _tick(self):
        now = self.get_clock().now()

        if self.spin_active:
            if now >= self.spin_end_time:
                self._stop_spin()
            else:
                if not self._safety_is_active():
                    self.cmd_pub.publish(self.spin_cmd)
            return

        if self._safety_is_active():
            return

        if (now - self.last_spin_end_time) < Duration(seconds=self.cooldown_sec):
            return

        # Do not start a new spin until robot translates after previous spin.
        if self.waiting_translation_after_spin:
            return

        if self.current_speed > self.idle_speed_threshold:
            return

        if (now - self.last_motion_time) >= Duration(seconds=self.idle_duration_sec):
            self._start_spin()


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationSpinAssistNode()
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
