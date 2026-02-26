#!/usr/bin/env python3
"""Emergency stop publisher based on near obstacle distance.

When obstacle is closer than stop_distance:
- Blocks FORWARD movement (positive linear.x)
- Allows BACKWARD movement and rotation (so operator can drive away)
- Releases when obstacle is farther than resume_distance

Manual override:
    ros2 param set /proximity_safety_node override true
    # drive robot away
    ros2 param set /proximity_safety_node override false
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ProximitySafetyNode(Node):
    """Blocks forward motion when obstacle is too close.

    Subscribes to teleop and nav2 cmd_vel topics. When safety is active,
    republishes commands with forward speed clamped to zero, allowing
    reverse and rotation so the operator can maneuver away.
    """

    def __init__(self):
        super().__init__('proximity_safety_node')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ultrasonic_topic', '/ultrasonic/ranges')
        self.declare_parameter('cmd_topic', '/safety/cmd_vel')
        self.declare_parameter('stop_distance', 0.20)
        self.declare_parameter('resume_distance', 0.25)
        self.declare_parameter('sensor_timeout_sec', 0.8)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('override', False)

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

        # Последняя команда от оператора/nav2 — для пропуска заднего хода
        self._last_teleop_cmd = Twist()
        self._last_nav_cmd = Twist()

        # Сенсоры
        self.create_subscription(
            LaserScan, self.scan_topic,
            self._scan_callback, qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, self.ultrasonic_topic,
            self._ultrasonic_callback, qos_profile_sensor_data,
        )

        # Подписка на команды движения (чтобы пропускать задний ход)
        self.create_subscription(
            Twist, '/teleop_keyboard/cmd_vel',
            self._teleop_callback, 10,
        )
        self.create_subscription(
            Twist, '/nav2/cmd_vel',
            self._nav_callback, 10,
        )

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            'proximity_safety_node started: '
            f'stop<{self.stop_distance:.2f}m, resume>{self.resume_distance:.2f}m, '
            f'override: ros2 param set {self.get_name()} override true'
        )

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'override' and p.value:
                self.get_logger().warn('OVERRIDE ENABLED — safety disabled')
            elif p.name == 'override' and not p.value:
                self.get_logger().info('Override disabled — safety restored')
        return SetParametersResult(successful=True)

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

    def _teleop_callback(self, msg: Twist):
        self._last_teleop_cmd = msg

    def _nav_callback(self, msg: Twist):
        self._last_nav_cmd = msg

    def _get_recent_min(self, min_range: float, stamp):
        if stamp is None:
            return math.inf
        if (self.get_clock().now() - stamp) > Duration(seconds=self.sensor_timeout_sec):
            return math.inf
        return min_range

    def _control_loop(self):
        # Проверка override
        override = self.get_parameter('override').value
        if override:
            # Override активен — safety не вмешивается
            if self.safety_active:
                self.safety_active = False
                self.get_logger().warn('Safety released by override')
            return

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
                    f'Obstacle too close: nearest={nearest:.3f} m, blocking forward'
                )

        if self.safety_active:
            # Берём команду от телеопа (приоритет) или nav2
            src = self._last_teleop_cmd
            if abs(src.linear.x) < 0.001 and abs(src.angular.z) < 0.001:
                src = self._last_nav_cmd

            filtered = Twist()
            # Блокируем только движение ВПЕРЁД, пропускаем назад и вращение
            filtered.linear.x = min(0.0, src.linear.x)  # ≤ 0 (назад OK)
            filtered.angular.z = src.angular.z           # вращение OK
            self.cmd_pub.publish(filtered)


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
