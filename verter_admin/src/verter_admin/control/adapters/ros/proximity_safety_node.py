#!/usr/bin/env python3
"""Emergency stop publisher based on near obstacle distance."""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from verter_admin.control.application import EvaluateSafety, SafetyEvaluationInput
from verter_admin.control.domain import (
    SafetyState,
    VelocityCommand,
    select_source_command,
)
from verter_admin.contracts.motion import MotionTimeouts, TopicContract


class ProximitySafetyNode(Node):
    """Blocks linear motion when obstacle is too close."""

    def __init__(self):
        super().__init__('proximity_safety_node')

        self.declare_parameter('scan_topic', TopicContract.SCAN)
        self.declare_parameter('ultrasonic_topic', TopicContract.ULTRASONIC_SCAN)
        self.declare_parameter('cmd_topic', TopicContract.SAFETY_CMD_VEL)
        self.declare_parameter('stop_distance', 0.20)
        self.declare_parameter('resume_distance', 0.25)
        self.declare_parameter('sensor_timeout_sec', MotionTimeouts.SENSOR_FRESHNESS_SEC)
        self.declare_parameter('command_timeout_sec', MotionTimeouts.TWIST_MUX_SOURCE_SEC)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('allow_degraded_motion', False)
        self.declare_parameter('override', False)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.ultrasonic_topic = self.get_parameter('ultrasonic_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.resume_distance = float(self.get_parameter('resume_distance').value)
        self.sensor_timeout_sec = float(self.get_parameter('sensor_timeout_sec').value)
        self.command_timeout_sec = float(self.get_parameter('command_timeout_sec').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.allow_degraded_motion = bool(self.get_parameter('allow_degraded_motion').value)

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

        self._last_teleop_cmd = Twist()
        self._last_nav_cmd = Twist()
        self._last_teleop_stamp = None
        self._last_nav_stamp = None
        self._state = SafetyState.READY
        self._last_sensor_health = (None, None)
        self._evaluator = EvaluateSafety()

        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, self.ultrasonic_topic, self._ultrasonic_callback, qos_profile_sensor_data
        )
        self.create_subscription(Twist, TopicContract.TELEOP_CMD_VEL, self._teleop_callback, 10)
        self.create_subscription(Twist, TopicContract.NAV2_CMD_VEL, self._nav_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_timer(1.0 / self.control_rate_hz, self._control_loop)
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            'proximity_safety_node started: '
            f'stop<{self.stop_distance:.2f}m, resume>{self.resume_distance:.2f}m, '
            f'sensor_timeout={self.sensor_timeout_sec:.2f}s, '
            f'cmd_timeout={self.command_timeout_sec:.2f}s, '
            f'override: ros2 param set {self.get_name()} override true'
        )

    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult

        for param in params:
            if param.name == 'override' and param.value:
                self.get_logger().warn('OVERRIDE ENABLED - safety disabled')
            elif param.name == 'override' and not param.value:
                self.get_logger().info('Override disabled - safety restored')
            elif param.name == 'allow_degraded_motion':
                self.allow_degraded_motion = bool(param.value)
                self.get_logger().warn(
                    'DEGRADED policy changed: '
                    f'allow_degraded_motion={self.allow_degraded_motion}'
                )
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

    @staticmethod
    def _to_velocity_command(msg: Twist) -> VelocityCommand:
        return VelocityCommand(
            linear_x=float(msg.linear.x),
            angular_z=float(msg.angular.z),
        )

    @staticmethod
    def _to_twist(cmd: VelocityCommand) -> Twist:
        twist = Twist()
        twist.linear.x = cmd.linear_x
        twist.angular.z = cmd.angular_z
        return twist

    def _scan_callback(self, msg: LaserScan):
        self.scan_min_range = self._min_valid_range(msg)
        self.scan_last_stamp = self.get_clock().now()

    def _ultrasonic_callback(self, msg: LaserScan):
        self.ultrasonic_min_range = self._min_valid_range(msg)
        self.ultrasonic_last_stamp = self.get_clock().now()

    def _teleop_callback(self, msg: Twist):
        self._last_teleop_cmd = msg
        self._last_teleop_stamp = self.get_clock().now()

    def _nav_callback(self, msg: Twist):
        self._last_nav_cmd = msg
        self._last_nav_stamp = self.get_clock().now()

    def _is_recent(self, stamp, timeout_sec: float) -> bool:
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp) <= Duration(seconds=timeout_sec)

    def _get_recent_min(self, min_range: float, stamp):
        if not self._is_recent(stamp, self.sensor_timeout_sec):
            return math.inf
        return min_range

    def _command_is_fresh(self) -> bool:
        return self._is_recent(
            self._last_teleop_stamp, self.command_timeout_sec
        ) or self._is_recent(
            self._last_nav_stamp, self.command_timeout_sec
        )

    def _select_source_cmd(self) -> VelocityCommand | None:
        teleop_fresh = self._is_recent(self._last_teleop_stamp, self.command_timeout_sec)
        nav_fresh = self._is_recent(self._last_nav_stamp, self.command_timeout_sec)
        return select_source_command(
            teleop_command=self._to_velocity_command(self._last_teleop_cmd),
            nav_command=self._to_velocity_command(self._last_nav_cmd),
            teleop_fresh=teleop_fresh,
            nav_fresh=nav_fresh,
        )

    def _set_state(self, new_state: SafetyState, reason: str):
        if self._state == new_state:
            return
        old_state = self._state
        self._state = new_state
        self.get_logger().warn(
            f'SAFETY_STATE {old_state.value} -> {new_state.value}: {reason}'
        )

    def _log_sensor_health(self, scan_fresh: bool, ultrasonic_fresh: bool):
        current_health = (scan_fresh, ultrasonic_fresh)
        if current_health == self._last_sensor_health:
            return
        self._last_sensor_health = current_health
        if scan_fresh or ultrasonic_fresh:
            self.get_logger().info(
                f'sensor freshness: scan={scan_fresh}, ultrasonic={ultrasonic_fresh}'
            )
        else:
            self.get_logger().warn(
                'sensor freshness lost: both scan and ultrasonic are stale'
            )

    def _control_loop(self):
        override = self.get_parameter('override').value
        if override:
            if self.safety_active:
                self.safety_active = False
                self.get_logger().warn('Safety released by override')
            self._set_state(SafetyState.DEGRADED, 'override enabled')
            return

        scan_fresh = self._is_recent(self.scan_last_stamp, self.sensor_timeout_sec)
        ultrasonic_fresh = self._is_recent(
            self.ultrasonic_last_stamp, self.sensor_timeout_sec
        )
        self._log_sensor_health(scan_fresh, ultrasonic_fresh)
        sensors_fresh = scan_fresh or ultrasonic_fresh

        scan_min = self._get_recent_min(self.scan_min_range, self.scan_last_stamp)
        ultrasonic_min = self._get_recent_min(
            self.ultrasonic_min_range, self.ultrasonic_last_stamp
        )
        nearest = min(scan_min, ultrasonic_min)
        command_fresh = self._command_is_fresh()

        decision = self._evaluator.execute(
            SafetyEvaluationInput(
                nearest_distance=nearest,
                sensors_fresh=sensors_fresh,
                command_fresh=command_fresh,
                safety_active=self.safety_active,
                stop_distance=self.stop_distance,
                resume_distance=self.resume_distance,
                allow_degraded_motion=self.allow_degraded_motion,
                source_command=self._select_source_cmd(),
            )
        )

        if decision.recovery_gate_opened:
            self.get_logger().info(
                f'FAULT -> READY gate opened: nearest={nearest:.3f} m, '
                f'cmd_fresh={command_fresh}'
            )
        if decision.obstacle_triggered:
            self.get_logger().warn(
                f'Obstacle too close: nearest={nearest:.3f} m, blocking linear motion'
            )

        self.safety_active = decision.safety_active
        if decision.publish_command is not None:
            self.cmd_pub.publish(self._to_twist(decision.publish_command))

        self._set_state(decision.state, decision.state_reason)


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
