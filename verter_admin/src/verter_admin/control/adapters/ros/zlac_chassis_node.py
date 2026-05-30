#!/usr/bin/env python3
"""ROS 2 lifecycle adapter: cmd_vel → ZLAC8015D, position feedback → /wheel_encoders.

Replaces the ESP32 chassis micro-ROS node. The ROS layer is a thin adapter
between three things:

  * ChassisPolicy  (domain)            — state machine, watchdog, kinematics.
  * ZLACDriver     (adapter)           — Modbus RTU to the hardware driver.
  * ROS interfaces (this file)         — subscriptions, publications, timer,
                                          lifecycle transitions.

Topic contracts (kept compatible with the previous ESP32 firmware so the
odometry node and twist_mux don't need to change):

  Sub  /cmd_vel              geometry_msgs/Twist            RELIABLE
  Pub  /wheel_encoders       std_msgs/Int64MultiArray       BEST_EFFORT  50 Hz
  Pub  /chassis/state        std_msgs/UInt8                 RELIABLE      2 Hz
  Pub  /chassis/fault        std_msgs/UInt32                RELIABLE      2 Hz
  Pub  /chassis/cmd_watchdog std_msgs/Bool                  RELIABLE      2 Hz
  Pub  /chassis/wheel_rpm    std_msgs/Int16MultiArray       BEST_EFFORT  20 Hz

Lifecycle transitions:
  configure  → open serial port, write velocity mode + comm-watchdog,
               read fw version to verify reachability. Stays in INACTIVE.
  activate   → enable driver, clear stale command, start 50 Hz control loop.
  deactivate → stop motors, disable driver, hold timer suspended.
  cleanup    → close serial port.
  on_error   → emergency_stop, disable, mark policy FAULT.

Safety chain (SDHR §3):
  L0 (ZLAC8015D firmware)         — hardware enable, comm-loss watchdog (200 ms),
                                    max RPM clamp, current/temperature limits.
  L1 (this node + ChassisPolicy)  — software watchdog (cmd_timeout = 500 ms),
                                    velocity clamping, sign mapping, fault latch.
  Operator / external             — physical E-stop NC button on driver EN line.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.lifecycle import State, TransitionCallbackReturn
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int16MultiArray, Int64MultiArray, UInt8, UInt32

from verter_admin.contracts.motion import MotionTimeouts, TopicContract
from verter_admin.control.adapters.zlac.zlac_driver import (
    WheelFeedback,
    ZLACDriver,
    ZLACDriverError,
)
from verter_admin.control.domain.chassis_policy import (
    ChassisParameters,
    ChassisPolicy,
    ChassisState,
)


# ROS QoS profiles aligned with existing chassis topic contracts.
_BEST_EFFORT = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)
_RELIABLE = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


class ZLACChassisNode(LifecycleNode):
    """L1 lifecycle node owning the chassis command path."""

    CONTROL_PERIOD_S = 0.02  # 50 Hz
    SLOW_PUBLISH_PERIOD_S = 0.5  # 2 Hz for state/fault/watchdog
    WHEEL_RPM_PUBLISH_PERIOD_S = 0.05  # 20 Hz debug

    def __init__(self) -> None:
        super().__init__("zlac_chassis_node")

        # --- Parameters ----------------------------------------------------
        self.declare_parameter("port", "/dev/zlac_chassis")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("slave", 1)
        self.declare_parameter("comm_watchdog_ms", 200)
        self.declare_parameter("wheel_diameter", 0.200)
        self.declare_parameter("wheel_base", 0.386)
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("max_motor_rpm", 200)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 1.0)
        self.declare_parameter("left_sign", 1)
        self.declare_parameter("right_sign", 1)
        self.declare_parameter(
            "cmd_timeout",
            MotionTimeouts.CHASSIS_WATCHDOG_MS / 1000.0,
        )

        # --- Runtime state -------------------------------------------------
        self._driver: ZLACDriver | None = None
        self._policy: ChassisPolicy | None = None
        self._control_timer = None
        self._slow_timer = None
        self._rpm_timer = None
        self._cmd_sub = None
        self._last_feedback: WheelFeedback | None = None

        # --- Publishers (created lazily in on_configure) -------------------
        self._pub_encoders = None
        self._pub_state = None
        self._pub_fault = None
        self._pub_watchdog = None
        self._pub_wheel_rpm = None

    # ====================================================== lifecycle =====

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        params = self._build_chassis_parameters()
        self._policy = ChassisPolicy(params)
        self._policy.configure()

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        slave = int(self.get_parameter("slave").value)
        watchdog_ms = int(self.get_parameter("comm_watchdog_ms").value)

        self._driver = ZLACDriver(port=port, baud=baud, slave=slave)
        try:
            self._driver.open()
            fw = self._driver.read_fw_version()
            self._driver.setup_velocity_mode(comm_watchdog_ms=watchdog_ms)
        except ZLACDriverError as e:
            self.get_logger().error(f"configure failed: {e}")
            self._safe_close_driver()
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info(
            f"ZLAC connected on {port} (fw=0x{fw:04X}), velocity mode set, "
            f"comm_watchdog={watchdog_ms}ms"
        )

        self._create_endpoints()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if self._driver is None or self._policy is None:
            return TransitionCallbackReturn.FAILURE

        try:
            self._driver.write_target_rpm(0, 0)
            self._driver.enable()
        except ZLACDriverError as e:
            self.get_logger().error(f"activate: enable failed: {e}")
            return TransitionCallbackReturn.FAILURE

        self._policy.activate()
        self._start_timers()

        # rclpy lifecycle transitions publishers to active for managed pubs;
        # we created plain publishers (not LifecyclePublisher) so nothing to do.
        super().on_activate(state)
        self.get_logger().info("ZLAC chassis ACTIVE — accepting /cmd_vel")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._stop_timers()
        if self._driver is not None:
            self._driver.stop_motors()
            try:
                self._driver.disable()
            except ZLACDriverError as e:
                self.get_logger().warn(f"deactivate: disable failed: {e}")

        if self._policy is not None:
            self._policy.deactivate()

        super().on_deactivate(state)
        self.get_logger().info("ZLAC chassis INACTIVE")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        self._destroy_endpoints()
        self._safe_close_driver()
        if self._policy is not None:
            self._policy.cleanup()
            self._policy = None
        self.get_logger().info("ZLAC chassis UNCONFIGURED")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, _state: State) -> TransitionCallbackReturn:
        self._stop_timers()
        self._destroy_endpoints()
        if self._driver is not None:
            self._driver.stop_motors()
            try:
                self._driver.disable()
            except ZLACDriverError:
                pass
        self._safe_close_driver()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, _state: State) -> TransitionCallbackReturn:
        # Latch into fault so the next activate is forced through a manual recover.
        if self._driver is not None:
            try:
                self._driver.emergency_stop()
                self._driver.disable()
            except ZLACDriverError:
                pass
        if self._policy is not None:
            self._policy.on_fault(code=0xFFFF)
        return TransitionCallbackReturn.SUCCESS

    # ================================================== internals ========

    def _build_chassis_parameters(self) -> ChassisParameters:
        gp = self.get_parameter
        return ChassisParameters(
            wheel_diameter=float(gp("wheel_diameter").value),
            wheel_base=float(gp("wheel_base").value),
            gear_ratio=float(gp("gear_ratio").value),
            max_motor_rpm=int(gp("max_motor_rpm").value),
            max_linear_velocity=float(gp("max_linear_velocity").value),
            max_angular_velocity=float(gp("max_angular_velocity").value),
            cmd_timeout=float(gp("cmd_timeout").value),
            left_sign=int(gp("left_sign").value),
            right_sign=int(gp("right_sign").value),
        )

    def _create_endpoints(self) -> None:
        self._cmd_sub = self.create_subscription(
            Twist, TopicContract.FINAL_CMD_VEL, self._on_cmd_vel, _RELIABLE
        )
        self._pub_encoders = self.create_publisher(
            Int64MultiArray, "/wheel_encoders", _BEST_EFFORT
        )
        self._pub_state = self.create_publisher(UInt8, "/chassis/state", _RELIABLE)
        self._pub_fault = self.create_publisher(UInt32, "/chassis/fault", _RELIABLE)
        self._pub_watchdog = self.create_publisher(
            Bool, "/chassis/cmd_watchdog", _RELIABLE
        )
        self._pub_wheel_rpm = self.create_publisher(
            Int16MultiArray, "/chassis/wheel_rpm", _BEST_EFFORT
        )

    def _destroy_endpoints(self) -> None:
        if self._cmd_sub is not None:
            self.destroy_subscription(self._cmd_sub)
            self._cmd_sub = None
        for attr in (
            "_pub_encoders",
            "_pub_state",
            "_pub_fault",
            "_pub_watchdog",
            "_pub_wheel_rpm",
        ):
            pub = getattr(self, attr)
            if pub is not None:
                self.destroy_publisher(pub)
                setattr(self, attr, None)

    def _start_timers(self) -> None:
        self._control_timer = self.create_timer(self.CONTROL_PERIOD_S, self._on_control_tick)
        self._slow_timer = self.create_timer(self.SLOW_PUBLISH_PERIOD_S, self._on_slow_publish)
        self._rpm_timer = self.create_timer(self.WHEEL_RPM_PUBLISH_PERIOD_S, self._on_rpm_publish)

    def _stop_timers(self) -> None:
        for t in (self._control_timer, self._slow_timer, self._rpm_timer):
            if t is not None:
                self.destroy_timer(t)
        self._control_timer = None
        self._slow_timer = None
        self._rpm_timer = None

    def _safe_close_driver(self) -> None:
        if self._driver is not None:
            self._driver.close()
            self._driver = None

    # -------------------------------------------------- ROS callbacks ----

    def _on_cmd_vel(self, msg: Twist) -> None:
        if self._policy is None:
            return
        now = self._now_seconds()
        self._policy.on_command(linear=msg.linear.x, angular=msg.angular.z, now=now)

    def _on_control_tick(self) -> None:
        if self._policy is None or self._driver is None:
            return

        now = self._now_seconds()
        targets = self._policy.tick(now=now)

        try:
            self._driver.write_target_rpm(targets.left_rpm, targets.right_rpm)
            fb = self._driver.read_feedback()
            self._last_feedback = fb
        except ZLACDriverError as e:
            self.get_logger().warn(f"control tick I/O: {e}", throttle_duration_sec=2.0)
            return

        if fb.has_fault and self._policy.state != ChassisState.FAULT:
            self.get_logger().error(
                f"driver reported fault 0x{fb.fault_code:04X} — latching FAULT"
            )
            try:
                self._driver.disable()
            except ZLACDriverError:
                pass
            self._policy.on_fault(code=fb.fault_code)

        # Publish encoder counts under the existing contract so odometry_node
        # consumes them unchanged.
        enc = Int64MultiArray()
        enc.data = [int(fb.position_left), int(fb.position_right)]
        self._pub_encoders.publish(enc)

    def _on_slow_publish(self) -> None:
        if self._policy is None:
            return
        st = UInt8()
        st.data = self._policy.state.value
        self._pub_state.publish(st)

        fault = UInt32()
        fault.data = int(self._policy.fault_code)
        self._pub_fault.publish(fault)

        wd = Bool()
        wd.data = self._policy.state == ChassisState.ACTIVE
        self._pub_watchdog.publish(wd)

    def _on_rpm_publish(self) -> None:
        if self._last_feedback is None:
            return
        msg = Int16MultiArray()
        msg.data = [
            int(self._last_feedback.actual_rpm_left),
            int(self._last_feedback.actual_rpm_right),
        ]
        self._pub_wheel_rpm.publish(msg)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ZLACChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
