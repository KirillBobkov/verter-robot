#!/usr/bin/env python3
"""Interactive calibration of chassis kinematic constants.

Subscribes to /wheel_encoders, takes snapshots before/after a user-driven
maneuver, asks for the measured real-world distance/angle, and prints the
resulting constant.  No publishing — drive the robot from a separate terminal
(teleop, rqt, or `ros2 topic pub /cmd_vel ...`).

Pre-requisites:
  * chassis_bringup.launch.py is running and state=ACTIVE responds to /cmd_vel.
  * Firmware has the encoder sign fix (so forward motion makes BOTH encoder
    values increase).  Verify quickly: drive forward, run `forward-check`
    subcommand below — both deltas should be positive and similar.

Subcommands:
  forward-check    Print encoder deltas while you drive forward briefly
                   (sanity test BEFORE the real calibration).
  encoder          Measure encoder_resolution by driving straight a known
                   distance (recommend 2-3 m for accuracy).
  wheelbase        Measure wheel_base by rotating in place a known angle
                   (recommend a full 360° for accuracy).

Usage:
  cd ~/verter-robot/verter_admin/scripts
  python3 chassis_calibrate.py forward-check
  python3 chassis_calibrate.py encoder
  python3 chassis_calibrate.py wheelbase

Both calibrations are independent.  Recommended order:
  1. forward-check (sanity)
  2. encoder       (gives meters_per_tick — needed before wheelbase)
  3. wheelbase     (uses encoder_resolution from step 2)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Int64MultiArray
except ImportError:
    sys.stderr.write("ERROR: rclpy not available.  Source your ROS 2 setup first:\n"
                     "  source /opt/ros/humble/setup.bash\n"
                     "  source ~/ros2_ws/install/setup.bash\n")
    sys.exit(2)


# Wheel diameter must match firmware constant. Change here only if you change
# both in the firmware and in odometry_policy.py.
# 0.196 m = effective rolling diameter under load (tire compression vs the
# geometric 200 mm of ZLLG80ASM250).
WHEEL_DIAMETER_M = 0.196
WHEEL_CIRCUMFERENCE_M = math.pi * WHEEL_DIAMETER_M


# ---------------------------------------------------------------- ROS glue


class EncoderTap(Node):
    """Latches the latest /wheel_encoders message for on-demand reads."""

    def __init__(self) -> None:
        super().__init__("chassis_calibrate")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._latest: tuple[int, int] | None = None
        self._msg_count = 0
        self.create_subscription(
            Int64MultiArray, "/wheel_encoders", self._on_enc, qos
        )

    def _on_enc(self, msg: Int64MultiArray) -> None:
        if len(msg.data) >= 2:
            self._latest = (int(msg.data[0]), int(msg.data[1]))
            self._msg_count += 1

    def wait_for_first(self, timeout: float = 5.0) -> tuple[int, int]:
        deadline = time.monotonic() + timeout
        while self._latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._latest is None:
            print("ERROR: no /wheel_encoders received in 5 s.")
            print("       Is chassis_bringup.launch.py running and state=ACTIVE?")
            sys.exit(2)
        return self._latest

    def snapshot(self) -> tuple[int, int]:
        """Force a fresh read by spinning briefly."""
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        assert self._latest is not None
        return self._latest


def _prompt_float(label: str) -> float:
    while True:
        s = input(label).strip()
        try:
            return float(s)
        except ValueError:
            print("  not a number, try again.")


# ----------------------------------------------------------- subcommands


def cmd_forward_check(_args: argparse.Namespace) -> int:
    rclpy.init()
    tap = EncoderTap()
    start = tap.wait_for_first()

    print()
    print("===== Forward-check =====")
    print(f"Start: L={start[0]}  R={start[1]}")
    print()
    print("Drive the robot FORWARD a short distance (≈ 0.5 m).")
    print("In another terminal:")
    print('  ros2 topic pub -r 10 -t 30 /cmd_vel geometry_msgs/msg/Twist \\')
    print('      "{linear: {x: 0.2}, angular: {z: 0.0}}"')
    print()
    input("Press ENTER after the robot has stopped ...")

    end = tap.snapshot()
    dl = end[0] - start[0]
    dr = end[1] - start[1]
    print(f"End:   L={end[0]}  R={end[1]}")
    print(f"Δ      L={dl:+d}  R={dr:+d}")
    print()

    if dl > 0 and dr > 0:
        ratio = max(dl, dr) / max(1, min(dl, dr))
        print(f"OK — both encoders increased.  L/R ratio = {ratio:.3f}")
        if ratio > 1.10:
            print("WARN: deltas differ by more than 10%.  Possible causes:")
            print("  - wheel slippage during the run")
            print("  - one motor weaker than the other under load")
            print("  - mechanical drag on one side")
    else:
        signs = ("+" if dl > 0 else ("-" if dl < 0 else "0"),
                 "+" if dr > 0 else ("-" if dr < 0 else "0"))
        print(f"FAIL — sign(ΔL)={signs[0]}, sign(ΔR)={signs[1]}.  Expected both '+'.")
        print("Action: the firmware encoder-sign fix is missing or wrong direction.")
        print("        Re-flash the firmware and verify LEFT_SIGN/RIGHT_SIGN.")
    rclpy.shutdown()
    return 0


def cmd_encoder(_args: argparse.Namespace) -> int:
    rclpy.init()
    tap = EncoderTap()
    start = tap.wait_for_first()

    print()
    print("===== Encoder-resolution calibration =====")
    print(f"Wheel diameter assumed: {WHEEL_DIAMETER_M:.3f} m "
          f"(circumference {WHEEL_CIRCUMFERENCE_M:.4f} m)")
    print(f"Start encoder: L={start[0]}  R={start[1]}")
    print()
    print("1. Mark the robot's current position on the floor (e.g., tape).")
    print("2. Drive the robot FORWARD in a straight line for 2-3 m.")
    print("   In another terminal:")
    print('     ros2 topic pub -r 10 -t 100 /cmd_vel geometry_msgs/msg/Twist \\')
    print('         "{linear: {x: 0.2}, angular: {z: 0.0}}"')
    print("   (or use any teleop).")
    print("3. Stop the robot and mark the final position on the floor.")
    print("4. Measure the actual distance between marks with a tape measure.")
    print()
    input("Press ENTER after stopping ...")

    end = tap.snapshot()
    dl = end[0] - start[0]
    dr = end[1] - start[1]
    print(f"End encoder:   L={end[0]}  R={end[1]}")
    print(f"Δ encoder:     L={dl:+d}  R={dr:+d}")

    if dl <= 0 or dr <= 0:
        print()
        print("ERROR: one or both encoder deltas are non-positive — that means")
        print("the robot was either driven backward, didn't move, or the encoder")
        print("sign convention is wrong.  Re-flash with the encoder sign fix and")
        print("run forward-check first.")
        rclpy.shutdown()
        return 1

    print()
    distance = _prompt_float("Measured straight-line distance in meters (e.g. 2.0): ")
    if distance <= 0:
        print("ERROR: distance must be positive.")
        rclpy.shutdown()
        return 1

    avg_delta = (dl + dr) / 2.0
    ticks_per_meter = avg_delta / distance
    meters_per_tick = distance / avg_delta
    encoder_resolution = WHEEL_CIRCUMFERENCE_M * ticks_per_meter

    print()
    print("===== Result =====")
    print(f"Avg ticks for {distance:.3f} m: {avg_delta:.1f}")
    print(f"ticks_per_meter   = {ticks_per_meter:.3f}")
    print(f"meters_per_tick   = {meters_per_tick:.6e}")
    print(f"encoder_resolution = {round(encoder_resolution)} "
          f"(ticks per wheel revolution)")
    print()
    print("Apply to verter_admin/src/verter_admin/control/domain/odometry_policy.py:")
    print(f"    encoder_resolution: int = {round(encoder_resolution)}")
    print()
    print(f"L/R asymmetry: {abs(dl - dr) / max(dl, dr) * 100:.1f}%")
    print("(>5% usually means floor friction was uneven during the run.")
    print(" Repeat the calibration with a smoother floor / longer distance.)")

    rclpy.shutdown()
    return 0


class _AutoRotator(Node):
    """Drives a closed-loop pure rotation: publishes /cmd_vel, watches
    /wheel_encoders, stops when the *estimated* yaw reaches the target.

    Estimated yaw is computed from (right_ticks - left_ticks) * meters_per_tick
    / wheel_base_estimate. If the estimate is wrong, the physical angle the
    robot ends up at will differ from the target — that mismatch is exactly
    what we use to correct wheel_base afterwards.
    """

    PHASE_WAITING = "waiting"
    PHASE_ROTATING = "rotating"
    PHASE_SETTLING = "settling"
    PHASE_DONE = "done"

    def __init__(
        self,
        target_rad: float,
        angular_velocity: float,
        wb_estimate: float,
        meters_per_tick: float,
        settle_secs: float = 2.0,
    ) -> None:
        super().__init__("chassis_wheelbase_auto")
        # Direction: angular_velocity sign follows target sign so we always
        # converge instead of running away from the target.
        self._omega = abs(angular_velocity) * (1.0 if target_rad >= 0 else -1.0)
        self._target_rad = target_rad
        self._wb_estimate = wb_estimate
        self._mpt = meters_per_tick
        self._settle_secs = settle_secs

        self._start_enc: tuple[int, int] | None = None
        self._latest_enc: tuple[int, int] | None = None
        self._phase = self.PHASE_WAITING
        self._settle_t0: float | None = None
        self._last_log = 0.0

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_re = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(Int64MultiArray, "/wheel_encoders",
                                 self._on_enc, qos_be)
        self._pub = self.create_publisher(Twist, "/cmd_vel", qos_re)
        # 10 Hz control loop — well within the firmware's 500 ms cmd_vel
        # watchdog window.
        self.create_timer(0.1, self._tick)

    @property
    def phase(self) -> str:
        return self._phase

    def estimated_rad(self) -> float:
        if self._start_enc is None or self._latest_enc is None:
            return 0.0
        dl = self._latest_enc[0] - self._start_enc[0]
        dr = self._latest_enc[1] - self._start_enc[1]
        return (dr - dl) * self._mpt / self._wb_estimate

    def deltas(self) -> tuple[int, int]:
        assert self._start_enc is not None and self._latest_enc is not None
        return (
            self._latest_enc[0] - self._start_enc[0],
            self._latest_enc[1] - self._start_enc[1],
        )

    def emergency_stop(self) -> None:
        # Send a few zero-twist messages to overcome best-effort drops.
        for _ in range(5):
            self._pub.publish(Twist())
            time.sleep(0.02)

    def _on_enc(self, msg: Int64MultiArray) -> None:
        if len(msg.data) >= 2:
            self._latest_enc = (int(msg.data[0]), int(msg.data[1]))

    def _tick(self) -> None:
        now = time.monotonic()
        twist = Twist()

        if self._phase == self.PHASE_WAITING:
            if self._latest_enc is not None:
                self._start_enc = self._latest_enc
                self._phase = self.PHASE_ROTATING
                print(f"Start encoder: L={self._start_enc[0]}  R={self._start_enc[1]}")
                print("Rotating ...")
            return

        if self._phase == self.PHASE_ROTATING:
            est = self.estimated_rad()
            if (now - self._last_log) > 1.0:
                self._last_log = now
                deg = math.degrees(est)
                tdeg = math.degrees(self._target_rad)
                print(f"  est {deg:+6.1f}° / target {tdeg:+6.1f}°")
            if abs(est) >= abs(self._target_rad):
                twist.angular.z = 0.0
                self._pub.publish(twist)
                self._phase = self.PHASE_SETTLING
                self._settle_t0 = now
                print(f"  target reached at est {math.degrees(est):+.2f}°"
                      f"  — settling for {self._settle_secs:.1f} s ...")
            else:
                twist.angular.z = float(self._omega)
                self._pub.publish(twist)
            return

        if self._phase == self.PHASE_SETTLING:
            twist.angular.z = 0.0
            self._pub.publish(twist)
            if self._settle_t0 is not None and (now - self._settle_t0) > self._settle_secs:
                self._phase = self.PHASE_DONE
            return


def cmd_wheelbase_auto(_args: argparse.Namespace) -> int:
    print()
    print("===== Wheel-base calibration (auto rotation) =====")
    print(f"Wheel diameter assumed: {WHEEL_DIAMETER_M:.3f} m "
          f"(circumference {WHEEL_CIRCUMFERENCE_M:.4f} m)")
    print()

    enc_res_str = input("encoder_resolution [4096]: ").strip() or "4096"
    encoder_resolution = float(enc_res_str)
    wb_str = input("wheel_base estimate [0.386]: ").strip() or "0.386"
    wb_estimate = float(wb_str)
    target_deg_str = input("target rotation in degrees [360]: ").strip() or "360"
    target_deg = float(target_deg_str)
    omega_str = input("angular velocity rad/s [0.5]: ").strip() or "0.5"
    omega = float(omega_str)

    if encoder_resolution <= 0 or wb_estimate <= 0 or omega <= 0:
        print("ERROR: encoder_resolution, wheel_base, omega must all be > 0.")
        return 1
    if target_deg == 0:
        print("ERROR: target_deg must be non-zero.")
        return 1

    meters_per_tick = WHEEL_CIRCUMFERENCE_M / encoder_resolution
    expected_secs = abs(math.radians(target_deg) / omega)

    print()
    print(f"Will rotate IN PLACE at ω = {omega:.2f} rad/s")
    print(f"Target: {target_deg:+.1f}°  (estimated duration ≈ {expected_secs:.1f} s)")
    print()
    print("SAFETY:")
    print("  - Clear 1 m around the robot.")
    print("  - Floor must not be slippery.")
    print("  - Mark a tape arrow under the front of the robot — you will")
    print("    measure how many degrees the arrow turned vs. its mark.")
    print("  - Keep hand on E-stop / main switch.")
    print()
    input("Press ENTER to start rotation ...")

    rclpy.init()
    node = _AutoRotator(
        target_rad=math.radians(target_deg),
        angular_velocity=omega,
        wb_estimate=wb_estimate,
        meters_per_tick=meters_per_tick,
    )

    # Spin until the rotator finishes its phase machine, with a hard cap.
    timeout = time.monotonic() + expected_secs * 4 + 10.0
    try:
        while node.phase != _AutoRotator.PHASE_DONE:
            if time.monotonic() > timeout:
                print("TIMEOUT — aborting and stopping the robot.")
                node.emergency_stop()
                rclpy.shutdown()
                return 1
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        print("\nInterrupted — stopping the robot.")
        node.emergency_stop()
        rclpy.shutdown()
        return 130

    dl, dr = node.deltas()
    estimated_final = node.estimated_rad()
    node.emergency_stop()
    rclpy.shutdown()

    print()
    print(f"Δ encoder:       L={dl:+d}  R={dr:+d}")
    print(f"Estimated angle: {math.degrees(estimated_final):+.2f}° "
          f"(by current wheel_base estimate {wb_estimate:.4f} m)")
    print()
    print("Now MEASURE THE PHYSICAL ANGLE the robot ended up rotated.")
    print("Compare the front-of-robot mark to its original orientation.")
    print("If the robot did multiple full revolutions, count them all")
    print("  (e.g. '1 full turn + 8° extra' → enter 368).")
    print("Direction matters — if the robot turned OPPOSITE to what was")
    print("  commanded, enter the angle with a negative sign.")
    print()
    actual_deg = _prompt_float("Actual measured angle in degrees: ")
    if actual_deg == 0:
        print("ERROR: actual angle cannot be zero.")
        return 1
    actual_rad = math.radians(actual_deg)

    arc_diff_m = (dr - dl) * meters_per_tick
    wb_corrected = abs(arc_diff_m / actual_rad)
    correction = wb_corrected / wb_estimate

    print()
    print("===== Result =====")
    print(f"Arc difference     = {arc_diff_m:+.4f} m")
    print(f"Actual angle       = {actual_rad:+.4f} rad ({actual_deg:+.2f}°)")
    print(f"wheel_base (corrected) = {wb_corrected:.4f} m")
    print(f"wheel_base (estimate)  = {wb_estimate:.4f} m")
    print(f"correction factor      = ×{correction:.4f}  "
          f"({(correction - 1) * 100:+.2f}%)")
    print()
    print("Apply to:")
    print("  firmware (esp32_chassis_modbus.ino):")
    print(f"      static constexpr float WHEEL_BASE = {wb_corrected:.4f}f;")
    print("  odometry (control/domain/odometry_policy.py):")
    print(f"      wheel_base: float = {wb_corrected:.4f}")
    print()
    if 0.95 < correction < 1.05:
        print("Correction < 5 % — initial estimate was already close.")
    elif 0.80 < correction < 1.20:
        print("Correction 5-20 % — typical for a frame change. Apply and "
              "optionally re-run for refinement.")
    else:
        print("Correction > 20 % — suspect: angle measurement error, wheel"
              " slip on slippery floor, or wrong encoder_resolution. Re-run"
              " on a non-slippery surface and double-check the angle.")
    return 0


def cmd_wheelbase(_args: argparse.Namespace) -> int:
    rclpy.init()
    tap = EncoderTap()
    start = tap.wait_for_first()

    print()
    print("===== Wheel-base calibration =====")
    print(f"Start encoder: L={start[0]}  R={start[1]}")
    print()
    encoder_resolution = _prompt_float(
        "encoder_resolution from previous calibration (e.g. 4096): "
    )
    if encoder_resolution <= 0:
        print("ERROR: encoder_resolution must be positive.")
        rclpy.shutdown()
        return 1
    meters_per_tick = WHEEL_CIRCUMFERENCE_M / encoder_resolution
    print(f"→ meters_per_tick = {meters_per_tick:.6e} m")
    print()

    print("1. Mark the robot's current orientation (front-of-robot line on floor).")
    print("2. Rotate the robot IN PLACE by a known angle.  Recommend a full")
    print("   360° turn or 180° if space is tight.  In another terminal:")
    print('     ros2 topic pub -r 10 -t 80 /cmd_vel geometry_msgs/msg/Twist \\')
    print('         "{linear: {x: 0.0}, angular: {z: 0.5}}"')
    print("3. Stop, measure actual rotation angle (mark on floor + protractor,")
    print("   or count revolutions if you turned a full multiple of 360°).")
    print()
    input("Press ENTER after stopping ...")

    end = tap.snapshot()
    dl = end[0] - start[0]
    dr = end[1] - start[1]
    print(f"End encoder:   L={end[0]}  R={end[1]}")
    print(f"Δ encoder:     L={dl:+d}  R={dr:+d}")
    print()

    if dl * dr >= 0:
        print("WARN: pure rotation should produce OPPOSITE-sign deltas (one")
        print("      wheel rolling forward, one rolling backward).  Got same")
        print("      signs — either rotation wasn't pure, or the robot drove")
        print("      forward/backward instead of spinning.  Repeat with")
        print("      linear.x = 0.0 and a clear angular.z.")

    angle_deg = _prompt_float("Measured rotation angle in degrees (e.g. 360.0): ")
    if angle_deg == 0:
        print("ERROR: angle cannot be zero.")
        rclpy.shutdown()
        return 1
    angle_rad = math.radians(angle_deg)

    # Pure rotation kinematics:
    #   right wheel arc - left wheel arc = wheel_base · θ
    # → wheel_base = (Δr - Δl) · meters_per_tick / θ
    # Use signed deltas so a CCW (positive θ) rotation with right-forward gives
    # (Δr - Δl) > 0, matching the sign of θ.
    arc_diff_m = (dr - dl) * meters_per_tick
    if arc_diff_m * angle_rad < 0:
        print()
        print("NOTE: sign(Δr-Δl) and sign(angle) differ.  Either you turned the")
        print("      opposite direction from what `angular.z` commanded, or one")
        print("      of LEFT_SIGN/RIGHT_SIGN is still wrong.  Taking absolute value.")
        wheel_base = abs(arc_diff_m) / abs(angle_rad)
    else:
        wheel_base = arc_diff_m / angle_rad

    print()
    print("===== Result =====")
    print(f"Δr - Δl = {dr - dl:+d} ticks")
    print(f"Arc difference  = {arc_diff_m:.4f} m")
    print(f"Rotation angle  = {angle_rad:.4f} rad ({angle_deg:.1f}°)")
    print(f"wheel_base      = {wheel_base:.4f} m")
    print()
    print("Apply to:")
    print("  1. firmware (esp32_chassis_modbus.ino):")
    print(f"       static constexpr float WHEEL_BASE = {wheel_base:.4f}f;")
    print("  2. odometry (control/domain/odometry_policy.py):")
    print(f"       wheel_base: float = {wheel_base:.4f}")
    print()

    rclpy.shutdown()
    return 0


# ---------------------------------------------------------------- main


def main() -> int:
    p = argparse.ArgumentParser(prog="chassis_calibrate")
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("forward-check",
                   help="verify both encoders increase on forward motion")
    sub.add_parser("encoder",
                   help="measure encoder_resolution from a straight-line drive")
    sub.add_parser("wheelbase",
                   help="measure wheel_base from a known rotation angle (manual)")
    sub.add_parser("wheelbase-auto",
                   help="rotate the robot automatically to a target angle, "
                        "then ask the operator for the actual angle observed")
    args = p.parse_args()

    handlers = {
        "forward-check":  cmd_forward_check,
        "encoder":        cmd_encoder,
        "wheelbase":      cmd_wheelbase,
        "wheelbase-auto": cmd_wheelbase_auto,
    }
    return handlers[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
