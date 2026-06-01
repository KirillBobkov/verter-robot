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
WHEEL_DIAMETER_M = 0.200
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
                   help="measure wheel_base from a known rotation angle")
    args = p.parse_args()

    handlers = {
        "forward-check": cmd_forward_check,
        "encoder":       cmd_encoder,
        "wheelbase":     cmd_wheelbase,
    }
    return handlers[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
