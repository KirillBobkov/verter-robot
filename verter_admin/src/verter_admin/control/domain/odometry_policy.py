"""Pure odometry domain primitives and math (no ROS dependencies)."""

from dataclasses import dataclass
from enum import Enum
import math


class OdometrySource(Enum):
    """Runtime source used to update odometry state."""

    CMD_VEL = 'cmd_vel'
    ENCODER = 'encoder'


@dataclass(frozen=True)
class OdometryParameters:
    """Robot and policy constants for odometry integration.

    Calibrated on 2026-06-02 using scripts/chassis_calibrate.py encoder.
    Seven independent straight-line runs (0.66 m … 2.03 m) converged on:

      * meters_per_tick ≈ 1.503e-4 m
      * encoder PPR     = 4096          (round, matches typical 1024 cpr × 4)
      * rolling Ø       ≈ 0.196 m       (5 mm less than geometric 200 mm due
                                          to tire compression under load)
      * L/R asymmetry   < 0.5 %         (driving straight is straight)

    Two runs at d=0.196 in firmware gave encoder_resolution = 4093 and 4094
    (0.04 % apart) — strong evidence the physical encoder is exactly 4096
    and the residual error is tape-measure noise on the floor marks.

    `meters_per_step` is the only kinematic constant odometry actually uses;
    it is now correct to ≤ 0.5 % of distance traveled — well below SLAM /
    Nav2 tolerance.

    `wheel_base` is still a carry-over from the old chassis frame; calibrate
    it via `scripts/chassis_calibrate.py wheelbase` before relying on yaw
    in odom-only operation.
    """

    wheel_circumference: float = math.pi * 0.196     # 0.6158 m, rolling-loaded
    gear_ratio: float = 1.0                          # direct-drive hub motor
    encoder_resolution: int = 4096                   # confirmed by 7 runs
    wheel_base: float = 0.386                        # TODO calibrate
    max_linear_velocity: float = 0.5
    max_angular_velocity: float = 1.0
    linear_scale: float = 1.0
    angular_scale: float = 1.0
    encoder_timeout: float = 1.0

    @property
    def meters_per_step(self) -> float:
        return self.wheel_circumference / (self.encoder_resolution * self.gear_ratio)


@dataclass
class OdometryState:
    """Mutable state snapshot used by odometry use-case."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vth: float = 0.0
    source: OdometrySource = OdometrySource.CMD_VEL
    encoder_initialized: bool = False
    last_left_steps: int | None = None
    last_right_steps: int | None = None
    last_time: float = 0.0
    current_time: float = 0.0
    last_encoder_time: float = 0.0
    last_encoder_callback_time: float | None = None


def clamp_cmd_velocity(
    linear_x: float,
    angular_z: float,
    params: OdometryParameters,
) -> tuple[float, float, float]:
    """Clamp and scale command velocities for open-loop fallback."""

    clamped_linear = max(min(linear_x, params.max_linear_velocity), -params.max_linear_velocity)
    clamped_angular = max(min(angular_z, params.max_angular_velocity), -params.max_angular_velocity)
    return (
        clamped_linear * params.linear_scale,
        clamped_angular * params.angular_scale,
        0.0,
    )


def normalize_angle(angle: float) -> float:
    """Normalize angle into [-pi, pi] range."""

    return math.atan2(math.sin(angle), math.cos(angle))


def integrate_cmd_vel_pose(
    x: float,
    y: float,
    theta: float,
    vx: float,
    vth: float,
    dt: float,
) -> tuple[float, float, float]:
    """Integrate differential-drive pose from command velocities."""

    delta_theta = vth * dt
    if abs(vth) < 1e-6:
        delta_x = vx * dt * math.cos(theta)
        delta_y = vx * dt * math.sin(theta)
    else:
        radius = vx / vth
        delta_x = radius * (math.sin(theta + delta_theta) - math.sin(theta))
        delta_y = -radius * (math.cos(theta + delta_theta) - math.cos(theta))

    return (
        x + delta_x,
        y + delta_y,
        normalize_angle(theta + delta_theta),
    )


def compute_encoder_delta(
    delta_left_steps: int,
    delta_right_steps: int,
    params: OdometryParameters,
) -> tuple[float, float]:
    """Convert encoder step deltas to traveled distance and heading delta."""

    dist_left = delta_left_steps * params.meters_per_step
    dist_right = delta_right_steps * params.meters_per_step
    delta_distance = (dist_left + dist_right) / 2.0
    delta_theta = (dist_right - dist_left) / params.wheel_base
    return delta_distance, delta_theta
