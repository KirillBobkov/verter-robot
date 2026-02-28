"""Odometry use-case orchestrating domain odometry policy."""

from dataclasses import dataclass
import math

from verter_admin.control.domain import (
    OdometryParameters,
    OdometrySource,
    OdometryState,
    clamp_cmd_velocity,
    compute_encoder_delta,
    integrate_cmd_vel_pose,
    normalize_angle,
)


@dataclass(frozen=True)
class OdometryEvent:
    """Lifecycle and transition markers for adapter logging."""

    movement_started: bool = False
    movement_stopped: bool = False
    switched_to_encoder: bool = False
    restored_encoder: bool = False
    switched_to_cmd_vel: bool = False
    large_dt: bool = False


@dataclass(frozen=True)
class OdometrySnapshot:
    """Current kinematic state for ROS publication."""

    x: float
    y: float
    theta: float
    vx: float
    vy: float
    vth: float
    source: OdometrySource
    current_time: float


class ComputeOdometry:
    """Stateful odometry use-case independent from ROS transport."""

    def __init__(self, params: OdometryParameters, now_sec: float):
        self._params = params
        self._state = OdometryState(
            last_time=now_sec,
            current_time=now_sec,
            last_encoder_time=now_sec,
        )
        self._is_moving = False

    def on_cmd_vel(self, linear_x: float, angular_z: float) -> OdometryEvent:
        if self._state.source != OdometrySource.ENCODER:
            self._state.vx, self._state.vth, self._state.vy = clamp_cmd_velocity(
                linear_x,
                angular_z,
                self._params,
            )

        moving_now = abs(linear_x) > 0.01 or abs(angular_z) > 0.01
        movement_started = moving_now and not self._is_moving
        movement_stopped = (not moving_now) and self._is_moving
        self._is_moving = moving_now
        return OdometryEvent(
            movement_started=movement_started,
            movement_stopped=movement_stopped,
        )

    def on_encoder(self, left_steps: int, right_steps: int, now_sec: float) -> OdometryEvent:
        self._state.last_encoder_time = now_sec

        if not self._state.encoder_initialized:
            self._state.last_left_steps = left_steps
            self._state.last_right_steps = right_steps
            self._state.last_encoder_callback_time = now_sec
            self._state.encoder_initialized = True
            self._state.source = OdometrySource.ENCODER
            return OdometryEvent(switched_to_encoder=True)

        restored_encoder = False
        if self._state.source != OdometrySource.ENCODER:
            self._state.source = OdometrySource.ENCODER
            restored_encoder = True

        delta_left = left_steps - int(self._state.last_left_steps)
        delta_right = right_steps - int(self._state.last_right_steps)
        dt = now_sec - float(self._state.last_encoder_callback_time)

        if dt <= 0 or dt > 1.0:
            self._state.last_left_steps = left_steps
            self._state.last_right_steps = right_steps
            self._state.last_encoder_callback_time = now_sec
            return OdometryEvent(restored_encoder=restored_encoder)

        if delta_left == 0 and delta_right == 0:
            self._state.vx = 0.0
            self._state.vth = 0.0
            self._state.vy = 0.0
            self._state.last_encoder_callback_time = now_sec
            return OdometryEvent(restored_encoder=restored_encoder)

        delta_distance, delta_theta = compute_encoder_delta(delta_left, delta_right, self._params)
        self._state.vx = delta_distance / dt
        self._state.vth = delta_theta / dt
        self._state.vy = 0.0

        avg_theta = self._state.theta + delta_theta / 2.0
        self._state.x += delta_distance * math.cos(avg_theta)
        self._state.y += delta_distance * math.sin(avg_theta)
        self._state.theta = normalize_angle(self._state.theta + delta_theta)

        self._state.last_left_steps = left_steps
        self._state.last_right_steps = right_steps
        self._state.last_encoder_callback_time = now_sec
        return OdometryEvent(restored_encoder=restored_encoder)

    def on_tick(self, now_sec: float) -> OdometryEvent:
        switched_to_cmd_vel = False
        if self._state.source == OdometrySource.ENCODER:
            if now_sec - self._state.last_encoder_time > self._params.encoder_timeout:
                self._state.source = OdometrySource.CMD_VEL
                switched_to_cmd_vel = True

        dt = now_sec - self._state.last_time
        large_dt = False
        if dt > 1.0:
            dt = 0.02
            large_dt = True

        # During CMD_VEL fallback: freeze position.
        # Integrating from cmd_vel would cause double-counting when encoder
        # returns (encoder delta already captures movement during the gap).
        # High covariance (published by the ROS adapter) signals EKF to trust
        # scan matching over odometry while position is frozen.

        self._state.last_time = now_sec
        self._state.current_time = now_sec
        return OdometryEvent(
            switched_to_cmd_vel=switched_to_cmd_vel,
            large_dt=large_dt,
        )

    def snapshot(self) -> OdometrySnapshot:
        return OdometrySnapshot(
            x=self._state.x,
            y=self._state.y,
            theta=self._state.theta,
            vx=self._state.vx,
            vy=self._state.vy,
            vth=self._state.vth,
            source=self._state.source,
            current_time=self._state.current_time,
        )
