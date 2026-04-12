"""Encoder-based differential drive odometry use-case.

Clean-slate implementation: pure encoder integration, no cmd_vel fallback,
no impossible-delta filters (those existed to compensate for firmware I2C bugs
that are now handled at the firmware level).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from verter_admin.control.domain.odometry_policy import (
    OdometryParameters,
    compute_encoder_delta,
    normalize_angle,
)


@dataclass(frozen=True)
class OdometrySnapshot:
    """Kinematic state snapshot for ROS publication."""

    x: float
    y: float
    theta: float
    vx: float        # forward velocity  (m/s)
    vth: float       # angular velocity  (rad/s)
    encoder_fresh: bool  # False when encoder timed out → inflate covariance


class ComputeOdometry:
    """Stateful encoder odometry use-case, independent of ROS transport.

    Call sequence every cycle:
        on_encoder() — when a new /wheel_encoders message arrives
        on_tick()    — at the publication timer rate (e.g. 50 Hz)
        snapshot()   — to read the current state for publishing
    """

    def __init__(self, params: OdometryParameters, now_sec: float) -> None:
        self._params = params

        self._x: float = 0.0
        self._y: float = 0.0
        self._theta: float = 0.0
        self._vx: float = 0.0
        self._vth: float = 0.0

        self._initialized: bool = False
        self._last_left: int = 0
        self._last_right: int = 0
        self._last_enc_sec: float = now_sec
        self._encoder_fresh: bool = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def on_encoder(self, left_steps: int, right_steps: int, now_sec: float) -> None:
        """Integrate a new encoder reading into the pose estimate."""

        if not self._initialized:
            self._last_left = left_steps
            self._last_right = right_steps
            self._last_enc_sec = now_sec
            self._initialized = True
            self._encoder_fresh = True
            return

        dt = now_sec - self._last_enc_sec
        if dt <= 0.0:
            # Duplicate or out-of-order message — ignore silently.
            return

        delta_left  = left_steps  - self._last_left
        delta_right = right_steps - self._last_right

        delta_dist, delta_theta = compute_encoder_delta(
            delta_left, delta_right, self._params
        )

        # Trapezoidal (mid-point) integration — more accurate than forward Euler
        avg_theta    = self._theta + delta_theta / 2.0
        self._x     += delta_dist * math.cos(avg_theta)
        self._y     += delta_dist * math.sin(avg_theta)
        self._theta  = normalize_angle(self._theta + delta_theta)

        self._vx  = delta_dist  / dt
        self._vth = delta_theta / dt

        self._last_left    = left_steps
        self._last_right   = right_steps
        self._last_enc_sec = now_sec
        self._encoder_fresh = True

    def on_tick(self, now_sec: float) -> None:
        """Called at the publication timer rate.

        When encoder data stops arriving (I2C hang or disconnected ESP32),
        velocity is zeroed so the ROS consumer sees a stopped robot.
        The adapter should inflate covariance when encoder_fresh is False.
        """

        if not self._initialized:
            return

        gap = now_sec - self._last_enc_sec
        if gap > self._params.encoder_timeout:
            self._encoder_fresh = False
            self._vx  = 0.0
            self._vth = 0.0

    def snapshot(self) -> OdometrySnapshot:
        return OdometrySnapshot(
            x=self._x,
            y=self._y,
            theta=self._theta,
            vx=self._vx,
            vth=self._vth,
            encoder_fresh=self._encoder_fresh,
        )
