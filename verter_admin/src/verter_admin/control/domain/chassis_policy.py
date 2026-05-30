"""ChassisPolicy — pure domain for ZLAC8015D-based differential drive.

Zero ROS imports, zero Modbus imports. This module owns:
  * the chassis lifecycle state machine (UNCONFIGURED/INACTIVE/ACTIVE/SAFE_STOP/FAULT),
  * cmd_vel watchdog (drops to SAFE_STOP when commands stop arriving),
  * differential drive kinematics (cmd_vel → per-wheel m/s → motor RPM),
  * bounds enforcement (linear/angular and motor RPM clamps),
  * direction-sign mapping (left/right wheel rotation convention).

SDHR rulebook §2.2 dependency rule: domain MUST NOT import adapters/infrastructure.
The Modbus adapter and ROS lifecycle node call into this policy.

State machine:

    UNCONFIGURED ──configure()──▶ INACTIVE ──activate()──▶ ACTIVE
         ▲                          │ ▲                      │  ▲
         │                          │ │                      │  │
    cleanup() ◀───────deactivate()──┘ │                  cmd_fresh=F
         │                            │                      │  │
         │                       reset_fault()                ▼  │
         │                            │                  SAFE_STOP
         │                            │                      │  ▲
         └──────── ←──── FAULT ◀──────┴────── on_fault(code) ┘──┘
                                                cmd_fresh=T (→ ACTIVE)

Invariants enforced here:
  * `tick()` returns zero wheel targets in any state except ACTIVE with fresh command.
  * `on_command()` is silently ignored outside ACTIVE/SAFE_STOP (drops stale teleop).
  * `activate()` discards any cached command so the chassis waits for a fresh one.
  * RPM output is clamped to `params.max_motor_rpm` regardless of input.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum, auto


class ChassisState(Enum):
    UNCONFIGURED = auto()
    INACTIVE = auto()
    ACTIVE = auto()
    SAFE_STOP = auto()
    FAULT = auto()


@dataclass(frozen=True)
class ChassisParameters:
    """Robot kinematics + policy constants.

    Defaults reflect the ZLLG80ASM250-L hub motor (Ø 200 mm, direct drive, 200 RPM
    rated, 24 VDC) paired with the ZLAC8015D driver. `wheel_base` MUST be
    re-measured on the assembled chassis; left/right signs MUST be confirmed on
    bench with the wheels in the air (see scripts/zlac_probe.py).
    """

    wheel_diameter: float = 0.200          # m — ZLLG80ASM250 outer diameter
    wheel_base: float = 0.386              # m — TODO re-measure new chassis
    gear_ratio: float = 1.0                # direct-drive hub motor
    max_motor_rpm: int = 200               # rated RPM of the motor; hard ceiling
    max_linear_velocity: float = 0.5       # m/s — policy cap (≈ 48 RPM at Ø 0.2)
    max_angular_velocity: float = 1.0      # rad/s — policy cap
    cmd_timeout: float = 0.5               # s — matches MotionTimeouts.CHASSIS_WATCHDOG_MS
    left_sign: int = +1                    # TODO calibrate
    right_sign: int = +1                   # TODO calibrate

    @property
    def wheel_circumference(self) -> float:
        return math.pi * self.wheel_diameter

    def mps_to_motor_rpm(self, v_mps: float) -> float:
        """Convert wheel linear velocity [m/s] to motor RPM."""
        wheel_rpm = (v_mps / self.wheel_circumference) * 60.0
        return wheel_rpm * self.gear_ratio


@dataclass(frozen=True)
class WheelTargets:
    """Output of one policy tick: target motor RPMs for left and right channels.

    Signs already account for `left_sign`/`right_sign` direction calibration;
    the Modbus adapter writes these values verbatim to ZLAC8015D target-speed
    registers.
    """

    left_rpm: int
    right_rpm: int

    @classmethod
    def zero(cls) -> "WheelTargets":
        return cls(left_rpm=0, right_rpm=0)


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def cmd_vel_to_wheel_velocities(
    linear: float, angular: float, wheel_base: float
) -> tuple[float, float]:
    """Differential-drive inverse kinematics: (v_x, ω_z) → (v_left, v_right) in m/s."""
    half = wheel_base * 0.5
    v_left = linear - angular * half
    v_right = linear + angular * half
    return v_left, v_right


class ChassisPolicy:
    """Lifecycle + watchdog + kinematics for the chassis. Stateful, single-instance."""

    def __init__(self, params: ChassisParameters | None = None) -> None:
        self.params = params or ChassisParameters()
        self._state = ChassisState.UNCONFIGURED
        self._last_cmd_linear: float = 0.0
        self._last_cmd_angular: float = 0.0
        self._last_cmd_time: float | None = None
        self._fault_code: int = 0

    # ------------------------------------------------------------------ state

    @property
    def state(self) -> ChassisState:
        return self._state

    @property
    def fault_code(self) -> int:
        return self._fault_code

    @property
    def last_command_time(self) -> float | None:
        return self._last_cmd_time

    # -------------------------------------------------------- lifecycle ops

    def configure(self) -> bool:
        if self._state != ChassisState.UNCONFIGURED:
            return False
        self._state = ChassisState.INACTIVE
        return True

    def cleanup(self) -> bool:
        if self._state not in (ChassisState.INACTIVE, ChassisState.FAULT):
            return False
        self._clear_command()
        self._fault_code = 0
        self._state = ChassisState.UNCONFIGURED
        return True

    def activate(self) -> bool:
        if self._state != ChassisState.INACTIVE:
            return False
        # I5: discard stale command — require fresh /cmd_vel after activation.
        self._clear_command()
        self._state = ChassisState.ACTIVE
        return True

    def deactivate(self) -> bool:
        if self._state not in (ChassisState.ACTIVE, ChassisState.SAFE_STOP):
            return False
        self._clear_command()
        self._state = ChassisState.INACTIVE
        return True

    def on_fault(self, code: int) -> None:
        """Driver reported a hardware fault; latch into FAULT regardless of prior state."""
        self._fault_code = code
        self._clear_command()
        self._state = ChassisState.FAULT

    def reset_fault(self) -> bool:
        """Explicit recovery — caller MUST have confirmed driver fault register is clear."""
        if self._state != ChassisState.FAULT:
            return False
        self._fault_code = 0
        self._state = ChassisState.INACTIVE
        return True

    # ------------------------------------------------------------- command

    def on_command(self, linear: float, angular: float, now: float) -> None:
        if self._state not in (ChassisState.ACTIVE, ChassisState.SAFE_STOP):
            return
        self._last_cmd_linear = clamp(
            linear, -self.params.max_linear_velocity, self.params.max_linear_velocity
        )
        self._last_cmd_angular = clamp(
            angular, -self.params.max_angular_velocity, self.params.max_angular_velocity
        )
        self._last_cmd_time = now

    def tick(self, now: float) -> WheelTargets:
        """Single control-loop tick.

        Side effects: may toggle ACTIVE ↔ SAFE_STOP based on cmd_vel freshness.
        Pure with respect to FAULT / UNCONFIGURED / INACTIVE — those always return
        zero targets without state change.
        """
        if self._state not in (ChassisState.ACTIVE, ChassisState.SAFE_STOP):
            return WheelTargets.zero()

        cmd_fresh = (
            self._last_cmd_time is not None
            and (now - self._last_cmd_time) <= self.params.cmd_timeout
        )

        if not cmd_fresh:
            if self._state == ChassisState.ACTIVE:
                self._state = ChassisState.SAFE_STOP
            return WheelTargets.zero()

        if self._state == ChassisState.SAFE_STOP:
            self._state = ChassisState.ACTIVE

        return self._compute_targets(self._last_cmd_linear, self._last_cmd_angular)

    # ---------------------------------------------------------- internals

    def _clear_command(self) -> None:
        self._last_cmd_linear = 0.0
        self._last_cmd_angular = 0.0
        self._last_cmd_time = None

    def _compute_targets(self, linear: float, angular: float) -> WheelTargets:
        v_left, v_right = cmd_vel_to_wheel_velocities(
            linear, angular, self.params.wheel_base
        )
        # Defensive re-clamp at wheel level — pure rotation at max angular can
        # push individual wheel velocity beyond max_linear_velocity.
        v_left = clamp(
            v_left, -self.params.max_linear_velocity, self.params.max_linear_velocity
        )
        v_right = clamp(
            v_right, -self.params.max_linear_velocity, self.params.max_linear_velocity
        )

        rpm_left = self.params.left_sign * self.params.mps_to_motor_rpm(v_left)
        rpm_right = self.params.right_sign * self.params.mps_to_motor_rpm(v_right)

        max_rpm = self.params.max_motor_rpm
        rpm_left = clamp(rpm_left, -max_rpm, max_rpm)
        rpm_right = clamp(rpm_right, -max_rpm, max_rpm)

        return WheelTargets(
            left_rpm=int(round(rpm_left)),
            right_rpm=int(round(rpm_right)),
        )
