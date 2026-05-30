"""Unit tests for ChassisPolicy. Zero rclpy/pymodbus imports.

Covers (per rulebook §3.3 invariants):
  I1: cmd_vel watchdog → SAFE_STOP after cmd_timeout without fresh command.
  I3: linear/angular clamped to params.max_*_velocity.
  I4: motor RPM clamped to params.max_motor_rpm.
  I5: fail-safe default — wheel targets are zero in UNCONFIGURED/INACTIVE/FAULT and
      on first tick after activate() until a fresh /cmd_vel arrives.
"""
from __future__ import annotations

import math

import pytest

from verter_admin.control.domain.chassis_policy import (
    ChassisParameters,
    ChassisPolicy,
    ChassisState,
    WheelTargets,
    clamp,
    cmd_vel_to_wheel_velocities,
)


# ---------------------------------------------------------------- fixtures


@pytest.fixture
def params() -> ChassisParameters:
    return ChassisParameters(
        wheel_diameter=0.200,
        wheel_base=0.400,
        gear_ratio=1.0,
        max_motor_rpm=200,
        max_linear_velocity=0.5,
        max_angular_velocity=1.0,
        cmd_timeout=0.5,
        left_sign=+1,
        right_sign=+1,
    )


@pytest.fixture
def policy(params: ChassisParameters) -> ChassisPolicy:
    return ChassisPolicy(params)


@pytest.fixture
def active_policy(policy: ChassisPolicy) -> ChassisPolicy:
    assert policy.configure()
    assert policy.activate()
    return policy


# ----------------------------------------------------- ChassisParameters


class TestChassisParameters:
    def test_wheel_circumference(self, params: ChassisParameters) -> None:
        assert params.wheel_circumference == pytest.approx(math.pi * 0.200)

    def test_mps_to_motor_rpm_zero(self, params: ChassisParameters) -> None:
        assert params.mps_to_motor_rpm(0.0) == 0.0

    def test_mps_to_motor_rpm_full_circumference(self, params: ChassisParameters) -> None:
        # Traveling one circumference per second → 60 RPM at the wheel, ×gear at motor.
        v = params.wheel_circumference  # ≈ 0.628 m/s
        assert params.mps_to_motor_rpm(v) == pytest.approx(60.0)

    def test_mps_to_motor_rpm_with_gear(self) -> None:
        p = ChassisParameters(wheel_diameter=0.200, gear_ratio=4.0)
        v = p.wheel_circumference  # 60 wheel RPM
        assert p.mps_to_motor_rpm(v) == pytest.approx(240.0)


# ---------------------------------------------------------- pure helpers


class TestKinematicsHelpers:
    def test_clamp_below_range(self) -> None:
        assert clamp(-5.0, -1.0, 1.0) == -1.0

    def test_clamp_above_range(self) -> None:
        assert clamp(5.0, -1.0, 1.0) == 1.0

    def test_clamp_within_range(self) -> None:
        assert clamp(0.3, -1.0, 1.0) == 0.3

    def test_pure_forward_equal_wheels(self) -> None:
        v_l, v_r = cmd_vel_to_wheel_velocities(0.5, 0.0, wheel_base=0.4)
        assert v_l == pytest.approx(0.5)
        assert v_r == pytest.approx(0.5)

    def test_pure_rotation_opposite_wheels(self) -> None:
        # ω=1 rad/s, base=0.4 → ±0.2 m/s
        v_l, v_r = cmd_vel_to_wheel_velocities(0.0, 1.0, wheel_base=0.4)
        assert v_l == pytest.approx(-0.2)
        assert v_r == pytest.approx(0.2)

    def test_stationary(self) -> None:
        v_l, v_r = cmd_vel_to_wheel_velocities(0.0, 0.0, wheel_base=0.4)
        assert v_l == 0.0 and v_r == 0.0


# ----------------------------------------------------- state machine


class TestStateMachine:
    def test_initial_state(self, policy: ChassisPolicy) -> None:
        assert policy.state == ChassisState.UNCONFIGURED

    def test_configure_transitions_to_inactive(self, policy: ChassisPolicy) -> None:
        assert policy.configure() is True
        assert policy.state == ChassisState.INACTIVE

    def test_configure_idempotent_rejected(self, policy: ChassisPolicy) -> None:
        policy.configure()
        assert policy.configure() is False  # already INACTIVE

    def test_activate_requires_inactive(self, policy: ChassisPolicy) -> None:
        assert policy.activate() is False  # from UNCONFIGURED
        policy.configure()
        assert policy.activate() is True
        assert policy.state == ChassisState.ACTIVE

    def test_deactivate_from_active(self, active_policy: ChassisPolicy) -> None:
        assert active_policy.deactivate() is True
        assert active_policy.state == ChassisState.INACTIVE

    def test_deactivate_from_unconfigured_rejected(self, policy: ChassisPolicy) -> None:
        assert policy.deactivate() is False

    def test_cleanup_from_inactive(self, policy: ChassisPolicy) -> None:
        policy.configure()
        assert policy.cleanup() is True
        assert policy.state == ChassisState.UNCONFIGURED

    def test_cleanup_from_active_rejected(self, active_policy: ChassisPolicy) -> None:
        assert active_policy.cleanup() is False

    def test_cleanup_from_fault_allowed(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_fault(code=0xDEAD)
        assert active_policy.state == ChassisState.FAULT
        assert active_policy.cleanup() is True
        assert active_policy.state == ChassisState.UNCONFIGURED


class TestFaultHandling:
    def test_on_fault_latches(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_fault(code=42)
        assert active_policy.state == ChassisState.FAULT
        assert active_policy.fault_code == 42

    def test_on_fault_from_any_state(self, policy: ChassisPolicy) -> None:
        # FAULT is a sink — accessible from UNCONFIGURED, INACTIVE, ACTIVE, SAFE_STOP.
        policy.on_fault(code=1)
        assert policy.state == ChassisState.FAULT

    def test_reset_fault_returns_to_inactive(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_fault(code=99)
        assert active_policy.reset_fault() is True
        assert active_policy.state == ChassisState.INACTIVE
        assert active_policy.fault_code == 0

    def test_reset_fault_rejected_when_not_in_fault(self, active_policy: ChassisPolicy) -> None:
        assert active_policy.reset_fault() is False

    def test_fault_returns_zero_targets(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.5, 0.0, now=1.0)
        active_policy.on_fault(code=1)
        targets = active_policy.tick(now=1.01)
        assert targets == WheelTargets.zero()


# ----------------------------------------------------- watchdog (I1)


class TestWatchdog:
    def test_fresh_command_keeps_active(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.3, 0.0, now=10.0)
        active_policy.tick(now=10.1)
        assert active_policy.state == ChassisState.ACTIVE

    def test_stale_command_drops_to_safe_stop(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.3, 0.0, now=10.0)
        # cmd_timeout = 0.5; advance past it
        active_policy.tick(now=10.6)
        assert active_policy.state == ChassisState.SAFE_STOP

    def test_safe_stop_returns_zero(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.3, 0.0, now=10.0)
        targets = active_policy.tick(now=10.6)
        assert targets == WheelTargets.zero()

    def test_safe_stop_recovers_on_fresh_command(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.3, 0.0, now=10.0)
        active_policy.tick(now=10.6)
        assert active_policy.state == ChassisState.SAFE_STOP
        active_policy.on_command(0.3, 0.0, now=11.0)
        targets = active_policy.tick(now=11.01)
        assert active_policy.state == ChassisState.ACTIVE
        assert targets != WheelTargets.zero()

    def test_no_command_yet_returns_zero(self, active_policy: ChassisPolicy) -> None:
        # Just activated, no /cmd_vel ever received.
        targets = active_policy.tick(now=0.5)
        assert targets == WheelTargets.zero()
        # Not having any command at all also lands us in SAFE_STOP defensively.
        assert active_policy.state == ChassisState.SAFE_STOP

    def test_activate_discards_stale_command(self, policy: ChassisPolicy) -> None:
        policy.configure()
        policy.activate()
        policy.on_command(0.3, 0.0, now=10.0)
        policy.deactivate()
        # 11.0: cached command is "fresh" by wall-clock — but activate() must wipe it.
        policy.activate()
        targets = policy.tick(now=10.05)
        assert targets == WheelTargets.zero()
        assert policy.state == ChassisState.SAFE_STOP


# ----------------------------------------------------- command gating


class TestCommandGating:
    def test_command_ignored_in_unconfigured(self, policy: ChassisPolicy) -> None:
        policy.on_command(0.3, 0.0, now=1.0)
        assert policy.last_command_time is None

    def test_command_ignored_in_inactive(self, policy: ChassisPolicy) -> None:
        policy.configure()
        policy.on_command(0.3, 0.0, now=1.0)
        assert policy.last_command_time is None

    def test_command_ignored_in_fault(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_fault(code=1)
        active_policy.on_command(0.3, 0.0, now=1.0)
        assert active_policy.last_command_time is None

    def test_command_accepted_in_active(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.3, 0.0, now=1.0)
        assert active_policy.last_command_time == 1.0

    def test_command_accepted_in_safe_stop(self, active_policy: ChassisPolicy) -> None:
        # Drive to SAFE_STOP by advancing time past watchdog.
        active_policy.on_command(0.3, 0.0, now=1.0)
        active_policy.tick(now=2.0)
        assert active_policy.state == ChassisState.SAFE_STOP
        active_policy.on_command(0.4, 0.0, now=2.5)
        assert active_policy.last_command_time == 2.5


# ----------------------------------------------------- clamping (I3/I4)


class TestVelocityClamping:
    def test_linear_clamped_to_max(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(99.0, 0.0, now=1.0)
        targets = active_policy.tick(now=1.0)
        # max_linear=0.5 m/s → motor RPM = 0.5 / (π·0.2) · 60 ≈ 47.75
        expected_rpm = round(0.5 / (math.pi * 0.200) * 60.0)
        assert targets.left_rpm == expected_rpm
        assert targets.right_rpm == expected_rpm

    def test_linear_clamped_negative(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(-99.0, 0.0, now=1.0)
        targets = active_policy.tick(now=1.0)
        expected_rpm = -round(0.5 / (math.pi * 0.200) * 60.0)
        assert targets.left_rpm == expected_rpm
        assert targets.right_rpm == expected_rpm

    def test_angular_clamped_to_max(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.0, 99.0, now=1.0)
        targets = active_policy.tick(now=1.0)
        # angular clamped to 1.0; wheel = ±ω · base/2 = ±0.2 m/s
        v_wheel = 1.0 * 0.400 / 2.0  # 0.2 m/s
        expected_rpm = round(v_wheel / (math.pi * 0.200) * 60.0)
        assert targets.left_rpm == -expected_rpm
        assert targets.right_rpm == expected_rpm

    def test_motor_rpm_hard_cap(self) -> None:
        # Even if velocity is "in range", a tiny max_motor_rpm must cap output.
        params = ChassisParameters(
            wheel_diameter=0.200,
            wheel_base=0.400,
            gear_ratio=1.0,
            max_motor_rpm=10,  # artificially tiny
            max_linear_velocity=0.5,
            cmd_timeout=0.5,
        )
        p = ChassisPolicy(params)
        p.configure()
        p.activate()
        p.on_command(0.5, 0.0, now=1.0)
        targets = p.tick(now=1.0)
        assert abs(targets.left_rpm) <= 10
        assert abs(targets.right_rpm) <= 10


class TestSignCalibration:
    def test_left_sign_negative_inverts_left_only(self, params: ChassisParameters) -> None:
        p = ChassisPolicy(
            ChassisParameters(
                wheel_diameter=params.wheel_diameter,
                wheel_base=params.wheel_base,
                gear_ratio=params.gear_ratio,
                max_motor_rpm=params.max_motor_rpm,
                max_linear_velocity=params.max_linear_velocity,
                max_angular_velocity=params.max_angular_velocity,
                cmd_timeout=params.cmd_timeout,
                left_sign=-1,
                right_sign=+1,
            )
        )
        p.configure()
        p.activate()
        p.on_command(0.3, 0.0, now=1.0)
        targets = p.tick(now=1.0)
        assert targets.left_rpm < 0
        assert targets.right_rpm > 0

    def test_both_signs_negative(self, params: ChassisParameters) -> None:
        p = ChassisPolicy(
            ChassisParameters(
                wheel_diameter=params.wheel_diameter,
                wheel_base=params.wheel_base,
                gear_ratio=params.gear_ratio,
                max_motor_rpm=params.max_motor_rpm,
                max_linear_velocity=params.max_linear_velocity,
                max_angular_velocity=params.max_angular_velocity,
                cmd_timeout=params.cmd_timeout,
                left_sign=-1,
                right_sign=-1,
            )
        )
        p.configure()
        p.activate()
        p.on_command(0.3, 0.0, now=1.0)
        targets = p.tick(now=1.0)
        assert targets.left_rpm < 0
        assert targets.right_rpm < 0


# ----------------------------------------------------- kinematics output


class TestKinematicsOutput:
    def test_zero_command_zero_rpm(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.0, 0.0, now=1.0)
        assert active_policy.tick(now=1.0) == WheelTargets.zero()

    def test_pure_forward_equal_rpm(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.3, 0.0, now=1.0)
        targets = active_policy.tick(now=1.0)
        assert targets.left_rpm == targets.right_rpm
        assert targets.left_rpm > 0

    def test_pure_rotation_opposite_rpm(self, active_policy: ChassisPolicy) -> None:
        active_policy.on_command(0.0, 0.5, now=1.0)
        targets = active_policy.tick(now=1.0)
        assert targets.left_rpm == -targets.right_rpm
        assert targets.right_rpm > 0

    def test_forward_left_curve(self, active_policy: ChassisPolicy) -> None:
        # Forward + slight CCW rotation: right wheel faster than left.
        active_policy.on_command(0.3, 0.2, now=1.0)
        targets = active_policy.tick(now=1.0)
        assert targets.right_rpm > targets.left_rpm
        assert targets.left_rpm > 0  # both still forward
