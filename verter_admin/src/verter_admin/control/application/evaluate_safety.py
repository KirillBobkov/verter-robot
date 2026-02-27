"""Safety evaluation use-case orchestrating domain policy."""

from dataclasses import dataclass

from verter_admin.control.domain import (
    SafetyDecision,
    SafetyState,
    VelocityCommand,
    clamp_forward_motion,
)


@dataclass(frozen=True)
class SafetyEvaluationInput:
    """All runtime facts required for one safety decision tick."""

    nearest_distance: float
    sensors_fresh: bool
    command_fresh: bool
    safety_active: bool
    stop_distance: float
    resume_distance: float
    allow_degraded_motion: bool
    source_command: VelocityCommand | None


class EvaluateSafety:
    """Evaluates safety state and optional filtered command publication."""

    def execute(self, data: SafetyEvaluationInput) -> SafetyDecision:
        safety_active = data.safety_active
        obstacle_triggered = False
        recovery_gate_opened = False

        if safety_active:
            recovery_gate_open = (
                data.nearest_distance >= data.resume_distance
                and data.sensors_fresh
                and data.command_fresh
            )
            if recovery_gate_open:
                safety_active = False
                recovery_gate_opened = True
        else:
            if data.nearest_distance <= data.stop_distance:
                safety_active = True
                obstacle_triggered = True

        if safety_active:
            source = data.source_command or VelocityCommand()
            return SafetyDecision(
                safety_active=True,
                state=SafetyState.FAULT,
                state_reason='obstacle stop active',
                publish_command=clamp_forward_motion(source),
                obstacle_triggered=obstacle_triggered,
                recovery_gate_opened=recovery_gate_opened,
            )

        if not data.sensors_fresh and not data.allow_degraded_motion:
            return SafetyDecision(
                safety_active=False,
                state=SafetyState.DEGRADED,
                state_reason='stale safety sensors and degraded motion is not allowed',
                publish_command=VelocityCommand(),
                obstacle_triggered=obstacle_triggered,
                recovery_gate_opened=recovery_gate_opened,
            )

        if not data.sensors_fresh:
            return SafetyDecision(
                safety_active=False,
                state=SafetyState.DEGRADED,
                state_reason='stale safety sensors',
                publish_command=None,
                obstacle_triggered=obstacle_triggered,
                recovery_gate_opened=recovery_gate_opened,
            )

        return SafetyDecision(
            safety_active=False,
            state=SafetyState.READY,
            state_reason='safety checks nominal',
            publish_command=None,
            obstacle_triggered=obstacle_triggered,
            recovery_gate_opened=recovery_gate_opened,
        )

