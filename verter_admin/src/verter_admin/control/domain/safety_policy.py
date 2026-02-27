"""Pure domain safety policies (no ROS dependencies)."""

from dataclasses import dataclass
from enum import Enum


class SafetyState(Enum):
    READY = 'READY'
    DEGRADED = 'DEGRADED'
    FAULT = 'FAULT'


@dataclass(frozen=True)
class VelocityCommand:
    """Transport-agnostic command payload for safety decisions."""

    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class SafetyDecision:
    """Result of safety policy evaluation."""

    safety_active: bool
    state: SafetyState
    state_reason: str
    publish_command: VelocityCommand | None = None
    obstacle_triggered: bool = False
    recovery_gate_opened: bool = False


def is_zero_command(command: VelocityCommand) -> bool:
    """Return True when command is close to zero for arbitration fallback."""

    return abs(command.linear_x) < 0.001 and abs(command.angular_z) < 0.001


def select_source_command(
    teleop_command: VelocityCommand,
    nav_command: VelocityCommand,
    teleop_fresh: bool,
    nav_fresh: bool,
) -> VelocityCommand | None:
    """Select command source preserving teleop-over-nav policy."""

    if teleop_fresh and not is_zero_command(teleop_command):
        return teleop_command
    if nav_fresh:
        return nav_command
    if teleop_fresh:
        return teleop_command
    return None


def clamp_forward_motion(command: VelocityCommand) -> VelocityCommand:
    """Block only forward linear motion, preserve reverse and rotation."""

    return VelocityCommand(
        linear_x=min(0.0, command.linear_x),
        angular_z=command.angular_z,
    )

