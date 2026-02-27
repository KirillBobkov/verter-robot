"""Command/safety contracts shared across launch and runtime nodes."""


class TopicContract:
    """Canonical topic names for mobility and safety pipeline."""

    SAFETY_CMD_VEL = '/safety/cmd_vel'
    TELEOP_CMD_VEL = '/teleop_keyboard/cmd_vel'
    TELEOP_JOY_CMD_VEL = '/teleop_joy/cmd_vel'
    TELEOP_LOCK = '/teleop_keyboard/lock'
    NAV2_CMD_VEL = '/nav2/cmd_vel'
    FINAL_CMD_VEL = '/cmd_vel'
    SCAN = '/scan'
    ULTRASONIC_SCAN = '/ultrasonic/ranges'


class ArbitrationPolicy:
    """Source priorities consumed by twist_mux."""

    SAFETY_PRIORITY = 200
    TELEOP_PRIORITY = 100
    TELEOP_JOY_PRIORITY = 90
    NAV2_PRIORITY = 10
    TELEOP_LOCK_PRIORITY = 100

    # Canonical aliases for QG-6 traceability (same values, different names)
    TWIST_MUX_SOURCE_SAFETY = SAFETY_PRIORITY
    TWIST_MUX_SOURCE_TELEOP_KB = TELEOP_PRIORITY
    TWIST_MUX_SOURCE_TELEOP_JOY = TELEOP_JOY_PRIORITY
    TWIST_MUX_SOURCE_NAV2 = NAV2_PRIORITY


class MotionTimeouts:
    """Timing contracts for safety and command chain."""

    TWIST_MUX_SOURCE_SEC = 0.5
    TWIST_MUX_LOCK_SEC = 0.0
    SENSOR_FRESHNESS_SEC = 0.8
    CHASSIS_WATCHDOG_MS = 500
