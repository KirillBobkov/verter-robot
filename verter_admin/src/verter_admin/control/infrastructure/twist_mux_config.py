"""Infrastructure wiring for twist_mux runtime parameters."""

from verter_admin.contracts.motion import ArbitrationPolicy, MotionTimeouts, TopicContract


def build_twist_mux_parameters() -> dict:
    """Return canonical twist_mux parameters built from contracts."""

    return {
        'topics': {
            'safety_stop': {
                'topic': TopicContract.SAFETY_CMD_VEL,
                'timeout': MotionTimeouts.TWIST_MUX_SOURCE_SEC,
                'priority': ArbitrationPolicy.SAFETY_PRIORITY,
            },
            'teleop_keyboard': {
                'topic': TopicContract.TELEOP_CMD_VEL,
                'timeout': MotionTimeouts.TWIST_MUX_SOURCE_SEC,
                'priority': ArbitrationPolicy.TELEOP_PRIORITY,
            },
            'teleop_joy': {
                'topic': TopicContract.TELEOP_JOY_CMD_VEL,
                'timeout': MotionTimeouts.TWIST_MUX_SOURCE_SEC,
                'priority': ArbitrationPolicy.TELEOP_JOY_PRIORITY,
            },
            'navigation': {
                'topic': TopicContract.NAV2_CMD_VEL,
                'timeout': MotionTimeouts.TWIST_MUX_SOURCE_SEC,
                'priority': ArbitrationPolicy.NAV2_PRIORITY,
            },
        },
        'locks': {
            'teleop_lock': {
                'topic': TopicContract.TELEOP_LOCK,
                'timeout': MotionTimeouts.TWIST_MUX_LOCK_SEC,
                'priority': ArbitrationPolicy.TELEOP_LOCK_PRIORITY,
            },
        },
    }
