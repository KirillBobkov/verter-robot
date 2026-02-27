"""PatrolPolicy state machine — pure Python, zero ROS imports.

Implements SI-6: consecutive navigation failures degrade the state:
  PATROLLING → DEGRADED (at warn_threshold failures)
  DEGRADED   → FAULT    (at fault_threshold failures)

FAULT is non-recoverable via on_failure/on_success; use reset() for explicit recovery.
The recovery path via managed lifecycle is: cleanup → configure → activate.
"""
from __future__ import annotations

from enum import Enum, auto


class PatrolState(Enum):
    IDLE = auto()
    PATROLLING = auto()
    DEGRADED = auto()
    FAULT = auto()


class PatrolPolicy:
    """State machine governing patrol behaviour and degraded-mode transitions.

    Parameters
    ----------
    warn_threshold:
        Number of consecutive failures that triggers PATROLLING → DEGRADED.
    fault_threshold:
        Total consecutive failures (from start of patrol) that trigger DEGRADED → FAULT.
        Must be strictly greater than warn_threshold.
    """

    def __init__(self, warn_threshold: int = 3, fault_threshold: int = 5) -> None:
        if warn_threshold < 1:
            raise ValueError("warn_threshold must be >= 1")
        if fault_threshold <= warn_threshold:
            raise ValueError("fault_threshold must be > warn_threshold")
        self.warn_threshold = warn_threshold
        self.fault_threshold = fault_threshold
        self._state = PatrolState.IDLE
        self._fail_count = 0

    @property
    def state(self) -> PatrolState:
        return self._state

    @property
    def fail_count(self) -> int:
        return self._fail_count

    def start(self) -> None:
        """Transition IDLE → PATROLLING. No-op if already PATROLLING."""
        if self._state == PatrolState.IDLE:
            self._state = PatrolState.PATROLLING
            self._fail_count = 0

    def on_success(self) -> None:
        """Reset failure counter. If DEGRADED, return to PATROLLING. No-op in FAULT."""
        if self._state == PatrolState.FAULT:
            return
        self._fail_count = 0
        if self._state == PatrolState.DEGRADED:
            self._state = PatrolState.PATROLLING

    def on_failure(self) -> None:
        """Increment failure counter and advance degradation state if threshold reached."""
        if self._state in (PatrolState.FAULT, PatrolState.IDLE):
            return
        self._fail_count += 1
        if self._state == PatrolState.PATROLLING and self._fail_count >= self.warn_threshold:
            self._state = PatrolState.DEGRADED
        elif self._state == PatrolState.DEGRADED and self._fail_count >= self.fault_threshold:
            self._state = PatrolState.FAULT

    def stop(self) -> None:
        """Transition to IDLE if not in FAULT. Resets fail_count."""
        if self._state != PatrolState.FAULT:
            self._state = PatrolState.IDLE
            self._fail_count = 0

    def reset(self) -> None:
        """Force any state → IDLE (explicit recovery path). Resets fail_count.

        This is the only way to leave FAULT state without a full node lifecycle cycle.
        Caller is responsible for ensuring the robot is in a safe state before calling.
        """
        self._state = PatrolState.IDLE
        self._fail_count = 0
