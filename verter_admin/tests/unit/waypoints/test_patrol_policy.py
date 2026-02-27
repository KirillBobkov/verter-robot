"""Unit tests for PatrolPolicy state machine. Zero rclpy imports.

Covers SI-6: PATROLLING → DEGRADED → FAULT on consecutive navigation failures.
Named test test_fault_threshold is the SI-6 scenario required by the brief.
"""
import pytest

from verter_admin.waypoints.domain.patrol_policy import PatrolPolicy, PatrolState


@pytest.fixture
def policy():
    # warn=2, fault=4 gives clear step counts for assertions
    return PatrolPolicy(warn_threshold=2, fault_threshold=4)


def test_initial_state(policy):
    assert policy.state == PatrolState.IDLE


def test_start_transitions_to_patrolling(policy):
    policy.start()
    assert policy.state == PatrolState.PATROLLING


def test_start_from_idle_only_noop_if_patrolling(policy):
    policy.start()
    policy.start()  # no-op
    assert policy.state == PatrolState.PATROLLING


def test_success_resets_fail_count(policy):
    policy.start()
    policy.on_failure()
    policy.on_success()
    assert policy.fail_count == 0
    assert policy.state == PatrolState.PATROLLING


def test_warn_threshold_triggers_degraded(policy):
    policy.start()
    policy.on_failure()
    policy.on_failure()  # fail_count == 2 == warn_threshold
    assert policy.state == PatrolState.DEGRADED


def test_fault_threshold(policy):
    """SI-6: PATROLLING → DEGRADED at warn_threshold; DEGRADED → FAULT at fault_threshold."""
    policy.start()
    # 2 failures → DEGRADED
    for _ in range(2):
        policy.on_failure()
    assert policy.state == PatrolState.DEGRADED

    # 2 more failures (total 4 == fault_threshold) → FAULT
    for _ in range(2):
        policy.on_failure()
    assert policy.state == PatrolState.FAULT


def test_fault_is_terminal_on_failure(policy):
    policy.start()
    for _ in range(4):
        policy.on_failure()
    assert policy.state == PatrolState.FAULT
    policy.on_failure()  # no-op
    assert policy.state == PatrolState.FAULT


def test_fault_is_terminal_on_success(policy):
    policy.start()
    for _ in range(4):
        policy.on_failure()
    policy.on_success()  # no-op in FAULT
    assert policy.state == PatrolState.FAULT


def test_reset_from_fault(policy):
    """reset() MUST transition FAULT → IDLE (explicit recovery path for SI-6)."""
    policy.start()
    for _ in range(4):
        policy.on_failure()
    assert policy.state == PatrolState.FAULT
    policy.reset()
    assert policy.state == PatrolState.IDLE
    assert policy.fail_count == 0


def test_success_in_degraded_returns_to_patrolling(policy):
    policy.start()
    policy.on_failure()
    policy.on_failure()  # → DEGRADED
    policy.on_success()
    assert policy.state == PatrolState.PATROLLING


def test_stop_resets_to_idle(policy):
    policy.start()
    policy.stop()
    assert policy.state == PatrolState.IDLE
    assert policy.fail_count == 0


def test_stop_does_not_recover_fault(policy):
    policy.start()
    for _ in range(4):
        policy.on_failure()
    policy.stop()  # no-op when FAULT
    assert policy.state == PatrolState.FAULT


def test_on_failure_noop_when_idle(policy):
    policy.on_failure()  # no-op in IDLE
    assert policy.state == PatrolState.IDLE
    assert policy.fail_count == 0


def test_invalid_warn_threshold():
    with pytest.raises(ValueError):
        PatrolPolicy(warn_threshold=0)


def test_invalid_fault_threshold():
    with pytest.raises(ValueError):
        PatrolPolicy(warn_threshold=3, fault_threshold=3)
