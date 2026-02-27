"""Unit tests for NavigateToUseCase. Zero rclpy imports."""
import pytest
from unittest.mock import MagicMock

from verter_admin.waypoints.domain.waypoint_store import Waypoint, WaypointStore
from verter_admin.waypoints.application.navigate_to import NavigateToUseCase


@pytest.fixture
def store():
    s = WaypointStore()
    s.save(Waypoint('kitchen', 1.0, 2.0, 0.5))
    return s


@pytest.fixture
def action_client():
    m = MagicMock()
    m.send_goal.return_value = True
    return m


@pytest.fixture
def use_case(store, action_client):
    return NavigateToUseCase(store=store, action_client=action_client)


def test_navigate_to_known_waypoint(use_case, action_client):
    result = use_case.execute('kitchen')
    assert result is True
    action_client.send_goal.assert_called_once()
    sent_waypoint = action_client.send_goal.call_args[0][0]
    assert sent_waypoint.name == 'kitchen'
    assert sent_waypoint.x == 1.0
    assert sent_waypoint.y == 2.0
    assert sent_waypoint.theta == 0.5


def test_navigate_to_unknown_raises_key_error(use_case, action_client):
    with pytest.raises(KeyError):
        use_case.execute('nonexistent')
    action_client.send_goal.assert_not_called()


def test_navigate_returns_false_when_goal_rejected(use_case, action_client):
    action_client.send_goal.return_value = False
    result = use_case.execute('kitchen')
    assert result is False
