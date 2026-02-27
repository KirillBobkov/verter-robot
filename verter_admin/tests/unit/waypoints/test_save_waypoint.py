"""Unit tests for SaveWaypointUseCase. Zero rclpy imports."""
import pytest
from unittest.mock import MagicMock

from verter_admin.waypoints.domain.waypoint_store import WaypointStore
from verter_admin.waypoints.application.save_waypoint import SaveWaypointUseCase


@pytest.fixture
def store():
    return WaypointStore()


@pytest.fixture
def port():
    return MagicMock()


@pytest.fixture
def use_case(store, port):
    return SaveWaypointUseCase(store=store, port=port)


def test_save_persists_to_port(use_case, port, store):
    use_case.execute('home', 1.0, 2.0, 0.0)
    port.save.assert_called_once()
    wp = store.get('home')
    assert wp.x == 1.0
    assert wp.y == 2.0
    assert wp.theta == 0.0


def test_save_passes_correct_dict_to_port(use_case, port):
    use_case.execute('kitchen', 3.0, 4.0, 1.57)
    called_with = port.save.call_args[0][0]
    assert 'kitchen' in called_with
    assert called_with['kitchen']['x'] == 3.0
    assert called_with['kitchen']['theta'] == 1.57


def test_duplicate_raises_value_error_port_not_called(use_case, port):
    use_case.execute('home', 1.0, 2.0, 0.0)
    port.reset_mock()
    with pytest.raises(ValueError):
        use_case.execute('home', 3.0, 4.0, 1.0)
    port.save.assert_not_called()
