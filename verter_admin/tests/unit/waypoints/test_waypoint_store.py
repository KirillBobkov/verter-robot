"""Unit tests for WaypointStore CRUD. Zero rclpy imports."""
import pytest

from verter_admin.waypoints.domain.waypoint_store import Waypoint, WaypointStore


@pytest.fixture
def store():
    return WaypointStore()


def test_save_and_get(store):
    wp = Waypoint(name='a', x=1.0, y=2.0, theta=0.5)
    store.save(wp)
    assert store.get('a') == wp


def test_list_all_insertion_order(store):
    store.save(Waypoint('first', 0.0, 0.0, 0.0))
    store.save(Waypoint('second', 1.0, 1.0, 1.0))
    names = [wp.name for wp in store.list_all()]
    assert names == ['first', 'second']


def test_duplicate_raises_value_error(store):
    store.save(Waypoint('dup', 0.0, 0.0, 0.0))
    with pytest.raises(ValueError):
        store.save(Waypoint('dup', 1.0, 1.0, 1.0))


def test_get_unknown_raises_key_error(store):
    with pytest.raises(KeyError):
        store.get('missing')


def test_delete(store):
    store.save(Waypoint('to_del', 0.0, 0.0, 0.0))
    store.delete('to_del')
    with pytest.raises(KeyError):
        store.get('to_del')


def test_delete_unknown_raises_key_error(store):
    with pytest.raises(KeyError):
        store.delete('no_such')


def test_round_trip_dict(store):
    store.save(Waypoint('wp1', 1.0, 2.0, 3.0))
    d = store.to_dict()
    store2 = WaypointStore()
    store2.load_from_dict(d)
    wp = store2.get('wp1')
    assert wp.x == 1.0
    assert wp.y == 2.0
    assert wp.theta == 3.0


def test_load_from_dict_replaces_existing(store):
    store.save(Waypoint('old', 0.0, 0.0, 0.0))
    store.load_from_dict({'new': {'x': 5.0, 'y': 6.0, 'theta': 1.0}})
    with pytest.raises(KeyError):
        store.get('old')
    wp = store.get('new')
    assert wp.x == 5.0


def test_to_dict_empty_store(store):
    assert store.to_dict() == {}


def test_list_all_empty(store):
    assert store.list_all() == []
