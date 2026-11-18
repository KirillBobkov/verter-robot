import unittest

from robot_nats.costmap_observer import scope_key
from robot_nats.map_data import MapStore


class CostmapScopeTests(unittest.TestCase):
    def test_map_owner_must_match_current_navigation_process(self):
        mode = {'actual': 'navigation', 'pid': 22, 'stale': False}
        metadata = {
            'available': True, 'stale': False, 'map_id': 'scope',
            'mode': 'navigation', 'mode_pid': 22,
        }
        self.assertEqual(scope_key(mode, metadata), (22, 'scope'))
        for changes in (
            {'mode_pid': 21}, {'mode_pid': None}, {'mode': 'localization'},
            {'stale': True}, {'available': False},
        ):
            self.assertIsNone(scope_key(mode, dict(metadata, **changes)))
        self.assertIsNone(scope_key(dict(mode, stale=True), metadata))
        self.assertIsNone(scope_key(dict(mode, pid=True), metadata))

    def test_map_metadata_includes_owner_even_before_a_grid(self):
        store = MapStore('test')
        store.set_scope('session', {'actual': 'navigation', 'pid': 22, 'stale': False})
        self.assertEqual(store.metadata()['mode_pid'], 22)
        store.put_grid('session', 1, 1, 0.05, {'x': 0, 'y': 0, 'yaw': 0}, [0], 1)
        self.assertEqual(store.metadata()['mode_pid'], 22)
