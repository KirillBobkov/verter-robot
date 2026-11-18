import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from robot_nats.places import Places, resolve_map


class PlacesTests(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.root = Path(self.directory.name)
        maps = self.root / 'data/maps'
        maps.mkdir(parents=True)
        for name in ('room', 'another'):
            (maps / (name + '.yaml')).write_text(
                'image: ' + name + '.pgm\nresolution: 0.03\norigin: [0, 0, 0]\n'
            )
            (maps / (name + '.pgm')).write_bytes(b'P5\n2 1\n255\n\x00\xfe')
        self.config = {'project_root': str(self.root), 'maps': {}, 'data_dir': str(self.root / 'state')}
        self.current = {'map': 'room', 'x': 1.2, 'y': -0.3, 'yaw': 0.1}
        self.localization = SimpleNamespace(current_pose=lambda: dict(self.current))
        self.places = Places(self.config, self.localization)

    def tearDown(self):
        self.places.close()
        self.directory.cleanup()

    def save_point(self, ident='desk', **extra):
        args = {'map': 'room', 'id': ident, 'label': 'Стол', 'x': 1, 'y': 2, 'yaw': 0}
        args.update(extra)
        return self.places.execute('waypoint.save', args)

    def test_map_isolation_and_persistent_coordinates(self):
        self.save_point()
        self.assertEqual(self.places.execute('waypoint.list', {'map': 'another'})['waypoints'], [])
        self.places.close()
        self.places = Places(self.config, self.localization)
        saved = self.places.execute('waypoint.list', {'map': 'room'})['waypoints'][0]
        self.assertEqual(saved['label'], 'Стол')
        self.assertEqual(saved['frame_id'], 'map')
        self.assertEqual(saved['x'], 1)

    def test_changed_saved_map_does_not_reuse_existing_points(self):
        self.save_point()
        (self.root / 'data/maps/room.pgm').write_bytes(b'P5\n2 1\n255\n\xfe\xfe')
        with self.assertRaisesRegex(ValueError, 'saved_map_changed'):
            self.places.execute('waypoint.list', {'map': 'room'})

    def test_current_pose_must_belong_to_requested_map(self):
        args = {'map': 'room', 'id': 'here', 'label': 'Здесь', 'source': 'current'}
        result = self.places.execute('waypoint.save', args)
        self.assertEqual(result['waypoint']['x'], 1.2)
        self.current['map'] = 'another'
        with self.assertRaisesRegex(ValueError, 'map_does_not_match'):
            self.places.execute('waypoint.save', args)

    def test_ordered_routes_references_and_delete_guard(self):
        self.save_point('one')
        self.save_point('two', x=3)
        route = {'map': 'room', 'id': 'trip', 'label': 'Маршрут', 'kind': 'through', 'points': ['two', 'one', 'two']}
        result = self.places.execute('route.save', route)
        self.assertEqual(result['route']['points'], ['two', 'one', 'two'])
        self.assertNotIn('running', result['route'])
        with self.assertRaisesRegex(ValueError, 'waypoint_is_used'):
            self.places.execute('waypoint.delete', {'map': 'room', 'id': 'two'})
        self.places.execute('route.delete', {'map': 'room', 'id': 'trip'})
        self.places.execute('waypoint.delete', {'map': 'room', 'id': 'two'})
        route['points'] = ['missing']
        with self.assertRaisesRegex(ValueError, 'unknown_waypoint'):
            self.places.execute('route.save', route)

    def test_reject_nonfinite_boolean_coordinates_and_path_escape(self):
        for value in (True, float('nan'), float('inf')):
            with self.assertRaises(ValueError):
                self.save_point(x=value)
        for name in ('../room', '/etc/passwd', '', True):
            with self.assertRaises(ValueError):
                resolve_map(self.config, name)
        self.assertEqual(self.places.execute('waypoint.list', {'map': 'room'})['waypoints'], [])

    def test_current_pose_does_not_accept_overrides_or_motion_fields(self):
        args = {'map': 'room', 'id': 'here', 'label': 'Здесь', 'source': 'current', 'x': 1}
        with self.assertRaises(ValueError):
            self.places.execute('waypoint.save', args)
        with self.assertRaises(ValueError):
            self.save_point(execute=True)
        self.assertNotIn('execute', json.dumps(self.places.snapshot('room')))

    def test_invalid_map_is_reported_without_breaking_state_snapshot(self):
        (self.root / 'data/maps/room.yaml').write_text('image: room.pgm\nresolution: .nan\norigin: [0, 0, 0]\n')
        state = self.places.snapshot('room')
        self.assertEqual(state['error'], 'invalid_map_resolution')
        self.assertEqual(state['waypoints'], [])


if __name__ == '__main__':
    unittest.main()
