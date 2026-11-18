import base64
import hashlib
import math
import random
import unittest
import zlib
from types import SimpleNamespace

from robot_nats.map_data import MapStore, MAX_CELLS, robot_model
from robot_nats.map_observer import yaw


class MapTests(unittest.TestCase):
    def setUp(self):
        self.now = 0
        self.store = MapStore('robot', clock=lambda: self.now)
        self.store.set_scope('session', {'actual': 'mapping', 'stale': False})
        self.values = [-1, 0, 100, 25, 50, 75]

    def grid(self, values=None, origin=None):
        self.store.put_grid(
            'session',
            3,
            2,
            0.05,
            origin or {'x': -2, 'y': 3, 'yaw': 0},
            self.values if values is None else values,
            1000,
        )

    def chunk(self, revision):
        return self.store.query({'part': 'chunk', 'map_id': 'session', 'revision': revision, 'index': 0})

    def test_round_trip_signed_cells_origin_and_hash(self):
        self.grid(origin={'x': -2, 'y': 3, 'yaw': math.pi / 2})
        metadata = self.store.metadata()
        packed = base64.b64decode(self.chunk(metadata['revision'])['data_b64'])
        raw = zlib.decompress(packed)
        self.assertEqual(list(raw), [255, 0, 100, 25, 50, 75])
        self.assertEqual(hashlib.sha256(packed).hexdigest(), metadata['sha256'])
        self.assertEqual(hashlib.sha256(raw).hexdigest(), metadata['decoded_sha256'])
        self.assertEqual(metadata['origin']['yaw'], math.pi / 2)

    def test_versions_pins_and_session_reset(self):
        self.grid()
        self.grid()
        self.assertEqual(self.store.metadata()['revision'], 1)
        self.chunk(1)  # Pin a slow client's snapshot.
        for value in [10, 20, 30]:
            self.grid([value] * 6)
        self.assertEqual(self.store.metadata()['revision'], 4)
        self.assertEqual(self.chunk(1)['revision'], 1)
        self.now = 61
        with self.assertRaises(ValueError):
            self.chunk(1)
        self.store.set_scope('new-session', {'actual': 'mapping', 'stale': False})
        self.assertFalse(self.store.metadata()['available'])
        self.assertNotIn('x', self.store.pose())
        with self.assertRaises(ValueError):
            self.chunk(4)

    def test_multiple_chunks_assemble_without_loss(self):
        generator = random.Random(1)
        values = [generator.randrange(101) for _ in range(256 * 256)]
        self.store.put_grid('session', 256, 256, 0.05, {'x': 0, 'y': 0, 'yaw': 0}, values, 1000)
        metadata = self.store.metadata()
        self.assertGreater(metadata['chunks'], 1)
        packed = bytearray()
        for index in range(metadata['chunks']):
            result = self.store.query({'part': 'chunk', 'map_id': 'session', 'revision': 1, 'index': index})
            self.assertLess(len(result['data_b64']), 70000)
            packed.extend(base64.b64decode(result['data_b64']))
        self.assertEqual(zlib.decompress(packed), bytes(values))

    def test_fresh_pose_and_missing_tf_are_not_zero_positions(self):
        self.grid()
        self.store.put_pose('session', 1000, 1.25, -0.6, 0.5, 0.01)
        self.assertTrue(self.store.pose()['valid'])
        self.assertEqual(self.store.pose()['frame_id'], 'map')
        self.assertEqual(self.store.pose()['x'], 1.25)
        self.now = 1.1
        self.assertFalse(self.store.pose()['valid'])
        self.assertNotIn('x', self.store.pose())
        self.store.invalidate_pose('no_tf')
        self.assertEqual(self.store.pose()['reason'], 'no_tf')

    def test_stale_slam_map_and_retained_navigation_map(self):
        self.grid()
        self.now = 16
        self.assertTrue(self.store.metadata()['stale'])
        self.store.set_scope('session', {'actual': 'navigation', 'stale': False})
        self.assertFalse(self.store.metadata()['stale'])
        self.store.set_scope('session', {'actual': 'navigation', 'stale': True})
        self.assertTrue(self.store.metadata()['stale'])

    def test_bounds_and_request_validation(self):
        with self.assertRaises(ValueError):
            self.store.put_grid('session', MAX_CELLS + 1, 1, 0.05, {'x': 0, 'y': 0, 'yaw': 0}, [], 1)
        for values in [[0], [101] * 6, [-2] * 6]:
            with self.assertRaises(ValueError):
                self.grid(values)
        self.grid()
        for args in [
            {'part': 'goal'},
            {'part': 'metadata', 'path': '/tmp'},
            {'part': 'chunk', 'map_id': 'session', 'revision': 1, 'index': True},
            {'part': 'chunk', 'map_id': 'session', 'revision': 1, 'index': 99},
        ]:
            with self.assertRaises(ValueError):
                self.store.query(args)

    def test_model_four_wheels_and_tf_heading(self):
        model = robot_model(
            {
                'body': {'bounds_ground': [[-0.4, -0.27, 0.05], [0.175, 0.27, 0.61]]},
                'drive_wheels': {
                    'diameter': 0.196,
                    'width': 0.05,
                    'centers_ground': [[0, 0.178, 0.098], [0, -0.178, 0.098]],
                },
                'rear_wheels': {
                    'diameter': 0.05,
                    'width': 0.04,
                    'centers_ground': [[-0.29, 0.109, 0.025], [-0.29, -0.109, 0.025]],
                },
                'footprint': [[0.175, 0.27], [-0.4, 0.27], [-0.4, -0.27], [0.175, -0.27]],
            }
        )
        self.assertEqual(len(model['polygons']), 5)
        self.assertEqual(model['frame_id'], 'base_footprint')
        angle = yaw(SimpleNamespace(x=0, y=0, z=math.sin(0.4), w=math.cos(0.4)))
        self.assertAlmostEqual(angle, 0.8)
        with self.assertRaises(ValueError):
            yaw(SimpleNamespace(x=0, y=0, z=0, w=0))
