"""Costmap geometry, freshness and bounded atomic transfers without ROS."""

from array import array
import base64
import hashlib
import math
import random
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import zlib

from robot_nats.costmap_data import CostmapStore, MAX_CELLS, planar_pose, transform_origin


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def pose(x=0.0, y=0.0, angle=0.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=math.sin(angle / 2), w=math.cos(angle / 2)),
    )


class CostmapTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.store = CostmapStore('test_robot', clock=self.clock)
        self.mode = {'actual': 'navigation', 'stale': False, 'pid': 123}
        self.store.set_scope('session', self.mode, 'map-session', 'room')
        self.stamp = 100.0

    def put(self, kind='local', values=None, **overrides):
        self.stamp += 0.1
        arguments = {
            'session': 'session', 'kind': kind, 'width': 3, 'height': 2,
            'resolution': 0.05, 'origin': {'x': -2.0, 'y': -1.0, 'yaw': 0.0},
            'values': values if values is not None else [0, 1, 252, 253, 254, 255],
            'stamp': self.stamp, 'sample_age': 0.0,
            'source_frame': 'odom' if kind == 'local' else 'map',
        }
        arguments.update(overrides)
        return self.store.put_grid(**arguments)

    def chunk(self, metadata, index=0):
        return self.store.query({
            'part': 'chunk', 'kind': metadata['kind'], 'costmap_id': metadata['costmap_id'],
            'revision': metadata['revision'], 'index': index,
        })

    def test_uint8_row_major_round_trip_and_checksums(self):
        self.put(values=array('B', [0, 1, 252, 253, 254, 255]))
        metadata = self.store.metadata('local')
        chunk = self.chunk(metadata)
        packed = base64.b64decode(chunk['data_b64'])
        raw = zlib.decompress(packed)
        self.assertEqual(raw, bytes([0, 1, 252, 253, 254, 255]))
        self.assertEqual(hashlib.sha256(raw).hexdigest(), metadata['decoded_sha256'])
        self.assertEqual(hashlib.sha256(packed).hexdigest(), metadata['sha256'])
        self.assertEqual(metadata['encoding'], 'zlib-uint8-row-major')
        self.assertEqual(metadata['map_id'], chunk['map_id'])
        self.assertEqual(metadata['costmap_id'], chunk['costmap_id'])
        self.assertEqual(metadata['frame_id'], 'map')
        self.assertEqual(metadata['source_frame_id'], 'odom')

    def test_planar_transform_preserves_rotation_and_translation(self):
        origin = pose(2.0, 3.0, math.pi / 4)
        mapping = pose(10.0, 20.0, math.pi / 2)
        transform = SimpleNamespace(translation=mapping.position, rotation=mapping.orientation)
        result = transform_origin(origin, transform)
        self.assertAlmostEqual(result['x'], 7.0)
        self.assertAlmostEqual(result['y'], 22.0)
        self.assertAlmostEqual(result['yaw'], 3 * math.pi / 4)

    def test_nonplanar_nonunit_and_nonfinite_origins_are_rejected(self):
        invalid = pose()
        invalid.orientation.x = 0.1
        invalid.orientation.w = math.sqrt(0.99)
        with self.assertRaisesRegex(ValueError, 'nonplanar'):
            planar_pose(invalid)
        invalid = pose()
        invalid.orientation.w = 2.0
        with self.assertRaisesRegex(ValueError, 'nonunit'):
            planar_pose(invalid)
        invalid = pose()
        invalid.position.x = math.nan
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            planar_pose(invalid)

    def test_same_cells_with_changed_origin_create_new_revision(self):
        self.put()
        first = self.store.metadata('local')
        self.put(origin={'x': -1.95, 'y': -1.0, 'yaw': 0.1})
        second = self.store.metadata('local')
        self.assertEqual(second['revision'], first['revision'] + 1)
        self.assertEqual(second['decoded_sha256'], first['decoded_sha256'])
        self.assertNotEqual(second['origin'], first['origin'])

    def test_identical_new_measurement_updates_age_without_revision_churn(self):
        self.put()
        first = self.store.metadata('local')
        self.clock.now = 1.0
        self.put(sample_age=0.1)
        second = self.store.metadata('local')
        self.assertEqual(first['revision'], second['revision'])
        self.assertGreater(second['stamp'], first['stamp'])
        self.assertEqual(second['age_s'], 0.1)
        self.assertFalse(second['stale'])

    def test_original_sample_age_is_included_in_expiry(self):
        self.put(sample_age=1.4)
        metadata = self.store.metadata('local')
        self.assertFalse(metadata['stale'])
        self.clock.now = 0.2
        self.assertTrue(self.store.metadata('local')['stale'])
        with self.assertRaisesRegex(ValueError, 'not_current'):
            self.chunk(metadata)

    def test_processing_time_counts_towards_observation_age(self):
        compress = zlib.compress

        def delayed_compress(raw, level):
            self.clock.now += 0.4
            return compress(raw, level=level)

        with patch('robot_nats.costmap_data.zlib.compress', delayed_compress):
            self.put(sample_age=0.1)

        metadata = self.store.metadata('local')
        self.assertEqual(metadata['age_s'], 0.5)
        self.assertFalse(metadata['stale'])
        self.clock.now += 1.1
        self.assertTrue(self.store.metadata('local')['stale'])

    def test_grid_becoming_stale_during_processing_is_not_committed(self):
        self.put()
        before = self.store.metadata('local')
        compress = zlib.compress

        def delayed_compress(raw, level):
            self.clock.now += 2.0
            return compress(raw, level=level)

        with patch('robot_nats.costmap_data.zlib.compress', delayed_compress):
            with self.assertRaisesRegex(ValueError, 'outside_fresh_window'):
                self.put(values=bytes([254]) * 6, sample_age=0.1)

        after = self.store.metadata('local')
        self.assertEqual(after['revision'], before['revision'])
        self.assertEqual(after['stamp'], before['stamp'])
        self.assertEqual(after['decoded_sha256'], before['decoded_sha256'])
        self.assertTrue(after['stale'])

    def test_repeated_or_old_stamp_never_refreshes_a_snapshot(self):
        self.put()
        metadata = self.store.metadata('local')
        self.clock.now = 1.0
        for stamp in (metadata['stamp'], metadata['stamp'] - 1.0):
            with self.assertRaisesRegex(ValueError, 'nonincreasing'):
                self.put(stamp=stamp)
        self.clock.now = 1.6
        self.assertTrue(self.store.metadata('local')['stale'])

    def test_timestamp_and_source_frame_validation(self):
        for arguments in (
            {'stamp': 0.0}, {'stamp': math.nan}, {'sample_age': -0.11},
            {'sample_age': 1.51}, {'source_frame': 'map'},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.put(**arguments)
        self.put(kind='global', sample_age=2.9)
        self.assertFalse(self.store.metadata('global')['stale'])

    def test_invalid_dimensions_and_cells_are_rejected(self):
        for arguments in (
            {'width': 0}, {'width': True}, {'width': MAX_CELLS + 1},
            {'values': [0] * 5}, {'values': [0, 1, 2, 3, 4, 256]},
            {'values': [0, 1, 2, 3, 4, -1]}, {'values': [0, 1, 2, 3, 4, 1.0]},
            {'values': [0, 1, 2, 3, 4, True]}, {'resolution': math.inf},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.put(**arguments)

    def test_scope_changes_reject_old_chunks_and_late_writes(self):
        self.put()
        first = self.store.metadata('local')
        self.store.set_scope('other-session', self.mode, 'other-map')
        with self.assertRaisesRegex(ValueError, 'revision_unavailable'):
            self.chunk(first)
        self.assertFalse(self.put())
        self.assertFalse(self.store.metadata('local')['available'])
        self.assertEqual(self.store.state()['map_id'], 'other-map')

    def test_reacquiring_same_pid_still_changes_transfer_identity(self):
        self.put()
        first = self.store.metadata('local')
        self.store.set_scope(None, {'actual': 'idle', 'stale': False}, None)
        self.assertEqual(self.store.state()['local']['reason'], 'nav2_not_active')
        self.store.set_scope('session', self.mode, 'map-session')
        self.put()
        second = self.store.metadata('local')
        self.assertNotEqual(first['costmap_id'], second['costmap_id'])
        with self.assertRaisesRegex(ValueError, 'revision_unavailable'):
            self.chunk(first)

    def test_failed_transform_hides_last_good_grid_until_fresh_success(self):
        self.put()
        self.store.fail('local', 'map_to_odom_unavailable')
        metadata = self.store.metadata('local')
        self.assertTrue(metadata['stale'])
        self.assertEqual(metadata['reason'], 'map_to_odom_unavailable')
        self.put()
        self.assertFalse(self.store.metadata('local')['stale'])

    def test_disabled_observer_and_strict_read_arguments(self):
        disabled = CostmapStore('test_robot', enabled=False)
        self.assertEqual(disabled.state()['local']['reason'], 'ros_observation_disabled')
        self.put()
        metadata = self.store.metadata('local')
        for arguments in (
            {'part': 'clear'}, {'part': 'state', 'extra': 1},
            {'part': 'metadata'}, {'part': 'metadata', 'kind': 'camera'},
            {'part': 'metadata', 'kind': []},
        ):
            with self.subTest(arguments=arguments), self.assertRaises(ValueError):
                self.store.query(arguments)
        for index in (-1, True, metadata['chunks']):
            with self.subTest(index=index), self.assertRaises(ValueError):
                self.chunk(metadata, index)

    def test_pinned_multichunk_revision_survives_updates_and_expires(self):
        raw = random.Random(123).randbytes(65536)
        self.put(width=256, height=256, values=raw)
        metadata = self.store.metadata('local')
        self.assertGreater(metadata['chunks'], 1)
        first = self.chunk(metadata)
        for index in range(5):
            self.clock.now += 0.1
            self.put(width=256, height=256, values=bytes([index]) * 65536)
        second = self.chunk(metadata, 1)
        packed = base64.b64decode(first['data_b64']) + base64.b64decode(second['data_b64'])
        self.assertEqual(zlib.decompress(packed), raw)
        self.clock.now = 10.1
        self.put(width=256, height=256, values=bytes([7]) * 65536)
        with self.assertRaisesRegex(ValueError, 'revision_unavailable'):
            self.chunk(metadata, 1)

    def test_compressed_cache_is_bounded_across_both_kinds(self):
        generator = random.Random(456)
        with patch('robot_nats.costmap_data.MAX_CACHE_BYTES', 260000):
            for _ in range(5):
                for kind in ('local', 'global'):
                    self.put(kind=kind, width=256, height=256, values=generator.randbytes(65536))
                    self.clock.now += 0.01
            stored = sum(
                len(entry[1])
                for stream in self.store.streams.values()
                for entry in stream['versions'].values()
            )
            self.assertLessEqual(stored, 260000)
            self.assertTrue(self.store.metadata('local')['available'])
            self.assertTrue(self.store.metadata('global')['available'])


if __name__ == '__main__':
    unittest.main()
