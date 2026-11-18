import asyncio
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from robot_nats.map_data import MapStore, project_scan
from robot_nats.map_save import MapSaver
from robot_nats.modes import Modes


class ScanTests(unittest.TestCase):
    def test_transform_filter_and_bounded_points(self):
        scan = SimpleNamespace(
            ranges=[1, float('inf'), 0.01, 2], angle_min=0, angle_increment=math.pi / 2, range_min=0.1, range_max=10
        )
        transform = SimpleNamespace(
            translation=SimpleNamespace(x=2, y=3),
            rotation=SimpleNamespace(x=0, y=0, z=math.sqrt(0.5), w=math.sqrt(0.5)),
        )
        self.assertEqual(project_scan(scan, transform), [[2, 4], [4, 3]])
        scan.ranges = [1] * 20000
        self.assertLessEqual(len(project_scan(scan, transform)), 1800)
        scan.angle_increment = float('nan')
        with self.assertRaises(ValueError):
            project_scan(scan, transform)

    def test_age_session_and_geometry_diagnostics(self):
        clock = [0]
        store = MapStore('test', clock=lambda: clock[0])
        store.set_scope('one', {'actual': 'mapping', 'stale': False})
        store.put_grid('one', 1, 1, 0.03, {'x': 0, 'y': 0, 'yaw': 0}, [0], 1)
        store.put_scan('one', 1, [[2, 3]], 0.2, 'laser', 1665)
        self.assertEqual(store.query({'part': 'scan'})['points'], [[2, 3]])
        store.note_scan(1665, 0.004, 0.1)
        store.note_scan(1666, 0.00399, 0.1)
        self.assertTrue(store.diagnostics()['variable_scan_geometry'])
        clock[0] = 0.9
        self.assertFalse(store.scan()['valid'])
        self.assertNotIn('points', store.scan())
        clock[0] = 11
        self.assertEqual(store.diagnostics()['samples_10s'], 0)
        store.set_scope('two', {'actual': 'mapping', 'stale': False})
        self.assertFalse(store.scan()['valid'])


class FakeSupervisor:
    def __init__(self):
        self.calls = []

    def call(self, request, timeout):
        self.calls.append(request)
        if request['command'] == 'save-map':
            base = Path(request['path'])
            base.with_suffix('.yaml').write_text(
                f'image: {base.name}.pgm\n'
                'resolution: 0.03\n'
                'origin: [0, 0, 0]\n'
                'negate: 0\n'
                'free_thresh: 0.25\n'
                'occupied_thresh: 0.65\n'
            )
            base.with_suffix('.pgm').write_bytes(b'P5\n1 1\n255\n\x00')
        elif request['command'] != 'status':
            raise AssertionError('Forbidden control RPC: ' + str(request))
        return {'ok': True, 'mode': 'mapping', 'groups': {'mode': 123}, 'last_exit': {}}


def fake_serializer(base):
    base.with_suffix('.posegraph').write_bytes(b'graph')
    base.with_suffix('.data').write_bytes(b'data')


class SaveTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.directory = TemporaryDirectory()
        self.base = Path(self.directory.name)
        self.config = {
            'robot_id': 'test',
            'project_root': str(self.base),
            'data_dir': str(self.base / 'state'),
            'maps': {},
            'ros_enabled': False,
            'allow_mode_changes': True,
        }
        self.client = FakeSupervisor()
        self.modes = Modes(self.config, None, client=self.client)
        self.saver = MapSaver(self.config, self.modes, serializer=fake_serializer)

    async def asyncTearDown(self):
        await self.saver.close()
        self.directory.cleanup()

    async def test_save_complete_and_no_overwrite_or_control(self):
        accepted = await self.saver.start({'name': 'room_1'}, 'id')
        self.assertTrue(accepted['busy'])
        await self.saver.task
        result = self.saver.snapshot()
        self.assertEqual(result['status'], 'succeeded', result)
        self.assertEqual(len(result['files']), 4)
        manifest = json.loads((self.base / 'data/maps/room_1.manifest.json').read_text())
        self.assertEqual(manifest['files'], result['files'])
        self.assertEqual(self.saver.list_maps(), [{'name': 'room_1', 'grid': True, 'pose_graph': True}])
        with self.assertRaisesRegex(ValueError, 'already_exists'):
            await self.saver.start({'name': 'room_1'}, 'second')
        self.assertEqual([r['command'] for r in self.client.calls].count('save-map'), 1)
        self.assertTrue(all(r['command'] in ('status', 'save-map') for r in self.client.calls))

    async def test_partial_failure_and_restart_do_not_replay(self):
        def broken_serializer(base):
            raise RuntimeError('test_graph_failure')

        self.saver.serializer = broken_serializer
        await self.saver.start({'name': 'partial'}, 'id')
        await self.saver.task
        state = self.saver.snapshot()
        self.assertEqual(state['status'], 'failed')
        self.assertTrue(state['grid_saved'])
        self.assertFalse(state['graph_saved'])
        state['status'] = 'running'
        self.saver.state_file.write_text(json.dumps(state))
        restored = MapSaver(self.config, self.modes, serializer=fake_serializer)
        self.assertEqual(restored.snapshot()['status'], 'interrupted')
        self.assertIsNone(restored.task)

    async def test_invalid_names_and_disabled_saving(self):
        for name in ('../other', '', 'a/b', 'hello.world', 'a' * 65):
            with self.assertRaises(ValueError):
                await self.saver.start({'name': name}, 'id')
        self.config['allow_map_save'] = False
        with self.assertRaisesRegex(ValueError, 'disabled'):
            await self.saver.start({'name': 'room'}, 'id')
        self.assertEqual(self.client.calls, [])
