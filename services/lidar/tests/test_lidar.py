"""Offline geometry, invalid scans, HTTP/WS guards. No ROS context or NATS connection."""

import asyncio
import json
import math
from pathlib import Path
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock

from aiohttp.test_utils import TestClient, TestServer
from geometry import scan_frame, robot_model
from server import Feed, make_app


def scan(ranges=None):
    return S(
        header=S(frame_id='lidar_link', stamp=S(sec=10, nanosec=20)),
        angle_min=0.0,
        angle_increment=0.01,
        range_min=0.1,
        range_max=30.0,
        ranges=ranges if ranges is not None else [1.0, float('inf'), float('nan'), 0.01, 31.0],
    )


def transform():
    return S(translation=S(x=0.0, y=0.0, z=0.86), rotation=S(x=0.0, y=0.0, z=1.0, w=0.0))


class GeometryTests(unittest.TestCase):
    def test_invalid_ranges_and_tf_kept(self):
        frame = scan_frame(scan(), transform(), 'robot', 1, 10)
        self.assertEqual(frame['ranges'], [1.0, None, None, None, None])
        self.assertEqual(frame['transform']['rotation_xyzw'], [0, 0, 1, 0])
        json.dumps(frame, allow_nan=False)

    def test_bounded_sampling_preserves_angles(self):
        frame = scan_frame(scan([1.0] * 10000), transform(), 'robot', 1, 10)
        self.assertLessEqual(len(frame['ranges']), 2048)
        self.assertEqual(frame['stride'], 5)
        self.assertAlmostEqual(frame['angle_increment'], 0.05)

    def test_invalid_geometry_rejected(self):
        for message in [scan([]), scan()]:
            message.angle_increment = float('nan')
            with self.assertRaises(ValueError):
                scan_frame(message, transform(), 'robot', 1, 10)
        tf = transform()
        tf.rotation.w = 2
        with self.assertRaises(ValueError):
            scan_frame(scan(), tf, 'robot', 1, 10)

    def test_urdf_dimensions(self):
        root = Path(__file__).resolve().parents[3]
        model = robot_model(root / 'ros_ws/src/verter_description/urdf/verter_robot.urdf', 'robot')
        self.assertEqual(len(model['polygons']), 5)
        body = next(p for p in model['polygons'] if p['name'] == 'base_link')
        self.assertAlmostEqual(max(p[0] for p in body['points']), 0.175, places=5)
        self.assertAlmostEqual(min(p[0] for p in body['points']), -0.413628571, places=5)
        wheel = next(p for p in model['polygons'] if p['name'] == 'wheel_left')
        self.assertAlmostEqual(sum(p[1] for p in wheel['points']) / len(wheel['points']), 0.178, places=5)


class WebTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.nc = Mock()
        self.feed = Feed()
        self.client = TestClient(TestServer(make_app({'web_allowed_hosts': ['127.0.0.1']}, self.nc, self.feed)))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        self.nc.assert_not_called()
        self.assertEqual(self.nc.mock_calls, [])

    async def test_no_commands_and_guards(self):
        self.assertEqual((await self.client.get('/')).status, 200)
        self.assertTrue((await (await self.client.get('/api/lidar/status')).json())['stale'])
        self.assertEqual((await self.client.get('/', headers={'Host': 'bad.invalid'})).status, 403)
        self.assertEqual((await self.client.get('/api/lidar/ws', headers={'Origin': 'http://bad.invalid'})).status, 403)
        self.assertEqual((await self.client.post('/api/command', json={})).status, 400)

    async def test_nats_cache_to_websocket(self):
        value = {'v': 1, 'type': 'status', 'stale': False}
        await self.feed.receive(S(subject='robot.lidar.update.status', data=json.dumps(value).encode()))
        ws = await self.client.ws_connect('/api/lidar/ws')
        reply = json.loads((await ws.receive(timeout=2)).data)
        self.assertEqual(reply['type'], 'status')
        await ws.send_str('command')
        await ws.receive(timeout=2)
        await ws.close()

    async def test_old_cache_is_stale(self):
        self.feed.latest['status'] = (0, {'v': 1, 'type': 'status', 'stale': False})
        self.assertTrue((await (await self.client.get('/api/lidar/status')).json())['stale'])

    async def test_limit_and_malformed(self):
        for data in [b'[]', b'null', b'{}', b'x' * 140000]:
            await self.feed.receive(S(subject='robot.lidar.update.scan', data=data))
        self.assertEqual(self.feed.latest, {})
        self.feed.clients = 4
        self.assertEqual((await self.client.get('/api/lidar/ws')).status, 429)
