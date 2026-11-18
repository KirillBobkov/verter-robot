"""HTTP access tests with NATS replaced: no ROS or robot connection."""

import json
import sys
import types
import unittest
from unittest.mock import AsyncMock, patch
from aiohttp.test_utils import TestClient, TestServer

try:
    import nats
except ModuleNotFoundError:
    # Permit offline tests on development PCs without the NATS client installed.
    nats = types.ModuleType('nats')
    nats.connect = AsyncMock()
    nats.errors = types.SimpleNamespace(TimeoutError=TimeoutError, NoRespondersError=RuntimeError)
    sys.modules['nats'] = nats

from robot_nats.web import make_app


class WebAccessTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.nc = types.SimpleNamespace(
            is_connected=True,
            close=AsyncMock(),
            request=AsyncMock(return_value=types.SimpleNamespace(data=json.dumps({'ok': True}).encode())),
        )
        self.mock_connect = patch('robot_nats.web.nats.connect', AsyncMock(return_value=self.nc))
        self.mock_connect.start()
        self.config = {
            'nats_url': 'nats://unused.invalid',
            'client_user': 'mock',
            'client_password': 'mock',
            'robot_id': 'mock_robot',
        }
        self.clients = []

    async def asyncTearDown(self):
        for client in self.clients:
            await client.close()
        self.mock_connect.stop()

    async def client(self, lan=False):
        config = dict(self.config)
        if lan:
            config['web_allowed_hosts'] = ['localhost', '127.0.0.1', '::1', '192.168.0.9']
        client = TestClient(TestServer(await make_app(config)))
        await client.start_server()
        self.clients.append(client)
        return client

    async def test_defaults_remain_local(self):
        client = await self.client()
        self.assertEqual((await client.get('/', headers={'Host': '127.0.0.1:8088'})).status, 200)
        self.assertEqual((await client.get('/', headers={'Host': '192.168.0.9:8088'})).status, 403)
        self.nc.request.assert_not_awaited()

    async def test_lan_and_loopback_with_no_commands(self):
        client = await self.client(lan=True)
        for host in ['localhost:8088', '127.0.0.1:8088', '[::1]:8088', '192.168.0.9:8088']:
            response = await client.get('/', headers={'Host': host})
            self.assertEqual(response.status, 200)
            self.assertEqual(response.headers['Cache-Control'], 'no-store')
        self.assertEqual((await client.get('/', headers={'Host': 'untrusted.invalid:8088'})).status, 403)
        self.nc.request.assert_not_awaited()

    async def test_origin_and_content_type_guards(self):
        client = await self.client(lan=True)
        good = {'Host': '192.168.0.9:8088', 'Origin': 'http://192.168.0.9:8088'}
        bad_origin = dict(good, Origin='http://other.invalid')
        self.assertEqual(
            (await client.post('/api/command', headers=bad_origin, json={'action': 'cmd_vel.stop'})).status, 403
        )
        self.assertEqual(
            (
                await client.post(
                    '/api/command',
                    headers=dict(good, **{'Sec-Fetch-Site': 'cross-site'}),
                    json={'action': 'cmd_vel.stop'},
                )
            ).status,
            403,
        )
        self.assertEqual((await client.post('/api/command', headers=good, data='text')).status, 415)
        self.nc.request.assert_not_awaited()
        self.assertEqual((await client.get('/api/drive', headers=good)).status, 200)
        subject = self.nc.request.await_args.args[0]
        self.assertEqual(subject, 'mock_robot.cmd_vel.command.status')


if __name__ == '__main__':
    unittest.main()
