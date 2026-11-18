from pathlib import Path
import json
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from aiohttp.test_utils import TestClient, TestServer
from robot_nats.history import History
from robot_nats.protocol import encode, envelope
from robot_nats.service import Service
from robot_nats.web import make_app
from robot_nats.access import configure_client


class HistoryApiTests(unittest.IsolatedAsyncioTestCase):
    async def test_history_reads_do_not_create_command_journal_or_control(self):
        with tempfile.TemporaryDirectory() as directory:
            service = Service.__new__(Service)
            service.history = History(Path(directory) / 'history.sqlite')
            # No motion, telemetry, NATS or command-journal object exists in this fixture.
            response = await service.execute(encode(envelope('battery.history', {'period_s': 3600})))
            self.assertTrue(response['ok'], response)
            self.assertEqual(response['result']['series'], {})

    async def test_same_origin_http_routes_only_read_history(self):
        nc = SimpleNamespace(is_connected=True, close=AsyncMock())
        nc.request = AsyncMock(return_value=SimpleNamespace(data=encode({'ok': True, 'result': {}})))
        config = {'nats_url': 'nats://mock', 'client_user': 'mock', 'client_password': 'mock',
                  'robot_id': 'test_robot', 'web_allowed_hosts': ['127.0.0.1']}
        with patch('robot_nats.web.nats.connect', new=AsyncMock(return_value=nc)):
            client = TestClient(TestServer(await make_app(config)))
            await client.start_server()
            try:
                self.assertEqual((await client.get('/battery-history')).status, 200)
                self.assertEqual(nc.request.await_count, 0)
                self.assertEqual((await client.get('/api/battery/history?period_s=3600&max_points=20')).status, 200)
                args = nc.request.call_args.args
                self.assertEqual(args[0], 'test_robot.battery.command.history')
                body = json.loads(args[1])
                self.assertNotIn('action', body)
                self.assertEqual(body['args'], {'period_s': 3600, 'max_points': 20})
                count = nc.request.await_count
                self.assertEqual((await client.get('/api/battery/history?period_s=oops')).status, 400)
                self.assertEqual((await client.get('/api/battery/history?other=1')).status, 400)
                self.assertEqual((await client.get('/api/battery/history', headers={'Origin': 'http://other'})).status, 403)
                self.assertEqual(nc.request.await_count, count)
            finally:
                await client.close()

    def test_history_acl_is_local_and_does_not_grant_bms_control(self):
        server = {'authorization': {'users': [
            {'user': name, 'permissions': {'publish': [], 'subscribe': []}}
            for name in ['client', 'backend', 'bms']
        ]}}
        config = {'client_user': 'client', 'backend_user': 'backend', 'robot_id': 'test_robot',
                  'bucket': 'TEST', 'domain': 'LOCAL', 'allow_manual_motion': False}
        configure_client(server, config)
        client, backend, bms = server['authorization']['users']
        self.assertIn('test_robot.battery.command.history', client['permissions']['publish'])
        self.assertIn('test_robot.battery.command.history', backend['permissions']['subscribe'])
        self.assertEqual(bms['permissions'], {'publish': [], 'subscribe': []})
        self.assertNotIn('test_robot.cmd_vel.command.set', client['permissions']['publish'])
        del backend['permissions']
        configure_client(server, config)
        self.assertNotIn('permissions', backend)
