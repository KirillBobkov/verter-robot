import json
from types import SimpleNamespace
import unittest
from unittest.mock import AsyncMock, patch

from aiohttp.test_utils import TestClient, TestServer
from robot_nats.map_data import MapStore
from robot_nats.protocol import encode, envelope
from robot_nats.service import Service
from robot_nats.web import make_app


class MapApiTests(unittest.IsolatedAsyncioTestCase):
    async def test_reads_without_ros_or_command_journal(self):
        service = Service.__new__(Service)
        service.mapping = SimpleNamespace(store=MapStore('test'))
        service.map_saver = SimpleNamespace(snapshot=lambda: {'status': 'idle'})
        result = await service.execute(encode(envelope('map.get', {'part': 'metadata'})))
        self.assertTrue(result['ok'])
        self.assertFalse(result['result']['available'])

    async def test_http_is_only_a_nats_read_adapter(self):
        nc = SimpleNamespace(is_connected=True, close=AsyncMock())
        nc.request = AsyncMock(return_value=SimpleNamespace(data=encode({'ok': True, 'result': {}})))
        config = {'nats_url': 'mock', 'client_user': 'mock', 'client_password': 'mock', 'robot_id': 'test'}
        with patch('robot_nats.web.nats.connect', new=AsyncMock(return_value=nc)):
            client = TestClient(TestServer(await make_app(config)))
        await client.start_server()
        try:
            self.assertEqual((await client.get('/map')).status, 200)
            self.assertEqual(nc.request.await_count, 0)
            self.assertEqual((await client.get('/api/map?part=chunk&map_id=s&revision=2&index=0')).status, 200)
            args = nc.request.call_args.args
            self.assertEqual(args[0], 'test.map.command.get')
            body = json.loads(args[1])
            self.assertNotIn('action', body)
            self.assertEqual(body['args'], {'part': 'chunk', 'map_id': 's', 'revision': 2, 'index': 0})
            response = await client.get('/api/map?part=saved_metadata&name=room_1')
            self.assertEqual(response.status, 200)
            subject, data = nc.request.await_args.args
            self.assertEqual(subject, 'test.map.command.get')
            self.assertEqual(json.loads(data)['args'], {'part': 'saved_metadata', 'name': 'room_1'})
            response = await client.post('/api/command', json=envelope('map.delete', {'name': 'room_1'}))
            self.assertEqual(response.status, 200)
            subject, data = nc.request.await_args.args
            self.assertEqual(subject, 'test.map.command.delete')
            self.assertEqual(json.loads(data)['args'], {'name': 'room_1'})
            count = nc.request.await_count
            self.assertEqual((await client.get('/api/map?index=nan')).status, 400)
            self.assertEqual((await client.get('/api/map?command=start')).status, 400)
            self.assertEqual((await client.get('/api/map', headers={'Origin': 'http://other'})).status, 403)
            self.assertEqual(nc.request.await_count, count)
        finally:
            await client.close()
