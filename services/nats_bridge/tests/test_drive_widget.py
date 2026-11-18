import unittest
from pathlib import Path

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
from robot_nats.drive_widget import mount_drive_widget


class WidgetProxyTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.commands = []

        async def backend(request):
            if request.method == 'POST':
                self.commands.append(await request.json())
            return web.json_response({'ok': True, 'result': {'active': False}})

        app = web.Application()
        app.router.add_get('/api/drive', backend)
        app.router.add_post('/api/command', backend)
        self.backend = TestServer(app)
        await self.backend.start_server()
        app = web.Application()
        mount_drive_widget(app, Path(__file__).resolve().parents[3], self.backend.port)
        self.client = TestClient(TestServer(app))
        await self.client.start_server()

    async def asyncTearDown(self):
        await self.client.close()
        await self.backend.close()

    async def test_read_only_load(self):
        for path in ['/api/drive', '/static/drive.js', '/static/drive-widget.js', '/static/drive-widget.css']:
            response = await self.client.get(path)
            self.assertEqual(response.status, 200, path)
            await response.read()
        self.assertEqual(self.commands, [])

    async def test_forward_without_rewrite(self):
        body = dict(v=1, id='test-only', issued_at=123, ttl_s=1, action='cmd_vel.set', args={'linear_x': 0.01})
        response = await self.client.post('/api/command', json=body)
        self.assertEqual(response.status, 200)
        self.assertEqual(self.commands, [body])

    async def test_reject_and_no_retry(self):
        for body in [{}, {'action': 'mode.set'}, []]:
            self.assertEqual((await self.client.post('/api/command', json=body)).status, 400)
        self.assertEqual((await self.client.post('/api/command', data='{}')).status, 415)
        response = await self.client.post(
            '/api/command', data='x' * 16385, headers={'Content-Type': 'application/json'}
        )
        self.assertEqual(response.status, 413)
        self.assertEqual(self.commands, [])
        await self.backend.close()
        self.assertEqual((await self.client.get('/api/drive')).status, 503)

    async def test_chunked_json(self):
        async def chunks():
            yield b'{"action":'
            yield b'"cmd_vel.status"}'

        response = await self.client.post('/api/command', data=chunks(), headers={'Content-Type': 'application/json'})
        self.assertEqual(response.status, 200)
        self.assertEqual(self.commands, [{'action': 'cmd_vel.status'}])
