"""Disposable NATS and HTTP, synthetic grids only; no ROS or robot sockets."""

import asyncio
import base64
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
from tempfile import TemporaryDirectory
import time
import zlib

from aiohttp import ClientSession, web
import nats

from robot_nats.access import configure_client
from robot_nats.costmap_data import CostmapStore
from robot_nats.protocol import encode, envelope
from robot_nats.service import Service
from robot_nats.web import make_app


class ReadOnlySupervisor:
    def __init__(self):
        self.calls = []

    def call(self, request, timeout):
        self.calls.append(request)
        assert request == {'command': 'status'}, request
        return {'ok': True, 'mode': 'idle', 'groups': {}, 'last_exit': {}}


def free_port():
    with socket.socket() as channel:
        channel.bind(('127.0.0.1', 0))
        return channel.getsockname()[1]


async def main():
    binary = os.environ['VERTER_TEST_NATS_SERVER']
    port = free_port()
    assert port not in (4222, 8088)
    with TemporaryDirectory(prefix='verter-costmap-api-') as directory:
        root = Path(directory)
        config = {
            'robot_id': 'costmap_test',
            'domain': 'COSTMAP_TEST',
            'bucket': 'COSTMAP_TEST_STATE',
            'nats_url': 'nats://127.0.0.1:' + str(port),
            'backend_user': 'backend',
            'backend_password': 'test-only-backend',
            'client_user': 'client',
            'client_password': 'test-only-client',
            'data_dir': str(root / 'state'),
            'project_root': str(root),
            'maps': {},
            'ros_enabled': False,
            'allow_mode_changes': False,
            'allow_manual_motion': False,
            'allow_initial_pose': False,
            'supervisor_socket': str(root / 'no-real-supervisor.sock'),
            'web_allowed_hosts': ['127.0.0.1'],
        }
        server = {
            'listen': '127.0.0.1:' + str(port),
            'jetstream': {'domain': config['domain'], 'store_dir': str(root / 'jetstream')},
            'authorization': {'users': [
                {'user': 'backend', 'password': config['backend_password'], 'permissions': {
                    'publish': ['costmap_test.>', 'robots.costmap_test.>', '_INBOX.>', '$JS.>', '$KV.>'],
                    'subscribe': ['costmap_test.>', 'robots.costmap_test.>', '_INBOX.>', '$KV.>'],
                }},
                {'user': 'client', 'password': config['client_password'], 'permissions': {
                    'publish': [], 'subscribe': ['_INBOX.>'],
                }},
            ]},
        }
        configure_client(server, config)
        server_path = root / 'server.json'
        server_path.write_text(json.dumps(server))
        with (root / 'server.log').open('wb') as log:
            process = subprocess.Popen([binary, '-c', str(server_path)], stdout=log, stderr=log)

        service = client = runner = None
        controls = []
        updates = []
        try:
            for _ in range(100):
                assert process.poll() is None, (root / 'server.log').read_text()
                try:
                    reader, writer = await asyncio.open_connection('127.0.0.1', port)
                    writer.close()
                    await writer.wait_closed()
                    break
                except OSError:
                    await asyncio.sleep(0.05)
            else:
                raise RuntimeError('Disposable NATS did not start')

            service = Service(config)
            supervisor = ReadOnlySupervisor()
            service.modes.client = supervisor
            frozen = [0.0]
            service.costmaps.store = CostmapStore('costmap_test', clock=lambda: frozen[0])
            await service.start()
            assert service.telemetry.thread is None
            assert service.mapping.thread is None
            assert service.costmaps.thread is None
            client = await nats.connect(config['nats_url'], user='client', password=config['client_password'])

            async def record_update(message):
                updates.append((message.subject, json.loads(message.data)))

            async def record_control(message):
                controls.append(message.subject)

            await client.subscribe('costmap_test.costmap.update.*', cb=record_update)
            for subject in (
                'costmap_test.cmd_vel.command.>',
                'costmap_test.service.command.mode_set',
                'costmap_test.localization.command.initialize',
            ):
                await service.nc.subscribe(subject, cb=record_control)
            await client.flush()
            await service.nc.flush()

            store = service.costmaps.store
            mode = {'actual': 'navigation', 'pid': 222, 'stale': False}
            store.set_scope('synthetic-nav', mode, 'synthetic-map', 'floor')
            cells = bytes([0, 253, 254, 255, 30, 150])
            for kind, source in (('local', 'odom'), ('global', 'map')):
                store.put_grid(
                    'synthetic-nav', kind, 3, 2, 0.05,
                    {'x': -1.0, 'y': 2.0, 'yaw': 0.3}, cells, time.time(), 0.0, source,
                )

            app = await make_app(config)
            runner = web.AppRunner(app)
            await runner.setup()
            http_port = free_port()
            assert http_port not in (4222, 8088)
            site = web.TCPSite(runner, '127.0.0.1', http_port)
            await site.start()
            origin = 'http://127.0.0.1:' + str(http_port)
            async with ClientSession() as http:
                async with http.get(origin + '/api/costmap') as response:
                    state = await response.json()
                assert state['ok'] and state['result']['local']['available'], state
                metadata = state['result']['local']
                packed = bytearray()
                for index in range(metadata['chunks']):
                    args = {
                        'part': 'chunk', 'kind': 'local', 'costmap_id': metadata['costmap_id'],
                        'revision': metadata['revision'], 'index': index,
                    }
                    async with http.get(origin + '/api/costmap', params=args) as response:
                        chunk = await response.json()
                    assert chunk['ok'], chunk
                    assert chunk['result']['map_id'] == 'synthetic-map'
                    assert chunk['result']['kind'] == 'local'
                    packed.extend(base64.b64decode(chunk['result']['data_b64']))
                assert hashlib.sha256(packed).hexdigest() == metadata['sha256']
                assert zlib.decompress(packed) == cells
                assert metadata['origin'] == {'x': -1.0, 'y': 2.0, 'yaw': 0.3}

                async with http.post(origin + '/api/command', json=envelope('costmap.get')) as response:
                    assert (await response.json())['ok']
                async with http.get(origin + '/api/costmap?linear_x=1') as response:
                    assert response.status == 400
                async with http.get(origin + '/api/costmap?part=chunk&kind=local&revision=bad') as response:
                    assert response.status == 400

            message = envelope('costmap.get', {'part': 'metadata', 'kind': 'global'})
            message.pop('action')
            raw = encode(message)
            for _ in range(2):
                reply = await client.request('costmap_test.costmap.command.get', raw, timeout=5)
                assert json.loads(reply.data)['ok']
            assert service.db.execute('SELECT count(*) FROM commands').fetchone()[0] == 0
            await asyncio.sleep(1.1)
            assert {item[0] for item in updates} == {
                'costmap_test.costmap.update.local', 'costmap_test.costmap.update.global'
            }
            for kind in ('local', 'global'):
                saved = json.loads((await service.kv.get('costmap.' + kind)).value)
                assert saved['kind'] == kind and 'data_b64' not in saved
            assert controls == [], controls
            assert all(request == {'command': 'status'} for request in supervisor.calls)
            assert service.catalog['contract']['revision'] == '1.8'
            report = {
                'ok': True, 'ros_enabled': False, 'isolated_nats_port': port,
                'isolated_http_port': http_port, 'cells_roundtrip': list(cells),
                'updates_and_kv': True, 'journal_rows': 0,
                'control_requests': controls, 'supervisor_calls': supervisor.calls,
            }
            print(json.dumps(report, indent=2))
        finally:
            if runner:
                await runner.cleanup()
            if client:
                await client.close()
            if service:
                await service.close()
            process.terminate()
            process.wait(timeout=10)


if __name__ == '__main__':
    asyncio.run(main())
