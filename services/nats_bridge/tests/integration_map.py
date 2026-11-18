"""Own disposable NATS + HTTP; synthetic map/pose only. ROS and motion are disabled."""

import asyncio
import base64
import hashlib
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import zlib

from aiohttp import web, ClientSession
import nats

from robot_nats.access import configure_client
from robot_nats.protocol import encode, envelope
from robot_nats.service import Service
from robot_nats.web import make_app
from test_map_tools import FakeSupervisor, fake_serializer

ROOT = Path(__file__).resolve().parents[3]


async def main():
    with tempfile.TemporaryDirectory(prefix='verter-map-test-') as directory:
        base = Path(directory)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        config = {
            'robot_id': 'map_test',
            'domain': 'MAP_TEST',
            'bucket': 'MAP_STATE',
            'nats_url': f'nats://127.0.0.1:{port}',
            'backend_user': 'backend',
            'backend_password': 'test-only-backend',
            'client_user': 'client',
            'client_password': 'test-only-client',
            'ros_enabled': False,
            'allow_manual_motion': False,
            'allow_mode_changes': True,
            'maps': {},
            'project_root': str(base),
            'data_dir': str(base / 'data'),
            'supervisor_socket': str(base / 'missing-supervisor.sock'),
        }
        server = {
            'listen': f'127.0.0.1:{port}',
            'jetstream': {'domain': 'MAP_TEST', 'store_dir': str(base / 'js')},
            'authorization': {
                'users': [
                    {'user': 'backend', 'password': 'test-only-backend'},
                    {
                        'user': 'client',
                        'password': 'test-only-client',
                        'permissions': {'publish': [], 'subscribe': ['_INBOX.>']},
                    },
                ]
            },
        }
        configure_client(server, config)
        path = base / 'nats.json'
        path.write_text(json.dumps(server))
        with (base / 'nats.log').open('w') as log:
            process = subprocess.Popen([str(ROOT / 'vendor/nats/nats-server'), '-c', str(path)], stdout=log, stderr=log)
        service = runner = client = None
        try:
            await asyncio.sleep(0.5)
            service = Service(config)
            supervisor = FakeSupervisor()
            service.modes.client = supervisor
            service.map_saver.serializer = fake_serializer
            await service.start()
            assert service.telemetry.thread is None and service.mapping.thread is None
            client = await nats.connect(config['nats_url'], user='client', password='test-only-client')
            updates = []
            controls = []

            async def update(message):
                updates.append((message.subject, json.loads(message.data)))

            async def control(message):
                controls.append(message.subject)

            await client.subscribe('map_test.map.update.*', cb=update)
            await client.flush()
            await service.nc.subscribe('map_test.cmd_vel.command.>', cb=control)
            store = service.mapping.store
            store.set_scope('synthetic-session', {'actual': 'mapping', 'stale': False})
            store.put_grid(
                'synthetic-session', 3, 2, 0.05, {'x': -1, 'y': 2, 'yaw': 0.3}, [-1, 0, 100, 25, 50, 75], 1000
            )
            store.put_pose('synthetic-session', 1000, 1.25, -0.6, 0.5, 0)
            store.put_scan('synthetic-session', 1000, [[2, 3]], 0, 'laser', 1665)
            store.note_scan(1665, 0.004, 0.1)

            async def request(args):
                body = envelope('map.get', args)
                body.pop('action')
                reply = await client.request('map_test.map.command.get', encode(body), timeout=3)
                value = json.loads(reply.data)
                assert value['ok'], value
                return value['result']

            metadata = await request({'part': 'metadata'})
            chunk = await request(
                {'part': 'chunk', 'map_id': metadata['map_id'], 'revision': metadata['revision'], 'index': 0}
            )
            packed = base64.b64decode(chunk['data_b64'])
            assert hashlib.sha256(packed).hexdigest() == metadata['sha256']
            assert list(zlib.decompress(packed)) == [255, 0, 100, 25, 50, 75]
            pose = await request({'part': 'pose'})
            assert pose['valid'] and pose['x'] == 1.25 and pose['frame_id'] == 'map'
            scan = await request({'part': 'scan'})
            assert scan['valid'] and scan['points'] == [[2, 3]]
            assert (await request({'part': 'diagnostics'}))['samples_10s'] == 1
            runner = web.AppRunner(await make_app(config))
            await runner.setup()
            site = web.TCPSite(runner, '127.0.0.1', 0)
            await site.start()
            http_port = runner.addresses[0][1]
            async with ClientSession() as http:
                async with http.get(f'http://127.0.0.1:{http_port}/api/map?part=metadata') as response:
                    body = await response.json()
                    assert body['result']['sha256'] == metadata['sha256']
                body = envelope('map.save', {'name': 'synthetic_save'})
                async with http.post(f'http://127.0.0.1:{http_port}/api/command', json=body) as response:
                    accepted = await response.json()
                    assert accepted['ok'] and accepted['result']['name'] == 'synthetic_save', accepted
                await service.map_saver.task
                assert (await request({'part': 'save_status'}))['status'] == 'succeeded'
                async with http.post(f'http://127.0.0.1:{http_port}/api/command', json=body) as response:
                    duplicate = await response.json()
                    assert duplicate == accepted
                assert sum(r['command'] == 'save-map' for r in supervisor.calls) == 1
                assert all(r['command'] in ('status', 'save-map') for r in supervisor.calls)
                saved = (await request({'part': 'saved'}))['maps'][0]
                assert saved['pose_graph'] and saved['grid'] and not saved['active']
                async with http.get(
                    f'http://127.0.0.1:{http_port}/api/map?part=saved_metadata&name=synthetic_save'
                ) as response:
                    saved_metadata = (await response.json())['result']
                saved_chunk = await request({
                    'part': 'saved_chunk',
                    'name': 'synthetic_save',
                    'map_id': saved_metadata['map_id'],
                    'revision': saved_metadata['revision'],
                    'index': 0,
                })
                assert zlib.decompress(base64.b64decode(saved_chunk['data_b64'])) == bytes([100])
                delete = envelope('map.delete', {'name': 'synthetic_save'})
                previous_call_count = len(supervisor.calls)
                async with http.post(f'http://127.0.0.1:{http_port}/api/command', json=delete) as response:
                    deleted = await response.json()
                    assert deleted['ok'] and deleted['result'] == {'deleted': 'synthetic_save'}, deleted
                async with http.post(f'http://127.0.0.1:{http_port}/api/command', json=delete) as response:
                    assert await response.json() == deleted
                assert (await request({'part': 'saved'}))['maps'] == []
                assert all(r['command'] == 'status' for r in supervisor.calls[previous_call_count:])
                store.invalidate_pose('test_no_tf')
                async with http.get(f'http://127.0.0.1:{http_port}/api/map?part=pose') as response:
                    body = await response.json()
                    assert not body['result']['valid'] and 'x' not in body['result']
            await asyncio.sleep(1.2)
            assert any(subject.endswith('.metadata') for subject, value in updates)
            assert any(subject.endswith('.scan') for subject, value in updates)
            assert any(subject.endswith('.pose') for subject, value in updates)
            kv = json.loads((await service.kv.get('map.live')).value)
            assert kv['map_id'] == 'synthetic-session'
            assert controls == []
            assert service.telemetry.thread is None and service.mapping.thread is None
            print('PASS isolated NATS/HTTP map scan/save/preview/delete, ACL, idempotency, Core/KV; no ROS or motion')
        finally:
            if runner:
                await runner.cleanup()
            if client:
                await client.close()
            if service:
                await asyncio.wait_for(service.close(), timeout=10)
            process.terminate()
            process.wait(timeout=5)


if __name__ == '__main__':
    asyncio.run(main())
