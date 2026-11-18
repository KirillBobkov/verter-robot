"""Isolated NATS + HTTP API with fake ROS and supervisor; never uses robot ports."""

import asyncio
import json
import os
from pathlib import Path
import socket
import subprocess
from tempfile import TemporaryDirectory
import time

from aiohttp import ClientSession, web

from robot_nats.access import configure_client
from robot_nats.protocol import envelope
from robot_nats.service import Service
from robot_nats.web import make_app


def free_port():
    with socket.socket() as channel:
        channel.bind(('127.0.0.1', 0))
        return channel.getsockname()[1]


async def main():
    binary = os.environ['VERTER_TEST_NATS_SERVER']
    port = free_port()
    assert port not in (4222, 8088)
    with TemporaryDirectory(prefix='verter-localization-api-') as temporary:
        base = Path(temporary)
        maps = base / 'data/maps'
        maps.mkdir(parents=True)
        yaml_path = maps / 'test_map.yaml'
        yaml_path.write_text('image: test_map.pgm\nresolution: 0.2\norigin: [-2, -2, 0]\n')
        (maps / 'test_map.pgm').write_bytes(b'P5\n20 20\n255\n' + bytes([254]) * 400)
        config = {
            'robot_id': 'test_robot', 'domain': 'TEST_LOCALIZATION', 'bucket': 'TEST_LOCALIZATION_STATE',
            'nats_url': 'nats://127.0.0.1:' + str(port),
            'backend_user': 'backend', 'backend_password': 'test-backend-only',
            'client_user': 'client', 'client_password': 'test-client-only',
            'data_dir': str(base / 'state'), 'project_root': str(base),
            'maps': {'test_map': str(yaml_path)}, 'ros_enabled': False,
            'allow_mode_changes': False, 'allow_initial_pose': True, 'allow_manual_motion': False,
            'supervisor_socket': str(base / 'missing-production-supervisor.sock'),
            'web_allowed_hosts': ['127.0.0.1'],
        }
        server_config = {
            'listen': '127.0.0.1:' + str(port),
            'jetstream': {'domain': config['domain'], 'store_dir': str(base / 'jetstream')},
            'authorization': {'users': [
                {'user': 'backend', 'password': 'test-backend-only', 'permissions': {
                    'publish': ['test_robot.>', 'robots.test_robot.>', '_INBOX.>', '$JS.>', '$KV.>'],
                    'subscribe': ['test_robot.>', 'robots.test_robot.>', '_INBOX.>', '$KV.>'],
                }},
                {'user': 'client', 'password': 'test-client-only', 'permissions': {
                    'publish': [], 'subscribe': ['_INBOX.>'],
                }},
            ]},
        }
        configure_client(server_config, config)
        server_file = base / 'server.json'
        server_file.write_text(json.dumps(server_config))
        with (base / 'server.log').open('wb') as log:
            process = subprocess.Popen([binary, '-c', str(server_file)], stdout=log, stderr=log)
        service = runner = None
        try:
            for _ in range(100):
                if process.poll() is not None:
                    raise RuntimeError((base / 'server.log').read_text())
                try:
                    reader, writer = await asyncio.open_connection('127.0.0.1', port)
                    writer.close()
                    await writer.wait_closed()
                    break
                except OSError:
                    await asyncio.sleep(0.05)
            service = Service(config)

            class FakeSupervisor:
                def call(self, request, timeout):
                    assert request == {'command': 'status'}, 'Only fake status is allowed'
                    return {'ok': True, 'mode': 'localization', 'map': str(yaml_path),
                            'groups': {'mode': 123}, 'last_exit': {}}

            service.modes.client = FakeSupervisor()
            publications = []

            def fake_publish(pose, guard):
                guard()
                publications.append(dict(pose))
                return {'published_at': time.time()}

            service.localization.publisher = fake_publish
            await service.start()
            service.localization.snapshot()

            def observations():
                stamp = time.time()
                service.telemetry.put('localization', {
                    'x': 0.6, 'y': -0.2, 'yaw': 0.0, 'frame_id': 'map',
                    'stamp': stamp, 'covariance': [0.0] * 36,
                })
                store = service.mapping.store
                store.set_scope('fake-map-session', service.modes.state())
                store.put_grid('fake-map-session', 20, 20, 0.2, {'x': -2, 'y': -2, 'yaw': 0}, [0] * 400, stamp)
                store.put_pose('fake-map-session', stamp, 0.6, -0.2, 0.0, 0.0)
                store.put_scan('fake-map-session', stamp, [[1, 1]], 0.0, 'lidar_link', 1800)

            observations()
            runner = web.AppRunner(await make_app(config))
            await runner.setup()
            site = web.TCPSite(runner, '127.0.0.1', 0)
            await site.start()
            http_port = site._server.sockets[0].getsockname()[1]
            async with ClientSession() as client:
                async def command(action, args=None, body=None):
                    async with client.post(
                        f'http://127.0.0.1:{http_port}/api/command', json=body or envelope(action, args)
                    ) as response:
                        assert response.status == 200, await response.text()
                        return await response.json()

                status = await command('localization.get')
                assert status['ok'] and not status['result']['verified']
                assert publications == []
                state = status['result']
                init = envelope('localization.initialize', {
                    'map': 'test_map', 'session_id': state['session_id'], 'x': 0.5, 'y': -0.2, 'yaw': 0.0,
                })
                first = await command(None, body=init)
                again = await command(None, body=init)
                assert first['ok'] and again == first, (first, again)
                assert len(publications) == 1
                observations()
                status = (await command('localization.get'))['result']
                confirmed = await command('localization.confirm', {
                    'map': 'test_map', 'session_id': status['session_id'],
                    'initialization_id': status['initialization_id'],
                })
                assert confirmed['ok'] and confirmed['result']['verified'], confirmed
                point = await command('waypoint.save', {'map': 'test_map', 'id': 'here', 'label': 'Здесь', 'source': 'current'})
                assert point['ok'] and point['result']['waypoint']['x'] == 0.6, point
                route = await command('route.save', {
                    'map': 'test_map', 'id': 'round', 'label': 'Порядок', 'kind': 'through', 'points': ['here', 'here'],
                })
                assert route['ok'], route
                blocked = await command('waypoint.delete', {'map': 'test_map', 'id': 'here'})
                assert not blocked['ok'] and blocked['error'] == 'waypoint_is_used_by_route', blocked
                listed = await command('route.list', {'map': 'test_map'})
                assert listed['result']['routes'][0]['points'] == ['here', 'here']
                async with client.post(
                    f'http://127.0.0.1:{http_port}/api/command',
                    json=envelope('route.execute', {'map': 'test_map', 'id': 'round'}),
                ) as response:
                    assert response.status == 400
                assert len(publications) == 1
                metrics = await command('metrics.get')
                assert metrics['result']['route.catalog']['value']['map'] == 'test_map'
                assert len(metrics['result']['waypoint.catalog']['value']['waypoints']) == 1
            print(json.dumps({
                'ok': True, 'transport': 'isolated real NATS + HTTP', 'ros_enabled': False,
                'fake_initialpose_publications': len(publications), 'robot_calls': [],
                'checks': ['request/reply ACL', 'explicit initialpose once', 'idempotent retry',
                           'operator confirmation', 'save current point', 'ordered route',
                           'referenced point delete guard', 'catalog metrics', 'no route execution'],
            }))
        finally:
            if runner:
                await runner.cleanup()
            if service:
                await service.close()
            process.terminate()
            process.wait(timeout=10)


if __name__ == '__main__':
    asyncio.run(main())
