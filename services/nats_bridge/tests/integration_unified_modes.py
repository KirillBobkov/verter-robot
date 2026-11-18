"""Disposable NATS and fake Unix supervisor. No ROS imports, publishers or launch processes."""

import asyncio
import json
from pathlib import Path
import socket
import subprocess
import tempfile

import nats

from robot_nats.access import configure_client
from robot_nats.protocol import encode, envelope
from robot_nats.service import Service

ROOT = Path(__file__).resolve().parents[3]


async def main():
    with tempfile.TemporaryDirectory(prefix='verter-unified-modes-') as directory:
        base = Path(directory)
        with socket.socket() as port_probe:
            port_probe.bind(('127.0.0.1', 0))
            port = port_probe.getsockname()[1]
        state = {'ok': True, 'mode': 'idle', 'map': None, 'groups': {'hardware': 111}, 'last_exit': {}}
        calls = []

        async def supervisor(reader, writer):
            try:
                request = json.loads(await reader.readline())
                calls.append(request)
                if request['command'] == 'mode':
                    state['mode'] = request['mode']
                    if state['mode'] != 'idle':
                        state['groups']['mode'] = 222
                    else:
                        state['groups'].pop('mode', None)
                writer.write(encode(state) + b'\n')
                await writer.drain()
            finally:
                writer.close()
                await writer.wait_closed()

        unix = await asyncio.start_unix_server(supervisor, path=str(base / 'supervisor.sock'))
        config = {
            'robot_id': 'mode_test',
            'domain': 'MODE_TEST',
            'bucket': 'MODE_STATE',
            'nats_url': f'nats://127.0.0.1:{port}',
            'backend_user': 'backend',
            'backend_password': 'test-only-backend',
            'client_user': 'client',
            'client_password': 'test-only-client',
            'ros_enabled': False,
            'allow_manual_motion': False,
            'allow_mode_changes': True,
            'maps': {},
            'data_dir': str(base / 'data'),
            'project_root': str(base),
            'supervisor_socket': str(base / 'supervisor.sock'),
        }
        server = {
            'listen': f'127.0.0.1:{port}',
            'jetstream': {'domain': 'MODE_TEST', 'store_dir': str(base / 'js')},
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
            broker = subprocess.Popen([str(ROOT / 'vendor/nats/nats-server'), '-c', str(path)], stdout=log, stderr=log)
        service = client = None
        try:
            await asyncio.sleep(0.5)
            service = Service(config)
            await service.start()
            assert service.telemetry.thread is None
            service.telemetry.put('graph', {'nodes': ['esp32_ctrl'], 'cmd_vel_publishers': ['esp32_ctrl']})
            client = await nats.connect(config['nats_url'], user='client', password='test-only-client')
            body = envelope('mode.set', {'mode': 'mapping'})
            body.pop('action')
            subject = 'mode_test.service.command.mode_set'
            first = json.loads((await client.request(subject, encode(body), timeout=3)).data)
            assert first['ok'] and first['result']['operation']['status'] == 'running', first
            await service.modes.task
            assert service.snapshot()['mode']['actual'] == 'mapping'
            duplicate = json.loads((await client.request(subject, encode(body), timeout=3)).data)
            assert duplicate == first
            assert sum(call['command'] == 'mode' for call in calls) == 1
            before = list(calls)
            await asyncio.wait_for(service.close(), timeout=10)
            service = None
            assert calls == before, 'Bridge shutdown must not send mode/stop/shutdown RPC'
            assert state['mode'] == 'mapping'
            service = Service(config)
            await service.start()
            assert service.modes.state()['actual'] == 'mapping'
            assert sum(call['command'] == 'mode' for call in calls) == 1
            state['mode'] = 'navigation'  # Simulate a mode changed by CLI.
            await service.modes.refresh()
            assert service.modes.state()['actual'] == 'navigation'
            assert not service.modes.state()['stale']
            assert service.telemetry.thread is None
            print(
                'PASS isolated NATS -> fake supervisor; accepted/ready, dedup, CLI observation, bridge restart preserves mode; ROS disabled'
            )
        finally:
            if client:
                await client.close()
            if service:
                await asyncio.wait_for(service.close(), timeout=10)
            unix.close()
            await unix.wait_closed()
            broker.terminate()
            broker.wait(timeout=5)


if __name__ == '__main__':
    asyncio.run(main())
