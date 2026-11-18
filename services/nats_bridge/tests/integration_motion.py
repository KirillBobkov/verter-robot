"""Isolated NATS+HTTP test. ROS is disabled; the velocity sink is a Python list."""

import asyncio
import json
from pathlib import Path
import subprocess
import tempfile
import time
import uuid
from aiohttp import web, ClientSession
from robot_nats.access import configure_client
from robot_nats.service import Service
from robot_nats.web import make_app

ROOT = Path(__file__).resolve().parents[3]


async def main():
    with tempfile.TemporaryDirectory(prefix='verter-motion-test-') as folder:
        base = Path(folder)
        config = {
            'robot_id': 'mock_robot',
            'domain': 'MOTION_TEST',
            'bucket': 'MOTION_STATE',
            'nats_url': 'nats://127.0.0.1:4424',
            'backend_user': 'backend',
            'backend_password': 'test-only-backend',
            'client_user': 'client',
            'client_password': 'test-only-client',
            'ros_enabled': False,
            'supervisor_socket': str(base / 'missing-supervisor.sock'),
            'allow_manual_motion': True,
            'allow_mode_changes': False,
            'maps': {},
            'data_dir': str(base / 'data'),
        }
        server = {
            'listen': '127.0.0.1:4424',
            'jetstream': {'domain': 'MOTION_TEST', 'store_dir': str(base / 'js')},
            'authorization': {
                'users': [
                    {'user': 'backend', 'password': 'test-only-backend'},
                    {
                        'user': 'client',
                        'password': 'test-only-client',
                        'permissions': {
                            'publish': ['mock_robot.service.command.metrics_get'],
                            'subscribe': ['_INBOX.>'],
                        },
                    },
                ]
            },
        }
        configure_client(server, config)
        path = base / 'server.json'
        path.write_text(json.dumps(server))
        with (base / 'server.log').open('w') as log:
            process = subprocess.Popen([str(ROOT / 'vendor/nats/nats-server'), '-c', str(path)], stdout=log, stderr=log)
        service = None
        runner = None
        try:
            await asyncio.sleep(0.5)
            service = Service(config)
            await service.start()
            assert service.telemetry.thread is None, 'ROS must not be running in this test'
            service.motion.ready = True
            output = []
            sink = lambda x, z: output.append((x, z))
            runner = web.AppRunner(await make_app(config))
            await runner.setup()
            await web.TCPSite(runner, '127.0.0.1', 18089).start()
            async with ClientSession() as client:
                for _ in range(3):
                    async with client.get('http://127.0.0.1:18089/api/drive') as reply:
                        assert (await reply.json())['result']['active'] is False
                    service.motion.tick(sink)
                assert output == [], 'Status polling must never publish even zero velocity'

                async def command(operation, args):
                    body = {
                        'v': 1,
                        'id': str(uuid.uuid4()),
                        'issued_at': time.time(),
                        'ttl_s': 1,
                        'action': 'cmd_vel.' + operation,
                        'args': args,
                    }
                    async with client.post('http://127.0.0.1:18089/api/command', json=body) as reply:
                        assert reply.status == 200
                        return await reply.json()

                session = str(uuid.uuid4())
                args = {'session_id': session, 'seq': 1, 'linear_x': 0.01, 'angular_z': -0.05}
                assert (await command('set', args))['ok']
                service.motion.tick(sink)
                assert output == [(0.01, -0.05)]
                assert (await command('stop', {}))['ok']
                service.motion.tick(sink)
                assert output[-1] == (0.0, 0.0)
                assert not (await command('set', dict(args, seq=2)))['ok']
                count = len(output)
                service.motion.tick(sink)
                assert len(output) == count
            print(
                'PASS HTTP -> isolated NATS -> fake velocity sink; polling silent, signs, stop, replay rejected; ROS disabled'
            )
        finally:
            if runner:
                await runner.cleanup()
            if service:
                await service.close()
            process.terminate()
            process.wait(timeout=5)


if __name__ == '__main__':
    asyncio.run(main())
