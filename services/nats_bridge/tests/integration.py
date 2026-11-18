"""Real two-server leaf integration, isolated ports/data, no ROS or robot commands."""

import asyncio
from contextlib import suppress
import json
from pathlib import Path
import subprocess
import tempfile
import time
from aiohttp import ClientSession, web
import nats
from robot_nats.protocol import encode, envelope
from robot_nats.service import Service
from robot_nats.web import make_app
from robot_nats.access import configure_client

ROOT = Path(__file__).resolve().parents[3]


async def main():
    original = json.loads((ROOT / 'config/nats/bridge.json').read_text())
    binary = str(ROOT / 'vendor/nats/nats-server')
    with tempfile.TemporaryDirectory(prefix='verter-nats-test-') as tmp:
        base = Path(tmp)
        processes = []
        clients = []
        service = None
        runner = None

        def launch(name, cfg):
            path = base / (name + '.conf')
            path.write_text(json.dumps(cfg))
            log = (base / (name + '.log')).open('ab')
            p = subprocess.Popen([binary, '-c', str(path)], stdout=log, stderr=log)
            log.close()
            processes.append(p)
            return p

        def stop(p):
            if p.poll() is None:
                p.terminate()
                p.wait(timeout=8)

        cfg = dict(
            original,
            robot_id='test_robot',
            domain='TEST_LEAF',
            bucket='TEST_STATE',
            ros_enabled=False,
            allow_mode_changes=False,
            supervisor_socket=str(base / 'missing-supervisor.sock'),
            data_dir=str(base / 'backend'),
            nats_url='nats://127.0.0.1:4422',
            web_port=18088,
            maps={},
        )
        server = json.loads((ROOT / 'config/nats/server.conf').read_text())
        # Same permissions as installed config, with test namespace substituted.
        server = json.loads(
            json.dumps(server)
            .replace(original['domain'], 'TEST_LEAF')
            .replace(original['bucket'], 'TEST_STATE')
            .replace('robots.TEST_LEAF', 'robots.test_robot')
        )
        configure_client(server, cfg)
        server.update(server_name='test-leaf', listen='127.0.0.1:4422', http='127.0.0.1:8422')
        server['jetstream']['store_dir'] = str(base / 'js')
        server['leafnodes'] = {'remotes': [{'url': 'nats://127.0.0.1:7423'}]}
        hubcfg = {
            'server_name': 'test-hub',
            'listen': '127.0.0.1:4522',
            'leafnodes': {'listen': '127.0.0.1:7423'},
            'jetstream': {'domain': 'TEST_HUB', 'store_dir': str(base / 'hub-js')},
        }
        try:
            hub = launch('hub', hubcfg)
            leaf = launch('leaf', server)
            await asyncio.sleep(1)
            assert hub.poll() is None and leaf.poll() is None
            service = Service(cfg)
            await service.start()
            client = await nats.connect(
                cfg['nats_url'],
                user=cfg['client_user'],
                password=cfg['client_password'],
                max_reconnect_attempts=-1,
                reconnect_time_wait=0.2,
                pending_size=0,
            )
            central = await nats.connect('nats://127.0.0.1:4522')
            clients.extend([client, central])
            subject = 'robots.test_robot.v1.request'

            async def request(nc, obj):
                return json.loads((await nc.request(subject, encode(obj), timeout=5)).data)

            async def wait_hub():
                for _ in range(30):
                    try:
                        reply = await request(central, envelope('ping'))
                        if reply['ok']:
                            return
                    except (nats.errors.NoRespondersError, nats.errors.TimeoutError):
                        pass
                    await asyncio.sleep(0.2)
                raise AssertionError('leaf not connected')

            await wait_hub()
            print('PASS hub -> leaf request/reply', flush=True)
            cmd = envelope('settings.set', {'note': 'integration-test'})
            first = await request(central, cmd)
            second = await request(client, cmd)
            assert first['ok'] and first == second
            modified = dict(cmd, args={'note': 'different'})
            assert (await request(client, modified))['error'] == 'id_reused_with_different_payload'
            expired = envelope('ping')
            expired['issued_at'] -= 60
            assert (await request(client, expired))['error'] == 'expired'
            assert (await request(client, envelope('mode.set', {'mode': 'mapping'})))[
                'error'
            ] == 'mode_execution_disabled_by_local_config'
            bad = await client.request(subject, b'not json', timeout=5)
            assert not json.loads(bad.data)['ok']
            print('PASS deduplication, conflict, expiry, invalid JSON, mode gate', flush=True)
            kv = await client.jetstream(domain=cfg['domain']).key_value(cfg['bucket'])
            assert json.loads((await kv.get('settings')).value)['note'] == 'integration-test'
            hubkv = await central.jetstream(domain=cfg['domain']).key_value(cfg['bucket'])
            assert json.loads((await hubkv.get('settings')).value)['note'] == 'integration-test'
            print('PASS KV read from local client and hub via domain', flush=True)
            runner = web.AppRunner(await make_app(cfg))
            await runner.setup()
            await web.TCPSite(runner, '127.0.0.1', 18088).start()
            async with ClientSession() as http:
                async with http.get('http://127.0.0.1:18088/api/state') as r:
                    assert r.status == 200
                async with http.get('http://127.0.0.1:18088/') as r:
                    assert r.status == 200 and 'NATS' in await r.text()
                headers = {}
                async with http.get('http://127.0.0.1:18088/api/state', headers=headers) as r:
                    assert (await r.json())['ok']
                async with http.post('http://127.0.0.1:18088/api/command', headers=headers, json=envelope('ping')) as r:
                    assert (await r.json())['result']['pong']
                async with http.post(
                    'http://127.0.0.1:18088/api/command',
                    headers={'Origin': 'https://unrelated.invalid'},
                    json=envelope('ping'),
                ) as r:
                    assert r.status == 403
                async with http.get('http://127.0.0.1:18088/api/state', headers={'Host': 'unrelated.invalid'}) as r:
                    assert r.status == 403
                bms = {
                    'v': 1,
                    'robot_id': 'test_robot',
                    'connected': True,
                    'stale': False,
                    'sample_age_s': 0,
                    'sample_at': time.time(),
                    'sample_id': 'test-bms:10',
                    'status': 'streaming',
                    'sample': {'voltage_V': 27.493, 'current_A': 2.0, 'soc_pct': 77},
                    'device': {'model': 'fixture'},
                }
                await central.publish('robots.test_robot.v1.bms', encode(bms))
                await central.flush()
                await asyncio.sleep(2.5)
                async with http.get('http://127.0.0.1:18088/api/state') as r:
                    battery = (await r.json())['result']['battery']
                    assert battery['sample']['voltage_V'] == 27.493 and not battery['stale']
                assert json.loads((await kv.get('battery')).value)['sample']['voltage_V'] == 27.493
                # Public fragment subjects and read-only KV watch work with client credentials.
                update = await client.subscribe('test_robot.battery.update.voltage')
                watcher = await kv.watch('battery.voltage')
                entry = await watcher.updates(timeout=5)
                assert json.loads(entry.value)['value'] == 27.493
                assert json.loads(entry.value)['sample_id'] == 'test-bms:10'
                await watcher.stop()
                message = await update.next_msg(timeout=5)
                assert json.loads(message.data)['unit'] == 'V'
                await update.unsubscribe()
                body = envelope('metrics.get')
                body.pop('action')
                response = json.loads(
                    (await central.request('test_robot.service.command.metrics_get', encode(body), timeout=5)).data
                )
                assert response['ok'] and response['result']['battery.soc']['value'] == 77
                assert response['result']['battery.charging_state']['value'] == 'charging'
                assert (
                    response['result']['battery.current']['sample_id']
                    == response['result']['battery.voltage']['sample_id']
                )
                expired = dict(body, id=__import__('uuid').uuid4().__str__(), issued_at=time.time() - 60)
                reply = json.loads(
                    (await client.request('test_robot.service.command.metrics_get', encode(expired), timeout=5)).data
                )
                assert reply['error'] == 'expired'
                async with http.get('http://127.0.0.1:18088/api/metrics') as r:
                    assert (await r.json())['result']['battery.soc']['value'] == 77
                print(
                    'PASS fragment subject, KV get/watch, canonical commands via leaf, HTTP metrics and frame identity',
                    flush=True,
                )

                await asyncio.sleep(6)
                async with http.get('http://127.0.0.1:18088/api/state') as r:
                    battery = (await r.json())['result']['battery']
                    assert (
                        battery['stale']
                        and battery['status'] == 'publisher_lost'
                        and battery['charging_state'] == 'unknown'
                    )
            print('PASS token-free HTTP, origin guard, BMS -> NATS -> KV/HTTP and stale detection', flush=True)
            await central.close()
            stop(hub)
            await asyncio.sleep(0.5)
            assert (await request(client, envelope('ping')))['ok']
            print('PASS local operation without hub', flush=True)
            stop(leaf)
            await asyncio.sleep(0.5)
            leaf = launch('leaf', server)
            for _ in range(40):
                await asyncio.sleep(0.25)
                if client.is_connected and service.nc.is_connected:
                    break
            assert (await request(client, envelope('ping')))['ok']
            assert json.loads((await kv.get('settings')).value)['note'] == 'integration-test'
            assert (await request(client, cmd)) == first
            print('PASS server restart, reconnect, persisted KV and command result', flush=True)
            await runner.cleanup()
            runner = None
            await service.close()
            service = Service(cfg)
            await service.start()
            assert (await request(client, cmd)) == first
            assert service.modes.state()['stale'] and service.modes.state()['actual'] == 'unknown'
            print('PASS backend restart, dedup persistence, no mode replay', flush=True)
            print('ALL INTEGRATION TESTS PASSED', flush=True)
        finally:
            if runner:
                await runner.cleanup()
            if service:
                await service.close()
            for nc in clients:
                if not nc.is_closed:
                    await nc.close()
            for p in processes:
                stop(p)


if __name__ == '__main__':
    asyncio.run(main())
