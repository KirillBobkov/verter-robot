"""Read-only client example. Credentials come from environment, never printed."""

import asyncio
import json
import os
import time
import uuid
import nats


def fresh(value):
    now = time.time()
    return value['quality'] == 'ok' and all(
        isinstance(value.get(field), (int, float)) and -2 <= now - value[field] <= value['max_age_s']
        for field in ('sample_at', 'updated_at')
    )


async def main():
    robot = os.environ.get('ROBOT_ID', 'verter_01')
    nc = await nats.connect(
        os.environ.get('NATS_URL', 'nats://127.0.0.1:4222'),
        user=os.environ['NATS_USER'],
        password=os.environ['NATS_PASSWORD'],
        name='contract-reader',
        pending_size=0,
    )
    try:

        async def request(name):
            body = {'v': 1, 'id': str(uuid.uuid4()), 'issued_at': time.time(), 'ttl_s': 10, 'args': {}}
            reply = await nc.request(f'{robot}.service.command.{name}', json.dumps(body).encode(), timeout=5)
            result = json.loads(reply.data)
            if not result['ok']:
                raise RuntimeError(result['error'])
            return result['result']

        schema = await request('schema_get')
        print('Contract:', schema['contract']['revision'])
        print('Ping:', await request('ping'))
        kv = await nc.jetstream(domain=os.environ.get('NATS_DOMAIN', schema['domain'])).key_value(
            os.environ.get('NATS_BUCKET', schema['bucket'])
        )
        value = json.loads((await kv.get('battery.soc')).value)
        print('SOC:', value, 'fresh:', fresh(value))
        # Initial snapshot followed by updates; run for a bounded test period.
        watcher = await kv.watch('battery.*')
        try:
            deadline = time.monotonic() + float(os.environ.get('WATCH_SECONDS', '10'))
            while time.monotonic() < deadline:
                try:
                    entry = await watcher.updates(timeout=min(1, max(0.01, deadline - time.monotonic())))
                except nats.errors.TimeoutError:
                    continue
                if entry is None:
                    print('Initial KV snapshot complete')
                elif entry.operation not in ('DEL', 'PURGE'):
                    metric = json.loads(entry.value)
                    # Snapshot can be large; only print three scalar metrics in this example.
                    if entry.key in ('battery.soc', 'battery.current', 'battery.charging_state'):
                        print(entry.key, metric['value'], metric['quality'], 'fresh:', fresh(metric))
        finally:
            await watcher.stop()
    finally:
        await nc.close()


if __name__ == '__main__':
    asyncio.run(main())
