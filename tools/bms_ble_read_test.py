#!/usr/bin/env python3
"""One-shot JK BLE read test. Only 0x97 device-info and 0x96 telemetry requests."""

import argparse
import asyncio
import json
import os
from pathlib import Path
import time
from bleak import BleakClient, BleakScanner


def request(code):
    assert code in (0x96, 0x97)
    packet = bytearray.fromhex('aa5590eb') + bytearray([code]) + bytearray(15)
    packet[19] = sum(packet[:19]) & 255
    return packet


async def main(address, out):
    os.umask(0o077)
    out.mkdir(parents=True, exist_ok=True)
    device = await BleakScanner.find_device_by_address(address, timeout=20)
    if device is None:
        raise RuntimeError('Requested BMS is not advertising')
    frames = []
    buffer = bytearray()
    invalid = 0
    chunks = 0
    with (out / 'notifications.jsonl').open('w') as log:

        def notification(sender, payload):
            nonlocal buffer, invalid, chunks
            chunks += 1
            log.write(json.dumps({'time': time.time(), 'handle': sender.handle, 'hex': payload.hex()}) + '\n')
            log.flush()
            buffer.extend(payload)
            while len(buffer) >= 4:
                pos = buffer.find(bytes.fromhex('55aaeb90'))
                if pos < 0:
                    buffer = buffer[-3:]
                    break
                del buffer[:pos]
                if len(buffer) < 300:
                    break
                frame = bytes(buffer[:300])
                if sum(frame[:299]) & 255 != frame[299]:
                    invalid += 1
                    del buffer[0]
                    continue
                del buffer[:300]
                frames.append(frame)
                print('VALID_FRAME', hex(frame[4]), len(frame), flush=True)

        async with BleakClient(device, timeout=25) as client:
            services = [
                {
                    'uuid': s.uuid,
                    'characteristics': [
                        {'uuid': c.uuid, 'handle': c.handle, 'properties': c.properties} for c in s.characteristics
                    ],
                }
                for s in client.services
            ]
            (out / 'gatt.json').write_text(json.dumps(services, indent=2))
            chars = [
                c
                for s in client.services
                if s.uuid.lower().startswith('0000ffe0-')
                for c in s.characteristics
                if c.uuid.lower().startswith('0000ffe1-')
            ]
            notify = [c for c in chars if 'notify' in c.properties]
            write = [c for c in chars if 'write-without-response' in c.properties or 'write' in c.properties]
            if len(notify) != 1 or len(write) != 1:
                raise RuntimeError('Ambiguous or unsupported GATT; inspect gatt.json before any command')
            await client.start_notify(notify[0], notification)
            for code in (0x97, 0x96):
                await client.write_gatt_char(
                    write[0], request(code), response='write-without-response' not in write[0].properties
                )
                print('SENT_READ_REQUEST', hex(code), flush=True)
                await asyncio.sleep(3)
            await asyncio.sleep(15)
            await client.stop_notify(notify[0])
    (out / 'frames.json').write_text(json.dumps([x.hex() for x in frames], indent=2))
    result = {
        'address': address,
        'name': device.name,
        'notification_chunks': chunks,
        'valid_frames': len(frames),
        'invalid_checksums': invalid,
        'disconnected': True,
    }
    info = next((x for x in frames if x[4] == 3), None)
    if info:

        def txt(a, n):
            return info[a : a + n].split(b'\0', 1)[0].decode('ascii', errors='replace')

        result['device'] = {'model': txt(6, 16), 'hardware': txt(22, 8), 'software': txt(30, 8)}
    (out / 'result.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--address', required=True)
    p.add_argument('--out', required=True, type=Path)
    args = p.parse_args()
    asyncio.run(main(args.address, args.out))
