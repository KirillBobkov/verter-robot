"""Independent BLE -> NATS publisher. Never resets the adapter or controls BMS/ROS."""

import argparse
import asyncio
from contextlib import suppress
import fcntl
import json
import logging
from pathlib import Path
import signal
import time
import uuid

from bleak import BleakClient, BleakScanner
import nats
from .jk_ble import Frames, request, device_info, decode
from .protocol import encode


class BmsReader:
    def __init__(self, config):
        self.config = config
        self.boot_id = str(uuid.uuid4())
        self.sample = None
        self.sample_at = None
        self.sample_mono = None
        self.device = None
        self.connected = False
        self.status = 'starting'
        self.error = None
        self.rssi = None
        self.valid_frames = 0
        self.invalid_checksums = 0
        self.decode_errors = 0
        self.connections = 0
        self.sample_sequence = 0
        self.nc = None

    def snapshot(self):
        age = max(0, time.monotonic() - self.sample_mono) if self.sample_mono is not None else None
        return {
            'v': 1,
            'robot_id': self.config['robot_id'],
            'boot_id': self.boot_id,
            'updated_at': time.time(),
            'address': self.config['address'],
            'source': 'ble',
            'status': self.status,
            'connected': self.connected,
            'device': self.device,
            'rssi_at_connect_dbm': self.rssi,
            'error': self.error,
            'sample_id': f'{self.boot_id}:{self.sample_sequence}' if self.sample_at is not None else None,
            'sample_at': self.sample_at,
            'sample_age_s': round(age, 3) if age is not None else None,
            'stale': not self.connected or self.status != 'streaming' or age is None or age > 8,
            'sample': self.sample,
            'diagnostics': {
                'valid_frames': self.valid_frames,
                'invalid_checksums': self.invalid_checksums,
                'decode_errors': self.decode_errors,
                'connections': self.connections,
            },
        }

    async def collect_once(self):
        self.status = 'scanning'
        # Target a known address; other devices are neither connected nor paired.
        found = asyncio.Event()
        target = None

        def advertisement(device, adv):
            nonlocal target
            if device.address.upper() == self.config['address'].upper():
                target = device
                self.rssi = adv.rssi
                found.set()

        scanner = BleakScanner(detection_callback=advertisement, bluez={'adapter': self.config.get('adapter', 'hci0')})
        async with scanner:
            await asyncio.wait_for(found.wait(), 20)
        self.status = 'connecting'
        lost = asyncio.Event()
        info_ready = asyncio.Event()
        parser = Frames()
        verified = False
        async with BleakClient(
            target,
            timeout=25,
            disconnected_callback=lambda _: lost.set(),
            bluez={'adapter': self.config.get('adapter', 'hci0')},
        ) as client:
            self.connected = True
            self.connections += 1
            self.status = 'identifying'
            self.device = None
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
                raise RuntimeError('Unsupported GATT layout')

            def notification(sender, payload):
                before = parser.invalid
                frames = parser.feed(payload)
                self.invalid_checksums += parser.invalid - before
                for frame in frames:
                    self.valid_frames += 1
                    try:
                        if frame[4] == 3:
                            self.device = device_info(frame)
                            info_ready.set()
                        elif frame[4] == 2 and verified:
                            self.sample = decode(frame)
                            self.sample_at = time.time()
                            self.sample_mono = time.monotonic()
                            self.sample_sequence += 1
                            self.status = 'streaming'
                            self.error = None
                    except ValueError:
                        self.decode_errors += 1

            await client.start_notify(notify[0], notification)
            await client.write_gatt_char(
                write[0], request(0x97), response='write-without-response' not in write[0].properties
            )
            await asyncio.wait_for(info_ready.wait(), 10)
            expected = self.config['expected_device']
            if self.device != expected:
                raise RuntimeError('Device model/version changed; decoder review required')
            verified = True
            # This JK 19.31 needs time after device-info; back-to-back requests
            # can be acknowledged at GATT level without starting telemetry.
            await asyncio.sleep(3)
            await client.write_gatt_char(
                write[0], request(0x96), response='write-without-response' not in write[0].properties
            )
            stream_start = time.monotonic()
            self.status = 'waiting_sample'
            while not lost.is_set():
                await asyncio.sleep(1)
                last = max(stream_start, self.sample_mono or 0)
                if time.monotonic() - last > 15:
                    raise RuntimeError('BMS telemetry timeout')
            raise RuntimeError('BMS disconnected')

    async def collect_loop(self):
        delay = 2
        while True:
            attempt = time.monotonic()
            try:
                await self.collect_once()
            except asyncio.CancelledError:
                raise
            except Exception as exc:
                self.error = type(exc).__name__ + ': ' + str(exc)
                logging.warning('BMS connection: %s', self.error)
            finally:
                self.connected = False
                self.status = 'reconnecting'
            if time.monotonic() - attempt > 60:
                delay = 2
            await asyncio.sleep(delay)
            delay = min(delay * 2, 30)

    async def publish_once(self):
        if self.nc and self.nc.is_connected:
            await self.nc.publish(f'robots.{self.config["robot_id"]}.v1.bms', encode(self.snapshot()))

    async def publish_loop(self):
        while True:
            try:
                await self.publish_once()
            except Exception as exc:
                logging.warning('BMS NATS publication failed: %s', type(exc).__name__)
            await asyncio.sleep(2)


async def run(config):
    data = Path(config['data_dir'])
    data.mkdir(parents=True, exist_ok=True)
    with (data / 'bms-owner.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        reader = BmsReader(config)
        reader.nc = await nats.connect(
            config['nats_url'],
            user=config['user'],
            password=config['password'],
            name='verter-bms',
            pending_size=0,
            max_reconnect_attempts=-1,
            reconnect_time_wait=2,
        )
        stop = asyncio.Event()
        loop = asyncio.get_running_loop()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stop.set)
        tasks = [asyncio.create_task(reader.collect_loop()), asyncio.create_task(reader.publish_loop())]
        try:
            await stop.wait()
        finally:
            for task in tasks:
                task.cancel()
            for task in tasks:
                with suppress(asyncio.CancelledError):
                    await task
            reader.status = 'stopped'
            reader.connected = False
            with suppress(Exception):
                await reader.publish_once()
                await reader.nc.flush(timeout=2)
            await reader.nc.close()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--config', required=True)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO)
    asyncio.run(run(json.loads(Path(args.config).read_text())))
