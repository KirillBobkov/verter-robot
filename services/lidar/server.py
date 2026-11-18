"""ROS -> NATS Core -> WebSocket viewer with shared manual-control widget."""

import argparse
import asyncio
from contextlib import suppress
import json
import logging
from pathlib import Path
import signal
import sys
import threading
import time
import uuid
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'services/nats_bridge'))
from robot_nats.drive_widget import mount_drive_widget

from aiohttp import web
import nats
from geometry import robot_model, scan_frame


def encode(value):
    return json.dumps(value, allow_nan=False, separators=(',', ':')).encode()


class Reader:
    def __init__(self, config):
        self.config = config
        self.boot_id = str(uuid.uuid4())
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.latest = None
        self.error = 'waiting_scan'
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.latest, self.error

    def run(self):
        node = context = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import LaserScan
            from tf2_ros import Buffer, TransformListener

            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node(
                'verter_lidar_observer', context=context, enable_rosout=False, start_parameter_services=False
            )
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            last = 0
            sequence = 0

            def receive(scan):
                nonlocal last, sequence
                now = time.monotonic()
                if now - last < 0.2:
                    return
                last = now
                try:
                    stamp = rclpy.time.Time.from_msg(scan.header.stamp)
                    if abs((node.get_clock().now() - stamp).nanoseconds / 1e9) > 2:
                        raise ValueError('scan_timestamp_stale')
                    tf = buffer.lookup_transform('base_footprint', scan.header.frame_id, stamp)
                    frame = scan_frame(scan, tf.transform, self.config['robot_id'], sequence, time.time())
                    frame['boot_id'] = self.boot_id
                    sequence += 1
                    with self.lock:
                        self.latest, self.error = (now, frame), None
                except Exception as exc:
                    with self.lock:
                        self.error = str(exc)

            subscription = node.create_subscription(LaserScan, '/scan', receive, qos_profile_sensor_data)
            while not self.stop.is_set():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            logging.exception('Lidar observer failed')
            with self.lock:
                self.error = str(exc)
        finally:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
            if context:
                with suppress(Exception):
                    context.shutdown()


class Feed:
    """Bounded, latest-only NATS cache shared by browser connections."""

    def __init__(self):
        self.latest = {}
        self.clients = 0
        self.sockets = set()

    async def receive(self, message):
        if len(message.data) > 128 * 1024:
            return
        try:
            value = json.loads(message.data)
            kind = message.subject.rsplit('.', 1)[-1]
            if value.get('v') != 1 or kind not in ('scan', 'model', 'status') or value.get('type') != kind:
                return
            self.latest[kind] = (time.monotonic(), value)
        except (ValueError, AttributeError):
            pass


def make_app(config, nc, feed):
    allowed = set(config['web_allowed_hosts'])

    @web.middleware
    async def guard(request, handler):
        if urlsplit('//' + request.host).hostname not in allowed:
            raise web.HTTPForbidden()
        origin = request.headers.get('Origin')
        if origin and origin != f'{request.scheme}://{request.host}':
            raise web.HTTPForbidden()
        if request.headers.get('Sec-Fetch-Site') == 'cross-site':
            raise web.HTTPForbidden()
        response = await handler(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = (
            "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; frame-ancestors 'none'"
        )
        return response

    app = web.Application(middlewares=[guard])
    static = Path(__file__).parent / 'static'

    async def index(request):
        return web.FileResponse(static / 'index.html')

    async def websocket(request):
        if feed.clients >= 4:
            raise web.HTTPTooManyRequests()
        ws = web.WebSocketResponse(heartbeat=10, max_msg_size=1024)
        feed.clients += 1

        async def send():
            sent = {}
            try:
                while not ws.closed:
                    for kind, (received, value) in list(feed.latest.items()):
                        if sent.get(kind) == received:
                            continue
                        packet = dict(value)
                        packet['delivery_age_s'] = max(0, time.monotonic() - received)
                        if kind == 'scan':
                            packet['delivery_age_s'] = max(packet['delivery_age_s'], time.time() - value['sample_at'])
                        await asyncio.wait_for(ws.send_bytes(encode(packet)), timeout=2)
                        sent[kind] = received
                    await asyncio.sleep(0.2)
            finally:
                await ws.close()

        task = None
        try:
            await ws.prepare(request)
            feed.sockets.add(ws)
            task = asyncio.create_task(send())
            # No application commands are accepted over the WebSocket.
            async for message in ws:
                if task.done():
                    break
                if message.type in (web.WSMsgType.TEXT, web.WSMsgType.BINARY):
                    await ws.close(code=1008, message=b'read-only')
        finally:
            if task:
                task.cancel()
                with suppress(asyncio.CancelledError, ConnectionError, TimeoutError):
                    await task
            feed.clients -= 1
            feed.sockets.discard(ws)
        return ws

    async def status(request):
        latest = feed.latest.get('status')
        if latest is None:
            return web.json_response({'v': 1, 'stale': True, 'error': 'waiting_nats'})
        received, value = latest
        age = time.monotonic() - received
        return web.json_response({**value, 'delivery_age_s': age, 'stale': value['stale'] or age > 3})

    async def shutdown(app):
        await asyncio.gather(*(socket.close(code=1001, message=b'shutdown') for socket in list(feed.sockets)))

    app.on_shutdown.append(shutdown)
    app.router.add_get('/', index)
    app.router.add_get('/api/lidar/status', status)
    app.router.add_get('/api/lidar/ws', websocket)
    mount_drive_widget(app, Path(__file__).resolve().parents[2])
    app.router.add_static('/static/', static)
    return app


async def main(config):
    model = robot_model(config['urdf'], config['robot_id'])
    nc = await nats.connect(
        config['nats_url'],
        user=config['user'],
        password=config['password'],
        pending_size=0,
        max_reconnect_attempts=-1,
        reconnect_time_wait=1,
    )
    prefix = config['robot_id'] + '.lidar.update.'
    feed = Feed()
    subscription = await nc.subscribe(
        prefix + '*', cb=feed.receive, pending_msgs_limit=4, pending_bytes_limit=512 * 1024
    )
    await nc.flush()
    reader = Reader(config)
    runner = None
    stop = asyncio.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        asyncio.get_running_loop().add_signal_handler(sig, stop.set)

    async def publish():
        sent = None
        next_status = 0
        while not stop.is_set():
            latest, error = reader.snapshot()
            age = time.monotonic() - latest[0] if latest else None
            try:
                if nc.is_connected:
                    if latest and age <= 1.5 and latest[1]['sequence'] != sent and not error:
                        await nc.publish(prefix + 'scan', encode(latest[1]))
                        sent = latest[1]['sequence']
                    if time.monotonic() >= next_status:
                        await nc.publish(prefix + 'model', encode(model))
                        await nc.publish(
                            prefix + 'status',
                            encode(
                                {
                                    'v': 1,
                                    'type': 'status',
                                    'robot_id': config['robot_id'],
                                    'updated_at': time.time(),
                                    'stale': age is None or age > 1.5 or error is not None,
                                    'error': error,
                                    'age_s': age,
                                    'frame_id': 'base_footprint',
                                    'topic': '/scan',
                                    'max_age_s': 3,
                                }
                            ),
                        )
                        next_status = time.monotonic() + 1
            except Exception:
                logging.exception('Lidar NATS publication failed')
            await asyncio.sleep(0.2)

    task = asyncio.create_task(publish())
    try:
        if config.get('web_enabled', True):
            runner = web.AppRunner(make_app(config, nc, feed))
            await runner.setup()
            await web.TCPSite(runner, config['web_host'], config['web_port']).start()
        await stop.wait()
    finally:
        task.cancel()
        with suppress(asyncio.CancelledError):
            await task
        if runner:
            await runner.cleanup()
        reader.stop.set()
        await asyncio.to_thread(reader.thread.join, 3)
        await nc.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main(json.loads(Path(args.config).read_text())))
