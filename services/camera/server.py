"""WebRTC media; NATS request/reply signaling. Debug HTTP is only a NATS client."""

import argparse
import asyncio
import hashlib
import json
import logging
import math
import signal
import sys
import time
import uuid
from fractions import Fraction
from pathlib import Path
from urllib.parse import urlsplit

import nats
import cv2
import rclpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'services/nats_bridge'))
from robot_nats.drive_widget import mount_drive_widget

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, VideoStreamTrack
from av import VideoFrame
from capture import Capture


def encode(value):
    return json.dumps(value, allow_nan=False, separators=(',', ':')).encode()


def validate(raw, operation):
    if len(raw) > 65536:
        raise ValueError('request_too_large')
    obj = json.loads(raw)
    if not isinstance(obj, dict) or set(obj) != {'v', 'id', 'issued_at', 'ttl_s', 'action', 'args'}:
        raise ValueError('invalid_envelope')
    if type(obj['v']) is not int or obj['v'] != 1:
        raise ValueError('unsupported_version')
    uuid.UUID(obj['id'])
    for key in ('issued_at', 'ttl_s'):
        if type(obj[key]) not in (int, float) or not math.isfinite(obj[key]):
            raise ValueError('invalid_time')
    if (
        not 0 < obj['ttl_s'] <= 30
        or obj['issued_at'] > time.time() + 2
        or time.time() > obj['issued_at'] + obj['ttl_s']
    ):
        raise ValueError('expired')
    if obj['action'] != 'camera.' + operation or not isinstance(obj['args'], dict):
        raise ValueError('invalid_action')
    return obj


class CameraTrack(VideoStreamTrack):
    def __init__(self, capture, stream, fps):
        super().__init__()
        self.capture = capture
        self.stream = stream
        self.fps = fps
        self.started = time.monotonic()
        self.seq = 0

    async def recv(self):
        await asyncio.sleep(max(0, self.started + self.seq / self.fps - time.monotonic()))
        item = self.capture.image(self.stream)
        while item is None:
            await asyncio.sleep(0.1)
            item = self.capture.image(self.stream)
        image, _, _ = item
        frame = VideoFrame.from_ndarray(image, format='bgr24')
        frame.pts = int((time.monotonic() - self.started) * 90000)
        frame.time_base = Fraction(1, 90000)
        self.seq = max(self.seq + 1, int((time.monotonic() - self.started) * self.fps))
        return frame


class CameraService:
    def __init__(self, config, capture):
        self.config = config
        self.capture = capture
        self.peers = {}
        self.cache = {}
        self.nc = None
        self.kv = None
        self.offer_lock = asyncio.Lock()
        self.tasks = set()

    def status(self):
        return dict(
            self.capture.status(),
            v=1,
            robot_id=self.config['robot_id'],
            sessions=len(self.peers),
            transport='webrtc',
            ros_topics={s: f'/camera/{s}/image_raw' for s in ('color', 'depth')},
            ice_configured=bool(self.config.get('ice_servers')),
            max_sessions=self.config['max_sessions'],
        )

    async def close_peer(self, key):
        value = self.peers.pop(key, None)
        if value:
            value['track'].stop()
            await value['pc'].close()

    async def execute(self, operation, args):
        if operation == 'status':
            if args:
                raise ValueError('unexpected_args')
            return self.status()
        if operation == 'ice':
            if args:
                raise ValueError('unexpected_args')
            return {'iceServers': self.config.get('ice_servers', [])}
        if operation == 'close':
            if set(args) != {'session_id'} or not isinstance(args['session_id'], str):
                raise ValueError('invalid_session')
            await self.close_peer(args['session_id'])
            return {'closed': True}
        if operation != 'offer':
            raise ValueError('unknown_operation')
        if (
            set(args) != {'type', 'sdp', 'stream'}
            or args['type'] != 'offer'
            or args['stream'] not in ('color', 'depth')
            or not isinstance(args['sdp'], str)
        ):
            raise ValueError('invalid_offer')
        if (
            len(args['sdp']) > 60000
            or args['sdp'].count('m=video') != 1
            or 'm=audio' in args['sdp']
            or 'm=application' in args['sdp']
        ):
            raise ValueError('one_video_only')
        if self.capture.image(args['stream']) is None:
            raise ValueError('camera_unavailable')
        async with self.offer_lock:
            if len(self.peers) >= self.config['max_sessions']:
                raise ValueError('session_limit')
            key = str(uuid.uuid4())
            pc = RTCPeerConnection(
                RTCConfiguration(iceServers=[RTCIceServer(**s) for s in self.config.get('ice_servers', [])])
            )
            track = CameraTrack(self.capture, args['stream'], self.config['video_fps'])
            self.peers[key] = {'pc': pc, 'track': track, 'created': time.monotonic()}

            @pc.on('connectionstatechange')
            async def state_changed():
                if pc.connectionState in ('failed', 'closed'):
                    await self.close_peer(key)

            try:
                await pc.setRemoteDescription(RTCSessionDescription(sdp=args['sdp'], type='offer'))
                pc.addTrack(track)
                await asyncio.wait_for(pc.setLocalDescription(await pc.createAnswer()), 15)
                return {
                    'session_id': key,
                    'type': pc.localDescription.type,
                    'sdp': pc.localDescription.sdp,
                    'stream': args['stream'],
                    'max_duration_s': 1800,
                }
            except BaseException:
                await self.close_peer(key)
                raise

    async def receive(self, msg):
        # Bound work even for valid-but-abusive clients; never accumulate offline requests.
        if len(self.tasks) >= 16:
            if msg.reply:
                await msg.respond(encode({'v': 1, 'id': None, 'ok': False, 'error': 'busy'}))
            return
        task = asyncio.create_task(self.dispatch(msg))
        self.tasks.add(task)
        task.add_done_callback(self.tasks.discard)

    async def dispatch(self, msg):
        obj = None
        try:
            operation = msg.subject.rsplit('.', 1)[-1]
            obj = validate(msg.data, operation)
            digest = hashlib.sha256(msg.data).hexdigest()
            now = time.monotonic()
            self.cache = {k: v for k, v in self.cache.items() if now - v[0] < 60 or not v[2].done()}
            if obj['id'] in self.cache:
                _, old, future = self.cache[obj['id']]
                if old != digest:
                    raise ValueError('id_conflict')
                reply = await asyncio.shield(future)
            else:
                if len(self.cache) >= 128:
                    raise ValueError('busy')
                future = asyncio.get_running_loop().create_future()
                self.cache[obj['id']] = (now, digest, future)
                try:
                    reply = {'v': 1, 'id': obj['id'], 'ok': True, 'result': await self.execute(operation, obj['args'])}
                except Exception as exc:
                    reply = {'v': 1, 'id': obj['id'], 'ok': False, 'error': str(exc)}
                future.set_result(reply)
        except Exception as exc:
            reply = {'v': 1, 'id': obj.get('id') if obj else None, 'ok': False, 'error': str(exc)}
        if msg.reply:
            try:
                await msg.respond(encode(reply))
            except Exception:
                logging.warning('Camera reply failed')

    async def start(self):
        c = self.config
        self.nc = await nats.connect(
            c['nats_url'],
            user=c['user'],
            password=c['password'],
            name='verter-camera',
            max_reconnect_attempts=-1,
            pending_size=0,
        )
        for operation in ('status', 'ice', 'offer', 'close'):
            await self.nc.subscribe(f'{c["robot_id"]}.camera.command.{operation}', cb=self.receive)

    async def maintain(self):
        while True:
            now = time.monotonic()
            for key, value in list(self.peers.items()):
                if now - value['created'] > 1800 or (
                    value['pc'].connectionState != 'connected' and now - value['created'] > 30
                ):
                    await self.close_peer(key)
            try:
                state = self.status()
                at = time.time()
                if self.kv is None:
                    self.kv = await self.nc.jetstream(domain=self.config['domain']).key_value(self.config['bucket'])
                values = {
                    'state': state['state'],
                    'stale': state['stale'],
                    'sessions': state['sessions'],
                    'device': state.get('device'),
                    'color_age': state['streams'].get('color', {}).get('age_s'),
                    'depth_age': state['streams'].get('depth', {}).get('age_s'),
                }
                for name, value in values.items():
                    record = {
                        'v': 1,
                        'value': value,
                        'unit': 's' if name.endswith('_age') else None,
                        'sample_at': at,
                        'updated_at': at,
                        'sample_id': str(at),
                        'max_age_s': 6,
                        'quality': 'unavailable' if value is None else 'ok',
                    }
                    raw = encode(record)
                    await self.nc.publish(f'{self.config["robot_id"]}.camera.update.{name}', raw)
                    await self.kv.put('camera.' + name, raw)
            except Exception:
                self.kv = None
                logging.warning('Camera telemetry disconnected; retrying')
            await asyncio.sleep(2)

    async def close(self):
        for task in list(self.tasks):
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        for key in list(self.peers):
            await self.close_peer(key)
        if self.nc:
            await self.nc.close()


async def make_app(service):
    config = service.config
    # Separate client credentials: HTTP cannot publish metrics or arbitrary robot commands.
    nc = await nats.connect(
        config['nats_url'],
        user=config['client_user'],
        password=config['client_password'],
        name='verter-camera-http',
        max_reconnect_attempts=-1,
        pending_size=0,
    )

    @web.middleware
    async def guard(request, handler):
        if urlsplit('//' + request.host).hostname not in config['web_allowed_hosts']:
            raise web.HTTPForbidden(text='Host not allowed')
        if request.path.startswith('/api/'):
            if (
                request.headers.get('Origin', f'{request.scheme}://{request.host}')
                != f'{request.scheme}://{request.host}'
                or request.headers.get('Sec-Fetch-Site') == 'cross-site'
            ):
                raise web.HTTPForbidden(text='Cross origin denied')
            if request.method == 'POST' and request.content_type != 'application/json':
                raise web.HTTPUnsupportedMediaType()
        response = await handler(request)
        response.headers.update(
            {
                'Cache-Control': 'no-store',
                'X-Content-Type-Options': 'nosniff',
                'Content-Security-Policy': "default-src 'self'; script-src 'self'; style-src 'self'; media-src 'self' blob:; frame-ancestors 'none'",
            }
        )
        return response

    app = web.Application(middlewares=[guard], client_max_size=65536)
    http_viewers = set()
    closing = asyncio.Event()

    async def mjpeg(request):
        stream = request.query.get('stream', 'color')
        if stream not in ('color', 'depth') or set(request.query) - {'stream'}:
            raise web.HTTPBadRequest(text='stream must be color or depth')
        if len(http_viewers) >= 2:
            raise web.HTTPTooManyRequests(text='HTTP viewer limit (2)')
        if service.capture.image(stream) is None:
            raise web.HTTPServiceUnavailable(text='No fresh camera frame')
        viewer = object()
        http_viewers.add(viewer)
        response = web.StreamResponse(
            headers={
                'Content-Type': 'multipart/x-mixed-replace; boundary=frame',
                'Cache-Control': 'no-store, no-cache, must-revalidate',
                'X-Content-Type-Options': 'nosniff',
                'X-Accel-Buffering': 'no',
                'Content-Security-Policy': "default-src 'none'; frame-ancestors 'none'",
            }
        )
        started = time.monotonic()
        previous = None

        def jpeg(image):
            ok, data = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if not ok:
                raise ValueError('JPEG encoding failed')
            return data.tobytes()

        try:
            await response.prepare(request)
            while not closing.is_set() and time.monotonic() - started < 1800:
                if request.transport is None or request.transport.is_closing():
                    break
                item = service.capture.image(stream)
                if item is None:
                    break
                image, sample_at, seq = item
                if seq != previous:
                    data = await asyncio.to_thread(jpeg, image)
                    part = (
                        (
                            f'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(data)}\r\nX-Sample-At: {sample_at}\r\n\r\n'
                        ).encode()
                        + data
                        + b'\r\n'
                    )
                    await asyncio.wait_for(response.write(part), 5)
                    previous = seq
                try:
                    await asyncio.wait_for(closing.wait(), 0.2)
                except asyncio.TimeoutError:
                    pass
        except (ConnectionError, asyncio.TimeoutError):
            pass
        finally:
            http_viewers.discard(viewer)
            response.force_close()
        return response

    async def shutdown(app):
        closing.set()

    app.on_shutdown.append(shutdown)

    async def api(request):
        operation = request.match_info['operation']
        if operation not in ('status', 'ice', 'offer', 'close'):
            raise web.HTTPNotFound()
        if request.method == 'GET' and operation not in ('status', 'ice'):
            raise web.HTTPMethodNotAllowed('GET', ['POST'])
        if request.method == 'POST':
            try:
                body = await request.json()
            except Exception:
                raise web.HTTPBadRequest(text='Invalid JSON')
        else:
            body = {
                'v': 1,
                'id': str(uuid.uuid4()),
                'issued_at': time.time(),
                'ttl_s': 30,
                'action': 'camera.' + operation,
                'args': {},
            }
        try:
            reply = await nc.request(f'{config["robot_id"]}.camera.command.{operation}', encode(body), timeout=20)
            return web.json_response(json.loads(reply.data))
        except (nats.errors.TimeoutError, nats.errors.NoRespondersError):
            raise web.HTTPServiceUnavailable(text='Camera signaling unavailable')

    static = Path(__file__).parent / 'static'

    async def index(request):
        return web.FileResponse(static / 'index.html')

    app.router.add_get('/', index)
    app.router.add_get('/api/camera/mjpeg', mjpeg, allow_head=False)
    app.router.add_get('/api/camera/{operation}', api)
    app.router.add_post('/api/camera/{operation}', api)
    mount_drive_widget(app, Path(__file__).resolve().parents[2])
    app.router.add_static('/static/', static)

    async def cleanup(app):
        await nc.close()

    app.on_cleanup.append(cleanup)
    return app


async def main(config):
    rclpy.init()
    capture = Capture(config)
    service = CameraService(config, capture)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    runner = None
    task = None
    try:
        await service.start()
        task = asyncio.create_task(service.maintain())
        if config.get('web_enabled', True):
            runner = web.AppRunner(await make_app(service))
            await runner.setup()
            await web.TCPSite(runner, config['web_host'], config['web_port']).start()
        await stop.wait()
    finally:
        if runner:
            await runner.cleanup()
        if task:
            task.cancel()
            await asyncio.gather(task, return_exceptions=True)
        await service.close()
        capture.close()
        rclpy.shutdown()


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--config', required=True)
    args = p.parse_args()
    logging.basicConfig(level=logging.INFO)
    asyncio.run(main(json.loads(Path(args.config).read_text())))
