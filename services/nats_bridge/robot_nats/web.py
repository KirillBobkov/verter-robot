"""HTTP is a NATS client: all API calls travel through NATS request/reply."""

import asyncio
import json
import math
from pathlib import Path
from urllib.parse import urlsplit

from aiohttp import web
import nats
from .protocol import encode, envelope
from .contracts import COMMANDS
from .places import SPATIAL_COMMANDS
from .history_export import mount_history_export


async def make_app(config):
    allowed_hosts = set(config.get('web_allowed_hosts', ['localhost', '127.0.0.1', '::1']))
    nc = await nats.connect(
        config['nats_url'],
        user=config['client_user'],
        password=config['client_password'],
        name='verter-test-web',
        max_reconnect_attempts=-1,
        pending_size=0,
    )

    @web.middleware
    async def guard(request, handler):
        if urlsplit('//' + request.host).hostname not in allowed_hosts:
            raise web.HTTPForbidden(text='Host is not allowed for this debug interface')
        if request.path.startswith('/api/'):
            origin = request.headers.get('Origin')
            if origin and origin != f'{request.scheme}://{request.host}':
                raise web.HTTPForbidden(text='Cross-origin access is disabled')
            if request.headers.get('Sec-Fetch-Site') == 'cross-site':
                raise web.HTTPForbidden(text='Cross-site access is disabled')
            if request.method == 'POST' and request.content_type != 'application/json':
                raise web.HTTPUnsupportedMediaType(text='application/json required')
        response = await handler(request)
        response.headers['Cache-Control'] = 'no-store'
        response.headers['X-Content-Type-Options'] = 'nosniff'
        response.headers['Content-Security-Policy'] = (
            "default-src 'self'; script-src 'self'; style-src 'self'; frame-ancestors 'none'"
        )
        return response

    app = web.Application(middlewares=[guard], client_max_size=16384)

    async def request_nats(body):
        if not nc.is_connected:
            raise web.HTTPServiceUnavailable(text='NATS disconnected')
        try:
            if not isinstance(body, dict):
                raise web.HTTPBadRequest(text='JSON object required')
            body = dict(body)
            action = body.pop('action', None)
            route = next((name for name, value in COMMANDS.items() if value == action), None)
            if action in SPATIAL_COMMANDS:
                domain, operation = SPATIAL_COMMANDS[action]
                subject = f'{config["robot_id"]}.{domain}.command.{operation}'
                timeout = 10
            elif action in ('map.get', 'map.save', 'map.delete'):
                subject = f'{config["robot_id"]}.map.command.{action.split(".")[1]}'
                timeout = 5
            elif action == 'costmap.get':
                subject = f'{config["robot_id"]}.costmap.command.get'
                timeout = 5
            elif action == 'battery.history':
                subject = f'{config["robot_id"]}.battery.command.history'
                timeout = 20
            elif action in ('cmd_vel.set', 'cmd_vel.keepalive', 'cmd_vel.stop', 'cmd_vel.status'):
                subject = f'{config["robot_id"]}.cmd_vel.command.{action.split(".")[1]}'
                timeout = 1
            elif route is not None:
                subject = f'{config["robot_id"]}.service.command.{route}'
                timeout = 20
            else:
                raise web.HTTPBadRequest(text='Unknown action')
            reply = await nc.request(subject, encode(body), timeout=timeout)
            return web.json_response(json.loads(reply.data))
        except (nats.errors.TimeoutError, nats.errors.NoRespondersError):
            raise web.HTTPServiceUnavailable(
                text='No response; execution outcome may be unknown. Inspect state before retrying.'
            )

    async def command(request):
        try:
            body = await request.json()
        except (ValueError, UnicodeError):
            raise web.HTTPBadRequest(text='Invalid JSON')
        return await request_nats(body)

    async def state(request):
        return await request_nats(envelope('state.get'))

    async def metrics(request):
        return await request_nats(envelope('metrics.get'))

    async def drive(request):
        body = envelope('cmd_vel.status')
        body['ttl_s'] = 1
        return await request_nats(body)

    async def history(request):
        if set(request.query) - {'from', 'to', 'max_points', 'period_s'}:
            raise web.HTTPBadRequest(text='Unknown history query parameter')
        try:
            args = {key: int(value) if key == 'max_points' else float(value) for key, value in request.query.items()}
        except ValueError:
            raise web.HTTPBadRequest(text='Invalid history query parameter')
        if any(not math.isfinite(value) for value in args.values()):
            raise web.HTTPBadRequest(text='Invalid history query parameter')
        return await request_nats(envelope('battery.history', args))

    async def live_map(request):
        if set(request.query) - {'part', 'name', 'map_id', 'revision', 'index'}:
            raise web.HTTPBadRequest(text='Unknown map query parameter')
        args = dict(request.query)
        try:
            for name in ('revision', 'index'):
                if name in args:
                    args[name] = int(args[name])
        except ValueError:
            raise web.HTTPBadRequest(text='Invalid map query parameter')
        return await request_nats(envelope('map.get', args))

    async def map_page(request):
        return web.FileResponse(static / 'map.html')

    async def costmap(request):
        if set(request.query) - {'part', 'kind', 'costmap_id', 'revision', 'index'}:
            raise web.HTTPBadRequest(text='Unknown costmap query parameter')
        args = dict(request.query)
        try:
            for name in ('revision', 'index'):
                if name in args:
                    args[name] = int(args[name])
        except ValueError:
            raise web.HTTPBadRequest(text='Invalid costmap query parameter')
        return await request_nats(envelope('costmap.get', args))

    async def history_page(request):
        return web.FileResponse(static / 'battery-history.html')

    static = Path(__file__).resolve().parents[1] / 'static'

    async def index(request):
        return web.FileResponse(static / 'index.html')

    app.router.add_get('/', index)
    app.router.add_get('/map', map_page)
    app.router.add_get('/api/map', live_map)
    app.router.add_get('/api/costmap', costmap)
    app.router.add_get('/battery-history', history_page)
    app.router.add_get('/api/battery/history', history)
    mount_history_export(app, config, nc)
    app.router.add_get('/api/drive', drive)
    app.router.add_get('/api/state', state)
    app.router.add_get('/api/metrics', metrics)
    app.router.add_post('/api/command', command)
    app.router.add_static('/static/', static)

    async def cleanup(app):
        await nc.close()

    app.on_cleanup.append(cleanup)
    return app
