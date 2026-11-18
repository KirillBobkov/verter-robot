"""Same-origin debug widget routes. Commands still execute in the existing NATS bridge."""

import asyncio
import json
from pathlib import Path

from aiohttp import ClientError, ClientSession, ClientTimeout, web


def mount_drive_widget(app, root, bridge_port=None):
    if bridge_port is None:
        config = Path(root) / 'config/nats/bridge.json'
        bridge_port = json.loads(config.read_text()).get('web_port', 8088) if config.exists() else 8088
    bridge_port = int(bridge_port)
    static = Path(root) / 'services/nats_bridge/static'
    session = None

    async def startup(app):
        nonlocal session
        session = ClientSession(timeout=ClientTimeout(total=2), trust_env=False)

    async def cleanup(app):
        if session:
            await session.close()

    async def asset(request):
        return web.FileResponse(static / request.match_info['asset'])

    async def proxy(request):
        body = None
        if request.method == 'POST':
            if request.content_type != 'application/json':
                raise web.HTTPUnsupportedMediaType()
            if request.content_length is not None and request.content_length > 16384:
                raise web.HTTPRequestEntityTooLarge(max_size=16384, actual_size=request.content_length)
            raw = bytearray()
            async for chunk in request.content.iter_chunked(4096):
                raw.extend(chunk)
                if len(raw) > 16384:
                    raise web.HTTPRequestEntityTooLarge(max_size=16384, actual_size=len(raw))
            try:
                body = json.loads(raw)
            except (ValueError, UnicodeError):
                raise web.HTTPBadRequest(text='Invalid JSON')
            if not isinstance(body, dict) or body.get('action') not in {
                'cmd_vel.status',
                'cmd_vel.set',
                'cmd_vel.keepalive',
                'cmd_vel.stop',
            }:
                raise web.HTTPBadRequest(text='Only manual-control actions are supported')
        path = '/api/drive' if request.method == 'GET' else '/api/command'
        try:
            async with session.request(
                request.method, f'http://127.0.0.1:{bridge_port}{path}', json=body, allow_redirects=False
            ) as response:
                return web.Response(
                    body=await response.read(), status=response.status, content_type=response.content_type
                )
        except (ClientError, OSError, asyncio.TimeoutError):
            raise web.HTTPServiceUnavailable(text='Control backend unavailable; no automatic retry')

    app.on_startup.append(startup)
    app.on_cleanup.append(cleanup)
    app.router.add_get('/api/drive', proxy)
    app.router.add_post('/api/command', proxy)
    app.router.add_get('/static/{asset:drive(?:-widget)?\\.(?:js|css)}', asset)
