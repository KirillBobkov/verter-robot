"""Observe and request supervisor modes; never launch or terminate ROS processes."""

import asyncio
from contextlib import suppress
from pathlib import Path
import time
import uuid

from .supervisor import SupervisorClient
from .places import resolve_map


class Modes:
    def __init__(self, config, telemetry, client=None):
        self.config = config
        self.telemetry = telemetry
        root = config.get('project_root', Path(__file__).resolve().parents[3])
        self.client = client or SupervisorClient(root, config.get('supervisor_socket'))
        self.cached = None
        self.observed = None
        self.updated_at = None
        self.error = None
        self.operation = None
        self.task = None
        self.poller = None
        self.lock = asyncio.Lock()

    def state(self):
        cached = self.cached or {}
        actual = cached.get('mode', 'unknown')
        running = self.operation and self.operation['status'] == 'running'
        map_path = cached.get('map')
        map_id = next((name for name, path in self.config['maps'].items() if path == map_path), None)
        if map_id is None and map_path:
            map_id = Path(map_path).stem
        return {
            'actual': actual,
            'desired': self.operation['requested_mode'] if running else actual,
            'map': map_id,
            'map_path': map_path,
            'started_at': None,
            'execution_enabled': self.config['allow_mode_changes'],
            'owner': 'supervisor',
            'scope': 'All supervisor-owned ROS modes; bridge restart does not stop them',
            'pid': cached.get('groups', {}).get('mode'),
            'exit_code': cached.get('last_exit', {}).get('mode'),
            'updated_at': self.updated_at,
            'stale': self.observed is None or time.monotonic() - self.observed > 8,
            'error': self.error or cached.get('error'),
            'operation': dict(self.operation) if self.operation else None,
            'localization_verified': False,
        }

    def accept(self, response):
        if not response.get('ok'):
            raise RuntimeError(response.get('error', 'supervisor_request_failed'))
        if response.get('mode') not in ('idle', 'mapping', 'localization', 'navigation', 'fault'):
            raise RuntimeError('unknown_supervisor_mode')
        if not isinstance(response.get('groups'), dict) or not isinstance(response.get('last_exit', {}), dict):
            raise RuntimeError('invalid_supervisor_process_state')
        if response.get('map') is not None and not isinstance(response['map'], str):
            raise RuntimeError('invalid_supervisor_map')
        self.cached = response
        self.observed = time.monotonic()
        self.updated_at = time.time()
        self.error = None

    async def refresh(self):
        if self.lock.locked():
            return
        async with self.lock:
            try:
                response = await asyncio.to_thread(self.client.call, {'command': 'status'}, 3)
                self.accept(response)
            except Exception as exc:
                self.error = str(exc)
                self.observed = None

    async def start(self):
        await self.refresh()
        self.poller = asyncio.create_task(self.observe())

    async def observe(self):
        while True:
            await asyncio.sleep(2)
            await self.refresh()

    async def switch(self, args, request_id=None, expires_at=None):
        if set(args) - {'mode', 'map', 'discard'} or args.get('mode') not in ('idle', 'mapping', 'localization', 'navigation'):
            raise ValueError('invalid_mode_args_use_idle_mapping_localization_navigation')
        if 'discard' in args and type(args['discard']) is not bool:
            raise ValueError('discard_must_be_boolean')
        if not self.config['allow_mode_changes']:
            raise ValueError('mode_execution_disabled_by_local_config')
        if self.task and not self.task.done():
            raise ValueError('mode_operation_in_progress')
        await self.refresh()
        if self.task and not self.task.done():
            raise ValueError('mode_operation_in_progress')
        if self.state()['stale']:
            raise ValueError('supervisor_unavailable')
        if expires_at is not None and time.time() > expires_at:
            raise ValueError('expired')
        mode = args['mode']
        map_id = args.get('map')
        path = None
        if mode in ('localization', 'navigation'):
            path = str(resolve_map(self.config, map_id))
        elif map_id is not None:
            raise ValueError('map_only_for_localization_or_navigation')
        if mode == self.cached['mode'] and (mode not in ('localization', 'navigation') or path == self.cached.get('map')):
            return self.state()
        if self.cached['mode'] == 'mapping' and not args.get('discard', False):
            raise ValueError('mapping_save_then_explicit_discard_required')
        if mode != 'idle' and self.cached['mode'] != 'idle':
            raise ValueError('switch_to_idle_before_starting_another_mode')
        if mode != 'idle':
            graph = self.telemetry.snapshot()['sources'].get('graph', {})
            if graph.get('stale', True):
                raise ValueError('fresh_ros_graph_required')
            nodes = graph['value']['nodes']
            if any(
                name in nodes for name in ('slam_toolbox', 'amcl', 'controller_server', 'bt_navigator', 'map_server')
            ):
                raise ValueError('conflicting_ros_nodes_outside_supervisor_mode')
        # Sensor and ROS lifecycle readiness are checked by the same supervisor path as CLI.
        # The existence of esp32_ctrl's publisher is not a SLAM readiness failure.
        self.operation = {
            'id': request_id or str(uuid.uuid4()),
            'status': 'running',
            'requested_mode': mode,
            'requested_at': time.time(),
            'finished_at': None,
            'error': None,
        }
        request = {'command': 'mode', 'mode': mode, 'map': path, 'discard': args.get('discard', False)}
        self.task = asyncio.create_task(self.change(request))
        return self.state()

    async def change(self, request):
        async with self.lock:
            try:
                response = await asyncio.to_thread(self.client.call, request, 150)
                self.accept(response)
                self.operation['status'] = 'succeeded'
            except Exception as exc:
                self.error = str(exc)
                self.observed = None
                self.operation['status'] = 'failed'
                self.operation['error'] = str(exc)
            finally:
                self.operation['finished_at'] = time.time()

    async def stop(self):
        # Disconnect the adapter only. Supervisor keeps ownership across bridge restarts.
        for task in (self.poller, self.task):
            if task:
                task.cancel()
                with suppress(asyncio.CancelledError):
                    await task
