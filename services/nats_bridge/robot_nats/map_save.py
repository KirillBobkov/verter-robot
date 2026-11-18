"""Save grid through supervisor and serialize SLAM graph. No velocity or navigation commands."""

import asyncio
from contextlib import suppress
import hashlib
import json
from pathlib import Path
import re
import time

NAME = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z')
SUFFIXES = ('.yaml', '.pgm', '.posegraph', '.data', '.manifest.json')


def serialize_graph(filename):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from slam_toolbox.srv import SerializePoseGraph

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'verter_map_serializer', context=context, enable_rosout=False, start_parameter_services=False
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        client = node.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=5):
            raise RuntimeError('slam_serializer_unavailable')
        request = SerializePoseGraph.Request()
        request.filename = str(filename)
        future = client.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=25)
        if not future.done() or future.result() is None:
            raise RuntimeError('slam_serialization_outcome_unknown_inspect_files')
        if future.result().result != 0:
            raise RuntimeError('slam_serialization_failed_' + str(future.result().result))
    finally:
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


class MapSaver:
    def __init__(self, config, modes, serializer=None):
        self.config = config
        self.modes = modes
        self.maps = Path(config.get('project_root', Path(__file__).resolve().parents[3])) / 'data/maps'
        self.state_file = Path(config['data_dir']) / 'map-save-state.json'
        self.serializer = serializer
        self.task = None
        self.state = {'status': 'idle', 'id': None, 'name': None, 'error': None, 'files': []}
        try:
            saved = json.loads(self.state_file.read_text())
            if isinstance(saved, dict) and saved.get('status') in ('running', 'succeeded', 'failed', 'interrupted'):
                self.state = saved
                if saved['status'] == 'running':
                    self.state.update(status='interrupted', error='bridge_restarted_inspect_saved_files')
        except (OSError, ValueError):
            pass

    @property
    def active(self):
        return self.task is not None and not self.task.done()

    def snapshot(self):
        return {
            **self.state,
            'busy': self.active,
            'enabled': self.config.get('allow_map_save', self.config.get('allow_mode_changes', False)),
        }

    def persist(self):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.state_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.state, ensure_ascii=False, indent=2) + '\n')
        temporary.replace(self.state_file)

    def list_maps(self):
        result = []
        for path in sorted(self.maps.glob('*.yaml')):
            if NAME.fullmatch(path.stem) and path.resolve().parent == self.maps.resolve():
                result.append(
                    {
                        'name': path.stem,
                        'grid': path.with_suffix('.pgm').is_file(),
                        'pose_graph': path.with_suffix('.posegraph').is_file() and path.with_suffix('.data').is_file(),
                    }
                )
        return result

    async def start(self, args, request_id, expires_at=None):
        if set(args) != {'name'} or not isinstance(args['name'], str) or not NAME.fullmatch(args['name']):
            raise ValueError('map_name_must_be_1_to_64_ascii_letters_digits_underscore_dash')
        if not self.snapshot()['enabled']:
            raise ValueError('map_saving_disabled')
        if self.active or (self.modes.task and not self.modes.task.done()):
            raise ValueError('wait_for_current_map_or_mode_operation')
        if not self.config.get('ros_enabled', True) and self.serializer is None:
            raise ValueError('ros_disabled')
        await self.modes.refresh()
        if self.active or (self.modes.task and not self.modes.task.done()):
            raise ValueError('wait_for_current_map_or_mode_operation')
        state = self.modes.state()
        if state['stale'] or state['actual'] != 'mapping' or not state['pid']:
            raise ValueError('fresh_mapping_mode_required_to_save')
        if expires_at is not None and time.time() > expires_at:
            raise ValueError('expired')
        name = args['name']
        self.maps.mkdir(parents=True, exist_ok=True)
        path = self.maps / name
        if any(path.with_suffix(suffix).exists() for suffix in SUFFIXES):
            raise ValueError('map_already_exists_choose_another_name')
        self.state = {
            'id': request_id,
            'status': 'running',
            'stage': 'queued',
            'name': name,
            'base_path': str(path),
            'source_pid': state['pid'],
            'started_at': time.time(),
            'finished_at': None,
            'grid_saved': False,
            'graph_saved': False,
            'error': None,
            'files': [],
        }
        self.persist()
        self.task = asyncio.create_task(self.save(path, state['pid']))
        return self.snapshot()

    def verify_mode(self, result, pid):
        if not result.get('ok'):
            raise RuntimeError(result.get('error', 'supervisor_save_failed'))
        if result.get('mode') != 'mapping' or result.get('groups', {}).get('mode') != pid:
            raise RuntimeError('mapping_session_changed')

    def verify_files(self, path):
        result = []
        for suffix in SUFFIXES[:4]:
            file = path.with_suffix(suffix)
            if not file.is_file() or file.stat().st_size == 0:
                raise RuntimeError('saved_map_file_missing_' + suffix)
            digest = hashlib.sha256()
            with file.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(chunk)
            result.append({'name': file.name, 'bytes': file.stat().st_size, 'sha256': digest.hexdigest()})
        manifest = {
            'name': path.name,
            'created_at': time.time(),
            'source_pid': self.state['source_pid'],
            'files': result,
        }
        path.with_suffix('.manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
        return result

    async def save(self, path, pid):
        try:
            async with self.modes.lock:
                current = await asyncio.to_thread(self.modes.client.call, {'command': 'status'}, 3)
                self.verify_mode(current, pid)
                self.state['stage'] = 'grid'
                self.persist()
                result = await asyncio.to_thread(self.modes.client.call, {'command': 'save-map', 'path': str(path)}, 40)
                self.verify_mode(result, pid)
                self.modes.accept(result)
            self.state.update(grid_saved=True, stage='pose_graph')
            self.persist()
            serializer = self.serializer or serialize_graph
            await asyncio.to_thread(serializer, path)
            async with self.modes.lock:
                current = await asyncio.to_thread(self.modes.client.call, {'command': 'status'}, 3)
                self.verify_mode(current, pid)
                self.modes.accept(current)
            self.state['files'] = await asyncio.to_thread(self.verify_files, path)
            self.state.update(graph_saved=True, status='succeeded', stage='complete')
        except asyncio.CancelledError:
            self.state.update(status='interrupted', error='save_interrupted_inspect_files')
            raise
        except Exception as exc:
            self.state.update(status='failed', error=str(exc))
        finally:
            self.state['finished_at'] = time.time()
            self.persist()

    async def close(self):
        if self.active:
            try:
                await asyncio.wait_for(asyncio.shield(self.task), timeout=25)
            except asyncio.TimeoutError:
                self.task.cancel()
                with suppress(asyncio.CancelledError):
                    await self.task
