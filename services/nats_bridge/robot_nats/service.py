import asyncio
from contextlib import suppress
import hashlib
import fcntl
import json
import logging
from pathlib import Path
import sqlite3
import time
import uuid

import nats
from nats.js.api import KeyValueConfig, StorageType
from nats.js.errors import BucketNotFoundError

from .protocol import encode, validate, schema
from .telemetry import Telemetry
from .modes import Modes
from .battery_state import BatteryState
from .history import History
from .map_observer import MapObserver
from .costmap_observer import CostmapObserver
from .map_save import MapSaver
from .saved_maps import SavedMaps, map_name
from .localization import Localization
from .places import Places, SPATIAL_COMMANDS, map_names
from .contracts import COMMANDS, project, observation, charging_state
from .motion import Motion, validate_request


class Service:
    def __init__(self, config):
        self.config = config
        self.boot_id = str(uuid.uuid4())
        self.started = time.time()
        self.nc = None
        self.motion = Motion(config.get('allow_manual_motion', False))
        self.telemetry = Telemetry(
            config.get('ros_enabled', True), self.motion, lambda: bool(self.nc and self.nc.is_connected)
        )
        self.modes = Modes(config, self.telemetry)
        self.mapping = MapObserver(config, self.modes.state)
        self.costmaps = CostmapObserver(config, self.modes.state, self.mapping.store.metadata)
        self.localization = Localization(config, self.modes, self.telemetry, self.mapping)
        self.places = Places(config, self.localization)
        self.map_task = None
        self.costmap_task = None
        self.map_saver = MapSaver(config, self.modes)
        self.saved_maps = SavedMaps(config, self.map_saver.maps)
        self.lock = asyncio.Lock()
        self.nc = None
        self.task = None
        self.closing = False
        self.history_task = None
        self.history = History(Path(config['data_dir']) / 'battery-history.sqlite')
        self.kv = None
        self.kv_error = None
        self.settings = {'note': ''}
        self.previous_metrics = {}
        self.battery = BatteryState(config['robot_id'])
        self.prefix = f'robots.{config["robot_id"]}.v1'
        self.catalog = schema(config['robot_id'], config['domain'], config['bucket'])
        self.catalog['maps'] = map_names(config)
        data = Path(config['data_dir'])
        data.mkdir(parents=True, exist_ok=True)
        self.owner = (data / 'owner.lock').open('a')
        fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.db = sqlite3.connect(data / 'commands.sqlite')
        self.db.execute(
            'CREATE TABLE IF NOT EXISTS commands (id TEXT PRIMARY KEY, digest TEXT, result TEXT, created REAL)'
        )
        self.db.commit()

    async def start(self):
        self.nc = await nats.connect(
            self.config['nats_url'],
            user=self.config['backend_user'],
            password=self.config['backend_password'],
            name='verter-backend',
            max_reconnect_attempts=-1,
            reconnect_time_wait=1,
            pending_size=0,
        )
        js = self.nc.jetstream(domain=self.config['domain'])
        try:
            self.kv = await js.key_value(self.config['bucket'])
        except BucketNotFoundError:
            self.kv = await js.create_key_value(
                KeyValueConfig(
                    bucket=self.config['bucket'],
                    history=1,
                    storage=StorageType.FILE,
                    max_bytes=8 * 1024 * 1024,
                    max_value_size=256 * 1024,
                )
            )
        from nats.js.errors import KeyNotFoundError

        try:
            self.settings = json.loads((await self.kv.get('settings')).value)
        except KeyNotFoundError:
            await self.kv.put('settings', encode(self.settings))
        await self.kv.put('schema', encode(self.catalog))
        await self.nc.subscribe(self.prefix + '.request', queue='robot-api', cb=self.receive)
        for name in COMMANDS:
            await self.nc.subscribe(
                f'{self.config["robot_id"]}.service.command.{name}', queue='robot-api', cb=self.receive
            )
        for operation in ('set', 'keepalive', 'stop', 'status'):
            await self.nc.subscribe(
                f'{self.config["robot_id"]}.cmd_vel.command.{operation}', queue='robot-motion', cb=self.receive_motion
            )
        await self.nc.subscribe(self.prefix + '.bms', cb=self.receive_battery)
        await self.nc.subscribe(f'{self.config["robot_id"]}.battery.command.history', cb=self.receive)
        await self.nc.subscribe(
            f'{self.config["robot_id"]}.map.command.get',
            cb=self.receive,
            pending_msgs_limit=32,
            pending_bytes_limit=512 * 1024,
        )
        await self.nc.subscribe(f'{self.config["robot_id"]}.map.command.save', cb=self.receive)
        await self.nc.subscribe(f'{self.config["robot_id"]}.map.command.delete', cb=self.receive)
        await self.nc.subscribe(
            f'{self.config["robot_id"]}.costmap.command.get',
            cb=self.receive,
            pending_msgs_limit=32,
            pending_bytes_limit=512 * 1024,
        )
        for domain, operation in SPATIAL_COMMANDS.values():
            await self.nc.subscribe(
                f'{self.config["robot_id"]}.{domain}.command.{operation}', cb=self.receive
            )
        await self.nc.flush()
        self.task = asyncio.create_task(self.publish_loop())
        self.history_task = asyncio.create_task(self.record_history())
        await self.modes.start()
        self.mapping.start()
        self.costmaps.start()
        self.map_task = asyncio.create_task(self.publish_map())
        self.costmap_task = asyncio.create_task(self.publish_costmaps())

    def snapshot(self):
        battery = self.battery.snapshot()
        battery['charging_state'] = charging_state(battery)
        return {
            'v': 1,
            'robot_id': self.config['robot_id'],
            'boot_id': self.boot_id,
            'updated_at': time.time(),
            'uptime_s': round(time.time() - self.started, 2),
            'mode': self.modes.state(),
            'live_map': self.map_result({'part': 'metadata'}),
            'costmaps': self.costmaps.store.query({'part': 'state'}),
            'localization': self.localization.snapshot(),
            'places': self.places.snapshot(self.modes.state().get('map')),
            'telemetry': self.telemetry.snapshot(),
            'settings': self.settings,
            'battery': battery,
            'transport': {'connected': bool(self.nc and self.nc.is_connected), 'kv_error': self.kv_error},
            'movement_api': self.motion.enabled,
            'manual_control': self.motion.snapshot(),
            'manager_replaced': False,
        }

    def metric_snapshot(self, state=None):
        state = state or self.snapshot()
        metrics = project(state)
        metrics['service.schema'] = observation(
            self.catalog, None, state['updated_at'], self.boot_id + ':schema', state['updated_at']
        )
        # Preserve explicit disappearance observations while this bridge is running.
        graph = state['telemetry']['sources'].get('graph', {})
        for key, previous in self.previous_metrics.items():
            if key in metrics or not (key.startswith('node.') or key.startswith('battery.cell_')):
                continue
            value = None
            if key.startswith('node.') and graph.get('stale', True):
                metrics[key] = observation(
                    previous['value'],
                    previous['unit'],
                    previous['sample_at'],
                    previous['sample_id'],
                    state['updated_at'],
                    previous['max_age_s'],
                    stale=True,
                )
                continue
            if key.startswith('node.') and not graph.get('stale', True):
                value = dict(previous['value'], present=False)
            metrics[key] = observation(
                value,
                previous['unit'],
                graph.get('received_at') if value else None,
                self.boot_id + ':absent',
                state['updated_at'],
                previous['max_age_s'],
            )
        self.previous_metrics = metrics
        # A null error means no error, not a missing observation.
        metrics['service.kv_error']['quality'] = 'ok'
        return metrics

    def map_result(self, args, defined_maps=()):
        part = args.get('part', 'metadata')
        if part in ('save_status', 'saved'):
            if set(args) != {'part'}:
                raise ValueError('unexpected_map_args')
            if part == 'save_status':
                return self.map_saver.snapshot()
            return self.saved_maps.list(self.modes.state(), defined_maps)
        if part in ('saved_metadata', 'saved_chunk'):
            return self.saved_maps.query(args)
        result = self.mapping.store.query(args)
        if part == 'metadata':
            result['save'] = self.map_saver.snapshot()
        return result

    async def delete_map(self, args, expires_at):
        if set(args) != {'name'}:
            raise ValueError('unexpected_saved_map_args')
        name = map_name(args['name'])
        if not self.config.get('allow_map_delete', self.map_saver.snapshot()['enabled']):
            raise ValueError('map_deletion_disabled')
        if self.map_saver.active or (self.modes.task and not self.modes.task.done()):
            raise ValueError('wait_for_current_map_or_mode_operation')
        async with self.modes.lock:
            try:
                current = await asyncio.to_thread(self.modes.client.call, {'command': 'status'}, 3)
                self.modes.accept(current)
            except Exception as exc:
                raise ValueError('supervisor_unavailable') from exc
            state = self.modes.state()
            if state['actual'] in ('localization', 'navigation') and not state['map_path']:
                raise ValueError('supervisor_unavailable')
            return await asyncio.to_thread(
                self.saved_maps.delete, name, state, self.places.defined_maps(), expires_at
            )

    async def publish_map(self):
        next_metadata = 0
        while not self.closing:
            try:
                if self.nc.is_connected:
                    prefix = self.config['robot_id'] + '.map.update.'
                    if time.monotonic() >= next_metadata:
                        next_metadata = time.monotonic() + 1
                        metadata = self.map_result({'part': 'metadata'})
                        await self.nc.publish(prefix + 'metadata', encode(metadata))
                        if self.closing:
                            return
                        if self.kv:
                            await self.kv.put('map.live', encode(metadata))
                    if self.closing:
                        return
                    await self.nc.publish(prefix + 'pose', encode(self.mapping.store.pose()))
                    if not self.closing:
                        await self.nc.publish(prefix + 'scan', encode(self.mapping.store.scan()))
            except Exception:
                logging.exception('Map publication failed; retrying without changing robot mode')
            await asyncio.sleep(0.2)

    async def publish_costmaps(self):
        next_kv = 0
        while not self.closing:
            try:
                if self.nc.is_connected:
                    state = self.costmaps.store.query({'part': 'state'})
                    write_kv = time.monotonic() >= next_kv
                    if write_kv:
                        next_kv = time.monotonic() + 1
                    for kind in ('local', 'global'):
                        if self.closing:
                            return
                        metadata = encode(state[kind])
                        await self.nc.publish(f'{self.config["robot_id"]}.costmap.update.{kind}', metadata)
                        if write_kv and self.kv and not self.closing:
                            await self.kv.put('costmap.' + kind, metadata)
            except Exception:
                logging.exception('Costmap publication failed; retrying without changing robot mode')
            await asyncio.sleep(0.5)

    async def record_history(self):
        while not self.closing:
            try:
                battery = self.battery.snapshot()
                battery['charging_state'] = charging_state(battery)
                await asyncio.to_thread(self.history.record, battery)
            except Exception:
                logging.exception('BMS history recording failed; live telemetry remains available')
            await asyncio.sleep(2)

    async def publish_loop(self):
        while not self.closing:
            try:
                if not self.nc.is_connected:
                    raise RuntimeError('nats_disconnected')
                state = self.snapshot()
                metrics = self.metric_snapshot(state)
                for key, value in metrics.items():
                    if self.closing:
                        return
                    domain, name = key.split('.', 1)
                    await self.kv.put(key, encode(value))
                    await self.nc.publish(f'{self.config["robot_id"]}.{domain}.update.{name}', encode(value))
                # Compatibility snapshots; new clients use namespaced keys.
                await self.kv.put('state', encode(state))
                await self.kv.put('battery', encode(state['battery']))
                await self.nc.publish(self.prefix + '.state', encode(state))
                self.kv_error = None
            except Exception as exc:
                self.kv_error = str(exc)
                logging.warning('State publication failed: %s', exc)
            await asyncio.sleep(2)

    async def receive_motion(self, msg):
        if not msg.reply or not msg.reply.startswith('_INBOX.'):
            return
        ident = None
        try:
            body = validate_request(msg.data)
            ident = body['id']
            operation = msg.subject.rsplit('.', 1)[-1]
            result = self.motion.command(operation, body['args'], body['issued_at'])
            if operation == 'status':
                result['odometry'] = self.telemetry.snapshot()['sources'].get('odometry')
            response = {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
        except Exception as exc:
            response = {'v': 1, 'id': ident, 'ok': False, 'at': time.time(), 'error': str(exc)}
        if self.nc.is_connected:
            with suppress(Exception):
                await msg.respond(encode(response))

    async def receive_battery(self, msg):
        try:
            if len(msg.data) > 16384:
                raise ValueError('Oversized BMS message')
            data = json.loads(msg.data)
            encode(data)
            self.battery.accept(data)
        except (ValueError, TypeError):
            logging.warning('Invalid BMS message ignored')

    async def receive(self, msg):
        # Require request/reply. Fire-and-forget control messages are ignored.
        if not msg.reply or not msg.reply.startswith('_INBOX.'):
            return
        if msg.subject == self.prefix + '.request':
            response = await self.execute(msg.data)
        else:
            try:
                if len(msg.data) > 16384:
                    raise ValueError('request_too_large')
                body = json.loads(msg.data)
                if not isinstance(body, dict) or 'action' in body:
                    raise ValueError('action_is_encoded_in_subject')
                name = msg.subject.rsplit('.', 1)[-1]
                if msg.subject.endswith('.map.command.save'):
                    body['action'] = 'map.save'
                elif msg.subject.endswith('.map.command.delete'):
                    body['action'] = 'map.delete'
                elif msg.subject.endswith('.map.command.get'):
                    body['action'] = 'map.get'
                elif msg.subject.endswith('.costmap.command.get'):
                    body['action'] = 'costmap.get'
                elif msg.subject.endswith('.battery.command.history'):
                    body['action'] = 'battery.history'
                else:
                    spatial = next((
                        action for action, (domain, operation) in SPATIAL_COMMANDS.items()
                        if msg.subject == f'{self.config["robot_id"]}.{domain}.command.{operation}'
                    ), None)
                    body['action'] = spatial or COMMANDS[name]
                response = await self.execute(encode(body))
            except (ValueError, TypeError, KeyError) as exc:
                response = {'v': 1, 'id': None, 'ok': False, 'at': time.time(), 'error': str(exc)}
        if self.nc.is_connected:
            with suppress(Exception):
                await msg.respond(encode(response))

    async def execute(self, raw):
        ident = None
        try:
            obj = validate(raw)
            ident = obj['id']
            if obj['action'] == 'localization.get':
                if obj['args']:
                    raise ValueError('unexpected_args')
                result = self.localization.snapshot()
                return {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
            if obj['action'] in ('waypoint.list', 'route.list'):
                result = self.places.execute(obj['action'], obj['args'])
                return {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
            if obj['action'] == 'map.get':
                definitions = self.places.defined_maps() if obj['args'].get('part') == 'saved' else ()
                result = await asyncio.to_thread(self.map_result, obj['args'], definitions)
                return {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
            if obj['action'] == 'costmap.get':
                result = await asyncio.to_thread(self.costmaps.store.query, obj['args'])
                return {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
            if obj['action'] == 'battery.history':
                result = await asyncio.to_thread(self.history.query, obj['args'])
                return {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
            # Polling reads must remain fresh and must not fill the durable command journal.
            if obj['action'] in ('ping', 'schema.get', 'state.get', 'metrics.get'):
                if obj['args']:
                    raise ValueError('unexpected_args')
                if obj['action'] == 'ping':
                    result = {'pong': True, 'boot_id': self.boot_id}
                elif obj['action'] == 'schema.get':
                    self.catalog['maps'] = map_names(self.config)
                    result = self.catalog
                elif obj['action'] == 'state.get':
                    result = self.snapshot()
                else:
                    result = self.metric_snapshot()
                return {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
            digest = hashlib.sha256(encode(obj)).hexdigest()
            async with self.lock:
                # Validate again after waiting: queued commands may have expired.
                validate(raw)
                row = self.db.execute('SELECT digest,result FROM commands WHERE id=?', (ident,)).fetchone()
                if row:
                    if row[0] != digest:
                        raise ValueError('id_reused_with_different_payload')
                    if row[1] is None:
                        raise ValueError('outcome_unknown_after_interruption_inspect_state')
                    return json.loads(row[1])
                self.db.execute('DELETE FROM commands WHERE created < ?', (time.time() - 86400,))
                self.db.execute('INSERT INTO commands VALUES (?,?,NULL,?)', (ident, digest, time.time()))
                self.db.commit()
                try:
                    action, args = obj['action'], obj['args']
                    if action in ('ping', 'schema.get', 'state.get', 'metrics.get') and args:
                        raise ValueError('unexpected_args')
                    if action == 'ping':
                        result = {'pong': True, 'boot_id': self.boot_id}
                    elif action == 'schema.get':
                        result = self.catalog
                    elif action == 'state.get':
                        result = self.snapshot()
                    elif action == 'metrics.get':
                        result = self.metric_snapshot()
                    elif action == 'settings.set':
                        if set(args) != {'note'} or not isinstance(args['note'], str) or len(args['note']) > 500:
                            raise ValueError('invalid_note')
                        await self.kv.put('settings', encode(args))
                        self.settings = dict(args)
                        result = self.settings
                    elif action == 'map.save':
                        result = await self.map_saver.start(args, ident, obj['issued_at'] + obj['ttl_s'])
                    elif action == 'map.delete':
                        result = await self.delete_map(args, obj['issued_at'] + obj['ttl_s'])
                    elif action == 'mode.set':
                        if self.map_saver.active:
                            raise ValueError('wait_for_map_save')
                        if self.motion.snapshot()['active']:
                            raise ValueError('finish_manual_session_before_mode_change')
                        result = await self.modes.switch(
                            args, request_id=ident, expires_at=obj['issued_at'] + obj['ttl_s']
                        )
                    elif action == 'localization.initialize':
                        if self.map_saver.active or self.motion.snapshot()['active']:
                            raise ValueError('finish_map_save_or_manual_session_before_initial_pose')
                        result = await self.localization.initialize(args, obj['issued_at'] + obj['ttl_s'])
                    elif action == 'localization.confirm':
                        result = self.localization.confirm(args)
                    elif action.startswith(('waypoint.', 'route.')):
                        result = self.places.execute(action, args)
                    response = {'v': 1, 'id': ident, 'ok': True, 'at': time.time(), 'result': result}
                except Exception as exc:
                    response = {'v': 1, 'id': ident, 'ok': False, 'at': time.time(), 'error': str(exc)}
                self.db.execute('UPDATE commands SET result=? WHERE id=?', (encode(response).decode(), ident))
                self.db.commit()
                return response
        except Exception as exc:
            return {'v': 1, 'id': ident, 'ok': False, 'at': time.time(), 'error': str(exc)}

    async def close(self):
        self.closing = True
        self.localization.close()
        await self.map_saver.close()
        if self.costmap_task:
            self.costmap_task.cancel()
            with suppress(asyncio.CancelledError):
                await self.costmap_task
        self.costmaps.close()
        if self.map_task:
            self.map_task.cancel()
            with suppress(asyncio.CancelledError):
                await self.map_task
        self.mapping.close()
        if self.history_task:
            self.history_task.cancel()
            with suppress(asyncio.CancelledError):
                await self.history_task
        if self.task:
            self.task.cancel()
            with suppress(asyncio.CancelledError):
                await self.task
        await self.modes.stop()
        if self.nc:
            await self.nc.close()
        self.telemetry.close()
        self.db.close()
        self.places.close()
        self.owner.close()
