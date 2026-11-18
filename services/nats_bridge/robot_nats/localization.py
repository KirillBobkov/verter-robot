"""Operator-confirmed localization. Construction and observations never command ROS."""

import asyncio
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


class Localization:
    def __init__(self, config, modes, telemetry, mapping, publisher=None):
        self.config = config
        self.modes = modes
        self.telemetry = telemetry
        self.mapping = mapping
        self.publisher = publisher
        self.lock = threading.RLock()
        self.boot_id = str(uuid.uuid4())
        self.scope = None
        self.session_id = None
        self.initialization_id = None
        self.minimum_stamp = None
        self.session_seen_at = None
        self.initializing = False
        self.confirmed = False
        self.last_initialization = None
        self.closed = False
        self.reset_seen = None
        self.pending_reset = None
        self.own_publication = None
        self.last_reset = None
        self.minimum_amcl_sequence = None

    def _own_reset(self, value):
        own = self.own_publication
        if not own or not number(value.get('stamp')) or abs(value['stamp'] - own['stamp']) > 0.000001:
            return False
        if not all(number(value.get(key)) for key in ('x', 'y', 'yaw')):
            return False
        angle = value['yaw'] - own['yaw']
        return (
            abs(value['x'] - own['x']) < 0.000001
            and abs(value['y'] - own['y']) < 0.000001
            and abs(math.atan2(math.sin(angle), math.cos(angle))) < 0.000001
        )

    def _observe_reset(self, record):
        value = record.get('value') or {}
        token = (record.get('sequence'), record.get('received_at'), value.get('stamp')) if record else None
        if token == self.reset_seen:
            return
        self.reset_seen = token
        if self.scope is None or not record:
            return
        self.confirmed = False
        stamp = value.get('stamp')
        reset_at = stamp if number(stamp) and 0 < stamp <= time.time() + 1 else time.time()
        self.minimum_stamp = max(self.minimum_stamp, reset_at)
        self.last_reset = {'stamp': stamp, 'received_at': record.get('received_at'), 'source': 'initialpose'}
        if self.initializing:
            self.pending_reset = {**value, 'observed_sequence': record.get('sequence')}
        elif not self._own_reset(value):
            self.initialization_id = str(uuid.uuid4())
            self.minimum_amcl_sequence = record.get('sequence')

    def _scope(self):
        state = self.modes.state()
        path = state.get('map_path')
        saved_map = state.get('map')
        if saved_map is None and isinstance(path, str) and path:
            saved_map = Path(path).stem
        key = None
        if (
            state.get('actual') in ('localization', 'navigation')
            and type(state.get('pid')) is int
            and state['pid'] > 0
            and isinstance(path, str)
            and path
            and isinstance(saved_map, str)
            and saved_map
        ):
            key = (state['pid'], path, saved_map)
        reset = self.telemetry.snapshot().get('sources', {}).get('initial_pose', {})
        with self.lock:
            if key != self.scope:
                self.scope = key
                self.confirmed = False
                self.last_initialization = None
                self.session_seen_at = time.time() if key else None
                self.minimum_stamp = self.session_seen_at
                self.initialization_id = str(uuid.uuid4()) if key else None
                raw = json.dumps([self.boot_id, key], separators=(',', ':')).encode()
                self.session_id = hashlib.sha256(raw).hexdigest() if key else None
                self.own_publication = None
                self.pending_reset = None
                self.last_reset = None
                self.minimum_amcl_sequence = None
                reset_value = reset.get('value') or {}
                self.reset_seen = (
                    (reset.get('sequence'), reset.get('received_at'), reset_value.get('stamp')) if reset else None
                )
            self._observe_reset(reset)
            return state, saved_map

    def _enabled(self):
        return bool(self.config.get('allow_initial_pose', self.config.get('allow_mode_changes', False)))

    def _check_session(self, args, expected_initialization=None):
        state, saved_map = self._scope()
        with self.lock:
            if self.closed:
                raise ValueError('localization_backend_closed')
            if self.scope is None or state.get('stale', True):
                raise ValueError('fresh_localization_mode_required')
            if args.get('map') != saved_map:
                raise ValueError('localization_map_mismatch')
            if args.get('session_id') != self.session_id:
                raise ValueError('localization_session_changed')
            if expected_initialization is not None and expected_initialization != self.initialization_id:
                raise ValueError('localization_initialization_changed')
            return state

    def _grid(self, state):
        metadata = self.mapping.store.metadata()
        if (
            not metadata.get('available')
            or metadata.get('stale', True)
            or metadata.get('mode') != state.get('actual')
        ):
            raise ValueError('localization_map_not_ready')
        store_mode = getattr(self.mapping.store, 'mode', {})
        if store_mode.get('pid') is not None and store_mode['pid'] != state['pid']:
            raise ValueError('localization_map_session_changed')
        if metadata.get('saved_map') is not None and metadata['saved_map'] != state.get('map'):
            raise ValueError('localization_map_mismatch')
        return metadata

    def _observed(self):
        state, saved_map = self._scope()
        with self.lock:
            result = {
                'map': saved_map,
                'session_id': self.session_id,
                'initialization_id': self.initialization_id,
                'enabled': self._enabled(),
                'operator_confirmed': self.confirmed,
                'verified': False,
                'ready_for_confirmation': False,
                'initializing': self.initializing,
                'last_initialization': dict(self.last_initialization) if self.last_initialization else None,
                'last_reset': dict(self.last_reset) if self.last_reset else None,
                'updated_at': time.time(),
                'frame_id': 'map',
                'status': 'unavailable',
            }
            try:
                if self.closed:
                    raise ValueError('localization_backend_closed')
                if self.scope is None or state.get('stale', True):
                    raise ValueError('fresh_localization_mode_required')
                if self.initializing:
                    raise ValueError('initial_pose_publication_in_progress')
                metadata = self._grid(state)
                amcl = self.telemetry.snapshot().get('sources', {}).get('localization', {})
                value = amcl.get('value') or {}
                stamp = value.get('stamp')
                if not number(stamp) or stamp < self.minimum_stamp or stamp > time.time() + 1:
                    raise ValueError('waiting_for_current_session_amcl_pose')
                if self.minimum_amcl_sequence is not None:
                    sequence = amcl.get('sequence')
                    if type(sequence) is not int or sequence <= self.minimum_amcl_sequence:
                        raise ValueError('waiting_for_current_session_amcl_pose')
                if value.get('frame_id') != 'map' or not all(number(value.get(k)) for k in ('x', 'y', 'yaw')):
                    raise ValueError('invalid_amcl_pose')
                covariance = value.get('covariance')
                if not isinstance(covariance, list) or len(covariance) != 36 or not all(number(v) for v in covariance):
                    raise ValueError('invalid_amcl_covariance')
                if any(covariance[index] < 0 for index in (0, 7, 35)):
                    raise ValueError('invalid_amcl_covariance')
                result['amcl'] = {
                    'stamp': stamp,
                    'sample_age_s': max(0, time.time() - stamp),
                    'receipt_age_s': amcl.get('age_s'),
                    'x': value['x'],
                    'y': value['y'],
                    'yaw': value['yaw'],
                    'covariance': list(covariance),
                }
                pose = self.mapping.store.pose()
                scan = self.mapping.store.scan()
                for sample, name in ((pose, 'pose'), (scan, 'scan')):
                    if (
                        not sample.get('valid')
                        or sample.get('map_id') != metadata.get('map_id')
                        or not number(sample.get('age_s'))
                        or not 0 <= sample['age_s'] <= 1
                        or not number(sample.get('stamp'))
                        or sample['stamp'] < self.minimum_stamp
                        or not -1 <= time.time() - sample['stamp'] <= 1
                    ):
                        raise ValueError('fresh_current_session_map_' + name + '_required')
                if not all(number(pose.get(key)) for key in ('x', 'y', 'yaw')):
                    raise ValueError('invalid_current_map_pose')
                result['pose'] = {
                    'x': pose['x'],
                    'y': pose['y'],
                    'yaw': pose['yaw'],
                    'sample_at': pose['stamp'],
                    'frame_id': 'map',
                    'map': saved_map,
                    'session_id': self.session_id,
                    'initialization_id': self.initialization_id,
                    'localization_sample_at': stamp,
                    'map_id': pose.get('map_id'),
                }
                result['ready_for_confirmation'] = True
                result['verified'] = self.confirmed
                result['reason'] = None if self.confirmed else 'operator_confirmation_required'
                result['status'] = 'verified' if self.confirmed else 'unverified'
            except ValueError as exc:
                result['reason'] = str(exc)
                if self.initializing:
                    result['status'] = 'publishing'
                elif self.last_initialization and self.last_initialization['status'] == 'published':
                    result['status'] = 'waiting_estimate'
            return result

    def snapshot(self):
        return self._observed()

    def current_pose(self):
        state = self._observed()
        if not state['verified']:
            raise ValueError(state['reason'] or 'operator_confirmation_required')
        return dict(state['pose'])

    async def initialize(self, args, expires_at=None):
        required = {'map', 'session_id', 'x', 'y', 'yaw'}
        if not isinstance(args, dict) or not required <= set(args) or set(args) - required - {'std_xy', 'std_yaw'}:
            raise ValueError('invalid_initial_pose_args')
        args = dict(args)
        if expires_at is not None and (not number(expires_at) or time.time() > expires_at):
            raise ValueError('expired')
        if not self._enabled():
            raise ValueError('initial_pose_disabled')
        if self.publisher is None and not self.config.get('ros_enabled', True):
            raise ValueError('ros_disabled')
        if not all(number(args[key]) for key in ('x', 'y', 'yaw')):
            raise ValueError('invalid_initial_pose')
        std_xy = args.get('std_xy', 0.25)
        std_yaw = args.get('std_yaw', math.pi / 12)
        if not number(std_xy) or not number(std_yaw) or not 0 < std_xy <= 10 or not 0 < std_yaw <= math.pi:
            raise ValueError('invalid_initial_pose_uncertainty')
        state = self._check_session(args)
        metadata = self._grid(state)
        origin = metadata['origin']
        dx, dy = args['x'] - origin['x'], args['y'] - origin['y']
        cosine, sine = math.cos(origin['yaw']), math.sin(origin['yaw'])
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        if (
            not 0 <= local_x < metadata['width'] * metadata['resolution']
            or not 0 <= local_y < metadata['height'] * metadata['resolution']
        ):
            raise ValueError('initial_pose_outside_map')
        with self.lock:
            if self.initializing:
                raise ValueError('initial_pose_publication_in_progress')
            self.confirmed = False
            self.initializing = True
            self.initialization_id = str(uuid.uuid4())
            initialization = self.initialization_id
            self.minimum_stamp = time.time()
            self.pending_reset = None
            self.minimum_amcl_sequence = None
            self.last_initialization = {'status': 'publishing', 'published_at': None, 'error': None}

        def guard():
            if expires_at is not None and time.time() > expires_at:
                raise ValueError('expired')
            state = self._check_session(args, initialization)
            self._grid(state)

        pose = {
            'x': float(args['x']),
            'y': float(args['y']),
            'yaw': math.atan2(math.sin(args['yaw']), math.cos(args['yaw'])),
            'std_xy': float(std_xy),
            'std_yaw': float(std_yaw),
            'frame_id': 'map',
        }
        try:
            publisher = self.publisher
            if publisher is None:
                from .localization_ros import publish_initial_pose

                publisher = publish_initial_pose
            published = await asyncio.to_thread(publisher, pose, guard)
            guard()
            stamp = published.get('published_at') if isinstance(published, dict) else published
            if not number(stamp) or abs(time.time() - stamp) > 10:
                raise ValueError('invalid_initial_pose_publication_time')
            with self.lock:
                self.minimum_stamp = max(self.minimum_stamp, stamp)
                self.own_publication = {'stamp': stamp, **pose}
                if self.pending_reset and not self._own_reset(self.pending_reset):
                    self.initialization_id = str(uuid.uuid4())
                    self.minimum_amcl_sequence = self.pending_reset.get('observed_sequence')
                self.pending_reset = None
                self.last_initialization = {'status': 'published', 'published_at': stamp, 'error': None}
        except asyncio.CancelledError:
            with self.lock:
                if initialization == self.initialization_id:
                    self.initialization_id = str(uuid.uuid4())
                    self.last_initialization = {
                        'status': 'interrupted',
                        'published_at': None,
                        'error': 'initial_pose_outcome_unknown_inspect_state',
                    }
            raise
        except Exception as exc:
            with self.lock:
                if initialization == self.initialization_id:
                    self.last_initialization = {'status': 'failed', 'published_at': None, 'error': str(exc)}
            raise
        finally:
            with self.lock:
                self.initializing = False
        result = self.snapshot()
        result['status'] = 'waiting_estimate'
        return result

    def confirm(self, args):
        if not isinstance(args, dict) or set(args) != {'map', 'session_id', 'initialization_id'}:
            raise ValueError('invalid_localization_confirmation_args')
        if not isinstance(args['initialization_id'], str) or not args['initialization_id']:
            raise ValueError('invalid_localization_confirmation_args')
        if not self._enabled():
            raise ValueError('initial_pose_disabled')
        self._check_session(args, args['initialization_id'])
        observed = self._observed()
        if not observed['ready_for_confirmation']:
            raise ValueError(observed['reason'])
        with self.lock:
            self._check_session(args, args['initialization_id'])
            self.confirmed = True
        return self.snapshot()

    def close(self):
        with self.lock:
            self.closed = True
            self.confirmed = False
