"""Bounded raw-costmap snapshots; no ROS imports or control operations."""

import base64
from collections import OrderedDict
import copy
import hashlib
import json
import math
import threading
import time
import zlib


MAX_CELLS = 4 * 1024 * 1024
CHUNK_BYTES = 48 * 1024
MAX_AGE = {'local': 1.5, 'global': 3.0}
MAX_CACHE_BYTES = 32 * 1024 * 1024
PIN_SECONDS = 10.0


def planar_pose(pose):
    return planar_components(pose.position, pose.orientation)


def planar_components(position, rotation):
    values = [position.x, position.y, position.z, rotation.x, rotation.y, rotation.z, rotation.w]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('nonfinite_costmap_pose')
    norm = sum(value * value for value in values[3:])
    if abs(norm - 1.0) > 0.001:
        raise ValueError('nonunit_costmap_rotation')
    if abs(rotation.x) > 0.00001 or abs(rotation.y) > 0.00001:
        raise ValueError('nonplanar_costmap_rotation')
    return {
        'x': float(position.x),
        'y': float(position.y),
        'yaw': math.atan2(2 * rotation.w * rotation.z, 1 - 2 * rotation.z * rotation.z),
    }


def transform_origin(origin, transform):
    """Compose map<-source with the grid origin without resampling its cells."""
    source = planar_pose(origin)
    mapping = planar_components(transform.translation, transform.rotation)
    cosine = math.cos(mapping['yaw'])
    sine = math.sin(mapping['yaw'])
    angle = source['yaw'] + mapping['yaw']
    return {
        'x': mapping['x'] + cosine * source['x'] - sine * source['y'],
        'y': mapping['y'] + sine * source['x'] + cosine * source['y'],
        'yaw': math.atan2(math.sin(angle), math.cos(angle)),
    }


class CostmapStore:
    def __init__(self, robot_id, clock=time.monotonic, enabled=True):
        self.robot_id = robot_id
        self.clock = clock
        self.enabled = enabled
        self.lock = threading.RLock()
        self.session = None
        self.generation = 0
        self.map_id = None
        self.saved_map = None
        self.mode = {'actual': 'unknown', 'stale': True}
        self.streams = self.empty_streams()

    @staticmethod
    def empty_streams():
        return {
            kind: {
                'versions': OrderedDict(), 'revision': 0, 'signature': None,
                'received': None, 'stamp': None, 'sample_age': None, 'error': None,
                'pins': OrderedDict(),
            }
            for kind in MAX_AGE
        }

    @staticmethod
    def check_kind(kind):
        if not isinstance(kind, str) or kind not in MAX_AGE:
            raise ValueError('invalid_costmap_kind')

    def set_scope(self, session, mode, map_id, saved_map=None):
        with self.lock:
            changed = session != self.session or map_id != self.map_id
            self.mode = dict(mode)
            self.saved_map = saved_map
            if changed:
                self.generation += 1
                self.session = session
                self.map_id = map_id
                self.streams = self.empty_streams()
            return changed

    def costmap_id(self, kind):
        if self.session is None:
            return None
        return f'{self.session}:{self.generation}:{kind}'

    def trim_cache(self):
        now = self.clock()
        for stream in self.streams.values():
            for revision, (until, _) in list(stream['pins'].items()):
                if now >= until:
                    stream['pins'].pop(revision)
            while len(stream['versions']) > 3:
                stream['versions'].popitem(last=False)

        def pins():
            return sorted(
                (until, kind, revision)
                for kind, stream in self.streams.items()
                for revision, (until, _) in stream['pins'].items()
            )

        for _, kind, revision in pins()[:-2]:
            self.streams[kind]['pins'].pop(revision)

        def total_bytes():
            entries = {}
            for kind, stream in self.streams.items():
                for revision, entry in stream['versions'].items():
                    entries[(kind, revision)] = entry
                for revision, (_, entry) in stream['pins'].items():
                    entries[(kind, revision)] = entry
            return sum(len(entry[1]) for entry in entries.values())

        while total_bytes() > MAX_CACHE_BYTES:
            pinned = pins()
            if pinned:
                _, kind, revision = pinned[0]
                self.streams[kind]['pins'].pop(revision)
                continue
            candidates = [
                (entry[2], kind, revision)
                for kind, stream in self.streams.items()
                for revision, entry in stream['versions'].items()
                if revision != stream['revision']
            ]
            if not candidates:
                break
            _, kind, revision = min(candidates)
            self.streams[kind]['versions'].pop(revision)

    def fail(self, kind, error):
        self.check_kind(kind)
        with self.lock:
            self.streams[kind]['error'] = str(error)[:250]

    def put_grid(
        self, session, kind, width, height, resolution, origin, values,
        stamp, sample_age, source_frame, source_origin=None,
    ):
        processing_started = self.clock()
        self.check_kind(kind)
        with self.lock:
            generation = self.generation
            if session is None or session != self.session:
                return False
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
            raise ValueError('invalid_costmap_dimensions')
        if width * height > MAX_CELLS or len(values) != width * height:
            raise ValueError('costmap_size_limit_or_length_mismatch')
        if type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid_costmap_resolution')
        if set(origin) != {'x', 'y', 'yaw'} or not all(math.isfinite(value) for value in origin.values()):
            raise ValueError('invalid_costmap_origin')
        if not all(math.isfinite(value) for value in (stamp, sample_age)):
            raise ValueError('invalid_costmap_timestamp')
        if stamp <= 0 or not -0.1 <= sample_age <= MAX_AGE[kind]:
            raise ValueError('costmap_timestamp_outside_fresh_window')
        if source_frame != ('odom' if kind == 'local' else 'map'):
            raise ValueError('unexpected_costmap_source_frame')
        if isinstance(values, (bytes, bytearray)):
            raw = bytes(values)
        elif getattr(values, 'typecode', None) == 'B':
            raw = values.tobytes()
        else:
            if any(type(value) is not int or not 0 <= value <= 255 for value in values):
                raise ValueError('costmap_cells_must_be_uint8')
            raw = bytes(values)
        if len(raw) != width * height:
            raise ValueError('costmap_cells_must_be_uint8')

        geometry = {
            'width': width, 'height': height, 'resolution': resolution,
            'origin': dict(origin), 'source_frame_id': source_frame,
        }
        if source_origin is not None:
            geometry['source_origin'] = dict(source_origin)
        raw_digest = hashlib.sha256(raw)
        signature = hashlib.sha256(
            raw_digest.digest() + json.dumps(geometry, sort_keys=True, allow_nan=False).encode()
        ).hexdigest()
        with self.lock:
            if session != self.session or generation != self.generation:
                return False
            stream = self.streams[kind]
            if stream['stamp'] is not None and stamp <= stream['stamp']:
                raise ValueError('nonincreasing_costmap_stamp')
        packed = zlib.compress(raw, level=1)
        with self.lock:
            if session != self.session or generation != self.generation:
                return False
            stream = self.streams[kind]
            if stream['stamp'] is not None and stamp <= stream['stamp']:
                raise ValueError('nonincreasing_costmap_stamp')
            now = self.clock()
            effective_age = max(0, sample_age) + max(0, now - processing_started)
            if effective_age > MAX_AGE[kind]:
                raise ValueError('costmap_timestamp_outside_fresh_window')
            if signature != stream['signature']:
                stream['revision'] += 1
                metadata = {
                    **geometry,
                    'v': 1, 'robot_id': self.robot_id, 'kind': kind,
                    'costmap_id': self.costmap_id(kind), 'map_id': self.map_id,
                    'revision': stream['revision'], 'frame_id': 'map',
                    'encoding': 'zlib-uint8-row-major',
                    'decoded_bytes': len(raw), 'compressed_bytes': len(packed),
                    'chunk_bytes': CHUNK_BYTES,
                    'chunks': (len(packed) + CHUNK_BYTES - 1) // CHUNK_BYTES,
                    'sha256': hashlib.sha256(packed).hexdigest(),
                    'decoded_sha256': raw_digest.hexdigest(),
                }
                stream['versions'][stream['revision']] = (metadata, packed, now, effective_age)
            else:
                metadata, cached, _, _ = stream['versions'][stream['revision']]
                stream['versions'][stream['revision']] = (metadata, cached, now, effective_age)
            stream['received'] = now
            stream['stamp'] = stamp
            stream['sample_age'] = effective_age
            stream['signature'] = signature
            stream['error'] = None
            self.trim_cache()
            return True

    def metadata(self, kind):
        self.check_kind(kind)
        with self.lock:
            stream = self.streams[kind]
            mode = self.mode.get('actual', 'unknown')
            base = {
                'v': 1, 'robot_id': self.robot_id, 'kind': kind,
                'costmap_id': self.costmap_id(kind),
                'map_id': self.map_id, 'saved_map': self.saved_map,
                'frame_id': 'map', 'mode': mode, 'max_age_s': MAX_AGE[kind],
                'updated_at': time.time(),
            }
            if not stream['versions']:
                reason = 'waiting_for_costmap'
                if not self.enabled:
                    reason = 'ros_observation_disabled'
                elif mode != 'navigation':
                    reason = 'nav2_not_active'
                elif self.mode.get('stale'):
                    reason = 'mode_state_stale'
                elif self.session is None:
                    reason = 'waiting_for_active_map'
                return {
                    **base, 'available': False, 'stale': True, 'age_s': None,
                    'reason': stream['error'] or reason,
                }
            metadata = copy.deepcopy(next(reversed(stream['versions'].values()))[0])
            age = stream['sample_age'] + max(0, self.clock() - stream['received'])
            reason = stream['error']
            if not reason and (self.mode.get('stale') or mode != 'navigation'):
                reason = 'mode_state_stale' if self.mode.get('stale') else 'nav2_not_active'
            if not reason and age > MAX_AGE[kind]:
                reason = 'costmap_stale'
            return {
                **metadata, **base, 'available': True, 'stale': bool(reason),
                'reason': reason, 'age_s': round(age, 3), 'stamp': stream['stamp'],
            }

    def state(self):
        with self.lock:
            return {
                'v': 1, 'robot_id': self.robot_id,
                'mode': self.mode.get('actual', 'unknown'), 'map_id': self.map_id,
                'updated_at': time.time(),
                'local': self.metadata('local'), 'global': self.metadata('global'),
            }

    def query(self, args):
        if not isinstance(args, dict):
            raise ValueError('invalid_costmap_args')
        part = args.get('part', 'state')
        if part == 'state':
            if set(args) - {'part'}:
                raise ValueError('unexpected_costmap_args')
            return self.state()
        if part == 'metadata':
            if set(args) != {'part', 'kind'}:
                raise ValueError('costmap_metadata_requires_kind')
            return self.metadata(args['kind'])
        if part != 'chunk':
            raise ValueError('unknown_costmap_part')
        if set(args) != {'part', 'kind', 'costmap_id', 'revision', 'index'}:
            raise ValueError('costmap_chunk_requires_kind_id_revision_index')
        kind = args['kind']
        self.check_kind(kind)
        if type(args['revision']) is not int or type(args['index']) is not int:
            raise ValueError('invalid_costmap_chunk_index')
        with self.lock:
            self.trim_cache()
            current = self.metadata(kind)
            stream = self.streams[kind]
            entry = stream['versions'].get(args['revision'])
            pinned = stream['pins'].get(args['revision'])
            if entry is None and pinned is not None:
                entry = pinned[1]
            if args['costmap_id'] != current['costmap_id'] or entry is None:
                raise ValueError('costmap_revision_unavailable_refetch_metadata')
            if current['stale']:
                raise ValueError('costmap_not_current_refetch_metadata')
            metadata, packed, created, sample_age = entry
            if not pinned and args['revision'] != stream['revision'] and self.clock() - created + sample_age > MAX_AGE[kind]:
                raise ValueError('costmap_revision_stale_refetch_metadata')
            index = args['index']
            if not 0 <= index < metadata['chunks']:
                raise ValueError('invalid_costmap_chunk_index')
            if index == 0 and pinned is None:
                stream['pins'][args['revision']] = (self.clock() + PIN_SECONDS, entry)
                self.trim_cache()
            data = packed[index * CHUNK_BYTES:(index + 1) * CHUNK_BYTES]
            return {
                'kind': kind, 'costmap_id': metadata['costmap_id'], 'map_id': metadata['map_id'],
                'revision': metadata['revision'], 'index': index, 'chunks': metadata['chunks'],
                'data_b64': base64.b64encode(data).decode(),
            }
