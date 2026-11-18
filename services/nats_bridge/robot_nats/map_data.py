"""Bounded live-map snapshots. Pure data code, with no ROS or control operations."""

import base64
from collections import OrderedDict, deque
import copy
import hashlib
import json
import math
import threading
import time
import zlib

MAX_CELLS = 4 * 1024 * 1024
CHUNK_BYTES = 48 * 1024


def rectangle(x0, y0, x1, y1):
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def project_scan(scan, transform):
    if (
        not 0 < len(scan.ranges) <= 20000
        or not all(
            math.isfinite(value) for value in (scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)
        )
        or scan.angle_increment <= 0
        or not 0 <= scan.range_min < scan.range_max
    ):
        raise ValueError('invalid_scan_geometry')
    q = transform.rotation
    t = transform.translation
    if not all(math.isfinite(value) for value in (q.x, q.y, q.z, q.w, t.x, t.y)):
        raise ValueError('invalid_scan_transform')
    if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1) > 0.001:
        raise ValueError('nonunit_scan_rotation')
    step = max(1, math.ceil(len(scan.ranges) / 1800))
    points = []
    for index in range(0, len(scan.ranges), step):
        radius = scan.ranges[index]
        if not math.isfinite(radius) or not scan.range_min <= radius <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        x, y = radius * math.cos(angle), radius * math.sin(angle)
        points.append(
            [
                round(t.x + (1 - 2 * (q.y * q.y + q.z * q.z)) * x + 2 * (q.x * q.y - q.z * q.w) * y, 4),
                round(t.y + 2 * (q.x * q.y + q.z * q.w) * x + (1 - 2 * (q.x * q.x + q.z * q.z)) * y, 4),
            ]
        )
    return points


def robot_model(geometry):
    low, high = geometry['body']['bounds_ground']
    polygons = [{'name': 'body', 'kind': 'body', 'points': rectangle(low[0], low[1], high[0], high[1])}]
    for group in ('drive_wheels', 'rear_wheels'):
        wheels = geometry[group]
        radius = wheels['diameter'] / 2
        half_width = wheels['width'] / 2
        for index, (x, y, _) in enumerate(wheels['centers_ground']):
            polygons.append(
                {
                    'name': f'{group}_{index}',
                    'kind': 'wheel',
                    'points': rectangle(x - radius, y - half_width, x + radius, y + half_width),
                }
            )
    return {
        'available': True,
        'frame_id': 'base_footprint',
        'unit': 'm',
        'polygons': polygons,
        'footprint': geometry['footprint'],
        'source': 'verter_description/config/geometry.json',
    }


class MapStore:
    def __init__(self, robot_id, model=None, clock=time.monotonic):
        self.robot_id = robot_id
        self.model = model or {'available': False, 'reason': 'robot_geometry_unavailable'}
        self.clock = clock
        self.lock = threading.Lock()
        self.session = None
        self.mode = {'actual': 'unknown', 'stale': True}
        self.versions = OrderedDict()
        self.pins = OrderedDict()
        self.revision = 0
        self.received = None
        self.stamp = None
        self.signature = None
        self.pose_record = None
        self.scan_record = None
        self.scan_headers = deque(maxlen=256)
        self.error = None

    def set_scope(self, session, mode):
        with self.lock:
            self.mode = dict(mode)
            changed = session != self.session
            if changed:
                self.session = session
                self.versions.clear()
                self.pins.clear()
                self.revision = 0
                self.received = None
                self.signature = None
                self.pose_record = None
                self.scan_record = None
                self.scan_headers.clear()
                self.error = None
            return changed

    def fail(self, error):
        with self.lock:
            self.error = str(error)[:250]

    def put_grid(self, session, width, height, resolution, origin, values, stamp):
        if type(width) is not int or type(height) is not int or width <= 0 or height <= 0:
            raise ValueError('invalid_map_dimensions')
        if width * height > MAX_CELLS or len(values) != width * height:
            raise ValueError('map_size_limit_or_length_mismatch')
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid_map_resolution')
        if set(origin) != {'x', 'y', 'yaw'} or not all(math.isfinite(v) for v in origin.values()):
            raise ValueError('invalid_map_origin')
        if not math.isfinite(stamp) or min(values) < -1 or max(values) > 100:
            raise ValueError('invalid_map_cells_or_timestamp')
        raw = values.tobytes() if hasattr(values, 'tobytes') else bytes(value & 255 for value in values)
        if len(raw) != width * height:
            raise ValueError('map_cells_must_be_int8')
        geometry = {'width': width, 'height': height, 'resolution': resolution, 'origin': dict(origin)}
        digest = hashlib.sha256(raw)
        signature = hashlib.sha256(digest.digest() + json.dumps(geometry, sort_keys=True).encode()).hexdigest()
        with self.lock:
            if session != self.session or session is None:
                return
            if signature == self.signature:
                self.received = self.clock()
                self.stamp = stamp
                self.error = None
                return
        packed = zlib.compress(raw, level=1)
        with self.lock:
            if session != self.session:
                return
            self.revision += 1
            metadata = {
                **geometry,
                'v': 1,
                'robot_id': self.robot_id,
                'map_id': session,
                'revision': self.revision,
                'frame_id': 'map',
                'encoding': 'zlib-int8-row-major',
                'decoded_bytes': len(raw),
                'compressed_bytes': len(packed),
                'chunk_bytes': CHUNK_BYTES,
                'chunks': (len(packed) + CHUNK_BYTES - 1) // CHUNK_BYTES,
                'sha256': hashlib.sha256(packed).hexdigest(),
                'decoded_sha256': digest.hexdigest(),
            }
            self.versions[self.revision] = (metadata, packed)
            while len(self.versions) > 2:
                self.versions.popitem(last=False)
            self.received = self.clock()
            self.stamp = stamp
            self.signature = signature
            self.error = None

    def metadata(self):
        with self.lock:
            mode = self.mode.get('actual', 'unknown')
            if not self.versions:
                return {
                    'v': 1,
                    'robot_id': self.robot_id,
                    'available': False,
                    'stale': True,
                    'map_id': self.session,
                    'mode': mode,
                    'mode_pid': self.mode.get('pid'),
                    'saved_map': self.mode.get('map'),
                    'frame_id': 'map',
                    'updated_at': time.time(),
                    'reason': self.error or ('waiting_for_map' if self.session else 'no_active_map'),
                }
            metadata = copy.deepcopy(next(reversed(self.versions.values()))[0])
            age = max(0, self.clock() - self.received)
            return {
                **metadata,
                'available': True,
                'age_s': round(age, 3),
                'stamp': self.stamp,
                'updated_at': time.time(),
                'mode': mode,
                'mode_pid': self.mode.get('pid'),
                'saved_map': self.mode.get('map'),
                'error': self.error,
                'stale': bool(self.error or self.mode.get('stale') or (mode == 'mapping' and age > 15)),
            }

    def put_pose(self, session, stamp, x, y, yaw, sample_age):
        if not all(math.isfinite(value) for value in (stamp, x, y, yaw, sample_age)) or abs(sample_age) > 1:
            raise ValueError('invalid_or_old_map_pose')
        with self.lock:
            if session == self.session and session is not None:
                self.pose_record = (
                    self.clock(),
                    {'stamp': stamp, 'x': x, 'y': y, 'yaw': yaw, 'sample_age_s': max(0, sample_age)},
                )

    def invalidate_pose(self, reason):
        with self.lock:
            self.pose_record = (self.clock(), {'reason': str(reason)[:250]})

    def pose(self):
        with self.lock:
            base = {
                'v': 1,
                'robot_id': self.robot_id,
                'map_id': self.session,
                'revision': self.revision,
                'frame_id': 'map',
                'child_frame_id': 'base_footprint',
                'updated_at': time.time(),
            }
            if not self.versions or not self.pose_record:
                return {**base, 'valid': False, 'reason': 'waiting_for_map_pose'}
            received, value = self.pose_record
            age = self.clock() - received + value.get('sample_age_s', 0)
            if 'reason' in value or age > 1 or self.mode.get('stale'):
                return {**base, 'valid': False, 'reason': value.get('reason', 'map_pose_stale')}
            return {**base, **value, 'valid': True, 'age_s': round(max(0, age), 3)}

    def note_scan(self, count, increment, scan_time):
        if type(count) is not int or count < 0 or not math.isfinite(increment) or not math.isfinite(scan_time):
            return
        with self.lock:
            self.scan_headers.append((self.clock(), count, increment, scan_time))

    def diagnostics(self):
        with self.lock:
            recent = [item for item in self.scan_headers if self.clock() - item[0] <= 10]
        if not recent:
            return {'samples_10s': 0, 'variable_scan_geometry': None}
        counts = [item[1] for item in recent]
        increments = [item[2] for item in recent]
        times = [item[3] for item in recent]
        return {
            'samples_10s': len(recent),
            'beam_count_min': min(counts),
            'beam_count_max': max(counts),
            'angle_increment_min': min(increments),
            'angle_increment_max': max(increments),
            'scan_time_min': min(times),
            'scan_time_max': max(times),
            'variable_scan_geometry': min(counts) != max(counts) or max(increments) - min(increments) > 1e-8,
        }

    def put_scan(self, session, stamp, points, sample_age, source_frame, raw_count):
        if not math.isfinite(stamp) or not math.isfinite(sample_age) or abs(sample_age) > 1:
            raise ValueError('invalid_or_old_map_scan')
        if len(points) > 1800 or any(len(point) != 2 or not all(math.isfinite(v) for v in point) for point in points):
            raise ValueError('invalid_scan_points')
        with self.lock:
            if session == self.session and session is not None:
                self.scan_record = (
                    self.clock(),
                    {
                        'stamp': stamp,
                        'sample_age_s': max(0, sample_age),
                        'source_frame_id': source_frame,
                        'raw_count': raw_count,
                        'points': points,
                    },
                )

    def invalidate_scan(self, reason):
        with self.lock:
            self.scan_record = (self.clock(), {'reason': str(reason)[:250]})

    def scan(self):
        with self.lock:
            base = {
                'v': 1,
                'robot_id': self.robot_id,
                'map_id': self.session,
                'revision': self.revision,
                'frame_id': 'map',
                'updated_at': time.time(),
            }
            if not self.versions or not self.scan_record:
                return {**base, 'valid': False, 'reason': 'waiting_for_map_scan'}
            received, value = self.scan_record
            age = self.clock() - received + value.get('sample_age_s', 0)
            if 'reason' in value or age > 1 or self.mode.get('stale'):
                return {**base, 'valid': False, 'reason': value.get('reason', 'map_scan_stale')}
            return {**base, **copy.deepcopy(value), 'valid': True, 'age_s': round(max(0, age), 3)}

    def query(self, args):
        part = args.get('part', 'metadata')
        if part != 'chunk':
            if set(args) - {'part'}:
                raise ValueError('unexpected_map_args')
            if part == 'metadata':
                return self.metadata()
            if part == 'pose':
                return self.pose()
            if part == 'model':
                return copy.deepcopy(self.model)
            if part == 'scan':
                return self.scan()
            if part == 'diagnostics':
                return self.diagnostics()
            raise ValueError('unknown_map_part')
        if set(args) != {'part', 'map_id', 'revision', 'index'}:
            raise ValueError('chunk_requires_map_id_revision_index')
        if type(args['revision']) is not int or type(args['index']) is not int:
            raise ValueError('invalid_map_chunk_index')
        with self.lock:
            expired = [revision for revision, (until, _) in self.pins.items() if self.clock() >= until]
            for revision in expired:
                self.pins.pop(revision)
            entry = self.versions.get(args['revision'])
            if entry is None and args['revision'] in self.pins:
                entry = self.pins[args['revision']][1]
            if args['map_id'] != self.session or entry is None:
                raise ValueError('map_revision_unavailable_refetch_metadata')
            metadata, data = entry
            index = args['index']
            if not 0 <= index < metadata['chunks']:
                raise ValueError('invalid_map_chunk_index')
            if index == 0 and args['revision'] not in self.pins:
                self.pins[args['revision']] = (self.clock() + 60, entry)
                while len(self.pins) > 2:
                    self.pins.popitem(last=False)
            chunk = data[index * CHUNK_BYTES : (index + 1) * CHUNK_BYTES]
            return {
                'map_id': self.session,
                'revision': args['revision'],
                'index': index,
                'chunks': metadata['chunks'],
                'data_b64': base64.b64encode(chunk).decode(),
            }
