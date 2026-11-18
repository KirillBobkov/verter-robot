import json
import math
import time
import uuid
from .places import SPATIAL_COMMANDS

ACTIONS = [
    'ping',
    'schema.get',
    'state.get',
    'metrics.get',
    'settings.set',
    'mode.set',
    'battery.history',
    'map.get',
    'map.save',
    'map.delete',
    'costmap.get',
    *SPATIAL_COMMANDS,
]


def encode(value):
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(',', ':')).encode()


def validate(raw, now=None):
    if len(raw) > 16384:
        raise ValueError('request_too_large')
    obj = json.loads(raw, parse_constant=lambda value: (_ for _ in ()).throw(ValueError('nonfinite')))
    if not isinstance(obj, dict) or set(obj) != {'v', 'id', 'issued_at', 'ttl_s', 'action', 'args'}:
        raise ValueError('invalid_envelope')
    if type(obj['v']) is not int or obj['v'] != 1:
        raise ValueError('unsupported_version')
    uuid.UUID(obj['id'])
    for key in ('issued_at', 'ttl_s'):
        if type(obj[key]) not in (float, int) or not math.isfinite(obj[key]):
            raise ValueError('invalid_time')
    now = time.time() if now is None else now
    if not 0 < obj['ttl_s'] <= 30 or obj['issued_at'] > now + 2:
        raise ValueError('invalid_time_window')
    if now > obj['issued_at'] + obj['ttl_s']:
        raise ValueError('expired')
    if obj['action'] not in ACTIONS or not isinstance(obj['args'], dict):
        raise ValueError('invalid_action_or_args')
    return obj


def envelope(action, args=None):
    return dict(v=1, id=str(uuid.uuid4()), issued_at=time.time(), ttl_s=30, action=action, args=args or {})


def schema(robot_id, domain, bucket):
    from .contracts import catalog

    return {
        'v': 1,
        'robot_id': robot_id,
        'domain': domain,
        'bucket': bucket,
        'contract': catalog(robot_id),
        'request_subject': f'robots.{robot_id}.v1.request',
        'state_subject': f'robots.{robot_id}.v1.state',
        'actions': {
            'costmap.get': {
                'part': 'state | metadata | chunk',
                'kind': 'local | global, required for metadata/chunk',
                'contract': 'COSTMAP_API_V1.md',
                'read_only': True,
            },
            'map.save': {'name': '1..64 ASCII letters/digits/_/-', 'contract': 'MAP_API_V1.md'},
            'map.delete': {'name': '1..64 ASCII letters/digits/_/-', 'contract': 'MAP_API_V1.md'},
            'map.get': {
                'part': 'metadata | chunk | pose | model | scan | diagnostics | save_status | saved | saved_metadata | saved_chunk',
                'name': 'saved map name, required for saved_metadata/saved_chunk',
                'contract': 'MAP_API_V1.md',
            },
            'battery.history': {
                'from': 'Unix seconds, optional',
                'to': 'Unix seconds, optional',
                'max_points': '20..360, default 240',
            },
            'ping': {},
            'schema.get': {},
            'state.get': {},
            'metrics.get': {},
            'settings.set': {'note': 'string <= 500 characters'},
            'mode.set': {
                'mode': 'idle | mapping | localization | navigation',
                'map': 'allowlisted or saved map id, localization/navigation only',
                'discard': 'boolean; required true when leaving mapping',
            },
            **{action: {'contract': 'LOCALIZATION_API_V1.md'} for action in SPATIAL_COMMANDS},
        },
        'kv_keys': {
            'schema': 'API catalog',
            'state': 'robot snapshot; check updated_at and source age_s',
            'settings': 'operator note; does not change robot behavior',
            'battery': 'BMS BLE snapshot; check stale, sample_age_s and heartbeat_age_s',
            'costmap.local': 'Local Nav2 costmap metadata only; COSTMAP_API_V1.md',
            'costmap.global': 'Global Nav2 costmap metadata only; COSTMAP_API_V1.md',
        },
        'metrics': {
            'imu.gyro_z': {'unit': 'rad/s', 'source': '/imu/data'},
            'imu.accel_norm': {
                'unit': 'm/s^2',
                'source': '/imu/data',
                'caveat': 'scale discrepancy under investigation',
            },
            'ultrasonics.meters': {
                'unit': 'm',
                'source': '/ultrasonics',
                'timeout': 10.0,
                'order': [
                    'right',
                    'front_right_outer',
                    'front_right_inner',
                    'front_center',
                    'front_left_inner',
                    'front_left_outer',
                    'left',
                ],
                'caveat': 'order from firmware, not physically verified; array publishes after each individual channel update',
            },
            'odometry': {'source': '/odometry/filtered', 'units': {'x': 'm', 'y': 'm', 'vx': 'm/s', 'wz': 'rad/s'}},
            'localization': {'source': '/amcl_pose', 'caveat': 'covariance does not prove correct physical location'},
            'imu_status': {'source': '/imu/status'},
            'lidar': {'source': '/scan'},
            'battery': {
                'source': f'robots.{robot_id}.v1.bms',
                'transport': 'BLE',
                'protocol': 'JK02_32S',
                'units': {
                    'voltage_V': 'V',
                    'current_A': 'A; positive=charging per protocol',
                    'soc_pct': '%',
                    'cells_V': 'V',
                    'cell_delta_V': 'V',
                    'mos_temperature_C': 'C',
                },
                'stale_after_s': 8,
                'flags_raw': 'uninterpreted for firmware 19.31',
                'capacity_fields': 'preliminary; compare with current JK app',
            },
        },
        'envelope_example': envelope('state.get'),
        'limits': {
            'max_request_bytes': 16384,
            'max_ttl_s': 30,
            'movement_api': 'manual API available; runtime manual_control.enabled is authoritative',
        },
        'semantics': 'Core request/reply; commands are not queued offline; duplicate ids return cached results. Persisted desired mode is never replayed.',
    }
