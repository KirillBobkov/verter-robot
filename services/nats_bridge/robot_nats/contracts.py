"""Public subject/KV contract. Pure projection of observations; no control code."""

import base64
import math
from .places import SPATIAL_COMMANDS

REVISION = '1.8'
COMMANDS = {
    'ping': 'ping',
    'schema_get': 'schema.get',
    'state_get': 'state.get',
    'metrics_get': 'metrics.get',
    'note_set': 'settings.set',
    'mode_set': 'mode.set',
}
# name: (decoder field, JSON value type, unit)
BATTERY = {
    'soc': ('soc_pct', 'number', '%'),
    'voltage': ('voltage_V', 'number', 'V'),
    'current': ('current_A', 'number', 'A'),
    'power': ('power_W', 'number', 'W'),
    'remaining_capacity': ('remaining_Ah', 'number', 'Ah'),
    'full_charge_capacity': ('full_charge_capacity_Ah', 'number', 'Ah'),
    'cycles': ('cycles', 'integer', None),
    'temperature_mos': ('mos_temperature_C', 'number', 'degC'),
    'temperature_1': ('temperature_1_C', 'number', 'degC'),
    'temperature_2': ('temperature_2_C', 'number', 'degC'),
    'cell_sum': ('cell_sum_V', 'number', 'V'),
    'cell_delta': ('cell_delta_V', 'number', 'V'),
    'balance_current': ('balance_current_A', 'number', 'A'),
    'balance_state': ('balance_state', 'integer', None),
    'charging_enabled': ('charging_enabled', 'boolean', None),
    'discharging_enabled': ('discharging_enabled', 'boolean', None),
    'flags_raw': ('flags_raw', 'string', None),
}
TOPICS = {
    'imu_gyro_z': ('imu', 'gyro_z', 'rad/s'),
    'imu_accel_norm': ('imu', 'accel_norm', 'm/s^2'),
    'scan_points': ('lidar', 'points', None),
    'scan_valid_points': ('lidar', 'valid_points', None),
    'scan_frame': ('lidar', 'frame', None),
    'odometry_x': ('odometry', 'x', 'm'),
    'odometry_y': ('odometry', 'y', 'm'),
    'odometry_vx': ('odometry', 'vx', 'm/s'),
    'odometry_wz': ('odometry', 'wz', 'rad/s'),
    'localization_x': ('localization', 'x', 'm'),
    'localization_y': ('localization', 'y', 'm'),
    'localization_covariance': ('localization', 'covariance', None),
    'map_width': ('map', 'width', 'cell'),
    'map_height': ('map', 'height', 'cell'),
    'map_resolution': ('map', 'resolution', 'm/cell'),
    'map_to_odom_x': ('map_to_odom', 'x', 'm'),
    'map_to_odom_y': ('map_to_odom', 'y', 'm'),
    'cmd_vel_publishers': ('graph', 'cmd_vel_publishers', None),
}
ULTRASONICS = [
    'right',
    'front_right_outer',
    'front_right_inner',
    'front_center',
    'front_left_inner',
    'front_left_outer',
    'left',
]


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def charging_state(battery):
    current = (battery.get('sample') or {}).get('current_A')
    if battery.get('stale', True) or not finite(current):
        return 'unknown'
    return 'charging' if current > 0.1 else 'discharging' if current < -0.1 else 'idle'


def observation(value, unit, sample_at, sample_id, now, max_age_s=8, stale=False):
    # Null is not a zero measurement. A saved record never refreshes its own age.
    quality = (
        'unavailable'
        if value is None or sample_at is None
        else 'stale'
        if stale or now - sample_at > max_age_s
        else 'ok'
    )
    return {
        'v': 1,
        'value': value,
        'unit': unit,
        'sample_at': sample_at,
        'updated_at': now,
        'sample_id': sample_id,
        'quality': quality,
        'max_age_s': max_age_s,
    }


def node_id(name):
    return base64.urlsafe_b64encode(name.encode()).decode().rstrip('=')


def project(state):
    """One consistent local capture -> a dictionary of independently published values."""
    now, boot = state['updated_at'], state['boot_id']
    out = {}

    def put(key, value, unit=None, sample_at=now, sample_id=None, max_age_s=8, stale=False):
        out[key] = observation(value, unit, sample_at, sample_id or f'{boot}:{sample_at}', now, max_age_s, stale)

    b = state['battery']
    sample = b.get('sample') or {}
    battery_time = b.get('sample_at')
    battery_id = b.get('sample_id') or (f'{b.get("boot_id")}:{battery_time}' if battery_time else None)

    def battery_value(name, value, unit=None):
        put('battery.' + name, value, unit, battery_time, battery_id, stale=b.get('stale', True))

    for name, (field, _, unit) in BATTERY.items():
        battery_value(name, sample.get(field), unit)
    battery_value('charging_state', charging_state(b))
    for cell, value in sample.get('cells_V', {}).items():
        battery_value(f'cell_{int(cell):02d}_voltage', value, 'V')
    # Snapshot quality describes sample freshness; its nested status remains useful when stale.
    battery_value('snapshot', b)
    put(
        'service.bms',
        {
            k: b.get(k)
            for k in (
                'status',
                'connected',
                'error',
                'device',
                'heartbeat_age_s',
                'sample_age_s',
                'rssi_at_connect_dbm',
                'diagnostics',
            )
        },
    )
    put('service.bridge', {'boot_id': boot, 'uptime_s': state['uptime_s'], 'ros_error': state['telemetry']['error']})
    put(
        'service.mode',
        state['mode'],
        sample_at=state['mode'].get('updated_at', now),
        stale=state['mode'].get('stale', False),
    )
    if 'manual_control' in state:
        put('cmd_vel.target', state['manual_control'])
    if 'localization' in state:
        put('localization.state', state['localization'])
    if 'places' in state:
        for name in ('waypoint', 'route'):
            plural = 'waypoints' if name == 'waypoint' else 'routes'
            put(name + '.catalog', {
                key: state['places'].get(key) for key in ('map', 'map_revision', plural, 'error')
            })
    put('service.transport_connected', state['transport']['connected'])
    put('service.kv_error', state['transport']['kv_error'])
    put('service.note', state['settings']['note'])
    put('service.snapshot', state)
    sources = state['telemetry']['sources']
    for name, (source, field, unit) in TOPICS.items():
        record = sources.get(source, {})
        stamp = record.get('received_at')
        put(
            'topic.' + name,
            record.get('value', {}).get(field),
            unit,
            stamp,
            f'{boot}:{source}:{stamp}',
            5,
            record.get('stale', True),
        )
    us = sources.get('ultrasonics', {})
    distances = us.get('value', {}).get('meters', [])
    for index, name in enumerate(ULTRASONICS):
        value = distances[index] if index < len(distances) else None
        # The firmware's 10 m sentinel means no usable echo, not a measured distance.
        if value == 10.0:
            value = None
        put(
            'topic.ultrasonic_' + name,
            value,
            'm',
            us.get('received_at'),
            f'{boot}:ultrasonics:{us.get("received_at")}',
            5,
            us.get('stale', True),
        )
    for source in ('imu_status', 'graph'):
        record = sources.get(source, {})
        put(
            'topic.' + source,
            record.get('value'),
            sample_at=record.get('received_at'),
            max_age_s=5,
            stale=record.get('stale', True),
        )
    graph = sources.get('graph', {})
    names = graph.get('value', {}).get('full_node_names', graph.get('value', {}).get('nodes'))
    put(
        'node.list',
        sorted(set(names)) if names is not None else None,
        sample_at=graph.get('received_at'),
        max_age_s=5,
        stale=graph.get('stale', True),
    )
    for name in sorted(set(names or [])):
        put(
            'node.' + node_id(name),
            {'name': name, 'present': True},
            sample_at=graph.get('received_at'),
            max_age_s=5,
            stale=graph.get('stale', True),
        )
    return out


def catalog(robot_id):
    metrics = {}

    def add(key, kind, unit=None, max_age_s=8, **extra):
        domain, name = key.split('.', 1)
        metrics[key] = {
            'subject': f'{robot_id}.{domain}.update.{name}',
            'kv_key': key,
            'value_type': kind,
            'nullable': True,
            'unit': unit,
            'max_age_s': max_age_s,
            **extra,
        }

    for name, (_, kind, unit) in BATTERY.items():
        add('battery.' + name, kind, unit)
    add('battery.charging_state', 'string', enum=['charging', 'discharging', 'idle', 'unknown'])
    add('battery.cell_{01..32}_voltage', 'number', 'V', dynamic=True)
    add('battery.snapshot', 'object')
    for name in ('bms', 'bridge', 'mode', 'snapshot', 'schema'):
        add('service.' + name, 'object')
    add('cmd_vel.target', 'object')
    add('localization.state', 'object')
    add('waypoint.catalog', 'object')
    add('route.catalog', 'object')
    add('service.transport_connected', 'boolean')
    add('service.kv_error', 'string')
    add('service.note', 'string')
    for name, (_, field, unit) in TOPICS.items():
        kind = 'array' if field in ('covariance', 'cmd_vel_publishers') else 'string' if field == 'frame' else 'number'
        add('topic.' + name, kind, unit, 5)
    for name in ULTRASONICS:
        add('topic.ultrasonic_' + name, 'number', 'm', 5)
    for name in ('imu_status', 'graph'):
        add('topic.' + name, 'object', max_age_s=5)
    add('node.list', 'array', max_age_s=5)
    add('node.{base64url_full_ros_name}', 'object', max_age_s=5, dynamic=True)
    return {
        'revision': REVISION,
        'subject_pattern': '<robot_id>.<domain>.<action>.<name>',
        'encoding': 'UTF-8 JSON',
        'publish_interval_s': 2,
        'metrics': metrics,
        'commands': {
            **{f'{robot_id}.service.command.{name}': action for name, action in COMMANDS.items()},
            **{f'{robot_id}.{domain}.command.{operation}': action
               for action, (domain, operation) in SPATIAL_COMMANDS.items()},
        },
        'battery_history': {
            'subject': f'{robot_id}.battery.command.history',
            'action': 'battery.history',
            'read_only': True,
            'retention_s': 604800,
            'sample_interval_s': 10,
            'export_http': 'GET /api/battery/export?format=csv|txt',
            'raw_pages': {'limit_max': 250, 'page_bytes_max': 524288, 'cursor': 'after'},
            'contract': 'BMS_HISTORY_V1.md',
        },
        'mode_control': {
            'owner': 'supervisor',
            'modes': ['idle', 'mapping', 'localization', 'navigation'],
            'subject': f'{robot_id}.service.command.mode_set',
            'acceptance_is_not_completion': True,
            'contract': 'NATS_MODES_V1.md',
        },
        'map_save': {
            'subject': f'{robot_id}.map.command.save',
            'action': 'map.save',
            'read_only': False,
            'acceptance_is_not_completion': True,
            'contract': 'MAP_API_V1.md',
        },
        'map_delete': {
            'subject': f'{robot_id}.map.command.delete',
            'action': 'map.delete',
            'read_only': False,
            'contract': 'MAP_API_V1.md',
        },
        'localization_and_places': {
            'contract': 'LOCALIZATION_API_V1.md',
            'route_execution_implemented': False,
            'initial_pose_is_explicit': True,
            'verified_requires_operator_confirmation': True,
        },
        'live_map': {
            'subject': f'{robot_id}.map.command.get',
            'action': 'map.get',
            'read_only': True,
            'parts': [
                'metadata', 'chunk', 'pose', 'model', 'scan', 'diagnostics', 'save_status', 'saved',
                'saved_metadata', 'saved_chunk',
            ],
            'updates': {
                'metadata': f'{robot_id}.map.update.metadata',
                'pose': f'{robot_id}.map.update.pose',
                'scan': f'{robot_id}.map.update.scan',
            },
            'kv_key': 'map.live',
            'encoding': 'zlib-int8-row-major',
            'max_cells': 4194304,
            'chunk_bytes': 49152,
            'contract': 'MAP_API_V1.md',
        },
        'command_envelope': ['v', 'id', 'issued_at', 'ttl_s', 'args'],
        'costmap': {
            'subject': f'{robot_id}.costmap.command.get',
            'action': 'costmap.get',
            'read_only': True,
            'parts': ['state', 'metadata', 'chunk'],
            'kinds': ['local', 'global'],
            'updates': {kind: f'{robot_id}.costmap.update.{kind}' for kind in ('local', 'global')},
            'kv_keys': ['costmap.local', 'costmap.global'],
            'encoding': 'zlib-uint8-row-major',
            'max_cells': 4194304,
            'chunk_bytes': 49152,
            'contract': 'COSTMAP_API_V1.md',
            'sensor_attribution': False,
        },
        'quality': ['ok', 'stale', 'unavailable'],
        'cmd_vel': {
            'implemented': True,
            'enabled_by_default': False,
            'subjects': {op: f'{robot_id}.cmd_vel.command.{op}' for op in ('set', 'keepalive', 'stop', 'status')},
            'contract': 'NATS_MANUAL_CONTROL_V1.md',
            'lease_s': 1,
            'max_linear': 0.5,
            'max_angular': 0.5,
        },
        'compatibility': {
            'legacy_request': f'robots.{robot_id}.v1.request',
            'legacy_state': f'robots.{robot_id}.v1.state',
        },
    }
