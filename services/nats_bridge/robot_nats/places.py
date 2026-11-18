"""Map-bound point and route definitions. This module never commands a robot."""

import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import time


IDENTIFIER = re.compile(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z')
SPATIAL_COMMANDS = {
    'localization.get': ('localization', 'get'),
    'localization.initialize': ('localization', 'initialize'),
    'localization.confirm': ('localization', 'confirm'),
    'waypoint.list': ('waypoint', 'list'),
    'waypoint.save': ('waypoint', 'save'),
    'waypoint.delete': ('waypoint', 'delete'),
    'route.list': ('route', 'list'),
    'route.save': ('route', 'save'),
    'route.delete': ('route', 'delete'),
}


def resolve_map(config, name):
    if not isinstance(name, str):
        raise ValueError('map_name_required')
    explicit = config.get('maps', {}).get(name)
    if explicit:
        path = Path(explicit).resolve()
    else:
        if not IDENTIFIER.fullmatch(name):
            raise ValueError('invalid_map_name')
        root = Path(config.get('project_root', Path(__file__).resolve().parents[3])) / 'data/maps'
        path = (root / (name + '.yaml')).resolve()
        if path.parent != root.resolve():
            raise ValueError('map_outside_saved_directory')
    if not path.is_file():
        raise ValueError('unknown_or_missing_map')
    return path


def map_names(config):
    names = set(config.get('maps', {}))
    root = Path(config.get('project_root', Path(__file__).resolve().parents[3])) / 'data/maps'
    for path in root.glob('*.yaml'):
        if IDENTIFIER.fullmatch(path.stem) and path.resolve().parent == root.resolve():
            names.add(path.stem)
    return sorted(names)


class Places:
    def __init__(self, config, localization):
        self.config = config
        self.localization = localization
        path = Path(config['data_dir']) / 'places.sqlite'
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(path)
        self.db.execute(
            'CREATE TABLE IF NOT EXISTS definitions '
            '(map TEXT PRIMARY KEY, revision TEXT NOT NULL, body TEXT NOT NULL)'
        )
        self.db.commit()
        self.cache = {}

    def close(self):
        self.db.close()

    def defined_maps(self):
        names = set()
        for name, payload in self.db.execute('SELECT map,body FROM definitions'):
            definitions = json.loads(payload)
            if definitions['waypoints'] or definitions['routes']:
                names.add(name)
        return names

    def snapshot(self, name):
        if name is None:
            return {'map': None, 'map_revision': None, 'waypoints': [], 'routes': [], 'error': None}
        try:
            return {**self.read(name), 'error': None}
        except (OSError, ValueError, sqlite3.Error) as exc:
            return {'map': name, 'map_revision': None, 'waypoints': [], 'routes': [], 'error': str(exc)}

    def revision(self, name):
        import yaml

        path = resolve_map(self.config, name)
        if path.stat().st_size > 256 * 1024:
            raise ValueError('map_yaml_too_large')
        metadata = yaml.safe_load(path.read_text())
        if not isinstance(metadata, dict):
            raise ValueError('invalid_map_yaml')
        resolution = metadata.get('resolution')
        origin = metadata.get('origin')
        if type(resolution) not in (int, float) or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid_map_resolution')
        if not isinstance(origin, list) or len(origin) != 3 or any(
            type(value) not in (int, float) or not math.isfinite(value) for value in origin
        ):
            raise ValueError('invalid_map_origin')
        image_name = metadata.get('image')
        if not isinstance(image_name, str):
            raise ValueError('invalid_map_image')
        image = Path(image_name)
        if not image.is_absolute():
            image = path.parent / image
        image = image.resolve()
        if image.parent != path.parent or not image.is_file() or image.stat().st_size > 64 * 1024 * 1024:
            raise ValueError('invalid_or_external_map_image')
        stat = path.stat()
        image_stat = image.stat()
        key = (str(path), stat.st_mtime_ns, stat.st_size, str(image), image_stat.st_mtime_ns, image_stat.st_size)
        if key not in self.cache:
            semantic = {field: metadata.get(field) for field in (
                'resolution', 'origin', 'negate', 'mode', 'occupied_thresh', 'free_thresh'
            )}
            try:
                serialized = json.dumps(semantic, sort_keys=True, allow_nan=False).encode()
            except (TypeError, ValueError) as exc:
                raise ValueError('invalid_map_semantic_metadata') from exc
            digest = hashlib.sha256(serialized)
            with image.open('rb') as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b''):
                    digest.update(block)
            self.cache = {key: digest.hexdigest()}
        return self.cache[key]

    def read(self, name):
        revision = self.revision(name)
        row = self.db.execute('SELECT revision,body FROM definitions WHERE map=?', (name,)).fetchone()
        if row and row[0] != revision:
            raise ValueError('saved_map_changed_existing_points_require_review')
        document = json.loads(row[1]) if row else {'waypoints': [], 'routes': []}
        return {'map': name, 'map_revision': revision, **document}

    def write(self, document):
        payload = {'waypoints': document['waypoints'], 'routes': document['routes']}
        encoded = json.dumps(payload, ensure_ascii=False, allow_nan=False)
        if len(encoded.encode()) > 200 * 1024:
            raise ValueError('map_definitions_size_limit')
        with self.db:
            self.db.execute(
                'INSERT INTO definitions VALUES (?,?,?) '
                'ON CONFLICT(map) DO UPDATE SET revision=excluded.revision,body=excluded.body',
                (document['map'], document['map_revision'], encoded),
            )

    def execute(self, action, args):
        kind, operation = action.split('.')
        if kind not in ('waypoint', 'route') or operation not in ('list', 'save', 'delete'):
            raise ValueError('unknown_definition_operation')
        if not isinstance(args, dict):
            raise ValueError('invalid_definition_args')
        expected = {'map'} if operation == 'list' else {'map', 'id'}
        if operation == 'save':
            expected |= {'label', 'source', 'x', 'y', 'yaw'} if kind == 'waypoint' else {'label', 'kind', 'points'}
        if set(args) - expected or 'map' not in args:
            raise ValueError('unexpected_definition_args')
        document = self.read(args['map'])
        plural = 'waypoints' if kind == 'waypoint' else 'routes'
        if operation == 'list':
            return {'map': document['map'], 'map_revision': document['map_revision'], plural: document[plural]}
        ident = args.get('id')
        if not isinstance(ident, str) or not IDENTIFIER.fullmatch(ident):
            raise ValueError('invalid_definition_id')
        existing = next((row for row in document[plural] if row['id'] == ident), None)
        if operation == 'delete':
            if not existing:
                raise ValueError('definition_not_found')
            if kind == 'waypoint' and any(ident in route['points'] for route in document['routes']):
                raise ValueError('waypoint_is_used_by_route')
            document[plural] = [row for row in document[plural] if row['id'] != ident]
            self.write(document)
            return {'map': document['map'], 'map_revision': document['map_revision'], 'deleted': ident}
        label = args.get('label')
        if not isinstance(label, str) or not 1 <= len(label.strip()) <= 100:
            raise ValueError('label_must_be_1_to_100_characters')
        record = {'id': ident, 'label': label.strip(), 'updated_at': time.time()}
        if kind == 'waypoint':
            source = args.get('source', 'coordinates')
            if source == 'current':
                if set(args) != {'map', 'id', 'label', 'source'}:
                    raise ValueError('current_pose_does_not_accept_explicit_coordinates')
                pose = self.localization.current_pose()
                if pose['map'] != args['map']:
                    raise ValueError('localized_map_does_not_match_waypoint_map')
            elif source == 'coordinates':
                if not {'x', 'y', 'yaw'} <= set(args):
                    raise ValueError('waypoint_coordinates_required')
                pose = args
            else:
                raise ValueError('unknown_waypoint_source')
            for field in ('x', 'y', 'yaw'):
                value = pose[field]
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError('finite_waypoint_coordinates_required')
                if field != 'yaw' and abs(value) > 10000:
                    raise ValueError('waypoint_coordinate_out_of_range')
                record[field] = math.atan2(math.sin(value), math.cos(value)) if field == 'yaw' else value
            record.update(frame_id='map', source=source)
        else:
            if set(args) != {'map', 'id', 'label', 'kind', 'points'}:
                raise ValueError('invalid_route_args')
            points = args['points']
            if args['kind'] not in ('visit', 'through') or not isinstance(points, list) or not 1 <= len(points) <= 100:
                raise ValueError('invalid_route_kind_or_length')
            known = {row['id'] for row in document['waypoints']}
            if any(not isinstance(point, str) or point not in known for point in points):
                raise ValueError('route_contains_unknown_waypoint')
            record.update(kind=args['kind'], points=list(points))
        if not existing and len(document[plural]) >= 200:
            raise ValueError('definition_count_limit')
        document[plural] = [row for row in document[plural] if row['id'] != ident] + [record]
        self.write(document)
        return {'map': document['map'], 'map_revision': document['map_revision'], kind: record}
