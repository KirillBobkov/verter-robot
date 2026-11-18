"""Preview and remove saved occupancy grids without changing ROS state."""

from array import array
from collections import OrderedDict
import hashlib
from itertools import islice
import logging
import math
import os
from pathlib import Path
import re
import stat
import struct
import tempfile
import threading
import time

import yaml

from .map_data import MAX_CELLS, MapStore
from .map_save import NAME, SUFFIXES


YAML_LIMIT = 256 * 1024
PGM_LIMIT = MAX_CELLS * 6 + 65536
PGM_TOKEN = re.compile(rb'#[^\r\n]*|[^\s#]+')


def map_name(value):
    if not isinstance(value, str) or not NAME.fullmatch(value):
        raise ValueError('map_name_must_be_1_to_64_ascii_letters_digits_underscore_dash')
    return value


def regular_file(path):
    entry = path.lstat()
    if not stat.S_ISREG(entry.st_mode):
        raise ValueError('saved_map_requires_regular_files')
    return entry


def read_file(path, limit):
    def open_regular(name, flags):
        return os.open(name, flags | getattr(os, 'O_NOFOLLOW', 0) | getattr(os, 'O_NONBLOCK', 0))

    regular_file(path)
    with open(path, 'rb', opener=open_regular) as stream:
        entry = os.fstat(stream.fileno())
        if not stat.S_ISREG(entry.st_mode):
            raise ValueError('saved_map_requires_regular_files')
        if entry.st_size > limit:
            raise ValueError('saved_map_file_too_large')
        data = stream.read(limit + 1)
    if len(data) > limit:
        raise ValueError('saved_map_file_too_large')
    return data


def read_yaml(path):
    try:
        metadata = yaml.safe_load(read_file(path, YAML_LIMIT))
    except (yaml.YAMLError, RecursionError) as exc:
        raise ValueError('invalid_map_yaml') from exc
    if not isinstance(metadata, dict):
        raise ValueError('invalid_map_yaml')
    return metadata


def image_reference(path, metadata):
    name = metadata.get('image')
    if not isinstance(name, str) or not name or '\x00' in name:
        raise ValueError('invalid_map_image')
    return path.parent / name


def local_image(path, metadata):
    image = image_reference(path, metadata)
    if image.parent != path.parent:
        raise ValueError('invalid_or_external_map_image')
    return image


def file_version(entry):
    return entry.st_ino, entry.st_size, entry.st_mtime_ns, entry.st_ctime_ns


def pgm_grid(data, metadata):
    tokens = (match for match in PGM_TOKEN.finditer(data) if not match.group().startswith(b'#'))
    try:
        magic, width, height, maximum = (next(tokens) for _ in range(4))
        width, height, max_value = (int(token.group()) for token in (width, height, maximum))
    except (ValueError, RuntimeError) as exc:
        raise ValueError('invalid_saved_map_pgm') from exc
    size = width * height
    if width <= 0 or height <= 0 or size > MAX_CELLS or not 0 < max_value <= 65535:
        raise ValueError('invalid_saved_map_pgm_dimensions_or_depth')
    if magic.group() == b'P5':
        start = maximum.end()
        separator = data[start:start + 1]
        bytes_per_pixel = 1 if max_value < 256 else 2
        if not separator or not separator.isspace():
            raise ValueError('invalid_saved_map_pgm')
        start += 1
        if separator == b'\r' and data[start:start + 1] == b'\n' and len(data) - start == size * bytes_per_pixel + 1:
            start += 1
        if len(data) - start != size * bytes_per_pixel:
            raise ValueError('invalid_saved_map_pgm_length')
        pixels = data[start:]
        if bytes_per_pixel == 2:
            pixels = (pixel for pixel, in struct.iter_unpack('>H', pixels))
    elif magic.group() == b'P2':
        pixels = (int(token.group()) for token in tokens)
    else:
        raise ValueError('unsupported_saved_map_image_use_pgm')

    mode = metadata.get('mode', 'trinary')
    negate = metadata.get('negate')
    occupied, free = metadata.get('occupied_thresh'), metadata.get('free_thresh')
    if (
        mode not in ('trinary', 'scale', 'raw')
        or type(negate) not in (int, bool)
        or any(type(value) not in (float, int) for value in (free, occupied))
        or not 0 <= free < occupied <= 1
    ):
        raise ValueError('invalid_saved_map_mode_or_thresholds')
    lookup = bytearray(max_value + 1)
    for pixel in range(max_value + 1):
        shade = pixel / max_value
        occupancy = shade if negate else 1 - shade
        if mode == 'raw':
            value = int(shade * 255 + 0.5)
            lookup[pixel] = value if value <= 100 else 255
        elif occupancy > occupied:
            lookup[pixel] = 100
        elif occupancy < free:
            lookup[pixel] = 0
        elif mode == 'scale':
            lookup[pixel] = round((occupancy - free) / (occupied - free) * 100)
        else:
            lookup[pixel] = 255
    if mode == 'trinary' and not negate and max_value == 255 and isinstance(pixels, bytes):
        # The Humble saver uses 205 for unknown cells even when the YAML's free threshold
        # would classify them as free. Preserve the saved palette for preview.
        if set(pixels) <= {0, 205, 254, 255}:
            lookup[205] = 255
    if isinstance(pixels, bytes) and max_value == 255:
        translated = pixels.translate(lookup)
    else:
        try:
            translated = bytearray()
            for pixel in islice(pixels, size + 1):
                if not 0 <= pixel <= max_value:
                    raise ValueError('invalid_saved_map_pgm_pixel')
                translated.append(lookup[pixel])
        except (ValueError, IndexError) as exc:
            raise ValueError('invalid_saved_map_pgm_pixel') from exc
    if len(translated) != size:
        raise ValueError('invalid_saved_map_pgm_length')
    # PGM starts at the top left; ROS's row zero is at the bottom left.
    if width == 1:
        cells = translated[::-1]
    else:
        cells = bytearray(size)
        for row in range(height):
            start = row * width
            end = size - start
            cells[start:start + width] = translated[end - width:end]
    return width, height, array('b', bytes(cells))


class SavedMaps:
    def __init__(self, config, directory):
        self.config = config
        self.directory = directory
        self.lock = threading.Lock()
        self.cache = OrderedDict()

    def root(self):
        if self.directory.is_symlink() or self.directory.parent.is_symlink():
            raise ValueError('saved_map_directory_is_symlink')
        return self.directory.resolve()

    def configured_paths(self):
        return {Path(path).resolve() for path in self.config.get('maps', {}).values() if path}

    def list(self, mode, defined_maps):
        root = self.root()
        configured = self.configured_paths()
        active = Path(mode['map_path']).resolve() if mode.get('map_path') else None
        rows = []
        with self.lock:
            for path in sorted(root.glob('*.yaml')):
                name = path.stem
                if not NAME.fullmatch(name) or path.is_symlink() or not path.is_file():
                    continue
                try:
                    image = local_image(path, read_yaml(path))
                    grid = image.suffix.lower() == '.pgm' and bool(regular_file(image))
                except (OSError, ValueError):
                    grid = False
                graph = (path.with_suffix('.posegraph'), path.with_suffix('.data'))
                rows.append({
                    'name': name,
                    'grid': grid,
                    'pose_graph': all(item.is_file() and not item.is_symlink() for item in graph),
                    'active': active == path,
                    'configured': path in configured,
                    'has_definitions': name in defined_maps,
                })
        return {'maps': rows}

    def query(self, args):
        part = args['part']
        expected = {'part', 'name'}
        if part == 'saved_chunk':
            expected |= {'map_id', 'revision', 'index'}
        if set(args) != expected:
            raise ValueError('unexpected_saved_map_args')
        name = map_name(args['name'])
        with self.lock:
            try:
                store = self.snapshot(name)
            except FileNotFoundError as exc:
                raise ValueError('saved_map_not_found') from exc
            if part == 'saved_metadata':
                return store.metadata()
            return store.query({
                'part': 'chunk',
                'map_id': args['map_id'],
                'revision': args['revision'],
                'index': args['index'],
            })

    def snapshot(self, name):
        path = self.root() / (name + '.yaml')
        yaml_info = regular_file(path)
        metadata = read_yaml(path)
        image = local_image(path, metadata)
        if image.suffix.lower() != '.pgm':
            raise ValueError('unsupported_saved_map_image_use_pgm')
        image_info = regular_file(image)
        signature = file_version(yaml_info), file_version(image_info)
        key = (name, image.name, signature)
        if key in self.cache:
            self.cache.move_to_end(key)
            return self.cache[key]
        resolution = metadata.get('resolution')
        origin = metadata.get('origin')
        if type(resolution) not in (float, int) or not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid_map_resolution')
        if not isinstance(origin, list) or len(origin) != 3 or any(
            type(value) not in (int, float) or not math.isfinite(value) for value in origin
        ):
            raise ValueError('invalid_map_origin')
        data = read_file(image, PGM_LIMIT)
        width, height, cells = pgm_grid(data, metadata)
        if signature != (file_version(regular_file(path)), file_version(regular_file(image))):
            raise ValueError('saved_map_changed_refetch_metadata')
        version = hashlib.sha256(repr(key).encode()).hexdigest()
        map_id = f'saved:{name}:{version}'
        store = MapStore(self.config['robot_id'])
        store.set_scope(map_id, {'actual': 'saved', 'stale': False, 'map': name})
        origin = dict(zip(('x', 'y', 'yaw'), origin))
        store.put_grid(map_id, width, height, resolution, origin, cells, max(yaml_info.st_mtime, image_info.st_mtime))
        self.cache[key] = store
        while len(self.cache) > 4:
            self.cache.popitem(last=False)
        return store

    def delete(self, name, mode, defined_maps, expires_at):
        name = map_name(name)
        root = self.root()
        path = root / (name + '.yaml')
        with self.lock:
            if expires_at is not None and time.time() > expires_at:
                raise ValueError('expired')
            configured = self.configured_paths()
            if path in configured:
                raise ValueError('map_configured')
            current = mode.get('map_path')
            if current and Path(current).resolve() == path:
                raise ValueError('map_in_use')
            if name in defined_maps:
                raise ValueError('map_has_waypoints_or_routes')
            try:
                metadata = read_yaml(path)
            except FileNotFoundError as exc:
                raise ValueError('saved_map_not_found') from exc
            owned_image = local_image(path, metadata)
            if owned_image.stem != name or owned_image.suffix.lower() != '.pgm':
                owned_image = None
            if owned_image:
                related_paths = set(root.glob('*.yaml')) | configured
                if current:
                    related_paths.add(Path(current).resolve())
                for other in related_paths - {path}:
                    try:
                        reference = image_reference(other, read_yaml(other))
                    except (OSError, ValueError):
                        continue
                    if reference.resolve() == owned_image:
                        raise ValueError('map_image_in_use')
            candidates = [path.with_suffix(suffix) for suffix in SUFFIXES[2:]]
            if owned_image:
                candidates.insert(0, owned_image)
            targets = [path]
            for target in candidates:
                try:
                    regular_file(target)
                except FileNotFoundError:
                    continue
                targets.append(target)
            if expires_at is not None and time.time() > expires_at:
                raise ValueError('expired')
            self.remove_files(root, targets)
            self.cache.clear()
            return {'deleted': name}

    @staticmethod
    def remove_files(root, targets):
        staging = Path(tempfile.mkdtemp(prefix='.map-delete-', dir=root))
        moved = []
        try:
            for path in targets:
                staged = staging / path.name
                path.rename(staged)
                moved.append((path, staged))
        except OSError as exc:
            try:
                for path, staged in reversed(moved):
                    staged.rename(path)
                staging.rmdir()
            except OSError as rollback_error:
                logging.exception('Could not restore saved map after delete failure in %s', staging)
                raise ValueError('map_delete_rollback_failed_inspect_files') from rollback_error
            raise ValueError('map_delete_failed_files_restored') from exc
        # The map has been removed from the library. Cleanup can't expose a partial map.
        try:
            for _, staged in moved:
                staged.unlink()
            staging.rmdir()
        except OSError:
            logging.exception('Could not remove staged files for deleted map in %s', staging)
