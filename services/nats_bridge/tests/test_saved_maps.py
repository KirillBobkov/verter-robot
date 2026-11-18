import base64
import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
import zlib

from robot_nats.map_data import MAX_CELLS
from robot_nats.saved_maps import SavedMaps


class SavedMapsTests(unittest.TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name) / 'maps'
        self.root.mkdir()
        self.config = {'robot_id': 'test', 'maps': {}}
        self.maps = SavedMaps(self.config, self.root)

    def write_map(self, name, image=None, pgm=None):
        path = self.root / f'{name}.yaml'
        if image is None:
            image = f'{name}.pgm'
            pgm = pgm or b'P5\n# Nav2 map\n2 2\n255\n\x00\xfe\x78\x0a'
            (self.root / image).write_bytes(pgm)
        path.write_text(
            f'image: {image}\n'
            'resolution: 0.03\n'
            'origin: [2.5, -1.0, 0.75]\n'
            'negate: 0\n'
            'free_thresh: 0.25\n'
            'occupied_thresh: 0.65\n'
        )
        return path

    def preview(self, name):
        return self.maps.query({'part': 'saved_metadata', 'name': name})

    def chunk(self, name, metadata, index=0):
        return self.maps.query({
            'part': 'saved_chunk',
            'name': name,
            'map_id': metadata['map_id'],
            'revision': metadata['revision'],
            'index': index,
        })

    def test_preview_keeps_yaml_geometry_and_flips_pgm_to_ros_order(self):
        self.write_map('room')
        metadata = self.preview('room')
        self.assertTrue(metadata['available'])
        self.assertEqual(metadata['saved_map'], 'room')
        self.assertEqual(metadata['mode'], 'saved')
        self.assertEqual(metadata['origin'], {'x': 2.5, 'y': -1.0, 'yaw': 0.75})
        self.assertEqual(metadata['resolution'], 0.03)
        self.assertEqual((metadata['width'], metadata['height']), (2, 2))
        self.assertEqual(metadata['encoding'], 'zlib-int8-row-major')
        chunk = self.chunk('room', metadata)
        packed = base64.b64decode(chunk['data_b64'])
        self.assertEqual(hashlib.sha256(packed).hexdigest(), metadata['sha256'])
        self.assertEqual(zlib.decompress(packed), bytes([255, 100, 100, 0]))

    def test_binary_pixel_whitespace_and_ascii_format(self):
        self.write_map('binary', pgm=b'P5\n1 2\n255\n\x0a\xfe')
        self.write_map('ascii', pgm=b'P2\n1 2\n255\n10 # pixel 1\n254')
        for name in ('binary', 'ascii'):
            metadata = self.preview(name)
            chunk = self.chunk(name, metadata)
            self.assertEqual(zlib.decompress(base64.b64decode(chunk['data_b64'])), bytes([0, 100]))

    def test_nav2_trinary_palette_keeps_205_unknown_despite_yaml_threshold(self):
        self.write_map('saved', pgm=b'P5\n3 1\n255\n\x00\xcd\xfe')
        self.write_map('custom', pgm=b'P5\n3 1\n255\n\x00\xcd\xa0')
        for name, expected in (('saved', [100, 255, 0]), ('custom', [100, 0, 255])):
            chunk = self.chunk(name, self.preview(name))
            self.assertEqual(zlib.decompress(base64.b64decode(chunk['data_b64'])), bytes(expected))

    def test_invalid_path_oversized_and_truncated_pgm(self):
        self.write_map('huge', pgm=f'P5\n{MAX_CELLS + 1} 1\n255\n'.encode())
        self.write_map('short', pgm=b'P5\n1 2\n255\n\x0a')
        self.write_map('external', image='../something.pgm')
        for name, error in (
            ('../bad', 'map_name_must_be'),
            ('huge', 'invalid_saved_map_pgm_dimensions'),
            ('short', 'invalid_saved_map_pgm_length'),
            ('external', 'invalid_or_external_map_image'),
        ):
            with self.subTest(name=name), self.assertRaisesRegex(ValueError, error):
                self.preview(name)

    def test_preview_and_delete_reject_symlink(self):
        external = self.root.parent / 'external.pgm'
        external.write_bytes(b'P5\n1 1\n255\n\x00')
        try:
            (self.root / 'link.pgm').symlink_to(external)
        except OSError:
            self.skipTest('Symlink creation is unavailable')
        self.write_map('link', image='link.pgm')
        with self.assertRaisesRegex(ValueError, 'saved_map_requires_regular_files'):
            self.preview('link')
        with self.assertRaisesRegex(ValueError, 'saved_map_requires_regular_files'):
            self.maps.delete('link', {}, set(), None)
        self.assertTrue(external.is_file())

    def test_replaced_or_deleted_map_rejects_inactive_chunks(self):
        self.write_map('room')
        old_version = self.preview('room')
        self.write_map('room', pgm=b'P5\n1 1\n255\n\x00')
        with self.assertRaisesRegex(ValueError, 'map_revision_unavailable'):
            self.chunk('room', old_version)
        version = self.preview('room')
        self.maps.delete('room', {}, set(), None)
        with self.assertRaisesRegex(ValueError, 'saved_map_not_found'):
            self.chunk('room', version)

    def test_deletion_guards_active_configured_defined_and_shared_maps(self):
        path = self.write_map('room')
        mode = {'map_path': str(path)}
        with self.assertRaisesRegex(ValueError, 'map_in_use'):
            self.maps.delete('room', mode, set(), None)
        self.config['maps']['room_alias'] = str(path)
        with self.assertRaisesRegex(ValueError, 'map_configured'):
            self.maps.delete('room', {}, set(), None)
        self.config['maps'].clear()
        with self.assertRaisesRegex(ValueError, 'map_has_waypoints_or_routes'):
            self.maps.delete('room', {}, {'room'}, None)
        self.write_map('shared', image='room.pgm')
        with self.assertRaisesRegex(ValueError, 'map_image_in_use'):
            self.maps.delete('room', {}, set(), None)
        self.assertEqual(self.maps.delete('shared', {}, set(), None), {'deleted': 'shared'})
        self.assertTrue(path.is_file())
        self.assertTrue(path.with_suffix('.pgm').is_file())

    def test_deletion_rollback_and_cleanup(self):
        path = self.write_map('room')
        for suffix in ('.posegraph', '.data', '.manifest.json'):
            path.with_suffix(suffix).write_bytes(b'file')
        original_names = sorted(item.name for item in self.root.iterdir())
        original_rename = Path.rename

        def fail_on_data(source, target):
            if source.parent == self.root and source.suffix == '.data':
                raise PermissionError('test')
            return original_rename(source, target)

        with patch.object(Path, 'rename', new=fail_on_data):
            with self.assertRaisesRegex(ValueError, 'map_delete_failed_files_restored'):
                self.maps.delete('room', {}, set(), None)
        self.assertEqual(sorted(item.name for item in self.root.iterdir()), original_names)
        self.assertEqual(self.maps.delete('room', {}, set(), None), {'deleted': 'room'})
        self.assertEqual(list(self.root.iterdir()), [])


if __name__ == '__main__':
    unittest.main()
