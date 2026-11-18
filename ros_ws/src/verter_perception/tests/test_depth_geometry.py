"""Pure geometry checks: real projection, wire layouts and optical/base axes."""

import struct
import unittest

import cv2
import numpy as np

from verter_perception.depth_geometry import (
    decode_depth,
    filter_cloud,
    project_depth,
    quaternion_matrix,
    sampled_rays,
)


class DepthGeometryTests(unittest.TestCase):
    def test_millimetres_with_row_padding(self):
        wire = struct.pack('<HHHHHH', 1000, 2000, 65000, 3000, 4000, 65000)
        depth = decode_depth(wire, 2, 2, 6, '16UC1')
        np.testing.assert_allclose(depth, [[1, 2], [3, 4]])

    def test_big_endian_millimetres_and_float_metres(self):
        mm = decode_depth(struct.pack('>HH', 250, 4000), 2, 1, 4, '16UC1', True)
        metres = decode_depth(struct.pack('>ff', 0.5, 3.25), 2, 1, 8, '32FC1', True)
        np.testing.assert_allclose(mm, [[0.25, 4.0]])
        np.testing.assert_allclose(metres, [[0.5, 3.25]])

    def test_invalid_wire_layouts(self):
        for values in [
            (b'\x00' * 4, 3, 1, 4, '16UC1'),
            (b'\x00' * 3, 2, 1, 4, '16UC1'),
            (b'\x00' * 8, 2, 1, 4, '16UC1'),
            (b'\x00' * 4, 2, 1, 4, 'mono16'),
        ]:
            with self.assertRaises(ValueError):
                decode_depth(*values)

    def test_pinhole_projection_and_invalid_returns(self):
        intrinsic = [2, 0, 1, 0, 2, 1, 0, 0, 1]
        rows, columns, rays = sampled_rays(3, 2, intrinsic, [], 'plumb_bob', 1)
        depth = np.array([[0, np.nan, np.inf], [0.2, 2, 5]], dtype=np.float32)
        points = project_depth(depth, rows, columns, rays, 0.25, 4)
        np.testing.assert_allclose(points, [[0, 0, 2]])

    def test_stride_samples_pixels_without_resizing_intrinsics(self):
        intrinsic = [100, 0, 4, 0, 100, 4, 0, 0, 1]
        rows, columns, rays = sampled_rays(8, 8, intrinsic, [], 'plumb_bob', 4)
        np.testing.assert_array_equal(rows, [0, 4])
        np.testing.assert_array_equal(columns, [0, 4])
        np.testing.assert_allclose(rays[-1], [0, 0, 1], atol=1.0e-7)
        np.testing.assert_allclose(rays[0], [-0.04, -0.04, 1], atol=1.0e-7)

    def test_raw_distortion_is_undone(self):
        intrinsic = np.array([[100, 0, 100], [0, 100, 100], [0, 0, 1]], dtype=float)
        distortion = np.array([0.1, -0.01, 0.001, -0.002, 0.0001, 0.01, 0, 0])
        _, _, rays = sampled_rays(200, 200, intrinsic.ravel(), distortion, 'rational_polynomial', 50)
        expected = cv2.undistortPoints(
            np.array([[[150.0, 150.0]]]), intrinsic, distortion
        )[0, 0]
        np.testing.assert_allclose(rays[-1, :2], expected, atol=1.0e-6)
        self.assertGreater(abs(float(rays[-1, 0]) - 0.5), 0.001)

    def test_invalid_calibration_rejected(self):
        good = [100, 0, 1, 0, 100, 1, 0, 0, 1]
        for intrinsic, distortion, model in [
            ([0] * 9, [], 'plumb_bob'),
            (good, [np.nan] * 5, 'plumb_bob'),
            (good, [0] * 5, 'rational_polynomial'),
            (good, [0] * 4, 'equidistant'),
        ]:
            with self.assertRaises(ValueError):
                sampled_rays(2, 2, intrinsic, distortion, model)

    def test_optical_axes_floor_retained_body_removed(self):
        # Optical (right, down, forward) -> base (forward, left, up).
        rotation = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        points = np.array([
            [0, 0, 1],       # forward obstacle
            [0, 0.65, 1],    # ground return retained for 3D clearing
            [0, 0, 0.1],     # own body footprint
            [0, -1, 1],      # above configured observation volume
            [np.nan, 0, 1],
        ], dtype=np.float32)
        filtered = filter_cloud(
            points, rotation, [-0.033, 0, 0.65], -0.05, 1.6,
            [-0.434, 0.195, -0.287, 0.287],
        )
        np.testing.assert_allclose(filtered, points[:2])

    def test_quaternion_yaw_and_bad_transform(self):
        half_angle = np.pi / 4
        matrix = quaternion_matrix([0, 0, np.sin(half_angle), np.cos(half_angle)])
        np.testing.assert_allclose(matrix @ [1, 0, 0], [0, 1, 0], atol=1.0e-7)
        for values in ([0, 0, 0, 0], [0, 0, np.inf, 1], [1, 2, 3]):
            with self.assertRaises(ValueError):
                quaternion_matrix(values)

    def test_empty_cloud_and_bad_limits(self):
        empty = filter_cloud(
            np.empty((0, 3)), np.eye(3), [0, 0, 0], -0.05, 1.6, [-0.4, 0.2, -0.3, 0.3]
        )
        self.assertEqual(empty.shape, (0, 3))
        with self.assertRaises(ValueError):
            project_depth(np.ones((1, 1)), [0], [0], np.ones((1, 3)), 2, 1)


if __name__ == '__main__':
    unittest.main()
