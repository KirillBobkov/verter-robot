"""Depth projection and geometric validation, independent of ROS."""

import math

import cv2
import numpy as np


def decode_depth(data, width, height, step, encoding, is_bigendian=False):
    """Return metres while preserving image row padding and wire byte order."""
    if encoding == '16UC1':
        dtype = np.dtype('>u2' if is_bigendian else '<u2')
        scale = 0.001
    elif encoding == '32FC1':
        dtype = np.dtype('>f4' if is_bigendian else '<f4')
        scale = 1.0
    else:
        raise ValueError('unsupported_depth_encoding')

    if width <= 0 or height <= 0 or step < width * dtype.itemsize:
        raise ValueError('invalid_depth_dimensions')

    raw = memoryview(data).cast('B')
    if len(raw) != step * height:
        raise ValueError('invalid_depth_buffer_length')

    pixels = np.ndarray(
        (height, width),
        dtype=dtype,
        buffer=raw,
        strides=(step, dtype.itemsize),
    )
    return pixels.astype(np.float32) * scale


def sampled_rays(width, height, intrinsic, distortion, model, stride=4):
    """Precompute optical-frame rays for a raw, possibly distorted depth image."""
    if width <= 0 or height <= 0 or stride < 1:
        raise ValueError('invalid_projection_dimensions')

    matrix = np.asarray(intrinsic, dtype=np.float64)
    coefficients = np.asarray(distortion, dtype=np.float64)
    if matrix.size != 9 or not np.all(np.isfinite(matrix)):
        raise ValueError('invalid_depth_intrinsics')
    matrix = matrix.reshape(3, 3)
    if matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
        raise ValueError('invalid_depth_focal_length')
    if not np.allclose(matrix[2], [0.0, 0.0, 1.0]):
        raise ValueError('invalid_depth_intrinsic_bottom_row')
    if not np.all(np.isfinite(coefficients)):
        raise ValueError('invalid_depth_distortion')
    if model == 'plumb_bob':
        if coefficients.size not in (0, 4, 5):
            raise ValueError('invalid_plumb_bob_coefficients')
    elif model == 'rational_polynomial':
        if coefficients.size not in (0, 8):
            raise ValueError('invalid_rational_polynomial_coefficients')
    else:
        raise ValueError('unsupported_depth_distortion_model')

    rows = np.arange(0, height, stride)
    columns = np.arange(0, width, stride)
    x, y = np.meshgrid(columns, rows)
    pixels = np.stack((x, y), axis=-1).astype(np.float64).reshape(-1, 1, 2)
    distortion_array = coefficients if coefficients.size else None
    try:
        normalized = cv2.undistortPoints(pixels, matrix, distortion_array).reshape(-1, 2)
    except cv2.error as error:
        raise ValueError('invalid_depth_calibration') from error
    rays = np.column_stack((normalized, np.ones(len(normalized)))).astype(np.float32)
    if not np.all(np.isfinite(rays)):
        raise ValueError('nonfinite_depth_rays')
    return rows, columns, rays


def project_depth(depth, rows, columns, rays, min_depth, max_depth):
    """Project finite real returns only; missing returns never become free rays."""
    if not math.isfinite(min_depth) or not math.isfinite(max_depth):
        raise ValueError('invalid_depth_limits')
    if min_depth <= 0 or max_depth <= min_depth:
        raise ValueError('invalid_depth_limits')

    sampled = depth[np.ix_(rows, columns)].reshape(-1)
    if rays.shape != (len(sampled), 3):
        raise ValueError('depth_ray_shape_mismatch')
    valid = np.isfinite(sampled) & (sampled >= min_depth) & (sampled <= max_depth)
    return np.ascontiguousarray(rays[valid] * sampled[valid, None], dtype=np.float32)


def quaternion_matrix(quaternion):
    """Convert a finite TF xyzw quaternion into a rotation matrix."""
    values = np.asarray(quaternion, dtype=np.float64)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError('invalid_transform_quaternion')
    length = float(np.linalg.norm(values))
    if length < 1.0e-12:
        raise ValueError('invalid_transform_quaternion')
    x, y, z, w = values / length
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def filter_cloud(points, rotation, translation, min_height, max_height, self_bounds):
    """Filter in the base frame, then retain original optical-frame coordinates."""
    points = np.asarray(points, dtype=np.float32)
    rotation = np.asarray(rotation, dtype=np.float64)
    translation = np.asarray(translation, dtype=np.float64)
    bounds = np.asarray(self_bounds, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError('invalid_cloud_shape')
    if rotation.shape != (3, 3) or translation.shape != (3,):
        raise ValueError('invalid_transform_shape')
    if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
        raise ValueError('nonfinite_transform')
    if bounds.shape != (4,) or not np.all(np.isfinite(bounds)):
        raise ValueError('invalid_self_bounds')
    if bounds[0] >= bounds[1] or bounds[2] >= bounds[3]:
        raise ValueError('invalid_self_bounds')
    if not math.isfinite(min_height) or not math.isfinite(max_height) or min_height >= max_height:
        raise ValueError('invalid_height_limits')

    in_base = points @ rotation.T + translation
    valid = np.all(np.isfinite(in_base), axis=1)
    valid &= (in_base[:, 2] >= min_height) & (in_base[:, 2] <= max_height)
    inside_body = (
        (in_base[:, 0] >= bounds[0])
        & (in_base[:, 0] <= bounds[1])
        & (in_base[:, 1] >= bounds[2])
        & (in_base[:, 1] <= bounds[3])
    )
    return np.ascontiguousarray(points[valid & ~inside_body], dtype=np.float32)
