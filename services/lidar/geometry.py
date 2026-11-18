"""Pure scan encoding and URDF top-view geometry; no ROS or device access."""

import math
import xml.etree.ElementTree as ET


def scan_frame(scan, transform, robot_id, sequence, sample_at):
    if not scan.ranges or not 0 <= scan.range_min < scan.range_max <= 10000:
        raise ValueError('invalid_scan_bounds')
    if not all(math.isfinite(x) for x in (scan.angle_min, scan.angle_increment)) or scan.angle_increment == 0:
        raise ValueError('invalid_scan_angles')
    translation = transform.translation
    rotation = transform.rotation
    xyz = [translation.x, translation.y, translation.z]
    quat = [rotation.x, rotation.y, rotation.z, rotation.w]
    if not all(math.isfinite(x) for x in xyz + quat) or abs(sum(x * x for x in quat) - 1) > 0.01:
        raise ValueError('invalid_transform')
    step = max(1, math.ceil(len(scan.ranges) / 2048))
    ranges = [
        round(x, 4) if math.isfinite(x) and scan.range_min <= x <= scan.range_max else None for x in scan.ranges[::step]
    ]
    return {
        'v': 1,
        'type': 'scan',
        'robot_id': robot_id,
        'sequence': sequence,
        'sample_at': sample_at,
        'max_age_s': 1.5,
        'frame_id': scan.header.frame_id,
        'target_frame': 'base_footprint',
        'ros_stamp': {'sec': scan.header.stamp.sec, 'nanosec': scan.header.stamp.nanosec},
        'angle_min': scan.angle_min,
        'angle_increment': scan.angle_increment * step,
        'range_min': scan.range_min,
        'range_max': scan.range_max,
        'original_points': len(scan.ranges),
        'stride': step,
        'ranges': ranges,
        'transform': {'translation': xyz, 'rotation_xyzw': quat},
    }


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def origin(element):
    if element is None:
        return identity()
    xyz = list(map(float, element.get('xyz', '0 0 0').split()))
    roll, pitch, yaw = map(float, element.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll),
        math.sin(roll),
        math.cos(pitch),
        math.sin(pitch),
        math.cos(yaw),
        math.sin(yaw),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0, 0, 0, 1],
    ]


def hull(points):
    points = sorted(set(points))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def half(values):
        result = []
        for p in values:
            while len(result) >= 2 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result

    return half(points)[:-1] + half(reversed(points))[:-1]


def robot_model(path, robot_id):
    tree = ET.parse(path).getroot()
    joints = {j.find('child').get('link'): j for j in tree.findall('joint')}

    def world(name, seen=()):
        if name == 'base_footprint':
            return identity()
        if name in seen:
            raise ValueError('cyclic_urdf')
        joint = joints[name]
        if joint.get('type') != 'fixed':
            raise ValueError('top_view_requires_fixed_joints')
        return multiply(world(joint.find('parent').get('link'), seen + (name,)), origin(joint.find('origin')))

    polygons = []
    for link in tree.findall('link'):
        name = link.get('name')
        if name != 'base_link' and not name.startswith(('wheel_', 'caster_')):
            continue
        collision = link.find('collision')
        geometry = collision.find('geometry')
        box, cylinder = geometry.find('box'), geometry.find('cylinder')
        if box is not None:
            x, y, z = [float(v) / 2 for v in box.get('size').split()]
            vertices = [(a, b, c, 1) for a in (-x, x) for b in (-y, y) for c in (-z, z)]
        elif cylinder is not None:
            radius, length = float(cylinder.get('radius')), float(cylinder.get('length'))
            vertices = [
                (radius * math.cos(i * math.tau / 32), radius * math.sin(i * math.tau / 32), z, 1)
                for i in range(32)
                for z in (-length / 2, length / 2)
            ]
        else:
            raise ValueError('unsupported_collision_geometry')
        matrix = multiply(world(name), origin(collision.find('origin')))
        points = [tuple(round(sum(matrix[i][j] * p[j] for j in range(4)), 6) for i in range(2)) for p in vertices]
        polygons.append({'name': name, 'kind': 'body' if name == 'base_link' else 'wheel', 'points': hull(points)})
    if len(polygons) != 5:
        raise ValueError('expected_body_and_four_wheels')
    return {
        'v': 1,
        'type': 'model',
        'robot_id': robot_id,
        'frame_id': 'base_footprint',
        'unit': 'm',
        'source': 'URDF collision projection',
        'polygons': polygons,
    }
