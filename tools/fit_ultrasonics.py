#!/usr/bin/env python3
"""Reproduce the user-authorized sensor fit at z=148 mm. No ROS/hardware use.

The convex envelope of the horizontal mesh section bridges mounting holes.
Front sensor lateral spacings are retained. Side sensors retain the 100 mm
longitudinal separation behind the front-center sensor. This is a CAD fit,
not a claim of measurement of the physical sensor mounting points.
"""

import argparse
import json
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / 'ros_ws/src/verter_description'


def section_hull():
    data = (DESC / 'meshes/platform_aligned_m.stl').read_bytes()
    count = struct.unpack_from('<I', data, 80)[0]
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    triangles = np.frombuffer(data, dtype=dtype, count=count, offset=84)['v'].astype(float)
    z = 0.148 - 0.050
    points = []
    for i, j in [(0, 1), (1, 2), (2, 0)]:
        a, b = triangles[:, i, :], triangles[:, j, :]
        mask = ((a[:, 2] - z) * (b[:, 2] - z) <= 0) & (abs(b[:, 2] - a[:, 2]) > 1e-12)
        a, b = a[mask], b[mask]
        t = (z - a[:, 2]) / (b[:, 2] - a[:, 2])
        points.extend((a + (b - a) * t[:, None])[:, :2].tolist())
    points = sorted(set(map(tuple, points)))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def chain(values):
        result = []
        for p in values:
            while len(result) > 1 and cross(result[-2], result[-1], p) <= 0:
                result.pop()
            result.append(p)
        return result

    return np.array(chain(points)[:-1] + chain(reversed(points))[:-1])


def fit():
    hull = section_hull()

    def intersections(axis, value):
        result = []
        for a, b in zip(hull, np.roll(hull, -1, axis=0)):
            if min(a[axis], b[axis]) <= value <= max(a[axis], b[axis]) and abs(b[axis] - a[axis]) > 1e-12:
                result.append(a + (b - a) * (value - a[axis]) / (b[axis] - a[axis]))
        if not result:
            raise ValueError('No section intersection')
        return result

    front = max(p[0] for p in intersections(1, 0))
    result = {'sensor_front_center': [round(front, 4), 0, 0.148]}
    for label, y in [('inner', 0.074), ('outer', 0.152)]:
        x = np.mean([max(p[0] for p in intersections(1, sign * y)) for sign in [-1, 1]])
        for side, sign in [('left', 1), ('right', -1)]:
            result[f'sensor_front_{side}_{label}'] = [round(float(x), 4), sign * y, 0.148]
    x = round(front - 0.100, 4)
    points = intersections(0, x)
    y = (max(p[1] for p in points) - min(p[1] for p in points)) / 2
    for side, sign in [('left', 1), ('right', -1)]:
        result[f'sensor_{side}'] = [x, sign * round(float(y), 4), 0.148]
    return result, hull


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    positions, hull = fit()
    print(json.dumps(positions, indent=2))
    if not args.apply:
        return
    file = DESC / 'urdf/verter_robot.urdf'
    root = ET.parse(file).getroot()
    root.insert(
        0,
        ET.Comment(
            'Lidar/IMU mounting confirmed unchanged by owner. Ultrasonic XYZ fitted to CAD at 148 mm, not surveyed.'
        ),
    )
    for name, xyz in positions.items():
        root.find(f"joint[@name='{name}_joint']/origin").set('xyz', f'{xyz[0]:.4f} {xyz[1]:.4f} 0.048')
        root.find(f"link[@name='{name}']/visual/material").set('name', 'ultrasonic_sensor')
    material = root.find("material[@name='unverified_sensor']")
    if material is not None:
        material.set('name', 'ultrasonic_sensor')
    root.find("material[@name='ultrasonic_sensor']/color").set('rgba', '0.1 0.65 0.4 1')
    ET.indent(root, space='  ')
    file.write_bytes(ET.tostring(root, encoding='utf-8', xml_declaration=True) + b'\n')
    file = DESC / 'config/geometry.json'
    config = json.loads(file.read_text())
    config['sensor_positions_status'] = (
        'Lidar/IMU unchanged, confirmed by owner. Ultrasonics fitted to CAD by owner request; height 0.148 m confirmed.'
    )
    config['ultrasonics'] = {
        'centers_ground': positions,
        'method': 'Convex envelope of mesh section at 0.148 m ground height; symmetric pairs rounded to 0.1 mm; original front lateral spacings retained; sides 0.100 m behind front center',
        'section_hull_xy': hull.tolist(),
        'status': 'CAD fit, not physical survey',
    }
    file.write_bytes((json.dumps(config, indent=2) + '\n').encode())


if __name__ == '__main__':
    main()
