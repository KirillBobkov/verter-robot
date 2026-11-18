"""Physical dimensions and compatibility checks independent of ROS runtime."""

import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
DESC = ROOT / 'ros_ws/src/verter_description'
URDF = ET.parse(DESC / 'urdf/verter_robot.urdf').getroot()
CONFIG = json.loads((DESC / 'config/geometry.json').read_text())


def ground_position(name):
    joints = {j.find('child').get('link'): j for j in URDF.findall('joint')}
    point = np.zeros(3)
    while name != 'base_footprint':
        j = joints[name]
        # All ancestors of wheel and sensor frames here are translation-only.
        point += np.array([float(v) for v in j.find('origin').get('xyz').split()])
        name = j.find('parent').get('link')
    return point


def test_single_connected_tree():
    links = {link.get('name') for link in URDF.findall('link')}
    children = [j.find('child').get('link') for j in URDF.findall('joint')]
    assert len(children) == len(set(children)) == len(links) - 1
    assert links - set(children) == {'base_footprint'}
    for name in links:
        ground_position(name)


@pytest.mark.parametrize('side,y', [('left', 0.178), ('right', -0.178)])
def test_drive_wheel_contact(side, y):
    center = ground_position('wheel_' + side)
    assert center == pytest.approx([0, y, 0.098])
    for typ in ['visual', 'collision']:
        g = URDF.find(f"link[@name='wheel_{side}']/{typ}/geometry/cylinder")
        assert float(g.get('radius')) == pytest.approx(center[2])
        assert float(g.get('length')) == 0.05


@pytest.mark.parametrize('side,y', [('left', 0.109), ('right', -0.109)])
def test_rear_wheel_contact_and_bumper_offset(side, y):
    center = ground_position('caster_back_' + side)
    assert center == pytest.approx([CONFIG['body']['bounds_ground'][0][0] + 0.122, y, 0.025])
    g = URDF.find(f"link[@name='caster_back_{side}']/visual/geometry/cylinder")
    assert float(g.get('radius')) == pytest.approx(center[2])
    assert float(g.get('length')) == 0.04


def test_mesh_bounds_and_footprint_enclosure():
    data = (DESC / 'meshes/platform_aligned_m.stl').read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    assert len(data) == 84 + n * 50
    dtype = np.dtype([('normal', '<f4', (3,)), ('v', '<f4', (3, 3)), ('attr', '<u2')])
    vertices = np.frombuffer(data, dtype=dtype, offset=84)['v'].reshape(-1, 3)
    assert np.isfinite(vertices).all()
    lo = vertices.min(0)
    hi = vertices.max(0)
    assert hi[0] == pytest.approx(0.175, abs=1e-6)
    assert lo[0] == pytest.approx(-0.413628571, abs=1e-6)
    polygon = np.array(CONFIG['footprint'])
    assert np.all(vertices[:, :2] >= polygon.min(0) - 1e-6)
    assert np.all(vertices[:, :2] <= polygon.max(0) + 1e-6)
    assert ground_position('base_link')[2] - 0.05 + lo[2] == pytest.approx(0.05)


def test_both_costmaps_use_measured_polygon():
    params = yaml.safe_load((ROOT / 'ros_ws/src/verter_bringup/config/nav2/nav2_navigation_params.yaml').read_text())
    for name in ['local_costmap', 'global_costmap']:
        p = params[name][name]['ros__parameters']
        assert 'robot_radius' not in p
        assert json.loads(p['footprint']) == CONFIG['footprint']
        assert p['footprint_padding'] == 0.01


def test_lidar_imu_and_other_unchanged_translations():
    old = ET.parse(DESC / 'urdf/verter_robot_minimal.urdf').getroot()
    for joint in old.findall('joint'):
        if joint.get('name') in [
            'lidar_joint',
            'imu_joint',
            'camera_tof_joint',
            'camera_tof_optical_joint',
            'base_footprint_joint',
        ]:
            new = URDF.find(f"joint[@name='{joint.get('name')}']/origin")
            assert new.get('xyz') == joint.find('origin').get('xyz')
    assert ground_position('lidar_link') == pytest.approx([0, 0, 0.86])
    assert ground_position('imu_link') == pytest.approx([0, 0, 0.405])


@pytest.mark.parametrize(
    'name',
    [
        'sensor_front_center',
        'sensor_front_left_inner',
        'sensor_front_right_inner',
        'sensor_front_left_outer',
        'sensor_front_right_outer',
        'sensor_left',
        'sensor_right',
    ],
)
def test_ultrasonics_at_section_boundary_and_height(name):
    center = ground_position(name)
    assert center[2] == pytest.approx(0.148)
    hull = np.array(CONFIG['ultrasonics']['section_hull_xy'])
    distances = []
    for a, b in zip(hull, np.roll(hull, -1, axis=0)):
        delta = b - a
        t = np.clip(np.dot(center[:2] - a, delta) / np.dot(delta, delta), 0, 1)
        distances.append(np.linalg.norm(center[:2] - (a + t * delta)))
    assert min(distances) < 0.0001  # Fit rounded to 0.1 mm; mesh tessellation itself is coarser.
    # Even the approximate 20-mm sensor box fits the padded navigation contour.
    origin = URDF.find(f"joint[@name='{name}_joint']/origin")
    yaw = float(origin.get('rpy').split()[2])
    radius = 0.01 * (abs(math.cos(yaw)) + abs(math.sin(yaw)))
    polygon = np.array(CONFIG['footprint'])
    assert np.all(center[:2] - radius >= polygon.min(0) - CONFIG['footprint_padding'])
    assert np.all(center[:2] + radius <= polygon.max(0) + CONFIG['footprint_padding'])


def test_confirmed_ultrasonic_angles():
    for name, deg in [
        ('sensor_front_left_inner', 5.7),
        ('sensor_front_left_outer', 13.2),
        ('sensor_left', 75.9),
        ('sensor_right', -75.9),
    ]:
        yaw = float(URDF.find(f"joint[@name='{name}_joint']/origin").get('rpy').split()[2])
        assert yaw == pytest.approx(math.radians(deg))
