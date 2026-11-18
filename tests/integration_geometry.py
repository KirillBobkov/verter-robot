"""Checks real robot_state_publisher TF in an unused localhost ROS domain."""

import os
import json
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

assert os.environ.get('ROS_DOMAIN_ID') == '178'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
ROOT = Path(__file__).resolve().parents[1]
rclpy.init()
node = rclpy.create_node('geometry_assertions')
process = None
try:
    # Refuse to mix this test with any existing ROS application in its domain.
    end = time.monotonic() + 2
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    others = [
        name
        for name, _ in node.get_node_names_and_namespaces()
        if name != 'geometry_assertions' and not name.startswith('_ros2cli')
    ]
    assert not others, others
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    diagnostics = ROOT / 'data/diagnostics/integration'
    diagnostics.mkdir(parents=True, exist_ok=True)
    log = open(diagnostics / 'geometry-rsp.log', 'w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'verter_description', 'view.launch.py', 'gui:=false'],
        start_new_session=True,
        stdout=log,
        stderr=subprocess.STDOUT,
    )
    expected = {
        'base_link': (0, 0, 0.1),
        'wheel_left': (0, 0.178, 0.098),
        'wheel_right': (0, -0.178, 0.098),
        'caster_back_left': (-0.291628571, 0.109, 0.025),
        'caster_back_right': (-0.291628571, -0.109, 0.025),
        'lidar_link': (0, 0, 0.86),
        'imu_link': (0, 0, 0.405),
    }
    config = json.loads((ROOT / 'ros_ws/src/verter_description/config/geometry.json').read_text())
    expected.update(config['ultrasonics']['centers_ground'])
    end = time.monotonic() + 15
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(buffer.can_transform('base_footprint', name, Time()) for name in expected):
            break
    for name, xyz in expected.items():
        t = buffer.lookup_transform('base_footprint', name, Time()).transform.translation
        assert all(abs(a - b) < 1e-8 for a, b in zip((t.x, t.y, t.z), xyz)), (name, t, xyz)
    print(f'PASS robot_state_publisher: {len(expected)} measured/fitted/preserved transforms')
    assert node.count_publishers('/cmd_vel') == 0
    assert node.count_publishers('/scan') == 0
    print('PASS geometry launch starts no command or sensor publishers')
finally:
    if process is not None:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
    node.destroy_node()
    rclpy.shutdown()
