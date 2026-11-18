#!/usr/bin/env python3
"""Synthetic Nav2 obstacle tests, restricted to localhost ROS domain 231.

This launches only two perception adapters and one standalone costmap at a time.
It never launches a controller, sends a velocity, sets an initial pose, or sends
a navigation goal. Run from a prepared /tmp/verter-costmap-stage-* directory:

    ROS_DOMAIN_ID=231 ROS_LOCALHOST_ONLY=1 python3 tests/costmap_integration.py .

The small C++ fixture creates only a named Costmap2DROS and publishes its
isCurrent() result. Local and global phases use their actual production node
names and parameters. Sources are synthetic throughout.
"""

import os


if os.environ.get('ROS_DOMAIN_ID') != '231':
    raise SystemExit('Refusing test outside isolated ROS_DOMAIN_ID=231')
if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Refusing test without ROS_LOCALHOST_ONLY=1')

from array import array
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.msg import Costmap, VoxelGrid
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2, Range
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import StaticTransformBroadcaster
import yaml


FORBIDDEN_TOPICS = ('/cmd_vel', '/initialpose', '/goal_pose', '/set_pose')
SENSOR_NAMES = (
    'right',
    'front_right_outer',
    'front_right_inner',
    'front_center',
    'front_left_inner',
    'front_left_outer',
    'left',
)


def stop_process(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)


def main():
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--background-depth-mm', type=int, default=3000)
    parser.add_argument('--local-window-m', type=int, choices=(4, 10))
    arguments = parser.parse_args()
    if not 1200 <= arguments.background_depth_mm <= 3500:
        raise SystemExit('Synthetic background must be between 1200 and 3500 mm')
    root = arguments.root.resolve()
    if not str(root).startswith('/tmp/verter-costmap-stage-'):
        raise SystemExit('A dedicated /tmp/verter-costmap-stage-* root is required')

    source = root / 'ros_ws/src/verter_perception'
    nav2_file = root / 'ros_ws/src/verter_bringup/config/nav2/nav2_navigation_params.yaml'
    perception_file = root / 'ros_ws/src/verter_bringup/config/perception.yaml'
    if not source.is_dir() or not nav2_file.is_file() or not perception_file.is_file():
        raise SystemExit('Candidate package and configuration are incomplete')

    output = root / 'costmap-smoke'
    output.mkdir(exist_ok=True)
    environment = dict(os.environ)
    environment['PYTHONPATH'] = str(source) + os.pathsep + environment.get('PYTHONPATH', '')
    environment['ROS_LOG_DIR'] = str(output / 'ros-logs')
    environment['OPENBLAS_NUM_THREADS'] = '1'
    environment['OMP_NUM_THREADS'] = '1'

    helper_source = root / 'tests/isolated_costmap'
    helper_build = root / 'costmap-test-helper-build'
    with (output / 'helper-build.log').open('wb') as build_log:
        subprocess.run(
            ['cmake', '-S', str(helper_source), '-B', str(helper_build)],
            check=True, stdout=build_log, stderr=build_log,
        )
        subprocess.run(
            ['cmake', '--build', str(helper_build), '--parallel', '1'],
            check=True, stdout=build_log, stderr=build_log,
        )
    executable = helper_build / 'verter_test_costmap'
    assert executable.is_file()

    rclpy.init()
    fixture = Fixture(arguments.background_depth_mm)
    processes = []
    logs = []
    report = {
        'ok': False, 'domain': 231, 'localhost_only': True, 'phases': {},
        'background_depth_mm': arguments.background_depth_mm,
        'test_local_window_override_m': arguments.local_window_m,
        'source_hashes': source_hashes(root),
        'synthetic_camera': {'width': 320, 'height': 256, 'fx': 160.0, 'stride': 4},
        'instrumentation_overrides': [
            'always_send_full_costmap=true',
            'camera_layer.publish_voxel_map=true',
        ],
    }
    try:
        fixture.spin_for(2)
        unexpected = set(fixture.node.get_node_names()) - {fixture.node.get_name()}
        if unexpected:
            raise AssertionError(('isolated_domain_not_empty', sorted(unexpected)))
        fixture.assert_no_commands()

        for module in ('depth_obstacles', 'ultrasonic_ranges'):
            stream = (output / (module + '.log')).open('wb')
            logs.append(stream)
            process = subprocess.Popen(
                [
                    sys.executable,
                    '-c',
                    'from verter_perception.' + module + ' import main\nmain()\n',
                    '--ros-args',
                    '--params-file',
                    str(perception_file),
                ],
                env=environment,
                stdout=stream,
                stderr=stream,
                start_new_session=True,
            )
            processes.append(process)

        fixture.spin_for(3)
        for process in processes:
            assert process.poll() is None, 'Perception adapter exited; inspect its log'

        fixture.send_depth = True
        fixture.send_sonars = True
        fixture.scene = 'floor'
        fixture.sonar_values = [10.0] * 7
        fixture.until(lambda: fixture.cloud is not None, 8, 'No projected depth cloud')
        assert fixture.cloud.header.frame_id == 'camera_depth_optical_frame'
        stamp = fixture.cloud.header.stamp.sec + fixture.cloud.header.stamp.nanosec * 1e-9
        assert stamp in fixture.sent_stamps, 'Cloud did not preserve an original image stamp'
        assert fixture.cloud.width * fixture.cloud.height > 0
        point = np.frombuffer(bytes(fixture.cloud.data), dtype='<f4')[:3]
        np.testing.assert_allclose(point, [0.4, 0.6, 1.0], atol=1e-5)
        assert not fixture.ranges, 'Timeout values became valid sonar measurements'

        config = yaml.safe_load(nav2_file.read_text())
        for kind in ('local_costmap', 'global_costmap'):
            parameters = copy.deepcopy(config[kind][kind]['ros__parameters'])
            expected_plugins = {'lidar_layer', 'camera_layer', 'ultrasonic_layer', 'inflation_layer'}
            assert expected_plugins.issubset(parameters['plugins'])
            assert parameters['camera_layer']['plugin'] == 'nav2_costmap_2d::VoxelLayer'
            assert parameters['ultrasonic_layer']['input_sensor_type'] == 'VARIABLE'
            assert not parameters['ultrasonic_layer']['clear_on_max_reading']
            assert set(parameters['ultrasonic_layer']['topics']) == {
                '/perception/ultrasonic/' + name for name in SENSOR_NAMES
            }

            # A full publication is needed to assert each state without patching
            # OccupancyGridUpdate messages. All sensor/layer parameters stay intact.
            parameters['always_send_full_costmap'] = True
            parameters['camera_layer']['publish_voxel_map'] = True
            if kind == 'local_costmap' and arguments.local_window_m is not None:
                parameters['width'] = arguments.local_window_m
                parameters['height'] = arguments.local_window_m
            test_file = output / (kind + '.yaml')
            test_file.write_text(yaml.safe_dump({'/' + kind + '/' + kind: {'ros__parameters': parameters}}))
            stream = (output / (kind + '.log')).open('wb')
            logs.append(stream)
            fixture.reset_scene()
            fixture.phase = kind
            # Drain volatile samples left in this observer's queues by the
            # preceding phase before measuring publications from invalid input.
            fixture.spin_for(2)
            fixture.ranges.clear()
            fixture.range_counts.clear()
            fixture.observe_costmap(kind)
            process = subprocess.Popen(
                [str(executable), kind, '--ros-args', '--params-file', str(test_file)],
                env=environment,
                stdout=stream,
                stderr=stream,
                start_new_session=True,
            )
            processes.append(process)
            configured = False
            active = False
            try:
                fixture.spin_for(2)
                fixture.transition(Transition.TRANSITION_CONFIGURE)
                configured = True
                fixture.transition(Transition.TRANSITION_ACTIVATE)
                active = True
                report['phases'][kind] = fixture.exercise_layers()
            finally:
                if active:
                    fixture.transition(Transition.TRANSITION_DEACTIVATE)
                if configured:
                    fixture.transition(Transition.TRANSITION_CLEANUP)
                stop_process(process)
                fixture.spin_for(1)
                fixture.grid = None

        fixture.send_depth = False
        fixture.send_sonars = False
        fixture.assert_no_commands()
        report.update(
            ok=True,
            projected_clouds=fixture.cloud_count,
            preserved_image_stamps=True,
            invalid_sonar_timeout_is_not_free_space=True,
            stopped_depth_is_stale_without_republication=True,
            fresh_depth_recovers_after_stale_without_restart=True,
            command_publishers={name: 0 for name in FORBIDDEN_TOPICS},
            nodes=sorted(set(fixture.node.get_node_names())),
        )
    finally:
        for process in reversed(processes):
            stop_process(process)
        fixture.node.destroy_node()
        rclpy.shutdown()
        for stream in logs:
            stream.close()
        report['snapshots'] = fixture.snapshots
        report['last_observations'] = fixture.snapshot('final')
        report['elapsed_s'] = time.monotonic() - started
        (output / 'result.json').write_text(json.dumps(report, indent=2))

    print(json.dumps(report, indent=2))


def source_hashes(root):
    files = list((root / 'ros_ws/src/verter_perception').rglob('*.py'))
    files += list((root / 'tests/isolated_costmap').glob('*'))
    files += [
        root / 'tests/costmap_integration.py',
        root / 'ros_ws/src/verter_bringup/config/nav2/nav2_navigation_params.yaml',
        root / 'ros_ws/src/verter_bringup/config/perception.yaml',
    ]
    return {
        str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(files)
        if path.is_file()
    }


class Fixture:
    def __init__(self, background_depth_mm):
        self.node = rclpy.create_node('verter_costmap_synthetic_fixture')
        self.cloud = None
        self.cloud_count = 0
        self.grid = None
        self.ranges = {}
        self.range_counts = {}
        self.depth_status = None
        self.voxel_grid = None
        self.clearing_endpoints = None
        self.snapshots = []
        self.phase = 'adapters'
        self.background_depth_mm = background_depth_mm
        self.inputs_current = None
        self.costmap_name = None
        self.sent_stamps = set()
        self.scene = 'floor'
        self.send_depth = False
        self.send_sonars = False
        self.sonar_values = [10.0] * 7
        self.subscriptions = []
        self.depth_pub = self.node.create_publisher(Image, '/camera/depth/image_raw', qos_profile_sensor_data)
        self.info_pub = self.node.create_publisher(CameraInfo, '/camera/depth/camera_info', qos_profile_sensor_data)
        self.sonar_pub = self.node.create_publisher(Float32MultiArray, '/ultrasonics', qos_profile_sensor_data)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/map', latched)
        self.subscriptions.append(self.node.create_subscription(
            PointCloud2, '/perception/depth/points', self.receive_cloud, qos_profile_sensor_data
        ))
        self.subscriptions.append(self.node.create_subscription(
            String, '/perception/depth/status', self.receive_status, 10
        ))
        self.latched_qos = latched
        self.costmap_subscriptions = []
        for name in SENSOR_NAMES:
            self.subscriptions.append(self.node.create_subscription(
                Range,
                '/perception/ultrasonic/' + name,
                lambda message, sensor=name: self.receive_range(sensor, message),
                qos_profile_sensor_data,
            ))

        self.static_tf = StaticTransformBroadcaster(self.node)
        definitions = [
            ('map', 'odom', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ('odom', 'base_footprint', [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
            ('base_footprint', 'camera_depth_optical_frame', [-0.033, 0.0, 0.65], [-0.5, 0.5, -0.5, 0.5]),
            ('base_footprint', 'lidar_link', [0.0, 0.0, 0.2], [0.0, 0.0, 0.0, 1.0]),
        ]
        sonar_geometry = [
            ('right', [0.055, -0.239, 0.148], -75.9),
            ('front_right_outer', [0.1369, -0.152, 0.148], -13.2),
            ('front_right_inner', [0.1509, -0.074, 0.148], -5.7),
            ('front_center', [0.155, 0.0, 0.148], 0.0),
            ('front_left_inner', [0.1509, 0.074, 0.148], 5.7),
            ('front_left_outer', [0.1369, 0.152, 0.148], 13.2),
            ('left', [0.055, 0.239, 0.148], 75.9),
        ]
        for name, xyz, yaw_degrees in sonar_geometry:
            yaw = math.radians(yaw_degrees)
            definitions.append((
                'base_footprint', 'sensor_' + name, xyz,
                [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)],
            ))
        transforms = []
        for parent, child, xyz, quaternion in definitions:
            message = TransformStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = parent
            message.child_frame_id = child
            message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = xyz
            (
                message.transform.rotation.x,
                message.transform.rotation.y,
                message.transform.rotation.z,
                message.transform.rotation.w,
            ) = quaternion
            transforms.append(message)
        self.static_tf.sendTransform(transforms)
        self.node.create_timer(0.1, self.publish_inputs)
        self.node.create_timer(0.5, self.publish_map)
        self.publish_map()

    def receive_cloud(self, message):
        self.cloud = message
        self.cloud_count += 1

    def receive_grid(self, message):
        self.grid = message

    def receive_voxels(self, message):
        self.voxel_grid = message

    def receive_endpoints(self, message):
        self.clearing_endpoints = message

    def receive_current(self, message):
        self.inputs_current = message.data

    def observe_costmap(self, name):
        for subscription in self.costmap_subscriptions:
            self.node.destroy_subscription(subscription)
        self.costmap_subscriptions = []
        self.costmap_name = '/' + name + '/' + name
        namespace = '/' + name
        self.costmap_subscriptions.append(self.node.create_subscription(
            Costmap, namespace + '/costmap_raw', self.receive_grid, self.latched_qos
        ))
        self.costmap_subscriptions.append(self.node.create_subscription(
            VoxelGrid, namespace + '/voxel_grid', self.receive_voxels, qos_profile_sensor_data
        ))
        self.costmap_subscriptions.append(self.node.create_subscription(
            PointCloud2, namespace + '/clearing_endpoints', self.receive_endpoints, qos_profile_sensor_data
        ))
        self.costmap_subscriptions.append(self.node.create_subscription(
            Bool, namespace + '/inputs_current', self.receive_current, 10
        ))

    def receive_status(self, message):
        self.depth_status = json.loads(message.data)

    def receive_range(self, name, message):
        self.ranges[name] = message
        self.range_counts[name] = self.range_counts.get(name, 0) + 1

    def publish_map(self):
        message = OccupancyGrid()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.info.resolution = 0.05
        message.info.width = 160
        message.info.height = 160
        message.info.origin.position.x = -4.0
        message.info.origin.position.y = -4.0
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (message.info.width * message.info.height)
        self.map_pub.publish(message)

    def publish_inputs(self):
        stamp = self.node.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'lidar_link'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2 * math.pi / 360
        scan.scan_time = 0.1
        scan.range_min = 0.05
        scan.range_max = 6.0
        scan.ranges = [5.5] * 361
        self.scan_pub.publish(scan)
        if self.send_sonars:
            message = Float32MultiArray()
            message.data = list(self.sonar_values)
            self.sonar_pub.publish(message)
        if not self.send_depth:
            return

        depth = np.zeros((256, 320), dtype='<u2')
        if self.scene == 'background':
            # A visible background is a plane, not six widely spaced rays.
            # At fx=160 and stride=4, samples are .025 m apart at 1 m;
            # the actual 640-wide Gemini profile samples more densely still.
            depth[:] = self.background_depth_mm
        elif self.scene == 'table':
            depth[96:116, 144:180] = 1000
        # Actual finite floor return: z_base = .65 - 96/160 * 1 = .05 m.
        depth[224:228, 224:228] = 1000
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = 'camera_depth_optical_frame'
        image.width = 320
        image.height = 256
        image.encoding = '16UC1'
        image.is_bigendian = False
        image.step = 640
        image.data = array('B', depth.tobytes())
        info = CameraInfo()
        info.header = copy.deepcopy(image.header)
        info.width = image.width
        info.height = image.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0] * 5
        info.k = [160.0, 0.0, 160.0, 0.0, 160.0, 128.0, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [160.0, 0.0, 160.0, 0.0, 0.0, 160.0, 128.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.sent_stamps.add(stamp.sec + stamp.nanosec * 1e-9)
        self.depth_pub.publish(image)
        self.info_pub.publish(info)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)

    def until(self, predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.03)
            if predicate():
                return
        raise AssertionError(reason)

    def transition(self, transition_id):
        client = self.node.create_client(ChangeState, self.costmap_name + '/change_state')
        try:
            assert client.wait_for_service(timeout_sec=12), 'Costmap lifecycle service did not appear'
            request = ChangeState.Request()
            request.transition.id = transition_id
            future = client.call_async(request)
            self.until(future.done, 12, 'Costmap lifecycle transition timed out')
            response = future.result()
            assert response.success, ('Costmap lifecycle transition failed', transition_id)
        finally:
            self.node.destroy_client(client)

    def near_cost(self, x, y, radius=0.06):
        if self.grid is None:
            return None
        metadata = self.grid.metadata
        origin = metadata.origin.position
        resolution = metadata.resolution
        minimum_x = max(0, math.floor((x - radius - origin.x) / resolution))
        maximum_x = min(metadata.size_x - 1, math.floor((x + radius - origin.x) / resolution))
        minimum_y = max(0, math.floor((y - radius - origin.y) / resolution))
        maximum_y = min(metadata.size_y - 1, math.floor((y + radius - origin.y) / resolution))
        values = [
            self.grid.data[cy * metadata.size_x + cx]
            for cy in range(minimum_y, maximum_y + 1)
            for cx in range(minimum_x, maximum_x + 1)
        ]
        return max(values) if values else None

    def reset_scene(self):
        self.scene = 'floor'
        self.sonar_values = [10.0] * 7
        self.grid = None
        self.voxel_grid = None
        self.clearing_endpoints = None
        self.inputs_current = None
        self.ranges.clear()
        self.range_counts.clear()

    def exercise_layers(self):
        self.until(lambda: self.grid is not None, 10, 'No raw costmap was published')
        self.spin_for(2)
        floor_cost = self.near_cost(0.967, -0.4)
        assert floor_cost == 0, ('Finite floor return marked an obstacle', floor_cost)
        assert not self.ranges, 'Invalid timeout array generated Range messages'

        self.scene = 'table'
        self.until(lambda: self.near_cost(0.967, 0.0) == 254, 8, 'Elevated table was not marked')
        table_cost = self.near_cost(0.967, 0.0)
        self.sonar_values[3] = 1.7
        self.spin_for(2)
        assert self.near_cost(0.967, 0.0) == 254, 'Ground-level free rays erased the elevated table'
        self.snapshots.append(self.snapshot('table_marked'))
        self.sonar_values[3] = 10.0

        self.scene = 'background'
        try:
            self.until(
                lambda: self.near_cost(0.967, 0.0) is not None and self.near_cost(0.967, 0.0) < 253,
                10,
                'A valid background depth ray did not clear the moved elevated obstacle',
            )
        finally:
            self.snapshots.append(self.snapshot('after_background_clearing'))
        cleared_table_cost = self.near_cost(0.967, 0.0)

        self.sonar_values[0] = 1.0
        yaw = math.radians(-75.9)
        target_x = 0.055 + math.cos(yaw)
        target_y = -0.239 + math.sin(yaw)
        self.until(
            lambda: self.near_cost(target_x, target_y) == 254,
            8,
            'Right sonar did not mark its correctly transformed obstacle',
        )
        assert 'right' in self.ranges
        reading = self.ranges['right']
        assert reading.header.frame_id == 'sensor_right'
        assert reading.radiation_type == Range.ULTRASOUND
        assert abs(reading.range - 1.0) < 1e-6
        assert 0.0 < reading.field_of_view < math.pi
        assert set(self.ranges).issubset({'right', 'front_center'}), self.ranges.keys()
        self.spin_for(1)
        assert self.near_cost(target_x, target_y) == 254, 'Free lidar erased the sonar obstacle'
        self.until(lambda: self.inputs_current is True, 5, 'Fresh costmap inputs were not current')

        paused_stamp = self.cloud.header.stamp.sec + self.cloud.header.stamp.nanosec * 1e-9
        self.send_depth = False
        self.until(lambda: self.inputs_current is False, 5, 'Stopped camera did not invalidate costmap inputs')
        self.until(
            lambda: self.depth_status is not None and self.depth_status['stale'],
            5,
            'Stopped depth adapter did not report stale data',
        )
        self.spin_for(1)
        paused_count = self.cloud_count
        self.spin_for(0.6)
        assert self.cloud_count == paused_count, 'An old depth frame was republished after input stopped'
        self.snapshots.append(self.snapshot('camera_paused'))

        self.send_depth = True
        self.until(lambda: self.cloud_count > paused_count, 5, 'Fresh depth did not resume after camera pause')
        resumed_stamp = self.cloud.header.stamp.sec + self.cloud.header.stamp.nanosec * 1e-9
        assert resumed_stamp > paused_stamp
        assert resumed_stamp in self.sent_stamps
        self.until(lambda: self.inputs_current is True, 5, 'Fresh camera data did not restore costmap currency')
        self.until(
            lambda: self.depth_status is not None
            and self.depth_status['ready']
            and not self.depth_status['stale'],
            5,
            'Fresh depth did not restore adapter readiness',
        )
        self.snapshots.append(self.snapshot('camera_resumed'))
        self.assert_no_commands()
        return {
            'floor_cost': floor_cost,
            'elevated_table_cost': table_cost,
            'free_low_beams_preserve_elevated_table': True,
            'cleared_table_cost': cleared_table_cost,
            'background_depth_clears_moved_obstacle': True,
            'sonar_target_xy': [target_x, target_y],
            'sonar_cost': 254,
            'free_lidar_preserves_sonar_obstacle': True,
            'camera_pause_invalidates_costmap': True,
            'camera_resume_restores_costmap_without_restart': True,
            'raw_costmap_frame': self.grid.header.frame_id,
        }

    def snapshot(self, label):
        result = {
            'label': label,
            'phase': self.phase,
            'scene': self.scene,
            'background_depth_mm': self.background_depth_mm,
            'table_near_cost': self.near_cost(0.967, 0.0),
            'cloud_count': self.cloud_count,
            'depth_status': self.depth_status,
            'inputs_current': self.inputs_current,
            'cloud_points_optical': self.cloud_points(self.cloud),
            'clearing_endpoints': self.cloud_points(self.clearing_endpoints),
        }
        if self.grid is not None:
            metadata = self.grid.metadata
            result['costmap'] = {
                'frame': self.grid.header.frame_id,
                'size': [metadata.size_x, metadata.size_y],
                'resolution': metadata.resolution,
                'origin': [metadata.origin.position.x, metadata.origin.position.y],
            }
            lethal_cells = []
            nearby_costs = []
            for index, value in enumerate(self.grid.data):
                cx = index % metadata.size_x
                cy = index // metadata.size_x
                wx = metadata.origin.position.x + (cx + 0.5) * metadata.resolution
                wy = metadata.origin.position.y + (cy + 0.5) * metadata.resolution
                if value == 254:
                    lethal_cells.append([wx, wy])
                if value and abs(wx - 0.967) < 0.25 and abs(wy) < 0.25:
                    nearby_costs.append([wx, wy, value])
            result['costmap']['lethal_cells'] = lethal_cells[:100]
            result['costmap']['lethal_count'] = len(lethal_cells)
            result['costmap']['nearby_nonzero_costs'] = nearby_costs
        if self.voxel_grid is not None:
            grid = self.voxel_grid
            x = math.floor((0.967 - grid.origin.x) / grid.resolutions.x)
            y = math.floor((0.0 - grid.origin.y) / grid.resolutions.y)
            columns = []
            for cy in range(max(0, y - 2), min(grid.size_y, y + 3)):
                for cx in range(max(0, x - 2), min(grid.size_x, x + 3)):
                    value = grid.data[cy * grid.size_x + cx]
                    columns.append({
                        'cell': [cx, cy],
                        'marked_bits': value >> 16,
                        'unknown_bits': (value & 65535) ^ (value >> 16),
                    })
            result['table_voxel_columns'] = columns
            marked_columns = []
            for index, value in enumerate(grid.data):
                if value >> 16:
                    cx = index % grid.size_x
                    cy = index // grid.size_x
                    marked_columns.append([
                        grid.origin.x + (cx + 0.5) * grid.resolutions.x,
                        grid.origin.y + (cy + 0.5) * grid.resolutions.y,
                        value >> 16,
                    ])
            result['marked_voxel_columns'] = marked_columns[:100]
        return result

    @staticmethod
    def cloud_points(message):
        if message is None or not message.data:
            return None
        fields = {field.name: field.offset for field in message.fields}
        if not {'x', 'y', 'z'}.issubset(fields):
            return {'error': 'missing_xyz_fields'}
        endian = '>f4' if message.is_bigendian else '<f4'
        dtype = np.dtype({
            'names': ['x', 'y', 'z'],
            'formats': [endian] * 3,
            'offsets': [fields[name] for name in ('x', 'y', 'z')],
            'itemsize': message.point_step,
        })
        points = np.frombuffer(bytes(message.data), dtype=dtype)
        return {
            'frame': message.header.frame_id,
            'count': len(points),
            'first_points': [[float(point[name]) for name in ('x', 'y', 'z')] for point in points[:12]],
        }

    def assert_no_commands(self):
        for topic in FORBIDDEN_TOPICS:
            assert self.node.count_publishers(topic) == 0, ('Unexpected command publisher', topic)
        forbidden_nodes = {
            'controller_server', 'behavior_server', 'bt_navigator', 'planner_server',
            'waypoint_follower', 'amcl', 'slam_toolbox',
        }
        assert not forbidden_nodes.intersection(self.node.get_node_names())


if __name__ == '__main__':
    main()
