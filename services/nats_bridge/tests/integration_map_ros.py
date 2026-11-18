"""Synthetic map/TF on loopback-only ROS domain 87. Never publishes velocity or goals."""

import math
from concurrent.futures import ThreadPoolExecutor
from tempfile import TemporaryDirectory
from robot_nats.map_save import serialize_graph
import os
from pathlib import Path
import time

from robot_nats.map_observer import MapObserver


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('This test requires isolated ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from slam_toolbox.srv import SerializePoseGraph
    from geometry_msgs.msg import TransformStamped
    from tf2_msgs.msg import TFMessage

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'synthetic_map_view_test', context=context, enable_rosout=False, start_parameter_services=False
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    assert context.get_domain_id() == 87
    mode = {'actual': 'mapping', 'pid': 123, 'stale': False}
    observer = MapObserver(
        {'robot_id': 'isolated_test', 'project_root': str(Path(__file__).resolve().parents[3])}, lambda: dict(mode)
    )
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    maps = node.create_publisher(OccupancyGrid, '/map', qos)
    transforms = node.create_publisher(TFMessage, '/tf', 10)
    scans = node.create_publisher(LaserScan, '/scan', 10)
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.header.stamp = node.get_clock().now().to_msg()
    message.info.width = 3
    message.info.height = 2
    message.info.resolution = 0.05
    message.info.origin.position.x = -2.0
    message.info.origin.position.y = 3.0
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, 100, 25, 50, 75]
    maps.publish(message)  # Publish exactly once: late subscribers must use transient-local durability.

    def tick_tf():
        values = []
        stamp = node.get_clock().now().to_msg()
        for parent, child, x, y, angle in [('map', 'odom', 1, 2, 0.3), ('odom', 'base_footprint', 2, -0.5, 0.2)]:
            value = TransformStamped()
            value.header.frame_id = parent
            value.child_frame_id = child
            value.header.stamp = stamp
            value.transform.translation.x = float(x)
            value.transform.translation.y = float(y)
            value.transform.rotation.z = math.sin(angle / 2)
            value.transform.rotation.w = math.cos(angle / 2)
            values.append(value)
        transforms.publish(TFMessage(transforms=values))
        scan = LaserScan()
        scan.header.frame_id = 'base_footprint'
        scan.header.stamp = stamp
        scan.angle_min = 0.0
        scan.angle_increment = 0.01
        scan.range_min = 0.1
        scan.range_max = 10.0
        scan.ranges = [1.0, float('inf')]
        scans.publish(scan)
        executor.spin_once(timeout_sec=0.02)

    def wait(predicate):
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            tick_tf()
            if predicate():
                return
        raise RuntimeError('Synthetic map/TF did not become ready: ' + str(observer.store.metadata()))

    try:

        def serialize(request, response):
            base = Path(request.filename)
            base.with_suffix('.posegraph').write_bytes(b'synthetic graph')
            base.with_suffix('.data').write_bytes(b'synthetic data')
            response.result = 0
            return response

        serializer = node.create_service(SerializePoseGraph, '/slam_toolbox/serialize_map', serialize)
        with TemporaryDirectory() as temporary, ThreadPoolExecutor(max_workers=1) as pool:
            base = Path(temporary) / 'map'
            future = pool.submit(serialize_graph, base)
            deadline = time.monotonic() + 10
            while not future.done() and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            future.result(timeout=1)
            assert base.with_suffix('.posegraph').is_file() and base.with_suffix('.data').is_file()
        node.destroy_service(serializer)
        observer.start()
        wait(
            lambda: (
                observer.store.metadata()['available']
                and observer.store.pose()['valid']
                and observer.store.scan()['valid']
            )
        )
        wait(lambda: observer.store.scan()['valid'])
        cloud = observer.store.scan()
        pose = observer.store.pose()
        assert abs(cloud['points'][0][0] - (pose['x'] + math.cos(0.5))) < 0.0002
        assert abs(cloud['points'][0][1] - (pose['y'] + math.sin(0.5))) < 0.0002
        assert abs(pose['x'] - (1 + 2 * math.cos(0.3) + 0.5 * math.sin(0.3))) < 0.00001
        assert abs(pose['y'] - (2 + 2 * math.sin(0.3) - 0.5 * math.cos(0.3))) < 0.00001
        assert abs(pose['yaw'] - 0.5) < 0.00001
        first = observer.store.metadata()['map_id']
        mode.update(actual='navigation', pid=124)
        wait(
            lambda: (
                observer.store.metadata()['available']
                and observer.store.metadata()['map_id'] != first
                and observer.store.pose()['valid']
                and observer.store.scan()['valid']
            )
        )
        assert observer.store.metadata()['map_id'].endswith(':124')
        time.sleep(1.4)  # No new TF messages: last known position must expire.
        assert not observer.store.scan()['valid']
        assert 'points' not in observer.store.scan()
        assert not observer.store.pose()['valid']
        assert 'x' not in observer.store.pose()
        assert not observer.store.metadata()['stale']  # Retained navigation map remains valid.
        print(
            'PASS isolated ROS: retained map, composed TF coordinates, mode/PID resubscription, stale pose hidden; no control publishers'
        )
    finally:
        observer.close()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


if __name__ == '__main__':
    main()
