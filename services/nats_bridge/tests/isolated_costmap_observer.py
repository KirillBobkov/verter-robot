"""Synthetic raw costmaps and delayed TF only, restricted to ROS domain 230."""

import os

if os.environ.get('ROS_DOMAIN_ID') != '230' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise SystemExit('Requires isolated ROS_DOMAIN_ID=230 and ROS_LOCALHOST_ONLY=1')

from array import array
import base64
import json
import math
from pathlib import Path
import sys
import time
import zlib

import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.msg import Costmap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from robot_nats.costmap_observer import CostmapObserver


def main():
    root = Path(sys.argv[1]).resolve()
    assert str(root).startswith('/tmp/verter-costmap-web-stage-')
    rclpy.init()
    node = rclpy.create_node('synthetic_costmap_viewer_inputs')
    mode = {'actual': 'navigation', 'pid': 222, 'stale': False}
    map_metadata = {
        'map_id': 'synthetic-map', 'saved_map': 'floor', 'available': True,
        'stale': False, 'mode': 'navigation', 'mode_pid': 222,
    }
    observer = CostmapObserver({'robot_id': 'test', 'ros_enabled': True}, lambda: dict(mode), lambda: dict(map_metadata))
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    publishers = {
        kind: node.create_publisher(Costmap, f'/{kind}_costmap/costmap_raw', qos)
        for kind in ('local', 'global')
    }
    transforms = TransformBroadcaster(node)
    pending = []
    enabled = [True]
    stamps = set()

    def publish_transform(stamp):
        message = TransformStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'map'
        message.child_frame_id = 'odom'
        message.transform.translation.x = 2.0
        message.transform.translation.y = 3.0
        message.transform.rotation.z = math.sqrt(0.5)
        message.transform.rotation.w = math.sqrt(0.5)
        transforms.sendTransform(message)

    def raw_inputs():
        if not enabled[0]:
            return
        stamp = node.get_clock().now().to_msg()
        stamps.add(stamp.sec + stamp.nanosec * 1e-9)
        for kind, publisher in publishers.items():
            message = Costmap()
            message.header.stamp = stamp
            message.header.frame_id = 'odom' if kind == 'local' else 'map'
            message.metadata.size_x = 3
            message.metadata.size_y = 2
            message.metadata.resolution = 0.05
            message.metadata.origin.position.x = -1.0
            message.metadata.origin.position.y = -2.0
            message.metadata.origin.orientation.z = math.sin(0.1)
            message.metadata.origin.orientation.w = math.cos(0.1)
            # metadata.update_time deliberately stays zero, as in Humble.
            message.data = array('B', [0, 253, 254, 255, 30, 150])
            publisher.publish(message)
        # The transform arrives later than the grid. Retrying at the full
        # 5 Hz source interval can continually select an even newer grid.
        pending.append((time.monotonic() + 0.12, stamp))

    def delayed_transforms():
        while pending and pending[0][0] <= time.monotonic():
            _, stamp = pending.pop(0)
            publish_transform(stamp)

    node.create_timer(0.2, raw_inputs)
    node.create_timer(0.01, delayed_transforms)

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def until(predicate, seconds, reason):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            rclpy.spin_once(node, timeout_sec=0.01)
        raise AssertionError(reason)

    def both_ready():
        return all(
            observer.store.metadata(kind)['available'] and not observer.store.metadata(kind)['stale']
            for kind in ('local', 'global')
        )

    report = {'ok': False, 'domain': 230, 'localhost_only': True, 'control_publishers': {}}
    try:
        spin_for(1)
        assert node.get_node_names() == [node.get_name()], node.get_node_names()
        observer.start()
        until(both_ready, 8, 'Raw grids or delayed TF were not observed')
        first = observer.store.metadata('local')
        until(lambda: observer.store.metadata('local').get('stamp', 0) > first['stamp'] + 0.5, 5,
              'Delayed TF starved subsequent local grids')
        metadata = observer.store.metadata('local')
        assert metadata['stamp'] in stamps
        assert abs(metadata['origin']['x'] - 4.0) < 1e-6, metadata
        assert abs(metadata['origin']['y'] - 2.0) < 1e-6, metadata
        assert abs(metadata['origin']['yaw'] - (math.pi / 2 + 0.2)) < 1e-6, metadata
        chunk = observer.store.query({
            'part': 'chunk', 'kind': 'local', 'costmap_id': metadata['costmap_id'],
            'revision': metadata['revision'], 'index': 0,
        })
        assert zlib.decompress(base64.b64decode(chunk['data_b64'])) == bytes([0, 253, 254, 255, 30, 150])

        enabled[0] = False
        spin_for(0.2)
        mode['actual'] = 'idle'
        until(lambda: observer.store.state()['mode'] == 'idle', 2, 'Inactive scope was not cleared')
        assert not observer.store.metadata('local')['available']
        mode['actual'] = 'navigation'
        spin_for(0.4)
        assert not observer.store.metadata('local')['available'], 'Retained pre-session grid was accepted'
        try:
            observer.store.query({
                'part': 'chunk', 'kind': 'local', 'costmap_id': metadata['costmap_id'],
                'revision': metadata['revision'], 'index': 0,
            })
        except ValueError:
            pass
        else:
            raise AssertionError('Old session chunks remained readable')
        enabled[0] = True
        until(both_ready, 5, 'New session did not recover')
        assert observer.store.metadata('local')['costmap_id'] != metadata['costmap_id']
        enabled[0] = False
        until(lambda: all(observer.store.metadata(kind)['stale'] for kind in ('local', 'global')), 5,
              'Paused raw streams did not expire')

        for topic in ('/cmd_vel', '/initialpose', '/goal_pose', '/set_pose'):
            report['control_publishers'][topic] = node.count_publishers(topic)
            assert report['control_publishers'][topic] == 0
        report.update({
            'ok': True, 'raw_uint8_roundtrip': True, 'map_normalized_origin': metadata['origin'],
            'original_header_stamp_preserved': True, 'delayed_tf_recovers_without_starvation': True,
            'scope_restart_rejects_retained_grid_and_chunks': True, 'paused_streams_expire': True,
        })
    finally:
        observer.close()
        node.destroy_node()
        rclpy.shutdown()
        (root / 'observer-smoke.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
