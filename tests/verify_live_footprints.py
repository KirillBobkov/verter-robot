"""Read-only assertions for actual Nav2 costmaps in the synthetic test domain."""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PolygonStamped
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data

assert os.environ.get('ROS_DOMAIN_ID') == '177'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
node = rclpy.create_node('verify_nav2_geometry')
expected = [[0.175, 0.267], [-0.414, 0.267], [-0.414, -0.267], [0.175, -0.267]]
received = {}
try:
    for name in ['local_costmap', 'global_costmap']:
        node.create_subscription(
            PolygonStamped,
            '/' + name + '/published_footprint',
            lambda msg, key=name: received.update({key: msg}),
            qos_profile_sensor_data,
        )
        client = node.create_client(GetParameters, '/' + name + '/' + name + '/get_parameters')
        assert client.wait_for_service(timeout_sec=5), name
        req = GetParameters.Request()
        req.names = ['footprint', 'footprint_padding']
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done(), name
        values = future.result().values
        assert json.loads(values[0].string_value) == expected, (name, values[0])
        assert abs(values[1].double_value - 0.01) < 1e-8
    end = time.monotonic() + 8
    while len(received) < 2 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    assert len(received) == 2, received.keys()
    for name, msg in received.items():
        pts = msg.polygon.points
        assert len(pts) == 4
        edges = sorted(math.hypot(pts[i].x - pts[(i + 1) % 4].x, pts[i].y - pts[(i + 1) % 4].y) for i in range(4))
        assert all(abs(a - b) < 1e-4 for a, b in zip(edges, [0.554, 0.554, 0.609, 0.609])), (name, edges)
    print('PASS both real costmaps loaded polygon + 10 mm padding and publish expected edges')
finally:
    node.destroy_node()
    rclpy.shutdown()
