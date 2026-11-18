"""ISOLATED TEST ONLY. No hardware access; refuses the normal robot domain."""

import os
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '177'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
rclpy.init()
node = Node('synthetic_verter_sensors')
scan = node.create_publisher(LaserScan, '/scan', 10)
odom = node.create_publisher(Odometry, '/odometry/filtered', 10)
imu = node.create_publisher(Imu, '/imu/data', 10)
pose = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
tf = TransformBroadcaster(node)
static = StaticTransformBroadcaster(node)
t = TransformStamped()
t.header.frame_id = 'base_footprint'
t.child_frame_id = 'lidar_link'
t.transform.rotation.w = 1.0
static.sendTransform(t)
count = 0


def tick():
    global count
    count += 1
    stamp = node.get_clock().now().to_msg()
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = 'odom'
    t.child_frame_id = 'base_footprint'
    t.transform.rotation.w = 1.0
    tf.sendTransform(t)
    o = Odometry()
    o.header.stamp = stamp
    o.header.frame_id = 'odom'
    o.child_frame_id = 'base_footprint'
    o.pose.pose.orientation.w = 1.0
    odom.publish(o)
    i = Imu()
    i.header.stamp = stamp
    i.header.frame_id = 'imu_link'
    i.orientation.w = 1.0
    imu.publish(i)
    if count % 5 == 0:
        s = LaserScan()
        s.header.stamp = stamp
        s.header.frame_id = 'lidar_link'
        s.angle_min = -math.pi
        s.angle_increment = math.pi / 180
        s.angle_max = s.angle_min + 359 * s.angle_increment
        s.range_min = 0.1
        s.range_max = 8.0
        s.scan_time = 0.1
        s.ranges = [
            3.0
            / max(
                abs(math.cos(s.angle_min + k * s.angle_increment)), abs(math.sin(s.angle_min + k * s.angle_increment))
            )
            for k in range(360)
        ]
        scan.publish(s)
    if count % 100 == 0:
        p = PoseWithCovarianceStamped()
        p.header.stamp = stamp
        p.header.frame_id = 'map'
        p.pose.pose.orientation.w = 1.0
        p.pose.covariance[0] = 0.25
        p.pose.covariance[7] = 0.25
        p.pose.covariance[35] = 0.1
        pose.publish(p)


node.create_timer(0.02, tick)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
