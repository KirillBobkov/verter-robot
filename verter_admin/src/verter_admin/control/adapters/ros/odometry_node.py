#!/usr/bin/env python3
"""ROS 2 adapter: /wheel_encoders → /odom + TF (odom → base_footprint).

Thin wrapper around ComputeOdometry.  All odometry math lives in the
application/domain layers; this node handles only ROS message I/O.

Published topics
  /odom  (nav_msgs/Odometry, 50 Hz)

Broadcast TF
  odom → base_footprint  (50 Hz, disabled with parameter publish_tf:=false)

Subscribed topics
  /wheel_encoders  (std_msgs/Int64MultiArray, BEST_EFFORT)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
import tf2_ros

from verter_admin.control.application.compute_odometry import ComputeOdometry
from verter_admin.control.domain.odometry_policy import OdometryParameters

# Pose and twist covariance matrices (diagonal 6×6, row-major).
# encoder_fresh=True  → small uncertainty (trust odometry).
# encoder_fresh=False → large uncertainty (EKF falls back to scan matching).
_SMALL = 1e-4
_LARGE = 0.5
_INF   = 1e6  # unused degrees of freedom (z, roll, pitch)


def _pose_cov(fresh: bool) -> list[float]:
    s = _SMALL if fresh else _LARGE
    return [
        s,    0,    0,    0,    0,    0,
        0,    s,    0,    0,    0,    0,
        0,    0, _INF,    0,    0,    0,
        0,    0,    0, _INF,    0,    0,
        0,    0,    0,    0, _INF,    0,
        0,    0,    0,    0,    0,    s,
    ]


def _twist_cov(fresh: bool) -> list[float]:
    s = _SMALL if fresh else _LARGE
    return [
        s,    0,    0,    0,    0,    0,
        0, _INF,    0,    0,    0,    0,
        0,    0, _INF,    0,    0,    0,
        0,    0,    0, _INF,    0,    0,
        0,    0,    0,    0, _INF,    0,
        0,    0,    0,    0,    0,    s,
    ]


class OdometryNode(Node):

    def __init__(self) -> None:
        super().__init__('odometry_node')

        self.declare_parameter('publish_tf', True)
        self._publish_tf: bool = self.get_parameter('publish_tf').value

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._odometry = ComputeOdometry(OdometryParameters(), now_sec)

        # Match the BEST_EFFORT QoS used by the ESP32 publisher.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._enc_sub = self.create_subscription(
            Int64MultiArray, '/wheel_encoders', self._on_encoders, sensor_qos
        )

        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        if self._publish_tf:
            self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 50 Hz publication timer
        self._timer = self.create_timer(0.02, self._on_tick)

    # ------------------------------------------------------------------

    def _on_encoders(self, msg: Int64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        now_sec = self.get_clock().now().nanoseconds / 1e9
        self._odometry.on_encoder(int(msg.data[0]), int(msg.data[1]), now_sec)

    def _on_tick(self) -> None:
        now = self.get_clock().now()
        self._odometry.on_tick(now.nanoseconds / 1e9)
        self._publish(now)

    def _publish(self, now: rclpy.time.Time) -> None:
        snap = self._odometry.snapshot()

        # Yaw → quaternion (2-D robot: roll=0, pitch=0)
        half = snap.theta * 0.5
        qw, qx, qy, qz = math.cos(half), 0.0, 0.0, math.sin(half)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'

        odom.pose.pose.position.x    = snap.x
        odom.pose.pose.position.y    = snap.y
        odom.pose.pose.orientation.w = qw
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz

        odom.twist.twist.linear.x  = snap.vx
        odom.twist.twist.angular.z = snap.vth

        odom.pose.covariance  = _pose_cov(snap.encoder_fresh)
        odom.twist.covariance = _twist_cov(snap.encoder_fresh)

        self._odom_pub.publish(odom)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp       = now.to_msg()
            tf.header.frame_id    = 'odom'
            tf.child_frame_id     = 'base_footprint'
            tf.transform.translation.x = snap.x
            tf.transform.translation.y = snap.y
            tf.transform.rotation.w    = qw
            tf.transform.rotation.x    = qx
            tf.transform.rotation.y    = qy
            tf.transform.rotation.z    = qz
            self._tf_broadcaster.sendTransform(tf)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
