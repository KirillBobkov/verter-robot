
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class ScanTfTiming(Node):
    def __init__(self):
        super().__init__('scan_tf_timing')

        self.latest_tf_ns = None

        tf_qos = QoSProfile(depth=1)
        tf_qos.reliability = ReliabilityPolicy.RELIABLE
        tf_qos.durability = DurabilityPolicy.VOLATILE

        scan_qos = QoSProfile(depth=1)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        scan_qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_qos,
        )

        self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            scan_qos,
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip('/')
            child = transform.child_frame_id.lstrip('/')

            if parent == 'odom' and child == 'base_footprint':
                self.latest_tf_ns = stamp_ns(transform.header.stamp)

    def scan_callback(self, msg):
        if self.latest_tf_ns is None:
            print('Ожидание odom -> base_footprint')
            return

        now_ns = self.get_clock().now().nanoseconds
        scan_ns = stamp_ns(msg.header.stamp)

        scan_age_ms = (now_ns - scan_ns) / 1e6
        tf_age_ms = (now_ns - self.latest_tf_ns) / 1e6
        coverage_ms = (self.latest_tf_ns - scan_ns) / 1e6

        status = (
            'OK: TF новее скана'
            if coverage_ms >= 0
            else 'ERROR: TF старее скана'
        )

        print(
            f'scan_age={scan_age_ms:8.3f} ms | '
            f'tf_age={tf_age_ms:7.3f} ms | '
            f'TF-scan={coverage_ms:+8.3f} ms | '
            f'{status}'
        )


rclpy.init()
node = ScanTfTiming()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()