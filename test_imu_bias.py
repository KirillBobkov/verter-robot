#!/usr/bin/env python3

import statistics
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class GyroAnalyzer(Node):
    def __init__(self):
        super().__init__('gyro_analyzer')

        self.values = []

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE

        self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            qos
        )

        self.timer = self.create_timer(5.0, self.print_stats)

    def imu_callback(self, msg: Imu):
        self.values.append(msg.angular_velocity.z)

        # Ограничиваем буфер примерно последними 6000 измерениями
        if len(self.values) > 6000:
            self.values.pop(0)

    def print_stats(self):
        if len(self.values) < 10:
            return

        mean = statistics.fmean(self.values)
        stddev = statistics.pstdev(self.values)
        minimum = min(self.values)
        maximum = max(self.values)

        self.get_logger().info(
            f'samples={len(self.values)} '
            f'mean={mean:+.8f} rad/s '
            f'std={stddev:.8f} rad/s '
            f'min={minimum:+.8f} '
            f'max={maximum:+.8f}'
        )


def main():
    rclpy.init()
    node = GyroAnalyzer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()