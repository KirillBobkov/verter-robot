"""Publish validated sonar hits for Nav2, without issuing robot commands."""

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray, String

from .sonar_logic import (
    SENSOR_FRAMES,
    SENSOR_NAMES,
    SonarMonitor,
    validate_settings,
)


class UltrasonicRanges(Node):
    def __init__(self):
        super().__init__('ultrasonic_ranges')
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 2.0)
        self.declare_parameter('field_of_view', 0.26)
        self.declare_parameter('output_rate_hz', 10.0)

        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.field_of_view = float(self.get_parameter('field_of_view').value)
        output_rate_hz = float(self.get_parameter('output_rate_hz').value)
        validate_settings(
            self.min_range,
            self.max_range,
            self.field_of_view,
            output_rate_hz,
        )
        self.monitor = SonarMonitor(
            self.min_range,
            self.max_range,
            output_rate_hz,
        )

        self.range_publishers = [
            self.create_publisher(
                Range,
                '/perception/ultrasonic/' + name,
                qos_profile_sensor_data,
            )
            for name in SENSOR_NAMES
        ]
        self.status_publisher = self.create_publisher(
            String,
            '/perception/ultrasonic/status',
            10,
        )
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/ultrasonics',
            self.receive_array,
            qos_profile_sensor_data,
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().warning(
            'Sonar indexing and field_of_view are provisional. Firmware has no '
            'per-sensor timestamps: Range stamps describe array receipt only. '
            'Invalid echoes are ignored, never published as free space.'
        )

    def receive_array(self, message):
        received_at = time.monotonic()
        receipt_stamp = self.get_clock().now().to_msg()
        sample = self.monitor.observe(message.data, received_at)
        if sample is None:
            return

        for index, value in enumerate(sample.values):
            if value is None:
                continue

            reading = Range()
            reading.header.stamp = receipt_stamp
            reading.header.frame_id = SENSOR_FRAMES[index]
            reading.radiation_type = Range.ULTRASOUND
            reading.field_of_view = self.field_of_view
            reading.min_range = self.min_range
            reading.max_range = self.max_range
            reading.range = value
            self.range_publishers[index].publish(reading)

    def publish_status(self):
        status = self.monitor.status(time.monotonic())
        status['configured_min_range_m'] = self.min_range
        status['configured_max_range_m'] = self.max_range
        status['configured_field_of_view_rad'] = self.field_of_view
        message = String()
        message.data = json.dumps(status, ensure_ascii=False, allow_nan=False)
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UltrasonicRanges()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
