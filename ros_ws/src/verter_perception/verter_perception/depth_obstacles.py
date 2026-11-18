"""Publish calibrated depth observations without accessing any motion interface."""

from array import array
from collections import OrderedDict
from copy import deepcopy
import json
import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .depth_geometry import decode_depth, filter_cloud, project_depth, quaternion_matrix, sampled_rays


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class DepthObstacles(Node):
    def __init__(self):
        super().__init__('depth_obstacles')
        defaults = {
            'image_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'points_topic': '/perception/depth/points',
            'status_topic': '/perception/depth/status',
            'base_frame': 'base_footprint',
            'stride': 4,
            'max_rate': 5.0,
            'min_depth': 0.25,
            'max_depth': 4.0,
            'min_height': -0.05,
            'max_height': 1.6,
            'self_bounds': [-0.434, 0.195, -0.287, 0.287],
            'max_frame_age': 0.5,
            'future_tolerance': 0.1,
            'status_timeout': 0.75,
        }
        self.settings = {}
        for name, value in defaults.items():
            self.settings[name] = self.declare_parameter(name, value).value
        self.validate_settings()

        self.buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(
            PointCloud2, self.settings['points_topic'], qos_profile_sensor_data
        )
        self.status_publisher = self.create_publisher(String, self.settings['status_topic'], 1)
        self.image_subscription = self.create_subscription(
            Image, self.settings['image_topic'], self.image_received, qos_profile_sensor_data
        )
        self.info_subscription = self.create_subscription(
            CameraInfo, self.settings['camera_info_topic'], self.info_received, qos_profile_sensor_data
        )
        self.images = OrderedDict()
        self.infos = OrderedDict()
        self.ray_signature = None
        self.rays = None
        self.last_frame_stamp = None
        self.last_frame_id = None
        self.last_frame_received = None
        self.last_published_stamp = None
        self.last_published_at = None
        self.last_error = 'waiting_for_depth_and_calibration'
        self.received = 0
        self.published = 0
        self.rejected = 0
        self.throttled = 0
        self.point_count = 0
        self.status_timer = self.create_timer(1.0, self.publish_status)

    def validate_settings(self):
        for name in ('image_topic', 'camera_info_topic', 'points_topic', 'status_topic', 'base_frame'):
            if not isinstance(self.settings[name], str) or not self.settings[name]:
                raise ValueError('invalid_parameter_' + name)
        if self.settings['stride'] < 1:
            raise ValueError('invalid_parameter_stride')
        for name in ('max_rate', 'min_depth', 'max_depth', 'max_frame_age', 'status_timeout'):
            if not math.isfinite(self.settings[name]) or self.settings[name] <= 0:
                raise ValueError('invalid_parameter_' + name)
        if not math.isfinite(self.settings['future_tolerance']) or self.settings['future_tolerance'] < 0:
            raise ValueError('invalid_parameter_future_tolerance')
        if self.settings['min_depth'] >= self.settings['max_depth']:
            raise ValueError('invalid_parameter_depth_limits')
        # Reuse the geometric validator for height and self-mask parameters.
        filter_cloud(
            np.empty((0, 3), dtype=np.float32), np.eye(3), np.zeros(3),
            self.settings['min_height'], self.settings['max_height'], self.settings['self_bounds'],
        )

    def reject(self, reason):
        self.rejected += 1
        self.last_error = reason

    def key(self, message):
        return stamp_ns(message.header.stamp), message.header.frame_id

    def frame_fresh(self, nanoseconds):
        age = (self.get_clock().now().nanoseconds - nanoseconds) * 1.0e-9
        return nanoseconds > 0 and -self.settings['future_tolerance'] <= age <= self.settings['max_frame_age']

    def image_received(self, image):
        self.received += 1
        key = self.key(image)
        self.last_frame_stamp = key[0]
        self.last_frame_id = key[1]
        self.last_frame_received = time.monotonic()
        if not key[1] or not self.frame_fresh(key[0]):
            self.reject('invalid_or_stale_depth_header')
            return
        if self.last_published_stamp is not None and key[0] <= self.last_published_stamp:
            self.reject('duplicate_or_out_of_order_depth')
            return
        self.images[key] = image
        self.process_pair(key)
        while len(self.images) > 5:
            self.images.popitem(last=False)
            self.reject('missing_matching_camera_info')

    def info_received(self, info):
        key = self.key(info)
        if not key[1] or not self.frame_fresh(key[0]):
            return
        self.infos[key] = info
        self.process_pair(key)
        while len(self.infos) > 5:
            self.infos.popitem(last=False)

    def process_pair(self, key):
        if key not in self.images or key not in self.infos:
            return
        image = self.images.pop(key)
        info = self.infos.pop(key)
        if self.last_published_stamp is not None and key[0] <= self.last_published_stamp:
            self.reject('duplicate_or_out_of_order_depth_pair')
            return
        now = time.monotonic()
        if self.last_published_at is not None:
            if now - self.last_published_at < 1.0 / self.settings['max_rate']:
                self.throttled += 1
                return
        if not self.frame_fresh(key[0]):
            self.reject('stale_depth_pair')
            return
        if image.width != info.width or image.height != info.height:
            self.reject('depth_calibration_size_mismatch')
            return
        if info.binning_x not in (0, 1) or info.binning_y not in (0, 1):
            self.reject('unsupported_binned_depth_calibration')
            return
        if info.roi.x_offset or info.roi.y_offset:
            self.reject('unsupported_depth_calibration_roi')
            return

        try:
            signature = (
                info.width, info.height, tuple(info.k), tuple(info.d), info.distortion_model,
                self.settings['stride'],
            )
            if signature != self.ray_signature:
                self.rays = sampled_rays(*signature)
                self.ray_signature = signature
            depth = decode_depth(
                image.data, image.width, image.height, image.step, image.encoding, image.is_bigendian
            )
            rows, columns, rays = self.rays
            points = project_depth(
                depth, rows, columns, rays, self.settings['min_depth'], self.settings['max_depth']
            )
            transform = self.buffer.lookup_transform(
                self.settings['base_frame'], image.header.frame_id, Time.from_msg(image.header.stamp)
            )
            position = transform.transform.translation
            orientation = transform.transform.rotation
            rotation = quaternion_matrix(
                [orientation.x, orientation.y, orientation.z, orientation.w]
            )
            points = filter_cloud(
                points, rotation, [position.x, position.y, position.z],
                self.settings['min_height'], self.settings['max_height'], self.settings['self_bounds'],
            )
        except TransformException:
            self.reject('depth_transform_unavailable')
            return
        except (ValueError, TypeError, OverflowError) as error:
            self.reject(str(error))
            return

        if not len(points):
            self.point_count = 0
            self.reject('no_valid_depth_returns')
            return
        if not self.frame_fresh(key[0]):
            self.reject('depth_became_stale_during_projection')
            return

        cloud = PointCloud2()
        cloud.header = deepcopy(image.header)
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = array('B', points.astype('<f4', copy=False).tobytes())
        cloud.is_dense = True
        self.publisher.publish(cloud)
        self.published += 1
        self.point_count = len(points)
        self.last_published_at = time.monotonic()
        self.last_published_stamp = key[0]
        self.last_error = None

    def publish_status(self):
        now = time.monotonic()
        frame_age = None
        header_age = None
        output_age = None
        if self.last_frame_received is not None:
            frame_age = max(0.0, now - self.last_frame_received)
        if self.last_frame_stamp is not None:
            header_age = (self.get_clock().now().nanoseconds - self.last_frame_stamp) * 1.0e-9
        if self.last_published_at is not None:
            output_age = max(0.0, now - self.last_published_at)
        stale = output_age is None or output_age > self.settings['status_timeout']
        status = {
            'ready': not stale and self.last_error is None,
            'stale': stale,
            'error': self.last_error,
            'frame_age_s': frame_age,
            'header_age_s': header_age,
            'output_age_s': output_age,
            'point_count': self.point_count,
            'received': self.received,
            'published': self.published,
            'rejected': self.rejected,
            'throttled': self.throttled,
            'frame_id': self.last_frame_id,
            'filter_frame': self.settings['base_frame'],
            'points_topic': self.settings['points_topic'],
        }
        self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DepthObstacles()
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
