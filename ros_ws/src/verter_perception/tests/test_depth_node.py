"""Exercise adapter callbacks with real messages and geometry, without a ROS node.

No context, executor, DDS participant or network publisher is created. Only the
clock, TF lookup and final publishers are replaced, so guards exercise the same
callback code that is used by the deployed node.
"""

from array import array
from collections import OrderedDict
import json
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import patch

from geometry_msgs.msg import TransformStamped
import numpy as np
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from verter_perception.depth_obstacles import DepthObstacles


class FakeClock:
    def __init__(self):
        self.nanoseconds = 100_000_000_000

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


class Collector:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeTransforms:
    def __init__(self):
        self.calls = []
        self.available = True
        self.on_lookup = None

    def lookup_transform(self, parent, child, at):
        self.calls.append((parent, child, at.nanoseconds))
        if not self.available:
            raise TransformException('fixture has no optical transform')
        if self.on_lookup is not None:
            self.on_lookup()

        transform = TransformStamped()
        transform.transform.translation.x = -0.033
        transform.transform.translation.z = 0.65
        transform.transform.rotation.x = -0.5
        transform.transform.rotation.y = 0.5
        transform.transform.rotation.z = -0.5
        transform.transform.rotation.w = 0.5
        return transform


def harness():
    clock = FakeClock()
    subject = SimpleNamespace(
        settings={
            'image_topic': '/camera/depth/image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'points_topic': '/perception/depth/points',
            'status_topic': '/perception/depth/status',
            'base_frame': 'base_footprint',
            'stride': 1,
            'max_rate': 5.0,
            'min_depth': 0.25,
            'max_depth': 4.0,
            'min_height': -0.05,
            'max_height': 1.6,
            'self_bounds': [-0.434, 0.195, -0.287, 0.287],
            'max_frame_age': 0.5,
            'future_tolerance': 0.1,
            'status_timeout': 0.75,
        },
        clock=clock,
        monotonic=1000.0,
        get_clock=lambda: clock,
        buffer=FakeTransforms(),
        publisher=Collector(),
        status_publisher=Collector(),
        images=OrderedDict(),
        infos=OrderedDict(),
        ray_signature=None,
        rays=None,
        last_frame_stamp=None,
        last_frame_id=None,
        last_frame_received=None,
        last_published_stamp=None,
        last_published_at=None,
        last_error='waiting_for_depth_and_calibration',
        received=0,
        published=0,
        rejected=0,
        throttled=0,
        point_count=0,
    )
    methods = (
        'validate_settings', 'key', 'frame_fresh', 'reject', 'image_received',
        'info_received', 'process_pair', 'publish_status',
    )
    for name in methods:
        setattr(subject, name, MethodType(getattr(DepthObstacles, name), subject))
    subject.validate_settings()
    return subject


def image_at(nanoseconds=100_000_000_000, frame='camera_depth_optical_frame', value=1000):
    image = Image()
    image.header.stamp.sec = nanoseconds // 1_000_000_000
    image.header.stamp.nanosec = nanoseconds % 1_000_000_000
    image.header.frame_id = frame
    image.width = 3
    image.height = 3
    image.encoding = '16UC1'
    image.step = 6
    image.data = array('B', np.full((3, 3), value, dtype='<u2').tobytes())
    return image


def info_for(image):
    info = CameraInfo()
    info.header = image.header
    info.width = image.width
    info.height = image.height
    info.distortion_model = 'plumb_bob'
    info.k = [4.0, 0.0, 1.0, 0.0, 4.0, 1.0, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    return info


class DepthNodeGuardTests(unittest.TestCase):
    def setUp(self):
        self.subject = harness()
        self.monotonic = patch(
            'verter_perception.depth_obstacles.time.monotonic',
            side_effect=lambda: self.subject.monotonic,
        )
        self.monotonic.start()
        self.addCleanup(self.monotonic.stop)

    def publish_pair(self, nanoseconds=100_000_000_000):
        image = image_at(nanoseconds)
        self.subject.image_received(image)
        self.subject.info_received(info_for(image))
        return image

    def test_both_message_orders_preserve_stamp_frame_and_tf_time(self):
        for calibration_first in (False, True):
            with self.subTest(calibration_first=calibration_first):
                self.subject = harness()
                image = image_at()
                info = info_for(image)
                if calibration_first:
                    self.subject.info_received(info)
                    self.subject.image_received(image)
                else:
                    self.subject.image_received(image)
                    self.assertEqual(len(self.subject.publisher.messages), 0)
                    self.subject.info_received(info)

                cloud = self.subject.publisher.messages[0]
                self.assertEqual(cloud.header, image.header)
                self.assertEqual(cloud.width, 9)
                self.assertEqual(cloud.row_step, 9 * 12)
                self.assertEqual(
                    self.subject.buffer.calls,
                    [('base_footprint', 'camera_depth_optical_frame', 100_000_000_000)],
                )

    def test_zero_stale_future_and_empty_frame_never_enter_pending_queue(self):
        for nanoseconds, frame in (
            (0, 'camera_depth_optical_frame'),
            (99_400_000_000, 'camera_depth_optical_frame'),
            (100_200_000_000, 'camera_depth_optical_frame'),
            (100_000_000_000, ''),
        ):
            with self.subTest(nanoseconds=nanoseconds, frame=frame):
                image = image_at(nanoseconds, frame)
                self.subject.image_received(image)
                self.subject.info_received(info_for(image))

        self.assertFalse(self.subject.images)
        self.assertFalse(self.subject.publisher.messages)
        self.assertEqual(self.subject.rejected, 4)

    def test_duplicate_and_out_of_order_images_are_not_republished(self):
        self.publish_pair()
        self.subject.monotonic += 0.3
        self.publish_pair()
        self.publish_pair(99_900_000_000)
        self.assertEqual(len(self.subject.publisher.messages), 1)
        self.assertEqual(self.subject.rejected, 2)
        self.assertEqual(self.subject.last_error, 'duplicate_or_out_of_order_depth')

    def test_new_calibration_cannot_refresh_an_old_image(self):
        self.subject.image_received(image_at())
        self.subject.clock.nanoseconds += 600_000_000
        newer_image = image_at(self.subject.clock.nanoseconds)
        self.subject.info_received(info_for(newer_image))
        self.assertFalse(self.subject.publisher.messages)
        # Even late calibration with the old stamp is rejected as stale.
        self.subject.info_received(info_for(image_at()))
        self.assertFalse(self.subject.publisher.messages)

    def test_different_camera_frame_does_not_pair(self):
        self.subject.image_received(image_at())
        other_camera = image_at(frame='another_depth_optical_frame')
        self.subject.info_received(info_for(other_camera))
        self.assertFalse(self.subject.publisher.messages)
        self.assertFalse(self.subject.buffer.calls)

    def test_calibration_size_mismatch_does_not_project(self):
        image = image_at()
        info = info_for(image)
        info.width = 4
        self.subject.image_received(image)
        self.subject.info_received(info)
        self.assertFalse(self.subject.publisher.messages)
        self.assertEqual(self.subject.last_error, 'depth_calibration_size_mismatch')

    def test_missing_tf_does_not_fall_back_to_latest_or_mount_constants(self):
        self.subject.buffer.available = False
        self.publish_pair()
        self.assertFalse(self.subject.publisher.messages)
        self.assertEqual(self.subject.last_error, 'depth_transform_unavailable')
        self.assertEqual(len(self.subject.buffer.calls), 1)

    def test_late_old_pair_is_rejected_after_a_newer_cloud(self):
        self.subject.image_received(image_at(99_900_000_000))
        self.publish_pair()
        self.subject.clock.nanoseconds += 300_000_000
        self.subject.monotonic += 0.3
        self.subject.info_received(info_for(image_at(99_900_000_000)))
        self.assertEqual(len(self.subject.publisher.messages), 1)
        self.assertEqual(self.subject.last_error, 'duplicate_or_out_of_order_depth_pair')

    def test_frame_that_becomes_stale_during_processing_is_not_published(self):
        def slow_lookup():
            self.subject.clock.nanoseconds += 600_000_000

        self.subject.buffer.on_lookup = slow_lookup
        self.publish_pair()
        self.assertFalse(self.subject.publisher.messages)
        self.assertEqual(self.subject.last_error, 'depth_became_stale_during_projection')

    def test_rate_limit_drops_frames_without_later_replaying_them(self):
        self.publish_pair()
        self.subject.monotonic += 0.1
        self.subject.clock.nanoseconds += 100_000_000
        self.publish_pair(self.subject.clock.nanoseconds)
        self.assertEqual(len(self.subject.publisher.messages), 1)
        self.assertEqual(self.subject.throttled, 1)

        self.subject.monotonic += 0.2
        self.subject.clock.nanoseconds += 200_000_000
        self.subject.publish_status()
        self.assertEqual(len(self.subject.publisher.messages), 1)
        self.publish_pair(self.subject.clock.nanoseconds)
        self.assertEqual(len(self.subject.publisher.messages), 2)

    def test_no_depth_returns_emit_neither_cloud_nor_fake_clearing_rays(self):
        image = image_at(value=0)
        self.subject.image_received(image)
        self.subject.info_received(info_for(image))
        self.subject.publish_status()
        status = json.loads(self.subject.status_publisher.messages[-1].data)
        self.assertFalse(self.subject.publisher.messages)
        self.assertFalse(status['ready'])
        self.assertEqual(status['error'], 'no_valid_depth_returns')

    def test_silent_sensor_becomes_stale_without_republishing_last_cloud(self):
        self.publish_pair()
        self.subject.publish_status()
        healthy = json.loads(self.subject.status_publisher.messages[-1].data)
        self.assertTrue(healthy['ready'])
        self.assertFalse(healthy['stale'])

        self.subject.monotonic += 1.0
        self.subject.clock.nanoseconds += 1_000_000_000
        self.subject.publish_status()
        stale = json.loads(self.subject.status_publisher.messages[-1].data)
        self.assertTrue(stale['stale'])
        self.assertFalse(stale['ready'])
        self.assertEqual(len(self.subject.publisher.messages), 1)


if __name__ == '__main__':
    unittest.main()
