"""Orbbec capture + ROS publisher. No motion publishers or subscriptions."""

from array import array
import traceback
import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from scipy.spatial.transform import Rotation
from pyorbbecsdk import Context, Pipeline, Config, OBSensorType, OBFormat


def bgr_frame(frame):
    w, h, fmt = frame.get_width(), frame.get_height(), frame.get_format()
    raw = np.frombuffer(frame.get_data(), np.uint8)
    if fmt == OBFormat.MJPG:
        return cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if fmt == OBFormat.RGB:
        return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if fmt == OBFormat.BGR:
        return raw.reshape(h, w, 3).copy()
    if fmt == OBFormat.YUYV:
        return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    raise ValueError('Unsupported color format: ' + str(fmt))


def camera_info(profile, frame_id, intr, dist):
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width, info.height = profile.get_width(), profile.get_height()
    info.distortion_model = 'rational_polynomial'
    info.d = [float(getattr(dist, k)) for k in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')]
    info.k = [float(intr.fx), 0.0, float(intr.cx), 0.0, float(intr.fy), float(intr.cy), 0.0, 0.0, 1.0]
    info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    info.p = [float(intr.fx), 0.0, float(intr.cx), 0.0, 0.0, float(intr.fy), float(intr.cy), 0.0, 0.0, 0.0, 1.0, 0.0]
    if intr.width != info.width or intr.height != info.height:
        raise ValueError('Calibration size differs from stream profile')
    return info


class Capture(Node):
    def __init__(self, config):
        super().__init__('verter_camera')
        self.config = config
        self.lock = threading.Lock()
        self.latest = {}
        self.metadata = {'state': 'starting', 'serial': config['serial']}
        self.stop_event = threading.Event()
        self.tf = StaticTransformBroadcaster(self)
        self.pubs = {
            s: self.create_publisher(Image, f'/camera/{s}/image_raw', qos_profile_sensor_data)
            for s in ('color', 'depth')
        }
        self.infos = {
            s: self.create_publisher(CameraInfo, f'/camera/{s}/camera_info', qos_profile_sensor_data)
            for s in ('color', 'depth')
        }
        self.thread = threading.Thread(target=self.run, name='orbbec-capture', daemon=True)
        self.thread.start()

    def status(self):
        with self.lock:
            result = dict(self.metadata)
            result['streams'] = {
                s: {
                    'width': v[0].shape[1],
                    'height': v[0].shape[0],
                    'sample_at': v[1],
                    'age_s': max(0.0, time.time() - v[1]),
                    'frames': v[2],
                }
                for s, v in self.latest.items()
            }
        result['stale'] = any(
            s not in result['streams'] or result['streams'][s]['age_s'] > 3 for s in ('color', 'depth')
        )
        return result

    def image(self, stream):
        with self.lock:
            item = self.latest.get(stream)
            if item is None or time.time() - item[1] > 3:
                return None
            image, sample_at, seq = item
        if stream == 'depth':
            # Display only: metric ROS depth is never colorized or rescaled.
            valid = image > 0
            image = cv2.applyColorMap(np.clip(image / 5000 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            image[~valid] = 0
        return image, sample_at, seq

    def transforms(self, params):
        # SDK: p_color = R * p_depth + t(mm). TF parent depth -> child color
        # needs the inverse coordinate mapping (pose of color in depth).
        rotation = np.asarray(params.transform.rot, dtype=float).reshape(3, 3)
        translation = np.asarray(params.transform.transform, dtype=float).reshape(3) / 1000.0
        definitions = [
            ('base_footprint', 'camera_link', self.config['mount_xyz'], [0.0, 0.0, 0.0, 1.0]),
            (
                'camera_link',
                'camera_depth_optical_frame',
                [0.0, 0.0, 0.0],
                Rotation.from_euler('xyz', [-np.pi / 2, 0, -np.pi / 2]).as_quat(),
            ),
            (
                'camera_depth_optical_frame',
                'camera_color_optical_frame',
                -rotation.T @ translation,
                Rotation.from_matrix(rotation.T).as_quat(),
            ),
        ]
        transforms = []
        for parent, child, xyz, quat in definitions:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, xyz)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(
                float, quat
            )
            transforms.append(t)
        self.tf.sendTransform(transforms)

    def publish_frame(self, stream, image, info, at):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = info.header.frame_id
        msg.height, msg.width = image.shape[:2]
        msg.encoding = 'bgr8' if stream == 'color' else '16UC1'
        msg.is_bigendian = False
        msg.step = msg.width * (3 if stream == 'color' else 2)
        msg.data = array('B', image.tobytes())
        info.header.stamp = msg.header.stamp
        self.pubs[stream].publish(msg)
        self.infos[stream].publish(info)
        with self.lock:
            seq = self.latest.get(stream, (None, None, 0))[2] + 1
            self.latest[stream] = (image, at, seq)

    def run(self):
        context = Context()
        while not self.stop_event.is_set():
            pipe = None
            try:
                device = context.query_devices().get_device_by_serial_number(self.config['serial'])
                identity = device.get_device_info()
                pipe = Pipeline(device)
                cfg = Config()
                profiles = {}
                for stream, kind, fmt in [
                    ('color', OBSensorType.COLOR_SENSOR, OBFormat.MJPG),
                    ('depth', OBSensorType.DEPTH_SENSOR, OBFormat.Y16),
                ]:
                    requested = self.config[stream]
                    profile = pipe.get_stream_profile_list(kind).get_video_stream_profile(
                        requested['width'], requested['height'], fmt, requested['fps']
                    )
                    cfg.enable_stream(profile)
                    profiles[stream] = profile
                pipe.start(cfg)
                params = pipe.get_camera_param()
                info = {
                    s: camera_info(
                        p,
                        f'camera_{s}_optical_frame',
                        getattr(params, 'rgb_intrinsic' if s == 'color' else 'depth_intrinsic'),
                        getattr(params, 'rgb_distortion' if s == 'color' else 'depth_distortion'),
                    )
                    for s, p in profiles.items()
                }
                self.transforms(params)
                with self.lock:
                    self.metadata = {
                        'state': 'streaming',
                        'serial': self.config['serial'],
                        'device': str(identity),
                        'error': None,
                    }
                last_frame = time.monotonic()
                while not self.stop_event.is_set():
                    frames = pipe.wait_for_frames(100)
                    if frames is None:
                        if time.monotonic() - last_frame > 8:
                            raise TimeoutError('No camera frames for 8 seconds')
                        continue
                    color, depth = frames.get_color_frame(), frames.get_depth_frame()
                    now = time.time()
                    if color:
                        rgb = bgr_frame(color)
                        if rgb is not None:
                            self.publish_frame('color', rgb, info['color'], now)
                    if depth:
                        raw = np.frombuffer(depth.get_data(), np.uint16).reshape(depth.get_height(), depth.get_width())
                        mm = np.clip(np.rint(raw.astype(np.float32) * depth.get_depth_scale()), 0, 65535).astype('<u2')
                        self.publish_frame('depth', mm, info['depth'], now)
                    if color or depth:
                        last_frame = time.monotonic()
            except Exception as exc:
                with self.lock:
                    self.metadata.update(state='disconnected', error=str(exc))
                self.get_logger().error(traceback.format_exc())
            finally:
                if pipe:
                    try:
                        pipe.stop()
                    except Exception:
                        pass
            self.stop_event.wait(3)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=10)
        self.destroy_node()
