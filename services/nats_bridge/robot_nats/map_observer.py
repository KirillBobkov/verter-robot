"""Read /map and TF in a separate ROS context. This node has no publishers or commands."""

import json
import logging
import math
from pathlib import Path
import threading
import time
import uuid

from .map_data import MapStore, robot_model, project_scan


def yaw(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    if not all(math.isfinite(value) for value in (x, y, z, w)):
        raise ValueError('invalid_rotation')
    if abs(x * x + y * y + z * z + w * w - 1) > 0.001:
        raise ValueError('nonunit_rotation')
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class MapObserver:
    def __init__(self, config, mode_reader):
        self.mode_reader = mode_reader
        self.enabled = config.get('ros_enabled', True)
        root = Path(config.get('project_root', Path(__file__).resolve().parents[3]))
        try:
            geometry = json.loads((root / 'ros_ws/src/verter_description/config/geometry.json').read_text())
            model = robot_model(geometry)
        except (OSError, ValueError, KeyError, TypeError):
            model = None
        self.store = MapStore(config['robot_id'], model)
        self.boot_id = str(uuid.uuid4())
        self.stop_event = threading.Event()
        self.thread = None

    def start(self):
        if self.enabled:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def close(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=3)

    def run(self):
        node = executor = context = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import LaserScan
            from nav_msgs.msg import OccupancyGrid
            from tf2_ros import Buffer, TransformListener

            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node(
                'verter_map_observer', context=context, enable_rosout=False, start_parameter_services=False
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
            )
            subscription = None
            session = None
            latest = None
            latest_scan = None
            scan_sequence = 0
            scan_processed = -1
            sequence = 0
            processed = -1
            first_map_at = None
            last_grid = last_pose = 0

            def receive(message, expected):
                nonlocal latest, sequence
                if expected == session:
                    sequence += 1
                    latest = (sequence, message)

            def receive_scan(message):
                nonlocal latest_scan, scan_sequence
                scan_sequence += 1
                latest_scan = (scan_sequence, message)
                self.store.note_scan(len(message.ranges), message.angle_increment, message.scan_time)

            scan_subscription = node.create_subscription(LaserScan, '/scan', receive_scan, qos_profile_sensor_data)

            while not self.stop_event.is_set():
                mode = self.mode_reader()
                pid = mode.get('pid')
                active = mode['actual'] in ('mapping', 'localization', 'navigation') and type(pid) is int
                current = f'{self.boot_id}:{pid}' if active else None
                changed = self.store.set_scope(current, mode)
                if changed:
                    if subscription:
                        node.destroy_subscription(subscription)
                        subscription = None
                    session = current
                    latest = None
                    first_map_at = None
                    processed = -1
                    scan_processed = -1
                    if session:
                        subscription = node.create_subscription(
                            OccupancyGrid, '/map', lambda msg, expected=session: receive(msg, expected), qos
                        )
                executor.spin_once(timeout_sec=0.05)
                now = time.monotonic()
                if session is None:
                    continue
                if latest is not None and latest[0] != processed and now - last_grid >= 1:
                    processed, message = latest
                    last_grid = now
                    try:
                        if message.header.frame_id != 'map':
                            raise ValueError('map_frame_must_be_map')
                        origin = message.info.origin
                        if abs(origin.orientation.x) > 0.00001 or abs(origin.orientation.y) > 0.00001:
                            raise ValueError('nonplanar_map_origin')
                        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                        self.store.put_grid(
                            session,
                            message.info.width,
                            message.info.height,
                            message.info.resolution,
                            {'x': origin.position.x, 'y': origin.position.y, 'yaw': yaw(origin.orientation)},
                            message.data,
                            stamp,
                        )
                        if first_map_at is None:
                            first_map_at = node.get_clock().now().nanoseconds / 1e9
                    except Exception as exc:
                        self.store.fail(exc)
                if first_map_at is not None and now - last_pose >= 0.1:
                    last_pose = now
                    try:
                        transform = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
                        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                        if stamp < first_map_at - 0.05:
                            raise ValueError('waiting_for_current_session_tf')
                        age = node.get_clock().now().nanoseconds / 1e9 - stamp
                        position = transform.transform.translation
                        self.store.put_pose(
                            session, stamp, position.x, position.y, yaw(transform.transform.rotation), age
                        )
                    except Exception as exc:
                        self.store.invalidate_pose(exc)
                if first_map_at is not None and latest_scan is not None and latest_scan[0] != scan_processed:
                    number, scan = latest_scan
                    stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
                    age = node.get_clock().now().nanoseconds / 1e9 - stamp
                    if abs(age) > 1 or stamp < first_map_at - 0.05:
                        scan_processed = number
                        self.store.invalidate_scan('scan_timestamp_outside_current_session')
                        continue
                    try:
                        transform = buffer.lookup_transform(
                            'map', scan.header.frame_id, rclpy.time.Time.from_msg(scan.header.stamp)
                        )
                    except Exception:
                        # The newest scan can precede odometry TF by a few milliseconds.
                        # Retry the bounded latest sample; a last good cloud expires after 1 s.
                        continue
                    scan_processed = number
                    try:
                        points = project_scan(scan, transform.transform)
                        self.store.put_scan(session, stamp, points, age, scan.header.frame_id, len(scan.ranges))
                    except Exception as exc:
                        self.store.invalidate_scan(exc)
        except Exception as exc:
            logging.exception('Map observer failed; other robot services remain independent')
            self.store.fail(exc)
        finally:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
            if context:
                context.shutdown()
