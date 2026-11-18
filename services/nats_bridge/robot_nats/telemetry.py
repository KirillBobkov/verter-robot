import json
import math
import logging
import threading
import time


def finite(v):
    return v if math.isfinite(v) else None


def pose_observation(message):
    pose = message.pose.pose
    rotation = pose.orientation
    components = (rotation.x, rotation.y, rotation.z, rotation.w)
    valid_rotation = all(math.isfinite(value) for value in components)
    valid_rotation = valid_rotation and abs(sum(value * value for value in components) - 1) < 0.001
    yaw = None
    if valid_rotation:
        yaw = math.atan2(
            2 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1 - 2 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
    return {
        'x': finite(pose.position.x),
        'y': finite(pose.position.y),
        'yaw': yaw,
        'frame_id': message.header.frame_id,
        'stamp': message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
        'covariance': [finite(value) for value in message.pose.covariance],
    }


class Telemetry:
    def __init__(self, enabled=True, motion=None, connected=lambda: True):
        self.motion, self.connected = motion, connected
        self.lock = threading.Lock()
        self.values = {}
        self.sequence = 0
        self.sequences = {}
        self.error = None
        self.stop = threading.Event()
        self.thread = None
        if enabled:
            self.thread = threading.Thread(target=self.run, daemon=True)
            self.thread.start()

    def put(self, key, value):
        with self.lock:
            self.sequence += 1
            self.sequences[key] = self.sequence
            self.values[key] = (time.monotonic(), time.time(), value)

    def snapshot(self):
        with self.lock:
            now = time.time()
            sources = {}
            for key, (mono, wall, value) in self.values.items():
                age = time.monotonic() - mono
                record = {'received_at': wall, 'age_s': round(age, 3), 'stale': age > 5, 'value': value}
                record['sequence'] = self.sequences.get(key)
                if key == 'localization':
                    stamp = value.get('stamp')
                    valid_stamp = type(stamp) in (int, float) and math.isfinite(stamp) and 0 < stamp <= now + 1
                    record['sample_at'] = stamp if valid_stamp else None
                    record['sample_age_s'] = max(0, now - stamp) if valid_stamp else None
                    record['stale'] = record['stale'] or not valid_stamp or now - stamp > 5
                sources[key] = record
            return {
                'error': self.error,
                'sources': sources,
            }

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=3)

    def run(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            from std_msgs.msg import String, Float32MultiArray
            from sensor_msgs.msg import LaserScan, Imu
            from nav_msgs.msg import Odometry, OccupancyGrid
            from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
            from tf2_msgs.msg import TFMessage

            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node(
                'verter_nats_telemetry', context=context, enable_rosout=False, start_parameter_services=False
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            q = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

            def health(m):
                try:
                    self.put('imu_status', json.loads(m.data))
                except ValueError:
                    self.put('imu_status', {'parse_error': True})

            def tf(m):
                for t in m.transforms:
                    if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
                        self.put(
                            'map_to_odom',
                            {'x': finite(t.transform.translation.x), 'y': finite(t.transform.translation.y)},
                        )

            specs = [
                (String, '/imu/status', health),
                (
                    Float32MultiArray,
                    '/ultrasonics',
                    lambda m: self.put(
                        'ultrasonics', {'meters': [finite(x) for x in m.data], 'timeout_sentinel': 10.0}
                    ),
                ),
                (
                    Imu,
                    '/imu/data',
                    lambda m: self.put(
                        'imu',
                        {
                            'gyro_z': finite(m.angular_velocity.z),
                            'accel_norm': finite(
                                math.hypot(m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)
                            ),
                        },
                    ),
                ),
                (
                    LaserScan,
                    '/scan',
                    lambda m: self.put(
                        'lidar',
                        {
                            'frame': m.header.frame_id,
                            'points': len(m.ranges),
                            'valid_points': sum(math.isfinite(x) and m.range_min <= x <= m.range_max for x in m.ranges),
                        },
                    ),
                ),
                (
                    Odometry,
                    '/odometry/filtered',
                    lambda m: self.put(
                        'odometry',
                        {
                            'x': finite(m.pose.pose.position.x),
                            'y': finite(m.pose.pose.position.y),
                            'vx': finite(m.twist.twist.linear.x),
                            'wz': finite(m.twist.twist.angular.z),
                        },
                    ),
                ),
                (
                    PoseWithCovarianceStamped,
                    '/amcl_pose',
                    lambda m: self.put('localization', pose_observation(m)),
                ),
                (TFMessage, '/tf', tf),
                (
                    OccupancyGrid,
                    '/map',
                    lambda m: self.put(
                        'map', {'width': m.info.width, 'height': m.info.height, 'resolution': m.info.resolution}
                    ),
                ),
            ]
            retained = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
            reset_qos = QoSProfile(
                depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE
            )
            subscriptions = [
                node.create_subscription(kind, topic, cb, retained if topic in ('/map', '/amcl_pose') else q)
                for kind, topic, cb in specs
            ]
            subscriptions.append(
                node.create_subscription(
                    PoseWithCovarianceStamped,
                    '/initialpose',
                    lambda message: self.put('initial_pose', pose_observation(message)),
                    reset_qos,
                )
            )
            velocity_publisher = None

            def send_velocity(linear, angular):
                nonlocal velocity_publisher
                if velocity_publisher is None:
                    velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 1)
                message = Twist()
                message.linear.x = float(linear)
                message.angular.z = float(angular)
                velocity_publisher.publish(message)

            if self.motion:
                self.motion.ready = True
            next_graph = 0
            try:
                while not self.stop.is_set():
                    executor.spin_once(timeout_sec=0.05)
                    if self.motion:
                        self.motion.tick(send_velocity, self.connected())
                    if time.monotonic() >= next_graph:
                        self.put(
                            'graph',
                            {
                                'nodes': node.get_node_names(),
                                'full_node_names': [
                                    namespace.rstrip('/') + '/' + name
                                    for name, namespace in node.get_node_names_and_namespaces()
                                ],
                                'cmd_vel_publishers': [
                                    x.node_name for x in node.get_publishers_info_by_topic('/cmd_vel')
                                ],
                            },
                        )
                        next_graph = time.monotonic() + 2
            finally:
                if self.motion:
                    self.motion.shutdown(send_velocity)
                executor.shutdown()
                node.destroy_node()
                context.shutdown()
        except Exception as exc:
            logging.exception('ROS telemetry thread failed')
            with self.lock:
                self.error = str(exc)
