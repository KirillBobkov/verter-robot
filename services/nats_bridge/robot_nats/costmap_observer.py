"""Observe Nav2 raw grids and TF in a separate, entirely passive ROS context."""

import logging
import threading
import time
import uuid

from .costmap_data import CostmapStore, MAX_AGE, planar_pose, transform_origin


def scope_key(mode, map_metadata):
    pid = mode.get('pid')
    map_id = map_metadata.get('map_id')
    active = (
        mode.get('actual') == 'navigation' and not mode.get('stale', True)
        and type(pid) is int and pid > 0 and map_metadata.get('available')
        and not map_metadata.get('stale', True) and isinstance(map_id, str)
        and map_metadata.get('mode') == 'navigation' and map_metadata.get('mode_pid') == pid
    )
    return (pid, map_id) if active else None


class CostmapObserver:
    def __init__(self, config, mode_reader, map_reader):
        self.mode_reader = mode_reader
        self.map_reader = map_reader
        self.enabled = config.get('ros_enabled', True)
        self.store = CostmapStore(config['robot_id'], enabled=self.enabled)
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
        subscriptions = {}
        try:
            import rclpy
            from nav2_msgs.msg import Costmap
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from tf2_ros import Buffer, TransformException, TransformListener

            context = Context()
            rclpy.init(context=context)
            node = rclpy.create_node(
                'verter_costmap_observer', context=context,
                enable_rosout=False, start_parameter_services=False,
            )
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            qos = QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            session = None
            previous_scope = None
            activation = 0
            scope_started = None
            latest = {}
            last_processed = {}
            last_attempt = {'local': 0.0, 'global': 0.0}
            sequence = {'local': 0, 'global': 0}

            def receive(kind, message, expected):
                if expected == session:
                    sequence[kind] += 1
                    latest[kind] = (sequence[kind], message)

            while not self.stop_event.is_set():
                mode = self.mode_reader()
                map_metadata = self.map_reader()
                map_id = map_metadata.get('map_id')
                pid = mode.get('pid')
                desired_key = scope_key(mode, map_metadata)
                active = desired_key is not None
                current = session
                if desired_key != previous_scope:
                    activation += 1
                    current = f'{self.boot_id}:{pid}:{activation}:{map_id}' if active else None
                    previous_scope = desired_key
                current_map = map_id if active else None
                changed = self.store.set_scope(current, mode, current_map, map_metadata.get('saved_map'))
                if changed:
                    for subscription in subscriptions.values():
                        node.destroy_subscription(subscription)
                    subscriptions.clear()
                    session = current
                    latest.clear()
                    last_processed.clear()
                    scope_started = node.get_clock().now().nanoseconds / 1e9
                    if session:
                        for kind in ('local', 'global'):
                            topic = f'/{kind}_costmap/costmap_raw'
                            subscriptions[kind] = node.create_subscription(
                                Costmap, topic,
                                lambda message, name=kind, expected=session: receive(name, message, expected),
                                qos,
                            )

                executor.spin_once(timeout_sec=0.05)
                if session is None:
                    continue
                now = time.monotonic()
                for kind in ('local', 'global'):
                    if kind not in latest:
                        continue
                    number, message = latest[kind]
                    interval = 0.2 if kind == 'local' else 1.0
                    if number == last_processed.get(kind) or now - last_attempt[kind] < interval:
                        continue
                    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
                    age = node.get_clock().now().nanoseconds / 1e9 - stamp
                    if stamp < scope_started - 0.05 or not -0.1 <= age <= MAX_AGE[kind]:
                        last_processed[kind] = number
                        last_attempt[kind] = now
                        self.store.fail(kind, 'costmap_timestamp_outside_current_session')
                        continue
                    expected_frame = 'odom' if kind == 'local' else 'map'
                    if message.header.frame_id != expected_frame:
                        last_processed[kind] = number
                        last_attempt[kind] = now
                        self.store.fail(kind, 'unexpected_costmap_source_frame')
                        continue
                    try:
                        source_origin = planar_pose(message.metadata.origin)
                        origin = source_origin
                        if kind == 'local':
                            transform = buffer.lookup_transform(
                                'map', 'odom', rclpy.time.Time.from_msg(message.header.stamp)
                            )
                            origin = transform_origin(message.metadata.origin, transform.transform)
                        self.store.put_grid(
                            session, kind,
                            message.metadata.size_x, message.metadata.size_y,
                            message.metadata.resolution, origin, message.data,
                            stamp, node.get_clock().now().nanoseconds / 1e9 - stamp,
                            message.header.frame_id, source_origin,
                        )
                        last_processed[kind] = number
                        last_attempt[kind] = now
                    except TransformException as error:
                        # Retry after the next executor turn, rather than waiting
                        # another raw-grid interval and selecting a newer sample.
                        # A previously valid snapshot retains its original TTL.
                        if not self.store.metadata(kind)['available']:
                            self.store.fail(kind, error)
                    except Exception as error:
                        last_processed[kind] = number
                        last_attempt[kind] = now
                        self.store.fail(kind, error)
        except Exception as error:
            logging.exception('Costmap observation failed; no robot mode is changed')
            for kind in ('local', 'global'):
                self.store.fail(kind, error)
        finally:
            if executor:
                executor.shutdown()
            if node:
                node.destroy_node()
            if context:
                context.shutdown()
