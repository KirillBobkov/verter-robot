#!/usr/bin/env python3
"""Bounded readiness checks. Never drive; 'zero' publishes zero Twist only."""

import sys
import time
import subprocess
import json
import math


LOCALIZATION_CONFLICT_NODES = {
    'slam_toolbox', 'controller_server', 'smoother_server', 'planner_server',
    'behavior_server', 'bt_navigator', 'waypoint_follower', 'lifecycle_manager_navigation',
}


def localization_conflicts(node_names, map_publishers, pose_publishers, allow_owned=False):
    names = [name.rsplit('/', 1)[-1] for name in node_names]
    blocked = set(LOCALIZATION_CONFLICT_NODES)
    if not allow_owned:
        blocked.update(['map_server', 'amcl', 'lifecycle_manager_localization'])
    conflicts = sorted(blocked.intersection(names))
    for name in ('map_server', 'amcl', 'lifecycle_manager_localization'):
        if allow_owned and names.count(name) > 1:
            conflicts.append('duplicate_node:' + name)
    maximum_publishers = 1 if allow_owned else 0
    if map_publishers > maximum_publishers:
        conflicts.append('map_publishers:' + str(map_publishers))
    if pose_publishers > maximum_publishers:
        conflicts.append('amcl_pose_publishers:' + str(pose_publishers))
    return conflicts


def map_snapshot(message):
    info = message.info
    result = {
        'valid': False, 'frame': message.header.frame_id,
        'width': info.width, 'height': info.height, 'resolution': info.resolution,
    }
    if message.header.frame_id != 'map' or info.width <= 0 or info.height <= 0:
        return result
    if not math.isfinite(info.resolution) or info.resolution <= 0:
        result['resolution'] = None
        return result
    if len(message.data) != info.width * info.height:
        return result
    if any(value < -1 or value > 100 for value in message.data):
        return result
    pose = info.origin
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if not all(math.isfinite(value) for value in values):
        return result
    norm = sum(value * value for value in values[3:])
    result['valid'] = abs(norm - 1.0) <= 0.01
    return result


def pose_snapshot(message, now):
    result = {'valid': False, 'frame': message.header.frame_id}
    pose = message.pose.pose
    covariance = list(message.pose.covariance)
    values = [pose.position.x, pose.position.y, pose.position.z,
              pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    if message.header.frame_id != 'map' or len(covariance) != 36:
        return result
    if not all(math.isfinite(value) for value in values + covariance):
        return result
    norm = sum(value * value for value in values[3:])
    if abs(norm - 1.0) > 0.01 or any(covariance[index] < 0 for index in (0, 7, 35)):
        return result
    if abs(covariance[1] - covariance[6]) > 1e-6:
        return result
    if covariance[0] * covariance[7] - covariance[1] * covariance[6] < -1e-9:
        return result
    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
    age = now - stamp
    if stamp <= 0 or age < -1.0:
        return result
    q = pose.orientation
    result.update({
        'valid': True, 'stamp': stamp, 'age_seconds': age,
        'fresh': age <= 2.0,
        'x': pose.position.x, 'y': pose.position.y,
        'yaw': math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
        'covariance_xy_yaw': [covariance[0], covariance[7], covariance[35]],
    })
    return result


def tf_snapshot(message, now):
    stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
    age = now - stamp
    pose = message.transform
    values = [pose.translation.x, pose.translation.y, pose.translation.z,
              pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w]
    norm = sum(value * value for value in values[3:])
    valid = all(math.isfinite(value) for value in values) and abs(norm - 1.0) <= 0.01
    valid = valid and stamp > 0
    return {'valid': valid, 'fresh': valid and -1.0 <= age <= 2.0,
            'stamp': stamp, 'age_seconds': age}


def localization_report(states, grid, pose, map_odom, map_base, conflicts):
    lifecycle_ready = all(states.get(name) == 3 for name in ('map_server', 'amcl'))
    map_available = grid is not None and grid.get('valid', False)
    ready = lifecycle_ready and map_available and not conflicts
    pose_valid = pose is not None and pose.get('valid', False)
    tf_fresh = map_odom is not None and map_odom.get('fresh', False)
    base_fresh = map_base is not None and map_base.get('fresh', False)
    initialized = ready and pose_valid and tf_fresh and base_fresh
    observed = pose_valid or (map_odom is not None and map_odom.get('valid', False))
    return {
        'ready': ready,
        'lifecycle_active': lifecycle_ready,
        'lifecycle_states': dict(states),
        'map_available': map_available,
        'map': grid,
        'initialized': initialized,
        'waiting_for_initial_pose': ready and not observed,
        'localization_verified': False,
        'amcl_pose': pose,
        'map_odom_tf': map_odom,
        'map_base_tf': map_base,
        'conflicts': list(conflicts),
        'scope': 'AMCL/map readiness only; no physical localization verification or motion permission',
    }


def main():
    from sensor_health import sample_valid, missing_inputs
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from sensor_msgs.msg import LaserScan, Imu
    from nav_msgs.msg import Odometry, OccupancyGrid
    from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.srv import ManageLifecycleNodes
    from tf2_ros import Buffer, TransformListener

    kind = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('verter_readiness_probe', enable_rosout=False, start_parameter_services=False)
    try:
        if kind == 'zero':
            pub = node.create_publisher(Twist, '/cmd_vel', 10)
            end = time.monotonic() + 1
            while time.monotonic() < end:
                pub.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
            return
        if kind == 'shutdown_nav':
            client = node.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
            if not client.wait_for_service(timeout_sec=1):
                return
            req = ManageLifecycleNodes.Request()
            req.command = 4
            f = client.call_async(req)
            rclpy.spin_until_future_complete(node, f, timeout_sec=3)
            return
        if kind == 'exclusive':
            for device in [
                '/dev/chassis',
                '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
                '/dev/esp32_imu',
                '/dev/esp32_ctrl',
            ]:
                check = subprocess.run(['fuser', device], capture_output=True)
                if check.returncode == 0:
                    raise RuntimeError('Device already in use: ' + device)
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            for topic in ['/odom_raw', '/scan_raw', '/scan']:
                if node.count_publishers(topic):
                    raise RuntimeError('Existing robot publisher: ' + topic)
            return
        if kind == 'localization-exclusive':
            # The supervisor invokes this only AFTER its old mode group exited.
            # Never stop an unmanaged source or change any parameter here.
            end = time.monotonic() + 2
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
            conflicts = localization_conflicts(
                node.get_node_names(), node.count_publishers('/map'), node.count_publishers('/amcl_pose'),
            )
            if conflicts:
                raise RuntimeError('Localization source already running: ' + ', '.join(conflicts))
            return
        if kind in ('localization', 'localization-status'):
            latest = {'map': None, 'pose': None}

            def receive_map(message):
                latest['map'] = map_snapshot(message)

            def receive_pose(message):
                latest['pose'] = message

            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            subscriptions = [
                node.create_subscription(OccupancyGrid, '/map', receive_map, qos),
                node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', receive_pose, qos),
            ]
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            clients = {name: node.create_client(GetState, '/' + name + '/get_state')
                       for name in ('map_server', 'amcl')}
            states = {name: None for name in clients}
            end = time.monotonic() + (3 if kind == 'localization-status' else 40)
            report = None
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                for name, client in clients.items():
                    states[name] = None
                    if not client.service_is_ready():
                        continue
                    future = client.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(node, future, timeout_sec=0.25)
                    if future.done() and future.result() is not None:
                        states[name] = future.result().current_state.id
                now = node.get_clock().now().nanoseconds * 1e-9
                frames = {}
                for child in ('odom', 'base_footprint'):
                    try:
                        transform = buffer.lookup_transform('map', child, rclpy.time.Time())
                        frames[child] = tf_snapshot(transform, now)
                    except Exception:
                        frames[child] = None
                pose = pose_snapshot(latest['pose'], now) if latest['pose'] is not None else None
                conflicts = localization_conflicts(
                    node.get_node_names(), node.count_publishers('/map'), node.count_publishers('/amcl_pose'),
                    allow_owned=True,
                )
                report = localization_report(states, latest['map'], pose, frames['odom'], frames['base_footprint'], conflicts)
                if kind == 'localization' and report['ready']:
                    return
            if kind == 'localization-status':
                print(json.dumps(report))
                return
            raise RuntimeError('Localization map or lifecycle nodes unavailable: ' + json.dumps(report))
        seen = {}

        def received(topic):
            def callback(msg):
                if kind not in ('hardware', 'hardware-status') or sample_valid(
                    topic, msg, node.get_clock().now().nanoseconds * 1e-9
                ):
                    seen[topic] = time.monotonic()

            return callback

        if kind in ('hardware', 'hardware-status'):
            topics = [('/scan', LaserScan), ('/odometry/filtered', Odometry), ('/imu/data', Imu)]
            for topic, typ in topics:
                node.create_subscription(typ, topic, received(topic), qos_profile_sensor_data)
            buffer = Buffer()
            listener = TransformListener(buffer, node)
            end = time.monotonic() + (3 if kind == 'hardware-status' else 60)
            missing = [t for t, _ in topics]
            tf_ready = False
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)
                missing = missing_inputs(seen, time.monotonic())
                tf_ready = False
                if buffer.can_transform('odom', 'base_footprint', rclpy.time.Time()):
                    tf = buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
                    tf_ready = sample_valid('tf', tf, node.get_clock().now().nanoseconds * 1e-9)
                if not missing and tf_ready:
                    if kind == 'hardware-status':
                        print(json.dumps({'ready': True, 'missing': [], 'odom_tf_fresh': True}))
                    return
            report = {'ready': False, 'missing': missing, 'odom_tf_fresh': tf_ready}
            if kind == 'hardware-status':
                print(json.dumps(report))
                return
            raise RuntimeError('Navigation inputs unavailable: ' + json.dumps(report))
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, '/map', received('/map'), qos)
        names = (
            ['map_server', 'amcl']
            if kind == 'navigation'
            else (
                [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                ]
                if kind == 'navigation-ready'
                else ['map_saver']
            )
        )
        clients = {n: node.create_client(GetState, '/' + n + '/get_state') for n in names}
        end = time.monotonic() + 40
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            if '/map' not in seen:
                continue
            ready = True
            for client in clients.values():
                if not client.service_is_ready():
                    ready = False
                    break
                f = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(node, f, timeout_sec=0.5)
                if not f.done() or f.result() is None or f.result().current_state.id != 3:
                    ready = False
                    break
            if ready:
                return
        raise RuntimeError('No map or lifecycle nodes not active')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
