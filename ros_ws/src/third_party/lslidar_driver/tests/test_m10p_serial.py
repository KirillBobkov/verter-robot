"""Synthetic UART -> actual driver -> rear filter, on loopback ROS domain 87 only."""

import argparse
import math
import os
from pathlib import Path
import pty
import struct
import subprocess
import tempfile
import time


def packet(block, turn):
    count = 67 + (block + turn) % 5
    words = []
    for index in range(count):
        angle = math.radians(block * 15 + 15 * index / count)
        radius = 1.5 / max(abs(math.cos(angle)), abs(math.sin(angle)))
        words.append(round(radius * 1000))
    words.insert(3, 0xFFFF)
    size = 20 + len(words) * 2
    header = struct.pack('>BBHHH', 0xA5, 0x5A, size, block * 1500, 0)
    return header + struct.pack('>' + 'H' * len(words), *words) + bytes(10) + b'\xfa\xfb'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('binary', type=Path)
    parser.add_argument('rear_filter', type=Path)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '87' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise RuntimeError('Requires ROS_DOMAIN_ID=87 and ROS_LOCALHOST_ONLY=1')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2

    rclpy.init()
    node = rclpy.create_node('m10p_uart_test', enable_rosout=False, start_parameter_services=False)
    scans = []
    filtered = []
    clouds = []
    subscriptions = [
        node.create_subscription(LaserScan, '/m10p_test_scan', scans.append, qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/m10p_test_filtered', filtered.append, qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/m10p_test_cloud', clouds.append, qos_profile_sensor_data),
    ]
    master, slave = pty.openpty()
    serial_path = os.ttyname(slave)
    assert serial_path.startswith('/dev/pts/')
    driver = filter_process = None
    with tempfile.TemporaryDirectory(prefix='m10p-uart-test-') as temporary:
        try:
            with (Path(temporary) / 'driver.log').open('w') as log:
                driver = subprocess.Popen(
                    [
                        str(args.binary.resolve()), '--ros-args',
                        '-r', '__node:=m10p_replay_driver',
                        '-p', 'lidar_name:=M10_P',
                        '-p', 'interface_selection:=serial',
                        '-p', 'serial_port_:=' + serial_path,
                        '-p', 'scan_topic:=/m10p_test_scan',
                        '-p', 'pointcloud_topic:=/m10p_test_cloud',
                        '-p', 'frame_id:=lidar_link',
                        '-p', 'm10p_scan_bins:=1800',
                        '-p', 'min_range:=0.1',
                        '-p', 'max_range:=25.0',
                        '-p', 'pubScan:=true',
                        '-p', 'pubPointCloud2:=true',
                        '-p', 'compensation:=false',
                        '-p', 'use_gps_ts:=false',
                    ], stdout=log, stderr=log,
                )
            with (Path(temporary) / 'filter.log').open('w') as log:
                filter_process = subprocess.Popen(
                    [
                        '/opt/ros/humble/lib/laser_filters/scan_to_scan_filter_chain', '--ros-args',
                        '-r', '__node:=laser_filter', '--params-file', str(args.rear_filter.resolve()),
                        '-r', 'scan:=/m10p_test_scan', '-r', 'scan_filtered:=/m10p_test_filtered',
                    ], stdout=log, stderr=log,
                )
            deadline = time.monotonic() + 7
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if node.count_publishers('/m10p_test_scan') and node.count_publishers('/m10p_test_filtered'):
                    break
                assert driver.poll() is None and filter_process.poll() is None
            assert node.count_publishers('/m10p_test_filtered') == 1
            start = time.monotonic()
            for sequence in range(24 * 48):
                data = packet(sequence % 24, sequence // 24)
                # Fragment the length field too: a read() is not a packet boundary.
                for fragment in (data[:1], data[1:3], data[3:]):
                    view = memoryview(fragment)
                    while view:
                        view = view[os.write(master, view):]
                due = start + (sequence + 1) / 288
                while time.monotonic() < due:
                    rclpy.spin_once(node, timeout_sec=min(.0005, max(0, due - time.monotonic())))
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=.01)
            assert len(scans) >= 30 and len(filtered) >= 30 and clouds
            previous_stamp = 0
            for scan in scans:
                stamp = scan.header.stamp.sec + scan.header.stamp.nanosec / 1e9
                assert stamp > previous_stamp
                previous_stamp = stamp
                assert len(scan.ranges) == 1800 and scan.header.frame_id == 'lidar_link'
                assert abs(scan.angle_min + 1799 * scan.angle_increment - scan.angle_max) < 1e-6
                assert .065 < scan.scan_time < .11
                assert scan.time_increment < 0
                assert abs(scan.time_increment * 1800 + scan.scan_time) < 1e-7
                errors = []
                for index, radius in enumerate(scan.ranges):
                    assert radius > 0
                    if math.isfinite(radius):
                        theta = scan.angle_min + index * scan.angle_increment
                        expected = 1.5 / max(abs(math.cos(theta)), abs(math.sin(theta)))
                        errors.append(abs(radius - expected))
                assert len(errors) > 1500 and max(errors) < .008
            for scan in filtered:
                assert len(scan.ranges) == 1800 and scan.time_increment < 0
                for index, radius in enumerate(scan.ranges):
                    angle = scan.angle_min + index * scan.angle_increment
                    if 2.45 < angle < 3.83:
                        assert not math.isfinite(radius)
            last = scans[-1]
            header = last.header.stamp.sec + last.header.stamp.nanosec / 1e9
            age = node.get_clock().now().nanoseconds / 1e9 - header
            assert .001 < age < .5
            cloud = clouds[-1]
            field = next(field for field in cloud.fields if field.name == 'timestamp')
            timestamp = struct.unpack_from('<d', cloud.data, field.offset)[0]
            cloud_stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec / 1e9
            assert cloud_stamp - .12 < timestamp <= cloud_stamp + 1e-6
            print(f'PASS isolated PTY -> actual M10P driver -> rear filter: {len(scans)} fixed scans, '
                  f'{len(filtered)} filtered; square geometry, timestamps, point cloud; no real UART or control')
        except Exception:
            for path in Path(temporary).glob('*.log'):
                print(path.name, path.read_text(errors='replace')[-5000:])
            raise
        finally:
            for process in (driver, filter_process):
                if process:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
            os.close(master)
            os.close(slave)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
