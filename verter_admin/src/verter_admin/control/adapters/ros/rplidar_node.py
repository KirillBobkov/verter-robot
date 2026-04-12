#!/usr/bin/env python3
"""Python RPLiDAR driver node using rplidar-roboticia + pyserial.

Workaround for tegra-xusb + cp210x poll/select incompatibility that
prevents the C++ rplidar_ros SDK from reading scan data on this Jetson.
pyserial uses blocking reads (VTIME > 0) which work correctly here.

Publishes: scan (sensor_msgs/LaserScan, BEST_EFFORT)
  — remapped to /scan_raw in mapping.launch.py

Parameters:
  serial_port  (str)   /dev/rplidar
  frame_id     (str)   lidar_link
  range_min    (float) 0.15 m
  range_max    (float) 6.0 m
  inverted     (bool)  false
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class RPLidarNode(Node):
    _NUM_BINS = 720  # 0.5° resolution

    def __init__(self) -> None:
        super().__init__('rplidar_node')

        self.declare_parameter('serial_port', '/dev/rplidar')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 6.0)
        self.declare_parameter('inverted', False)

        self._port = self.get_parameter('serial_port').value
        self._frame_id = self.get_parameter('frame_id').value
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._inverted = self.get_parameter('inverted').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(LaserScan, 'scan', sensor_qos)

        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'RPLidar Python node started on {self._port}')

    # ------------------------------------------------------------------

    def _scan_loop(self) -> None:
        import serial

        while self._running and rclpy.ok():
            ser = None
            try:
                self.get_logger().info(f'Connecting to {self._port} ...')
                ser = serial.Serial(self._port, 115200, timeout=3)

                # Stop + reset for clean state
                ser.write(bytes([0xA5, 0x25]))   # STOP
                time.sleep(0.1)
                ser.reset_input_buffer()
                ser.write(bytes([0xA5, 0x40]))   # RESET
                time.sleep(2.5)
                ser.reset_input_buffer()

                # Verify connection
                ser.write(bytes([0xA5, 0x50]))   # GET_DEVICE_INFO
                resp = ser.read(27)
                if len(resp) < 27 or resp[0] != 0xA5 or resp[1] != 0x5A:
                    raise RuntimeError(f'Device info failed: {resp.hex()}')
                self.get_logger().info(
                    f'Connected: model={resp[7]} fw={resp[9]}.{resp[8]} hw={resp[10]}'
                )

                # Start Standard scan
                ser.write(bytes([0xA5, 0x20]))   # START_SCAN
                desc = ser.read(7)
                if len(desc) < 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
                    raise RuntimeError(f'Bad scan descriptor: {desc.hex()}')
                self.get_logger().info('Scan started.')

                scan: list = []
                while self._running and rclpy.ok():
                    raw = ser.read(5)
                    if len(raw) < 5:
                        continue

                    new_scan = bool(raw[0] & 0x01)
                    inv_scan = bool((raw[0] >> 1) & 0x01)
                    quality  = raw[0] >> 2
                    check    = raw[1] & 0x01

                    if new_scan == inv_scan or check != 1:
                        continue  # malformed packet

                    angle_deg = ((raw[1] >> 1) | (raw[2] << 7)) / 64.0
                    dist_mm   = (raw[3] | (raw[4] << 8)) / 4.0

                    if new_scan and scan:
                        self._publish_scan(scan)
                        scan = []
                    if quality > 0 and dist_mm > 0:
                        scan.append((quality, angle_deg, dist_mm))

            except Exception as exc:
                self.get_logger().error(f'Lidar error: {exc!r} — reconnecting in 2 s')
                time.sleep(2.0)
            finally:
                if ser is not None:
                    try:
                        ser.write(bytes([0xA5, 0x25]))  # STOP
                        ser.close()
                    except Exception:
                        pass

    def _publish_scan(self, scan) -> None:
        n = self._NUM_BINS
        angle_min = -math.pi
        angle_inc = 2.0 * math.pi / n
        ranges = [float('inf')] * n

        for quality, angle_deg, dist_mm in scan:
            if quality == 0 or dist_mm <= 0:
                continue
            dist_m = dist_mm / 1000.0
            if dist_m < self._range_min or dist_m > self._range_max:
                continue

            # RPLIDAR: 0° = forward, increases CCW — same as ROS convention.
            # Map (0°, 360°) → (0, 2π) → (-π, π)
            angle_rad = math.radians(angle_deg % 360.0)
            if angle_rad > math.pi:
                angle_rad -= 2.0 * math.pi

            if self._inverted:
                angle_rad = -angle_rad

            idx = int((angle_rad - angle_min) / angle_inc)
            idx = max(0, min(n - 1, idx))
            if dist_m < ranges[idx]:
                ranges[idx] = dist_m

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.angle_min = angle_min
        msg.angle_max = math.pi - angle_inc
        msg.angle_increment = angle_inc
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / 5.5  # A1M8 Standard mode
        msg.range_min = self._range_min
        msg.range_max = self._range_max
        msg.ranges = ranges
        self._pub.publish(msg)

    def destroy_node(self) -> None:
        self._running = False
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RPLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
