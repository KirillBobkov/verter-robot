#!/usr/bin/env python3
"""RPLiDAR Standard scan node using os.open() + blocking termios.

Workaround for tegra-xusb + cp210x incompatibility:
  - C++ rplidar_ros SDK / pyserial / rplidar-roboticia all use poll()/select()
    internally, which is broken on this Jetson's cp210x USB serial driver.
  - Fix: open port with os.open() (no O_NONBLOCK), configure VMIN=1/VTIME=0
    BEFORE any I/O, then use os.read() which blocks in-kernel without poll().

Standard scan protocol (cmd 0xA5 0x20):
  - One 5-byte packet per measurement, no packet-pair dependency.
  - Format: quality/sync(1) + angle_q6(2) + dist_q2(2)
  - ~5–6 Hz, ~360 points per revolution.

Publishes: scan (sensor_msgs/LaserScan, BEST_EFFORT)
  — remapped to /scan_raw in mapping.launch.py

Parameters:
  serial_port  (str)   /dev/rplidar
  frame_id     (str)   lidar_link
  range_min    (float) 0.15 m
  range_max    (float) 6.0 m
  inverted     (bool)  false
"""
import fcntl
import math
import os
import struct
import termios
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

_SCAN_CMD = bytes([0xA5, 0x20])

# Linux IOCTL for DTR (A1M8 motor control: DTR=LOW → motor ON)
_TIOCMBIC = 0x5417
_TIOCM_DTR = 0x002


def _open_serial_blocking(port: str) -> int:
    """Open serial port in blocking mode, termios configured BEFORE any I/O.

    os.open() without O_NONBLOCK → fd is blocking by default.
    VMIN=1, VTIME=0 → os.read() blocks until ≥1 byte arrives (no poll needed).
    TCSAFLUSH before any I/O → buffer flush is harmless.
    """
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY)
    attr = termios.tcgetattr(fd)
    attr[0] = termios.IGNPAR | termios.IGNBRK  # iflag: ignore parity/break
    attr[1] = 0                                  # oflag: raw output
    attr[2] = termios.CS8 | termios.CLOCAL | termios.CREAD  # cflag: 8N1
    attr[3] = 0                                  # lflag: raw mode
    attr[4] = termios.B115200
    attr[5] = termios.B115200
    attr[6][termios.VMIN] = 1                    # block until ≥1 byte
    attr[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSAFLUSH, attr)
    return fd


def _read_exact(fd: int, n: int) -> bytes:
    """Block until exactly n bytes are read."""
    buf = bytearray()
    while len(buf) < n:
        chunk = os.read(fd, n - len(buf))
        if not chunk:
            raise RuntimeError(f'EOF: wanted {n} bytes, got {len(buf)}')
        buf += chunk
    return bytes(buf)


def _parse_std_packet(pkt: bytes):
    """Parse one 5-byte Standard scan packet.

    Byte 0: quality[7:2] | S_inv[1] | S[0]
      S=1 → start of new scan. S_inv must equal ~S (validity check).
    Byte 1: angle_q6[6:0] | check[0]   (check should be 1)
    Byte 2: angle_q6[14:7]
    Byte 3: dist_q2 LSB
    Byte 4: dist_q2 MSB

    Returns (new_scan, quality, angle_deg, dist_mm) or None if packet invalid.
    """
    s_bit     = pkt[0] & 0x01
    s_inv_bit = (pkt[0] >> 1) & 0x01
    if s_bit == s_inv_bit:
        return None   # S and ~S must be complementary

    quality  = pkt[0] >> 2
    angle_q6 = (pkt[1] >> 1) | (pkt[2] << 7)   # 14-bit angle in 1/64 °
    angle_deg = angle_q6 / 64.0

    dist_q2  = pkt[3] | (pkt[4] << 8)           # 16-bit distance in 0.25 mm units
    dist_mm  = dist_q2 / 4.0

    return bool(s_bit), quality, angle_deg, dist_mm


class RPLidarNode(Node):
    _NUM_BINS = 720  # 0.5° resolution

    def __init__(self) -> None:
        super().__init__('rplidar_node')

        self.declare_parameter('serial_port', '/dev/rplidar')
        self.declare_parameter('frame_id', 'lidar_link')
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 6.0)
        self.declare_parameter('inverted', False)

        self._port      = self.get_parameter('serial_port').value
        self._frame_id  = self.get_parameter('frame_id').value
        self._range_min = float(self.get_parameter('range_min').value)
        self._range_max = float(self.get_parameter('range_max').value)
        self._inverted  = self.get_parameter('inverted').value

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(LaserScan, 'scan', sensor_qos)

        self._running = True
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()
        self.get_logger().info(f'RPLidar node started on {self._port}')

    # ------------------------------------------------------------------

    def _scan_loop(self) -> None:
        while self._running and rclpy.ok():
            fd = None
            try:
                self.get_logger().info(f'Opening {self._port} (blocking)...')
                fd = _open_serial_blocking(self._port)

                # A1M8: DTR=LOW → motor ON
                fcntl.ioctl(fd, _TIOCMBIC, struct.pack('I', _TIOCM_DTR))
                time.sleep(1.0)

                # Stop any running scan, then reset for clean state
                os.write(fd, bytes([0xA5, 0x25]))   # STOP
                time.sleep(0.1)
                termios.tcflush(fd, termios.TCIFLUSH)

                os.write(fd, bytes([0xA5, 0x40]))   # RESET
                time.sleep(2.5)
                termios.tcflush(fd, termios.TCIFLUSH)

                # Verify device
                os.write(fd, bytes([0xA5, 0x50]))   # GET_DEVICE_INFO
                resp = _read_exact(fd, 27)
                if resp[0] != 0xA5 or resp[1] != 0x5A:
                    raise RuntimeError(f'Bad device info: {resp[:4].hex()}')
                self.get_logger().info(
                    f'Connected: model={resp[7]} fw={resp[9]}.{resp[8]} hw={resp[10]}'
                )

                # Start Standard scan
                os.write(fd, _SCAN_CMD)
                desc = _read_exact(fd, 7)
                if desc[0] != 0xA5 or desc[1] != 0x5A:
                    raise RuntimeError(f'Bad scan descriptor: {desc.hex()}')
                self.get_logger().info(f'Standard scan started. Descriptor: {desc.hex()}')

                scan: list[tuple[int, float, float]] = []
                last_scan_t = 0.0
                scan_time = 1.0 / 6.0
                bad_pkts = 0

                while self._running and rclpy.ok():
                    pkt = _read_exact(fd, 5)
                    parsed = _parse_std_packet(pkt)

                    if parsed is None:
                        bad_pkts += 1
                        if bad_pkts > 20:
                            raise RuntimeError(f'Too many bad packets ({bad_pkts}), restarting')
                        # Try to resync: search for a packet with check bit=1
                        self.get_logger().warn(f'Bad packet ({bad_pkts}): {pkt.hex()} — skipping')
                        continue

                    bad_pkts = 0
                    new_scan, quality, angle_deg, dist_mm = parsed

                    if new_scan and scan:
                        now = time.monotonic()
                        if last_scan_t > 0.0:
                            scan_time = now - last_scan_t
                        last_scan_t = now
                        self._publish_scan(scan, scan_time)
                        scan = []

                    if quality > 0 and dist_mm > 0:
                        scan.append((quality, angle_deg, dist_mm))

            except Exception as exc:
                self.get_logger().error(f'Lidar error: {exc!r} — reconnecting in 2 s')
                time.sleep(2.0)
            finally:
                if fd is not None:
                    try:
                        os.write(fd, bytes([0xA5, 0x25]))  # STOP
                        os.close(fd)
                    except Exception:
                        pass

    def _publish_scan(self, scan, scan_time: float) -> None:
        n = self._NUM_BINS
        angle_min = -math.pi
        angle_inc = 2.0 * math.pi / n
        ranges = [float('inf')] * n

        for _quality, angle_deg, dist_mm in scan:
            dist_m = dist_mm / 1000.0
            if dist_m < self._range_min or dist_m > self._range_max:
                continue

            # RPLIDAR: 0° = forward, increases CCW — same as ROS convention.
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
        msg.scan_time = scan_time
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
