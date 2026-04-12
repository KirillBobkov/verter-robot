#!/usr/bin/env python3
"""Python RPLiDAR Express scan node using pyserial.

Workaround for tegra-xusb + cp210x incompatibility:
  - C++ rplidar_ros SDK: poll/select fails → no scan data.
  - tcsetattr(TCSANOW) after initial read flushes rx buffer → os.read() blocks.
  - Fix: use pyserial ser.read() (VMIN=0, VTIME=timeout) throughout.
    84-byte Express packets arrive fast enough that this works reliably.

Express scan protocol (cmd 0x82):
  - 84-byte response packets, each holding 16 "cabins" (32 measurements).
  - Angles are reconstructed from two consecutive packets.

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

# Express Scan command (0x82), payload_size=5, payload=[0,0,0,0,0]
# checksum = XOR(0xA5, 0x82, 0x05, 0x00×5) = 0x22
_EXPRESS_CMD = bytes([0xA5, 0x82, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x22])
_PACKET_SIZE = 84    # bytes per Express response packet
_CABINS = 16         # cabins per packet
_MEAS = 32           # measurements per packet (16 cabins × 2)


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
    # Low-level I/O

    @staticmethod
    def _resync_pyserial(ser) -> bytes:
        """Byte-by-byte resync using pyserial (avoids raw os.read issues).

        Express packets start with high nibble 0xA (byte 0) and 0x5 (byte 1).
        Returns a complete 84-byte packet.
        """
        b0 = ser.read(1)
        b1 = ser.read(1)
        if not b0 or not b1:
            raise RuntimeError('lidar: resync read timeout')
        b0, b1 = b0[0], b1[0]
        for _ in range(_PACKET_SIZE * 8):
            if (b0 >> 4) == 0xA and (b1 >> 4) == 0x5:
                rest = ser.read(_PACKET_SIZE - 2)
                if len(rest) < _PACKET_SIZE - 2:
                    raise RuntimeError('lidar: resync short read')
                return bytes([b0, b1]) + rest
            b0 = b1
            b = ser.read(1)
            if not b:
                raise RuntimeError('lidar: resync read timeout')
            b1 = b[0]
        raise RuntimeError('lidar: express resync failed')

    # ------------------------------------------------------------------
    # Express packet decode

    @staticmethod
    def _parse_packet(pkt: bytes):
        """Parse one 84-byte Express scan packet.

        Packet layout:
          byte 0  : sync1 (high nibble = 0xA) | flags
          byte 1  : sync2 (high nibble = 0x5) | checksum nibble
          bytes 2-3: start_angle_sync_q6 (little-endian uint16)
                     bit 15 = new_scan flag
                     bits 14:0 = start_angle in 1/64° units
          bytes 4-83: 16 cabins × 5 bytes each

        Each cabin (5 bytes):
          w1 = bytes[0:2] uint16: bits 15:2 = dist1 in Q2 (×0.25 mm), bit 0 = sign of d_angle1
          w2 = bytes[2:4] uint16: bits 15:2 = dist2 in Q2 (×0.25 mm), bit 0 = sign of d_angle2
          b4 = byte[4]: low nibble = magnitude of d_angle1 (Q3), high nibble = magnitude of d_angle2 (Q3)

        Returns:
          new_scan (bool)
          start_deg (float): start angle [0, 360)
          cabins (list[tuple]): (dist1_mm, dist2_mm, d_angle1_deg, d_angle2_deg) × 16
        """
        raw_angle = (pkt[3] << 8) | pkt[2]
        new_scan = bool(raw_angle & 0x8000)
        start_deg = (raw_angle & 0x7FFF) / 64.0

        cabins = []
        for i in range(_CABINS):
            o = 4 + i * 5
            w1 = pkt[o] | (pkt[o + 1] << 8)
            w2 = pkt[o + 2] | (pkt[o + 3] << 8)
            b4 = pkt[o + 4]

            dist1_mm = (w1 >> 2) * 0.25
            dist2_mm = (w2 >> 2) * 0.25

            # Delta angle: magnitude in Q3 (÷8 → degrees), sign from dist-word LSB
            da1 = (b4 & 0x0F) / 8.0 * (-1.0 if (w1 & 0x01) else 1.0)
            da2 = ((b4 >> 4) & 0x0F) / 8.0 * (-1.0 if (w2 & 0x01) else 1.0)

            cabins.append((dist1_mm, dist2_mm, da1, da2))

        return new_scan, start_deg, cabins

    @staticmethod
    def _decode_cabins(cabins, start_curr: float, start_next: float):
        """Reconstruct 32 (angle_deg, dist_mm) points from one packet's cabins.

        Angles are linearly interpolated between start_curr and start_next,
        then corrected by each cabin's delta angle.
        """
        diff = (start_next - start_curr) % 360.0
        pts = []
        for i, (d1, d2, da1, da2) in enumerate(cabins):
            a1 = (start_curr + diff * (2 * i) / _MEAS + da1) % 360.0
            a2 = (start_curr + diff * (2 * i + 1) / _MEAS + da2) % 360.0
            pts.append((a1, d1))
            pts.append((a2, d2))
        return pts

    # ------------------------------------------------------------------

    def _scan_loop(self) -> None:
        import serial

        while self._running and rclpy.ok():
            ser = None
            try:
                self.get_logger().info(f'Connecting to {self._port} ...')
                ser = serial.Serial(self._port, 115200, timeout=3)

                # A1M8 motor controlled via DTR: False = motor ON.
                ser.dtr = False
                time.sleep(1.0)

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

                # Start Express scan
                ser.write(_EXPRESS_CMD)
                desc = ser.read(7)
                if len(desc) < 7 or desc[0] != 0xA5 or desc[1] != 0x5A:
                    raise RuntimeError(f'Bad express descriptor: {desc.hex()}')
                self.get_logger().info(f'Express descriptor: {desc.hex()}')
                if desc[6] != 0x82:
                    raise RuntimeError(
                        f'Express scan unsupported by firmware '
                        f'(data_type=0x{desc[6]:02x})'
                    )

                # On tegra-xusb + cp210x, tcsetattr(TCSANOW) flushes the rx
                # buffer — subsequent os.read() blocks even though the lidar
                # is streaming.  pyserial's ser.read() (VMIN=0, VTIME=timeout)
                # works correctly for 84-byte Express packets, so we use it
                # throughout without switching to raw blocking I/O.
                first_pkt = ser.read(_PACKET_SIZE)
                if len(first_pkt) < _PACKET_SIZE:
                    raise RuntimeError(
                        f'Express scan: no data after descriptor '
                        f'(got {len(first_pkt)}/{_PACKET_SIZE} bytes)'
                    )
                self.get_logger().info(
                    f'Express scan started. '
                    f'First packet sync: 0x{first_pkt[0]:02x} 0x{first_pkt[1]:02x}'
                )

                # Validate sync; byte-level resync via pyserial if needed
                if (first_pkt[0] >> 4) == 0xA and (first_pkt[1] >> 4) == 0x5:
                    pkt = first_pkt
                else:
                    self.get_logger().warn('Bad sync on first Express packet — resyncing')
                    pkt = self._resync_pyserial(ser)

                prev_start = None
                prev_cabins = None
                scan = []
                last_scan_t = 0.0
                scan_time = 1.0 / 10.0
                _pkt_count = 0
                _new_scan_count = 0
                _last_log = time.monotonic()

                while self._running and rclpy.ok():
                    new_scan, start_deg, cabins = self._parse_packet(pkt)
                    _pkt_count += 1

                    # Decode previous packet now that we have the next start angle
                    if prev_cabins is not None:
                        pts = self._decode_cabins(prev_cabins, prev_start, start_deg)
                        scan.extend(pts)

                    if new_scan:
                        _new_scan_count += 1
                        if scan:
                            now = time.monotonic()
                            if last_scan_t > 0.0:
                                scan_time = now - last_scan_t
                            last_scan_t = now
                            self._publish_scan(scan, scan_time)
                            scan = []
                        else:
                            self.get_logger().warn(
                                f'[DBG] new_scan=True but scan empty (pkt #{_pkt_count})'
                            )

                    prev_start = start_deg
                    prev_cabins = cabins

                    now = time.monotonic()
                    if now - _last_log > 2.0:
                        self.get_logger().info(
                            f'[DBG] pkts={_pkt_count} new_scans={_new_scan_count}'
                            f' scan_buf={len(scan)} start={start_deg:.1f}° '
                            f'b0=0x{pkt[0]:02x} b2b3=0x{pkt[2]:02x}{pkt[3]:02x}'
                        )
                        _last_log = now

                    # Read next packet via pyserial (avoids tcsetattr flush bug)
                    pkt = ser.read(_PACKET_SIZE)
                    if len(pkt) < _PACKET_SIZE:
                        raise RuntimeError(
                            f'Express: short read {len(pkt)}/{_PACKET_SIZE}'
                        )
                    if (pkt[0] >> 4) != 0xA or (pkt[1] >> 4) != 0x5:
                        self.get_logger().warn('Express sync lost — resyncing')
                        pkt = self._resync_pyserial(ser)
                        prev_cabins = None
                        prev_start = None

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

    def _publish_scan(self, scan, scan_time: float = 1.0 / 10.0) -> None:
        n = self._NUM_BINS
        angle_min = -math.pi
        angle_inc = 2.0 * math.pi / n
        ranges = [float('inf')] * n

        for angle_deg, dist_mm in scan:
            if dist_mm <= 0:
                continue
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
