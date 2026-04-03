#!/usr/bin/env python3

"""ROS adapter for odometry use-case."""

from dataclasses import dataclass, replace
import math
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster
from verter_admin.control.application import ComputeOdometry
from verter_admin.control.domain import OdometryParameters, OdometrySource


@dataclass(frozen=True)
class EncoderTelemetry:
    """Decoded debug payload from /wheel_encoders."""

    payload_len: int
    loop_dt_ms: int | None = None
    read_left_ms: int | None = None
    read_right_ms: int | None = None
    spin_ms: int | None = None
    prev_publish_ms: int | None = None
    sequence: int | None = None
    boot_id: int | None = None
    reset_reason: int | None = None
    spin_rc: int | None = None
    publish_rc: int | None = None
    transport_write_drop_count: int | None = None
    transport_short_write_count: int | None = None
    transport_last_available: int | None = None
    transport_last_required: int | None = None
    spin_error_count: int | None = None
    publish_error_count: int | None = None
    left_i2c_tx_fail_count: int | None = None
    left_i2c_short_read_count: int | None = None
    right_i2c_tx_fail_count: int | None = None
    right_i2c_short_read_count: int | None = None
    encoder_jump_reject_count: int | None = None
    long_loop_count: int | None = None
    max_loop_dt_ms: int | None = None
    cmd_timeout_stop_count: int | None = None
    cmd_age_ms: int | None = None
    free_heap_bytes: int | None = None
    min_free_heap_bytes: int | None = None
    velocity_mode: bool | None = None


class OdometryNode(Node):
    """Publishes /odom (+ optional TF) from encoder or cmd_vel fallback."""

    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('publish_tf', True)
        self.publish_tf = self.get_parameter('publish_tf').value

        self._params = OdometryParameters()
        self.declare_parameter('encoder_timeout', self._params.encoder_timeout)
        self.declare_parameter('encoder_warn_gap', 0.2)
        self._params = replace(
            self._params,
            encoder_timeout=float(self.get_parameter('encoder_timeout').value),
        )
        self._encoder_warn_gap = float(self.get_parameter('encoder_warn_gap').value)

        now_sec = time.time()
        self._odometry = ComputeOdometry(self._params, now_sec)
        self._last_log_time = now_sec
        self._last_encoder_host_time: float | None = None
        self._last_encoder_device_time: float | None = None
        self._last_encoder_telemetry: EncoderTelemetry | None = None
        self._last_encoder_seq: int | None = None
        self._last_encoder_boot_id: int | None = None
        self._encoder_timeout_count = 0
        self._encoder_rare_packet_count = 0
        self._encoder_seq_gap_total = 0
        self._encoder_restart_count = 0
        self._logged_legacy_encoder_format = False

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        wheel_encoders_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(
            Int64MultiArray,
            '/wheel_encoders',
            self.encoder_callback,
            wheel_encoders_qos,
        )

        update_frequency = 50.0
        self.create_timer(1.0 / update_frequency, self.update_odometry_callback)

        self.get_logger().info('Публикатор одометрии создан для топика /odom')
        self.get_logger().info('TF broadcaster инициализирован')
        self.get_logger().info('Подписчик создан для топика /cmd_vel')
        self.get_logger().info('Подписчик создан для топика /wheel_encoders')
        self.get_logger().info(f'Таймер одометрии запущен с частотой {update_frequency} Гц')
        self.get_logger().info(
            f'Порог таймаута энкодеров: {self._params.encoder_timeout:.3f} сек '
            f'(warn gap: {self._encoder_warn_gap:.3f} сек)'
        )
        self.get_logger().info('=' * 80)
        self.get_logger().info('Нода одометрии успешно инициализирована!')
        self.get_logger().info('Начальная позиция: x=0.0, y=0.0, theta=0.0')
        self.get_logger().info('Ожидание данных энкодеров (fallback на cmd_vel)')
        self.get_logger().info('=' * 80)

    def cmd_vel_callback(self, msg: Twist):
        event = self._odometry.on_cmd_vel(float(msg.linear.x), float(msg.angular.z))
        if event.movement_started:
            self.get_logger().info(
                f'Начало движения: cmd_vx={msg.linear.x:.3f} м/с, cmd_vth={msg.angular.z:.3f} рад/с'
            )
        elif event.movement_stopped:
            self.get_logger().info('Остановка')

    def encoder_callback(self, msg: Int64MultiArray):
        if len(msg.data) < 3:
            return

        now_sec = time.time()
        device_stamp_sec = float(msg.data[2]) / 1000.0
        telemetry = self._parse_encoder_telemetry(msg)
        host_gap = None
        device_gap = None

        if self._last_encoder_host_time is not None:
            host_gap = now_sec - self._last_encoder_host_time
        if self._last_encoder_device_time is not None and device_stamp_sec >= self._last_encoder_device_time:
            device_gap = device_stamp_sec - self._last_encoder_device_time

        seq_gap = self._track_encoder_stream(telemetry, host_gap, device_gap)

        event = self._odometry.on_encoder(
            left_steps=int(msg.data[0]),
            right_steps=int(msg.data[1]),
            now_sec=now_sec,
        )

        if event.switched_to_encoder:
            self.get_logger().info('Переключение на энкодерную одометрию (closed-loop)')
        elif event.restored_encoder:
            suffix = self._format_encoder_gap_suffix(
                host_gap,
                device_gap,
                telemetry,
                seq_gap=seq_gap,
            )
            self.get_logger().info(f'Восстановление энкодерной одометрии (closed-loop{suffix})')

        if host_gap is not None and host_gap > self._encoder_warn_gap:
            self._encoder_rare_packet_count += 1
            suffix = self._format_encoder_gap_suffix(
                host_gap,
                device_gap,
                telemetry,
                seq_gap=seq_gap,
            )
            self.get_logger().warn(f'Редкий пакет /wheel_encoders{suffix}')

        self._last_encoder_host_time = now_sec
        self._last_encoder_device_time = device_stamp_sec
        self._last_encoder_telemetry = telemetry

    def update_odometry_callback(self):
        event = self._odometry.on_tick(now_sec=time.time())
        if event.switched_to_cmd_vel:
            self._encoder_timeout_count += 1
            suffix = self._format_timeout_suffix()
            self.get_logger().warn(
                'Таймаут данных энкодеров - переключение на cmd_vel (open-loop); '
                f'gap={event.encoder_gap_sec:.3f}s{suffix}'
            )
        if event.large_dt:
            self.get_logger().warn('Большой временной интервал: >1.000 сек. Сбрасываем.')

        self._publish_odometry()
        if self.publish_tf:
            self._publish_transform()

    def _publish_odometry(self):
        state = self._odometry.snapshot()
        is_fallback = state.source == OdometrySource.CMD_VEL

        # During CMD_VEL fallback position is frozen — inflate covariance so
        # EKF reduces trust in odometry and defers to scan matching instead.
        pose_xy_cov = 5.0 if is_fallback else 0.05
        pose_yaw_cov = 5.0 if is_fallback else 0.15

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        odom_msg.pose.pose.position.x = state.x
        odom_msg.pose.pose.position.y = state.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self._euler_to_quaternion(0.0, 0.0, state.theta)

        odom_msg.pose.covariance = [
            pose_xy_cov, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, pose_xy_cov, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, pose_yaw_cov,
        ]

        odom_msg.twist.twist.linear.x = state.vx
        odom_msg.twist.twist.linear.y = state.vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = state.vth

        odom_msg.twist.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1e6, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]

        self.odom_publisher.publish(odom_msg)

        if state.current_time - self._last_log_time > 1.0:
            source_label = '[ENC]' if state.source == OdometrySource.ENCODER else '[CMD]'
            self.get_logger().info(
                f'{source_label} Позиция: x={state.x:.3f}м, y={state.y:.3f}м, '
                f'θ={math.degrees(state.theta):.1f}° | '
                f'Скорость: {state.vx:.2f}м/с, {math.degrees(state.vth):.1f}°/с'
            )
            self._last_log_time = state.current_time

    def _publish_transform(self):
        state = self._odometry.snapshot()

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.translation.x = state.x
        transform.transform.translation.y = state.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self._euler_to_quaternion(0.0, 0.0, state.theta)
        self.tf_broadcaster.sendTransform(transform)

    def _parse_encoder_telemetry(self, msg: Int64MultiArray) -> EncoderTelemetry:
        payload_len = len(msg.data)
        velocity_mode_raw = self._msg_int(msg.data, 34)
        telemetry = EncoderTelemetry(
            payload_len=payload_len,
            loop_dt_ms=self._msg_int(msg.data, 7),
            read_left_ms=self._msg_int(msg.data, 8),
            read_right_ms=self._msg_int(msg.data, 9),
            spin_ms=self._msg_int(msg.data, 10),
            prev_publish_ms=self._msg_int(msg.data, 11),
        )

        if payload_len < 35:
            if not self._logged_legacy_encoder_format:
                self.get_logger().warn(
                    'Получен legacy /wheel_encoders без расширенной диагностики; '
                    f'payload_len={payload_len}'
                )
                self._logged_legacy_encoder_format = True
            return telemetry

        return EncoderTelemetry(
            payload_len=payload_len,
            loop_dt_ms=telemetry.loop_dt_ms,
            read_left_ms=telemetry.read_left_ms,
            read_right_ms=telemetry.read_right_ms,
            spin_ms=telemetry.spin_ms,
            prev_publish_ms=telemetry.prev_publish_ms,
            sequence=self._msg_int(msg.data, 12),
            boot_id=self._msg_int(msg.data, 13),
            reset_reason=self._msg_int(msg.data, 14),
            spin_rc=self._msg_int(msg.data, 15),
            publish_rc=self._msg_int(msg.data, 16),
            transport_write_drop_count=self._msg_int(msg.data, 17),
            transport_short_write_count=self._msg_int(msg.data, 18),
            transport_last_available=self._msg_int(msg.data, 19),
            transport_last_required=self._msg_int(msg.data, 20),
            spin_error_count=self._msg_int(msg.data, 21),
            publish_error_count=self._msg_int(msg.data, 22),
            left_i2c_tx_fail_count=self._msg_int(msg.data, 23),
            left_i2c_short_read_count=self._msg_int(msg.data, 24),
            right_i2c_tx_fail_count=self._msg_int(msg.data, 25),
            right_i2c_short_read_count=self._msg_int(msg.data, 26),
            encoder_jump_reject_count=self._msg_int(msg.data, 27),
            long_loop_count=self._msg_int(msg.data, 28),
            max_loop_dt_ms=self._msg_int(msg.data, 29),
            cmd_timeout_stop_count=self._msg_int(msg.data, 30),
            cmd_age_ms=self._msg_int(msg.data, 31),
            free_heap_bytes=self._msg_int(msg.data, 32),
            min_free_heap_bytes=self._msg_int(msg.data, 33),
            velocity_mode=bool(velocity_mode_raw) if velocity_mode_raw is not None else None,
        )

    def _track_encoder_stream(
        self,
        telemetry: EncoderTelemetry,
        host_gap: float | None,
        device_gap: float | None,
    ) -> int | None:
        seq_gap = None

        if telemetry.boot_id is not None:
            if self._last_encoder_boot_id is None:
                self.get_logger().info(
                    'Расширенная диагностика /wheel_encoders активна; '
                    f'boot_id={telemetry.boot_id}, reset_reason={telemetry.reset_reason}, '
                    f'seq={telemetry.sequence}'
                )
            elif telemetry.boot_id != self._last_encoder_boot_id:
                self._encoder_restart_count += 1
                suffix = self._format_encoder_gap_suffix(host_gap, device_gap, telemetry)
                self.get_logger().error(
                    'Поток /wheel_encoders перезапущен на ESP32; '
                    f'boot_id={self._last_encoder_boot_id}->{telemetry.boot_id}{suffix}'
                )
                self._last_encoder_seq = None

        if telemetry.sequence is not None and self._last_encoder_seq is not None:
            same_boot = telemetry.boot_id is None or telemetry.boot_id == self._last_encoder_boot_id
            if same_boot and telemetry.sequence > self._last_encoder_seq + 1:
                seq_gap = telemetry.sequence - self._last_encoder_seq - 1
                self._encoder_seq_gap_total += seq_gap
                suffix = self._format_encoder_gap_suffix(host_gap, device_gap, telemetry, seq_gap=seq_gap)
                self.get_logger().error(f'Пропуск пакетов /wheel_encoders{suffix}')
            elif same_boot and telemetry.sequence <= self._last_encoder_seq:
                suffix = self._format_encoder_gap_suffix(host_gap, device_gap, telemetry)
                self.get_logger().warn(f'Немонотонная последовательность /wheel_encoders{suffix}')

        if telemetry.boot_id is not None:
            self._last_encoder_boot_id = telemetry.boot_id
        if telemetry.sequence is not None:
            self._last_encoder_seq = telemetry.sequence
        return seq_gap

    def _format_timeout_suffix(self) -> str:
        parts = [
            f'timeouts={self._encoder_timeout_count}',
            f'rare={self._encoder_rare_packet_count}',
            f'seq_loss={self._encoder_seq_gap_total}',
            f'restarts={self._encoder_restart_count}',
        ]
        if self._last_encoder_telemetry is not None:
            parts.append(f'last_payload={self._last_encoder_telemetry.payload_len}')
            if self._last_encoder_telemetry.sequence is not None:
                parts.append(f'last_seq={self._last_encoder_telemetry.sequence}')
            if self._last_encoder_telemetry.boot_id is not None:
                parts.append(f'last_boot={self._last_encoder_telemetry.boot_id}')
            if self._last_encoder_telemetry.reset_reason is not None:
                parts.append(f'last_reset={self._last_encoder_telemetry.reset_reason}')
            if self._last_encoder_telemetry.loop_dt_ms is not None:
                parts.append(f'last_loop_dt={self._last_encoder_telemetry.loop_dt_ms}ms')
            if self._last_encoder_telemetry.spin_ms is not None:
                parts.append(f'last_spin={self._last_encoder_telemetry.spin_ms}ms')
            if self._last_encoder_telemetry.prev_publish_ms is not None:
                parts.append(f'last_pub={self._last_encoder_telemetry.prev_publish_ms}ms')
        return '; ' + ', '.join(parts)

    @staticmethod
    def _msg_int(data, index: int) -> int | None:
        if len(data) <= index:
            return None
        return int(data[index])

    @staticmethod
    def _format_encoder_gap_suffix(
        host_gap: float | None,
        device_gap: float | None,
        telemetry: EncoderTelemetry | None,
        seq_gap: int | None = None,
    ) -> str:
        parts: list[str] = []
        if host_gap is not None:
            parts.append(f'host_gap={host_gap:.3f}s')
        if device_gap is not None:
            parts.append(f'esp_gap={device_gap:.3f}s')
        if telemetry is not None:
            parts.append(f'payload={telemetry.payload_len}')
            if telemetry.sequence is not None:
                parts.append(f'seq={telemetry.sequence}')
            if seq_gap is not None:
                parts.append(f'seq_gap={seq_gap}')
            if telemetry.boot_id is not None:
                parts.append(f'boot={telemetry.boot_id}')
            if telemetry.reset_reason is not None:
                parts.append(f'reset={telemetry.reset_reason}')
            if telemetry.loop_dt_ms is not None:
                parts.append(f'loop_dt={telemetry.loop_dt_ms}ms')
            if telemetry.read_left_ms is not None:
                parts.append(f'i2c_l={telemetry.read_left_ms}ms')
            if telemetry.read_right_ms is not None:
                parts.append(f'i2c_r={telemetry.read_right_ms}ms')
            if telemetry.spin_ms is not None:
                parts.append(f'spin={telemetry.spin_ms}ms')
            if telemetry.prev_publish_ms is not None:
                parts.append(f'pub_prev={telemetry.prev_publish_ms}ms')
            if telemetry.cmd_age_ms is not None:
                parts.append(f'cmd_age={telemetry.cmd_age_ms}ms')
            if telemetry.velocity_mode is not None:
                parts.append(f'vel_mode={int(telemetry.velocity_mode)}')
            if telemetry.spin_rc not in (None, 0):
                parts.append(f'spin_rc={telemetry.spin_rc}')
            if telemetry.publish_rc not in (None, 0):
                parts.append(f'pub_rc={telemetry.publish_rc}')
            if telemetry.transport_write_drop_count:
                parts.append(f'tx_drop={telemetry.transport_write_drop_count}')
            if telemetry.transport_short_write_count:
                parts.append(f'tx_short={telemetry.transport_short_write_count}')
            if telemetry.transport_last_available not in (None, 0):
                parts.append(f'tx_avail={telemetry.transport_last_available}')
            if telemetry.transport_last_required not in (None, 0):
                parts.append(f'tx_need={telemetry.transport_last_required}')
            if telemetry.spin_error_count:
                parts.append(f'spin_err={telemetry.spin_error_count}')
            if telemetry.publish_error_count:
                parts.append(f'pub_err={telemetry.publish_error_count}')
            if telemetry.left_i2c_tx_fail_count:
                parts.append(f'i2c_l_tx_fail={telemetry.left_i2c_tx_fail_count}')
            if telemetry.left_i2c_short_read_count:
                parts.append(f'i2c_l_short={telemetry.left_i2c_short_read_count}')
            if telemetry.right_i2c_tx_fail_count:
                parts.append(f'i2c_r_tx_fail={telemetry.right_i2c_tx_fail_count}')
            if telemetry.right_i2c_short_read_count:
                parts.append(f'i2c_r_short={telemetry.right_i2c_short_read_count}')
            if telemetry.encoder_jump_reject_count:
                parts.append(f'enc_jump_reject={telemetry.encoder_jump_reject_count}')
            if telemetry.long_loop_count:
                parts.append(f'long_loops={telemetry.long_loop_count}')
            if telemetry.max_loop_dt_ms is not None:
                parts.append(f'max_loop_dt={telemetry.max_loop_dt_ms}ms')
            if telemetry.cmd_timeout_stop_count:
                parts.append(f'cmd_to={telemetry.cmd_timeout_stop_count}')
            if telemetry.free_heap_bytes is not None:
                parts.append(f'heap={telemetry.free_heap_bytes}')
            if telemetry.min_free_heap_bytes is not None:
                parts.append(f'heap_min={telemetry.min_free_heap_bytes}')
        if not parts:
            return ''
        return '; ' + ', '.join(parts)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        quaternion = Quaternion()
        quaternion.w = cr * cp * cy + sr * sp * sy
        quaternion.x = sr * cp * cy - cr * sp * sy
        quaternion.y = cr * sp * cy + sr * cp * sy
        quaternion.z = cr * cp * sy - sr * sp * cy
        return quaternion

    def destroy_node(self):
        state = self._odometry.snapshot()
        self.get_logger().info('Завершение работы ноды одометрии')
        self.get_logger().info(
            f'Финальная позиция: x={state.x:.3f}м, y={state.y:.3f}м, θ={math.degrees(state.theta):.1f}°'
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()
    try:
        rclpy.spin(odometry_node)
    except KeyboardInterrupt:
        odometry_node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        odometry_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
