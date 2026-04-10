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
    left_pwm: int | None = None
    right_pwm: int | None = None
    left_velocity_mps: float | None = None
    right_velocity_mps: float | None = None
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
        self._last_encoder_issue_cause: str | None = None
        self._last_cmd_linear_x = 0.0
        self._last_cmd_angular_z = 0.0
        self._move_debug_until_sec = 0.0
        self._last_move_debug_log_time = 0.0
        self._last_direction_mismatch_log_time = 0.0
        self._last_raw_left_steps: int | None = None
        self._last_raw_right_steps: int | None = None
        self._encoder_timeout_count = 0
        self._encoder_rare_packet_count = 0
        self._encoder_seq_gap_total = 0
        self._encoder_restart_count = 0
        self._logged_legacy_encoder_format = False
        # Dead-reckoning компенсация правого энкодера при I2C блокировках.
        # Накапливается при каждом right_i2c_block, применяется ко всем
        # последующим right_steps — так delta следующего пакета остаётся корректной.
        # Сбрасывается при рестарте ESP32 (boot_id изменился).
        self._right_steps_offset: int = 0
        self._right_steps_dr_count: int = 0  # сколько раз применялась компенсация

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
        now_sec = time.time()
        self._last_cmd_linear_x = float(msg.linear.x)
        self._last_cmd_angular_z = float(msg.angular.z)
        event = self._odometry.on_cmd_vel(float(msg.linear.x), float(msg.angular.z))
        if event.movement_started:
            self._move_debug_until_sec = now_sec + 2.5
            self._last_move_debug_log_time = 0.0
            self.get_logger().info(
                f'[MOVE] start cmd_vx={msg.linear.x:.3f} m/s, cmd_vth={msg.angular.z:.3f} rad/s'
            )
        elif event.movement_stopped:
            self._move_debug_until_sec = 0.0
            self.get_logger().info('[MOVE] stop')

    def encoder_callback(self, msg: Int64MultiArray):
        if len(msg.data) < 3:
            return

        now_sec = time.time()
        device_stamp_sec = float(msg.data[2]) / 1000.0
        telemetry = self._parse_encoder_telemetry(msg)
        previous_telemetry = self._last_encoder_telemetry
        left_steps = int(msg.data[0])
        right_steps = int(msg.data[1])
        raw_delta_left_steps = None
        raw_delta_right_steps = None
        if self._last_raw_left_steps is not None:
            raw_delta_left_steps = left_steps - self._last_raw_left_steps
        if self._last_raw_right_steps is not None:
            raw_delta_right_steps = right_steps - self._last_raw_right_steps
        host_gap = None
        device_gap = None

        if self._last_encoder_host_time is not None:
            host_gap = now_sec - self._last_encoder_host_time
        if self._last_encoder_device_time is not None and device_stamp_sec >= self._last_encoder_device_time:
            device_gap = device_stamp_sec - self._last_encoder_device_time

        # Сброс dead-reckoning оффсета при рестарте ESP32 (totalSteps обнуляется).
        is_esp_restart = (
            telemetry.boot_id is not None
            and self._last_encoder_boot_id is not None
            and telemetry.boot_id != self._last_encoder_boot_id
        )
        if is_esp_restart:
            self._right_steps_offset = 0

        seq_gap = self._track_encoder_stream(
            telemetry,
            host_gap,
            device_gap,
            previous_telemetry=previous_telemetry,
        )
        issue_cause = self._infer_encoder_issue_cause(
            telemetry,
            previous_telemetry,
            seq_gap=seq_gap,
            host_gap=host_gap,
            device_gap=device_gap,
        )
        if issue_cause is not None:
            self._last_encoder_issue_cause = issue_cause

        # Dead-reckoning компенсация: при right_i2c_block правый энкодер
        # не накопил шаги за время зависания (~1с). Оцениваем пропущенные
        # шаги через скорость из предыдущего пакета × device_gap.
        # Оффсет накапливается и применяется ко всем следующим пакетам,
        # поэтому delta следующего пакета остаётся корректной (оффсет вычтется).
        if issue_cause == 'right_i2c_block' and device_gap is not None:
            prev_right_vel = (
                previous_telemetry.right_velocity_mps
                if previous_telemetry is not None
                else None
            )
            if prev_right_vel is not None and abs(prev_right_vel) > 0.01:
                meters_per_step = self._params.meters_per_step
                estimated_delta = round(prev_right_vel * device_gap / meters_per_step)
                if estimated_delta != 0:
                    self._right_steps_offset += estimated_delta
                    self._right_steps_dr_count += 1
                    self.get_logger().debug(
                        f'[DR] right_i2c_block: +{estimated_delta} steps компенсация '
                        f'(vel={prev_right_vel:.3f} m/s, gap={device_gap:.3f}s, '
                        f'offset={self._right_steps_offset}, n={self._right_steps_dr_count})'
                    )

        event = self._odometry.on_encoder(
            left_steps=left_steps,
            right_steps=right_steps + self._right_steps_offset,
            now_sec=now_sec,
        )
        self._log_motion_diagnostics(
            now_sec,
            telemetry,
            raw_delta_left_steps,
            raw_delta_right_steps,
            issue_cause,
        )

        if event.switched_to_encoder:
            self.get_logger().info('Переключение на энкодерную одометрию (closed-loop)')
        elif event.restored_encoder:
            suffix = self._format_encoder_gap_suffix(
                host_gap,
                device_gap,
                telemetry,
                previous_telemetry=previous_telemetry,
                seq_gap=seq_gap,
                cause=issue_cause,
            )
            self.get_logger().info(f'Восстановление энкодерной одометрии (closed-loop{suffix})')

        if host_gap is not None and host_gap > self._encoder_warn_gap:
            self._encoder_rare_packet_count += 1
            suffix = self._format_encoder_gap_suffix(
                host_gap,
                device_gap,
                telemetry,
                previous_telemetry=previous_telemetry,
                seq_gap=seq_gap,
                cause=issue_cause,
            )
            self.get_logger().warn(f'Редкий пакет /wheel_encoders{suffix}')

        self._last_encoder_host_time = now_sec
        self._last_encoder_device_time = device_stamp_sec
        self._last_encoder_telemetry = telemetry
        self._last_raw_left_steps = left_steps
        self._last_raw_right_steps = right_steps

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
            left_pwm=self._msg_int(msg.data, 3),
            right_pwm=self._msg_int(msg.data, 4),
            left_velocity_mps=self._msg_velocity(msg.data, 5),
            right_velocity_mps=self._msg_velocity(msg.data, 6),
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
            left_pwm=telemetry.left_pwm,
            right_pwm=telemetry.right_pwm,
            left_velocity_mps=telemetry.left_velocity_mps,
            right_velocity_mps=telemetry.right_velocity_mps,
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
        previous_telemetry: EncoderTelemetry | None = None,
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
                suffix = self._format_encoder_gap_suffix(
                    host_gap,
                    device_gap,
                    telemetry,
                    previous_telemetry=previous_telemetry,
                    cause='esp_restart',
                )
                self.get_logger().error(
                    'Поток /wheel_encoders перезапущен на ESP32; '
                    f'boot_id={self._last_encoder_boot_id}->{telemetry.boot_id}{suffix}'
                )
                self._last_encoder_seq = None
                self._last_encoder_issue_cause = 'esp_restart'

        if telemetry.sequence is not None and self._last_encoder_seq is not None:
            same_boot = telemetry.boot_id is None or telemetry.boot_id == self._last_encoder_boot_id
            if same_boot and telemetry.sequence > self._last_encoder_seq + 1:
                seq_gap = telemetry.sequence - self._last_encoder_seq - 1
                self._encoder_seq_gap_total += seq_gap
                suffix = self._format_encoder_gap_suffix(
                    host_gap,
                    device_gap,
                    telemetry,
                    previous_telemetry=previous_telemetry,
                    seq_gap=seq_gap,
                    cause='packet_drop',
                )
                self.get_logger().error(f'Пропуск пакетов /wheel_encoders{suffix}')
                self._last_encoder_issue_cause = 'packet_drop'
            elif same_boot and telemetry.sequence <= self._last_encoder_seq:
                suffix = self._format_encoder_gap_suffix(
                    host_gap,
                    device_gap,
                    telemetry,
                    previous_telemetry=previous_telemetry,
                    cause='non_monotonic_seq',
                )
                self.get_logger().warn(f'Немонотонная последовательность /wheel_encoders{suffix}')
                self._last_encoder_issue_cause = 'non_monotonic_seq'

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
        if self._last_encoder_issue_cause is not None:
            parts.insert(0, f'cause={self._last_encoder_issue_cause}')
        if self._last_encoder_telemetry is not None:
            if self._last_encoder_telemetry.sequence is not None:
                parts.append(f'last_seq={self._last_encoder_telemetry.sequence}')
            if self._last_encoder_telemetry.boot_id is not None:
                parts.append(f'last_boot={self._last_encoder_telemetry.boot_id}')
            if self._last_encoder_telemetry.loop_dt_ms is not None:
                parts.append(f'loop={self._last_encoder_telemetry.loop_dt_ms}ms')
            if self._last_encoder_telemetry.read_left_ms is not None and self._last_encoder_telemetry.read_left_ms >= 20:
                parts.append(f'i2c_l={self._last_encoder_telemetry.read_left_ms}ms')
            if self._last_encoder_telemetry.read_right_ms is not None and self._last_encoder_telemetry.read_right_ms >= 20:
                parts.append(f'i2c_r={self._last_encoder_telemetry.read_right_ms}ms')
            if self._last_encoder_telemetry.spin_ms is not None and self._last_encoder_telemetry.spin_ms >= 20:
                parts.append(f'spin={self._last_encoder_telemetry.spin_ms}ms')
            if self._last_encoder_telemetry.prev_publish_ms is not None and self._last_encoder_telemetry.prev_publish_ms >= 20:
                parts.append(f'pub={self._last_encoder_telemetry.prev_publish_ms}ms')
        return '; ' + ', '.join(parts)

    @staticmethod
    def _msg_int(data, index: int) -> int | None:
        if len(data) <= index:
            return None
        return int(data[index])

    @staticmethod
    def _msg_velocity(data, index: int) -> float | None:
        raw_value = OdometryNode._msg_int(data, index)
        if raw_value is None:
            return None
        return float(raw_value) / 10000.0

    @staticmethod
    def _sign(value: float | int, threshold: float = 1e-6) -> int:
        if value > threshold:
            return 1
        if value < -threshold:
            return -1
        return 0

    def _max_loop_increased(
        self,
        telemetry: EncoderTelemetry | None,
        previous_telemetry: EncoderTelemetry | None,
    ) -> bool:
        if telemetry is None or telemetry.max_loop_dt_ms is None:
            return False
        if previous_telemetry is None or previous_telemetry.max_loop_dt_ms is None:
            return False
        return telemetry.max_loop_dt_ms > previous_telemetry.max_loop_dt_ms

    def _format_motion_debug_suffix(
        self,
        telemetry: EncoderTelemetry | None,
        raw_delta_left_steps: int | None,
        raw_delta_right_steps: int | None,
        include_odom: bool = True,
    ) -> str:
        state = self._odometry.snapshot()
        parts = [
            f'cmd_vx={self._last_cmd_linear_x:.3f}',
            f'cmd_vth={self._last_cmd_angular_z:.3f}',
        ]
        if include_odom:
            parts.extend(
                [
                    f'odom_vx={state.vx:.3f}',
                    f'odom_vth={state.vth:.3f}',
                ]
            )
        if raw_delta_left_steps is not None:
            parts.append(f'dL={raw_delta_left_steps}')
        if raw_delta_right_steps is not None:
            parts.append(f'dR={raw_delta_right_steps}')
        if telemetry is not None:
            if telemetry.left_pwm is not None:
                parts.append(f'pwm_l={telemetry.left_pwm}')
            if telemetry.right_pwm is not None:
                parts.append(f'pwm_r={telemetry.right_pwm}')
            if telemetry.left_velocity_mps is not None:
                parts.append(f'vel_l={telemetry.left_velocity_mps:.3f}')
            if telemetry.right_velocity_mps is not None:
                parts.append(f'vel_r={telemetry.right_velocity_mps:.3f}')
            if telemetry.sequence is not None:
                parts.append(f'seq={telemetry.sequence}')
        return ', '.join(parts)

    def _infer_direction_mismatch(
        self,
        telemetry: EncoderTelemetry | None,
        raw_delta_left_steps: int | None,
        raw_delta_right_steps: int | None,
    ) -> str | None:
        expected_forward = self._sign(self._last_cmd_linear_x, 0.05)
        if expected_forward == 0:
            return None

        state = self._odometry.snapshot()
        odom_sign = self._sign(state.vx, 0.05)
        left_vel_sign = self._sign(telemetry.left_velocity_mps, 0.05) if telemetry and telemetry.left_velocity_mps is not None else 0
        right_vel_sign = self._sign(telemetry.right_velocity_mps, 0.05) if telemetry and telemetry.right_velocity_mps is not None else 0
        left_delta_sign = self._sign(raw_delta_left_steps, 2.0) if raw_delta_left_steps is not None else 0
        right_delta_sign = self._sign(raw_delta_right_steps, 2.0) if raw_delta_right_steps is not None else 0
        left_pwm_sign = self._sign(telemetry.left_pwm, 5.0) if telemetry and telemetry.left_pwm is not None else 0
        right_pwm_sign = self._sign(telemetry.right_pwm, 5.0) if telemetry and telemetry.right_pwm is not None else 0

        reverse_feedback = (
            left_pwm_sign == expected_forward
            and right_pwm_sign == expected_forward
            and left_vel_sign == -expected_forward
            and right_vel_sign == -expected_forward
        )
        reverse_deltas = (
            left_delta_sign == -expected_forward
            and right_delta_sign == -expected_forward
        )

        if reverse_feedback:
            return 'closed_loop_polarity_mismatch'
        if reverse_deltas and odom_sign == -expected_forward:
            return 'encoder_direction_reversed'
        if odom_sign == -expected_forward:
            return 'odom_reverse_vs_cmd'
        return None

    def _log_motion_diagnostics(
        self,
        now_sec: float,
        telemetry: EncoderTelemetry | None,
        raw_delta_left_steps: int | None,
        raw_delta_right_steps: int | None,
        issue_cause: str | None,
    ) -> None:
        mismatch_cause = self._infer_direction_mismatch(
            telemetry,
            raw_delta_left_steps,
            raw_delta_right_steps,
        )
        if mismatch_cause is not None and now_sec - self._last_direction_mismatch_log_time >= 0.5:
            self._last_direction_mismatch_log_time = now_sec
            self.get_logger().error(
                f'[MOVECHK] {mismatch_cause}; '
                + self._format_motion_debug_suffix(
                    telemetry,
                    raw_delta_left_steps,
                    raw_delta_right_steps,
                )
            )
            return

        if issue_cause == 'packet_drop':
            if now_sec - self._last_move_debug_log_time >= 0.5:
                self._last_move_debug_log_time = now_sec
                self.get_logger().warn(
                    f'[MOVECHK] packet_drop context; '
                    + self._format_motion_debug_suffix(
                        telemetry,
                        raw_delta_left_steps,
                        raw_delta_right_steps,
                    )
                )
            return

        if now_sec > self._move_debug_until_sec:
            return
        if now_sec - self._last_move_debug_log_time < 0.35:
            return

        self._last_move_debug_log_time = now_sec
        self.get_logger().info(
            f'[MOVECHK] start window; '
            + self._format_motion_debug_suffix(
                telemetry,
                raw_delta_left_steps,
                raw_delta_right_steps,
            )
        )

    def _counter_delta(
        self,
        telemetry: EncoderTelemetry | None,
        previous_telemetry: EncoderTelemetry | None,
        attr_name: str,
    ) -> int | None:
        if telemetry is None or previous_telemetry is None:
            return None
        if (
            telemetry.boot_id is not None
            and previous_telemetry.boot_id is not None
            and telemetry.boot_id != previous_telemetry.boot_id
        ):
            return None

        current_value = getattr(telemetry, attr_name)
        previous_value = getattr(previous_telemetry, attr_name)
        if current_value is None or previous_value is None:
            return None

        delta = current_value - previous_value
        if delta <= 0:
            return None
        return delta

    def _infer_encoder_issue_cause(
        self,
        telemetry: EncoderTelemetry | None,
        previous_telemetry: EncoderTelemetry | None,
        seq_gap: int | None,
        host_gap: float | None,
        device_gap: float | None,
    ) -> str | None:
        if telemetry is None:
            return None
        if seq_gap:
            return 'packet_drop'
        if telemetry.read_right_ms is not None and telemetry.read_right_ms >= 900:
            return 'right_i2c_block'
        if telemetry.read_left_ms is not None and telemetry.read_left_ms >= 900:
            return 'left_i2c_block'
        if (
            self._counter_delta(telemetry, previous_telemetry, 'right_i2c_tx_fail_count')
            or self._counter_delta(telemetry, previous_telemetry, 'right_i2c_short_read_count')
        ):
            return 'right_i2c_errors'
        if (
            self._counter_delta(telemetry, previous_telemetry, 'left_i2c_tx_fail_count')
            or self._counter_delta(telemetry, previous_telemetry, 'left_i2c_short_read_count')
        ):
            return 'left_i2c_errors'
        if telemetry.spin_ms is not None and telemetry.spin_ms >= 900:
            return 'executor_block'
        if telemetry.prev_publish_ms is not None and telemetry.prev_publish_ms >= 900:
            return 'publish_block'
        if (
            self._counter_delta(telemetry, previous_telemetry, 'transport_write_drop_count')
            or self._counter_delta(telemetry, previous_telemetry, 'transport_short_write_count')
        ):
            return 'serial_tx_issue'
        if (
            telemetry.loop_dt_ms is not None
            and telemetry.loop_dt_ms >= 900
            or self._counter_delta(telemetry, previous_telemetry, 'long_loop_count')
        ):
            return 'esp_loop_gap'
        if host_gap is not None and device_gap is not None and host_gap >= 0.9 and device_gap >= 0.9:
            return 'esp_or_transport_gap'
        if host_gap is not None and host_gap >= 0.9:
            return 'host_gap'
        return None

    def _format_encoder_gap_suffix(
        self,
        host_gap: float | None,
        device_gap: float | None,
        telemetry: EncoderTelemetry | None,
        previous_telemetry: EncoderTelemetry | None = None,
        seq_gap: int | None = None,
        cause: str | None = None,
    ) -> str:
        parts: list[str] = []
        if cause is not None:
            parts.append(f'cause={cause}')
        if host_gap is not None:
            parts.append(f'host={host_gap:.3f}s')
        if device_gap is not None:
            parts.append(f'esp={device_gap:.3f}s')
        if telemetry is not None:
            if telemetry.sequence is not None:
                parts.append(f'seq={telemetry.sequence}')
            if seq_gap is not None:
                parts.append(f'seq_gap={seq_gap}')
            if telemetry.boot_id is not None:
                parts.append(f'boot={telemetry.boot_id}')
            if telemetry.loop_dt_ms is not None:
                parts.append(f'loop={telemetry.loop_dt_ms}ms')
            if telemetry.read_left_ms is not None and (
                telemetry.read_left_ms >= 20 or cause in {'left_i2c_block', 'left_i2c_errors'}
            ):
                parts.append(f'i2c_l={telemetry.read_left_ms}ms')
            if telemetry.read_right_ms is not None and (
                telemetry.read_right_ms >= 20 or cause in {'right_i2c_block', 'right_i2c_errors'}
            ):
                parts.append(f'i2c_r={telemetry.read_right_ms}ms')
            if telemetry.spin_ms is not None and (
                telemetry.spin_ms >= 20 or cause == 'executor_block'
            ):
                parts.append(f'spin={telemetry.spin_ms}ms')
            if telemetry.prev_publish_ms is not None and (
                telemetry.prev_publish_ms >= 20 or cause == 'publish_block'
            ):
                parts.append(f'pub={telemetry.prev_publish_ms}ms')
            if (
                telemetry.max_loop_dt_ms is not None
                and (
                    cause == 'esp_loop_gap'
                    or self._max_loop_increased(telemetry, previous_telemetry)
                )
            ):
                parts.append(f'max_loop={telemetry.max_loop_dt_ms}ms')
            if cause == 'packet_drop':
                if telemetry.cmd_age_ms is not None:
                    parts.append(f'cmd_age={telemetry.cmd_age_ms}ms')
                if telemetry.velocity_mode is not None:
                    parts.append(f'vel_mode={int(telemetry.velocity_mode)}')
                if telemetry.left_pwm is not None:
                    parts.append(f'pwm_l={telemetry.left_pwm}')
                if telemetry.right_pwm is not None:
                    parts.append(f'pwm_r={telemetry.right_pwm}')
                if telemetry.left_velocity_mps is not None:
                    parts.append(f'vel_l={telemetry.left_velocity_mps:.3f}')
                if telemetry.right_velocity_mps is not None:
                    parts.append(f'vel_r={telemetry.right_velocity_mps:.3f}')

            counter_labels = (
                ('right_i2c_tx_fail_count', 'i2c_r_tx'),
                ('right_i2c_short_read_count', 'i2c_r_short'),
                ('left_i2c_tx_fail_count', 'i2c_l_tx'),
                ('left_i2c_short_read_count', 'i2c_l_short'),
                ('transport_write_drop_count', 'tx_drop'),
                ('transport_short_write_count', 'tx_short'),
                ('spin_error_count', 'spin_err'),
                ('publish_error_count', 'pub_err'),
                ('encoder_jump_reject_count', 'enc_jump'),
                ('long_loop_count', 'loops'),
                ('cmd_timeout_stop_count', 'cmd_to'),
            )
            for attr_name, label in counter_labels:
                delta = self._counter_delta(telemetry, previous_telemetry, attr_name)
                if delta is not None:
                    parts.append(f'{label}=+{delta}')
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
