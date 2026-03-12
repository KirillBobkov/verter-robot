#!/usr/bin/env python3

"""ROS adapter for odometry use-case."""

from dataclasses import replace
import math
import time

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster
from verter_admin.control.application import ComputeOdometry
from verter_admin.control.domain import OdometryParameters, OdometrySource


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

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int64MultiArray, '/wheel_encoders', self.encoder_callback, 10)

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
        loop_dt_ms = int(msg.data[7]) if len(msg.data) >= 8 else None
        host_gap = None
        device_gap = None

        if self._last_encoder_host_time is not None:
            host_gap = now_sec - self._last_encoder_host_time
        if self._last_encoder_device_time is not None and device_stamp_sec >= self._last_encoder_device_time:
            device_gap = device_stamp_sec - self._last_encoder_device_time

        event = self._odometry.on_encoder(
            left_steps=int(msg.data[0]),
            right_steps=int(msg.data[1]),
            now_sec=now_sec,
        )

        if event.switched_to_encoder:
            self.get_logger().info('Переключение на энкодерную одометрию (closed-loop)')
        elif event.restored_encoder:
            suffix = self._format_encoder_gap_suffix(host_gap, device_gap, loop_dt_ms)
            self.get_logger().info(f'Восстановление энкодерной одометрии (closed-loop{suffix})')

        if host_gap is not None and host_gap > self._encoder_warn_gap:
            suffix = self._format_encoder_gap_suffix(host_gap, device_gap, loop_dt_ms)
            self.get_logger().warn(f'Редкий пакет /wheel_encoders{suffix}')

        self._last_encoder_host_time = now_sec
        self._last_encoder_device_time = device_stamp_sec

    def update_odometry_callback(self):
        event = self._odometry.on_tick(now_sec=time.time())
        if event.switched_to_cmd_vel:
            self.get_logger().warn(
                'Таймаут данных энкодеров - переключение на cmd_vel (open-loop); '
                f'gap={event.encoder_gap_sec:.3f}s'
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

    @staticmethod
    def _format_encoder_gap_suffix(
        host_gap: float | None,
        device_gap: float | None,
        loop_dt_ms: int | None,
    ) -> str:
        parts: list[str] = []
        if host_gap is not None:
            parts.append(f'host_gap={host_gap:.3f}s')
        if device_gap is not None:
            parts.append(f'esp_gap={device_gap:.3f}s')
        if loop_dt_ms is not None:
            parts.append(f'loop_dt={loop_dt_ms}ms')
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
