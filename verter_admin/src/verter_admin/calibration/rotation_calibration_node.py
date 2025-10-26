#!/usr/bin/env python3
"""
===================================================================================
ROTATION CALIBRATION NODE - Калибровка поворотов робота
===================================================================================

Назначение:
-----------
Помогает откалибровать параметр angular_scale в odometry_node.

Что делает:
-----------
1. Командует роботу повернуться на 360° (полный оборот)
2. Вы измеряете реальный поворот робота
3. Нода рассчитывает правильный angular_scale

Как использовать:
-----------------
1. Поставьте метку на роботе (например, стрелку) указывающую вперёд
2. Запустите ноду:
   ros2 run verter_admin rotation_calibration_node

3. Нода повернёт робота на 360°
4. Измерьте РЕАЛЬНЫЙ поворот робота:
   - Если робот повернулся ровно на 360° → angular_scale = 1.0 (идеально!)
   - Если робот повернулся МЕНЬШЕ (например, 320°) → angular_scale нужно УМЕНЬШИТЬ
   - Если робот повернулся БОЛЬШЕ (например, 400°) → angular_scale нужно УВЕЛИЧИТЬ

5. Формула:
   angular_scale_новый = angular_scale_старый * (360° / реальный_поворот)

   Примеры:
   - Робот повернулся на 320° → angular_scale = 1.0 * (360/320) = 1.125
   - Робот повернулся на 400° → angular_scale = 1.0 * (360/400) = 0.9

6. Обновите angular_scale в odometry_node.py и повторите тест

Параметры:
----------
- angular_speed: скорость поворота (рад/с), по умолчанию 0.3
- num_rotations: количество полных оборотов, по умолчанию 1

===================================================================================
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class RotationCalibrationNode(Node):
    """Нода для калибровки поворотов робота."""

    def __init__(self):
        super().__init__('rotation_calibration_node')

        # Параметры
        self.declare_parameter('angular_speed', 0.3)
        self.declare_parameter('num_rotations', 1)

        self.angular_speed = self.get_parameter('angular_speed').value
        self.num_rotations = self.get_parameter('num_rotations').value

        # Целевой угол поворота (в радианах)
        self.target_angle = 2 * math.pi * self.num_rotations

        # Состояние
        self.start_theta = None
        self.current_theta = 0.0
        self.previous_theta = 0.0
        self.accumulated_angle = 0.0  # Накопленный угол (может быть > 2*pi)
        self.calibration_started = False
        self.calibration_done = False

        # Подписчик на одометрию
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Издатель команд скорости
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Таймер для управления калибровкой
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('='*70)
        self.get_logger().info('ROTATION CALIBRATION NODE')
        self.get_logger().info('='*70)
        self.get_logger().info(f'Целевой угол: {self.num_rotations} оборот(а) = {math.degrees(self.target_angle):.0f}°')
        self.get_logger().info(f'Угловая скорость: {self.angular_speed} рад/с')
        self.get_logger().info('')
        self.get_logger().info('ИНСТРУКЦИИ:')
        self.get_logger().info('1. Поставьте метку на роботе (стрелку вперёд)')
        self.get_logger().info('2. Запомните начальное положение метки')
        self.get_logger().info('3. Калибровка начнётся через 5 секунд...')
        self.get_logger().info('4. После остановки измерьте РЕАЛЬНЫЙ поворот')
        self.get_logger().info('='*70)

        # Задержка перед началом
        self.start_time = time.time()
        self.delay = 5.0  # 5 секунд

    def odom_callback(self, msg: Odometry):
        """Получаем текущую ориентацию из одометрии."""
        # Извлекаем угол из кватерниона
        orientation_q = msg.pose.pose.orientation
        self.current_theta = self.quaternion_to_yaw(orientation_q)

    def quaternion_to_yaw(self, q):
        """Конвертирует кватернион в угол yaw (поворот вокруг Z)."""
        # Формула из https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def control_loop(self):
        """Главный цикл управления калибровкой."""
        if self.calibration_done:
            return

        current_time = time.time()

        # Ждём задержку перед началом
        if not self.calibration_started:
            elapsed = current_time - self.start_time
            if elapsed < self.delay:
                remaining = self.delay - elapsed
                if int(remaining) != int(remaining + 0.1):  # Логируем каждую секунду
                    self.get_logger().info(f'Начало через {int(remaining)+1} сек...')
                return
            else:
                # Начинаем калибровку
                self.start_theta = self.current_theta
                self.previous_theta = self.current_theta
                self.accumulated_angle = 0.0
                self.calibration_started = True
                self.rotation_start_time = current_time
                self.get_logger().info('')
                self.get_logger().info('='*70)
                self.get_logger().info('КАЛИБРОВКА НАЧАЛАСЬ!')
                self.get_logger().info(f'Начальный угол: {math.degrees(self.start_theta):.1f}°')
                self.get_logger().info('='*70)
                return

        # Вычисляем изменение угла с прошлого измерения
        delta = self.current_theta - self.previous_theta

        # Обрабатываем переход через 0/2π
        if delta > math.pi:
            delta -= 2 * math.pi
        elif delta < -math.pi:
            delta += 2 * math.pi

        # Накапливаем угол (используем абсолютное значение, т.к. поворачиваем в одну сторону)
        self.accumulated_angle += abs(delta)
        self.previous_theta = self.current_theta

        # Проверяем, достигли ли целевого угла
        if self.accumulated_angle >= self.target_angle:
            # Калибровка завершена
            self.stop_robot()
            self.calibration_done = True

            elapsed_time = current_time - self.rotation_start_time
            actual_angle_deg = math.degrees(self.accumulated_angle)

            self.get_logger().info('')
            self.get_logger().info('='*70)
            self.get_logger().info('КАЛИБРОВКА ЗАВЕРШЕНА!')
            self.get_logger().info('='*70)
            self.get_logger().info(f'Время поворота: {elapsed_time:.1f} сек')
            self.get_logger().info(f'Одометрия показывает: {actual_angle_deg:.1f}°')
            self.get_logger().info('')
            self.get_logger().info('ТЕПЕРЬ ИЗМЕРЬТЕ РЕАЛЬНЫЙ ПОВОРОТ РОБОТА:')
            self.get_logger().info('')
            self.get_logger().info('Если робот повернулся:')
            self.get_logger().info('  - Ровно 360° → angular_scale = 1.0 (отлично!)')
            self.get_logger().info('  - Меньше 360° (например 320°) → нужно УМЕНЬШИТЬ angular_scale')
            self.get_logger().info('  - Больше 360° (например 400°) → нужно УВЕЛИЧИТЬ angular_scale')
            self.get_logger().info('')
            self.get_logger().info('Формула для расчёта:')
            self.get_logger().info('  angular_scale_новый = angular_scale_старый * (360° / реальный_поворот)')
            self.get_logger().info('')
            self.get_logger().info('Примеры:')
            self.get_logger().info('  - Реальный поворот 320° → angular_scale = 1.0 * (360/320) = 1.125')
            self.get_logger().info('  - Реальный поворот 400° → angular_scale = 1.0 * (360/400) = 0.9')
            self.get_logger().info('  - Реальный поворот 340° → angular_scale = 1.0 * (360/340) = 1.059')
            self.get_logger().info('  - Реальный поворот 380° → angular_scale = 1.0 * (360/380) = 0.947')
            self.get_logger().info('')
            self.get_logger().info('Обновите angular_scale в:')
            self.get_logger().info('  src/verter_admin/odometry/odometry_node.py (строка 148)')
            self.get_logger().info('')
            self.get_logger().info('='*70)

            self.control_timer.cancel()
            return

        # Продолжаем вращение
        cmd = Twist()
        cmd.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(cmd)

        # Логируем прогресс каждые 45 градусов
        progress_deg = math.degrees(self.accumulated_angle)
        if int(progress_deg / 45) != int((progress_deg + 1) / 45):
            self.get_logger().info(f'Прогресс: {progress_deg:.0f}° / {math.degrees(self.target_angle):.0f}°')

    def normalize_angle(self, angle):
        """Нормализует угол в диапазон [0, 2*pi]."""
        while angle < 0:
            angle += 2 * math.pi
        while angle > 2 * math.pi:
            angle -= 2 * math.pi
        return angle

    def stop_robot(self):
        """Остановить робота."""
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)

    def destroy_node(self):
        """Корректное завершение работы."""
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RotationCalibrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прервано (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
