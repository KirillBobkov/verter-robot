#!/usr/bin/env python3
"""
Калибровочная нода для робота Verter.

Три теста:
1. Прямая линия — робот проезжает 1м по одометрии, пользователь замеряет реальное расстояние
2. Поворот — робот поворачивается на 360° по одометрии, пользователь проверяет точность
3. Квадрат — робот проезжает квадрат 1м x 1м, проверка общей точности

Использование:
    # Сначала запустить mapping.launch.py (или хотя бы micro_ros_agent + odometry_node + twist_mux)
    ros2 run verter_admin calibration_node
"""

import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CalibrationNode(Node):

    # Текущие параметры робота (из odometry_node.py)
    WHEEL_CIRCUMFERENCE = 0.63
    WHEEL_BASE = 0.356

    # Скорости для калибровки (медленные для точности)
    LINEAR_SPEED = 0.15   # м/с
    ANGULAR_SPEED = 0.3   # рад/с

    # Частота публикации cmd_vel
    CMD_RATE = 20.0  # Гц

    def __init__(self):
        super().__init__('calibration_node')

        self.cmd_pub = self.create_publisher(
            Twist, '/teleop_keyboard/cmd_vel', 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Текущая позиция из одометрии
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        # Стартовая позиция для текущего теста
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        # Накопленный угол (для отслеживания полных оборотов)
        self.total_yaw = 0.0
        self.prev_yaw = 0.0

        self.get_logger().info('Калибровочная нода запущена')

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Кватернион → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def send_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def stop(self):
        for _ in range(5):
            self.send_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def save_start(self):
        self.start_x = self.x
        self.start_y = self.y
        self.start_yaw = self.yaw
        self.total_yaw = 0.0
        self.prev_yaw = self.yaw

    def distance_from_start(self):
        dx = self.x - self.start_x
        dy = self.y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def update_total_yaw(self):
        delta = self.yaw - self.prev_yaw
        # Нормализация: обработка перехода через ±π
        if delta > math.pi:
            delta -= 2.0 * math.pi
        elif delta < -math.pi:
            delta += 2.0 * math.pi
        self.total_yaw += delta
        self.prev_yaw = self.yaw

    def wait_for_odom(self):
        self.get_logger().info('Ожидание данных одометрии...')
        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Одометрия получена: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°')

    # =========================================================================
    # ТЕСТ 1: ПРЯМАЯ ЛИНИЯ
    # =========================================================================
    def test_straight(self, target_distance=1.0):
        self.get_logger().info(f'=== ТЕСТ: ПРЯМАЯ ЛИНИЯ ({target_distance}м) ===')
        self.get_logger().info('Поставь робота у отметки на полу. Нажми Enter для старта.')
        input()

        rclpy.spin_once(self, timeout_sec=0.1)
        self.save_start()

        self.get_logger().info(f'Старт! Еду вперёд {target_distance}м...')

        rate_period = 1.0 / self.CMD_RATE
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=rate_period)
            dist = self.distance_from_start()

            if dist >= target_distance:
                break

            self.send_cmd(self.LINEAR_SPEED, 0.0)

        self.stop()

        final_dist = self.distance_from_start()
        self.get_logger().info(f'Остановка. Одометрия: {final_dist:.4f}м')
        self.get_logger().info(f'Замерь реальное расстояние рулеткой и введи в метрах (например 0.95):')

        try:
            actual = float(input('Реальное расстояние (м): '))
        except (ValueError, EOFError):
            self.get_logger().warn('Неверный ввод, пропускаю расчёт')
            return

        if actual > 0 and final_dist > 0:
            ratio = actual / final_dist
            new_circumference = self.WHEEL_CIRCUMFERENCE * ratio
            self.get_logger().info(f'')
            self.get_logger().info(f'=== РЕЗУЛЬТАТ ===')
            self.get_logger().info(f'Одометрия: {final_dist:.4f}м, Реальность: {actual:.4f}м')
            self.get_logger().info(f'Коэффициент: {ratio:.4f}')
            self.get_logger().info(f'Текущий WHEEL_CIRCUMFERENCE: {self.WHEEL_CIRCUMFERENCE}')
            self.get_logger().info(f'Новый WHEEL_CIRCUMFERENCE:   {new_circumference:.6f}')
            self.get_logger().info(f'')
            self.get_logger().info(f'Измени WHEEL_CIRCUMFERENCE в odometry_node.py и esp32_chassis_microros.ino')

    # =========================================================================
    # ТЕСТ 2: ПОВОРОТ НА 360°
    # =========================================================================
    def test_rotation(self, target_degrees=360.0):
        target_rad = math.radians(target_degrees)

        self.get_logger().info(f'=== ТЕСТ: ПОВОРОТ ({target_degrees}°) ===')
        self.get_logger().info('Отметь направление робота (малярный скотч/лазер). Нажми Enter для старта.')
        input()

        rclpy.spin_once(self, timeout_sec=0.1)
        self.save_start()

        self.get_logger().info(f'Старт! Поворачиваю на {target_degrees}°...')

        rate_period = 1.0 / self.CMD_RATE
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=rate_period)
            self.update_total_yaw()

            if abs(self.total_yaw) >= target_rad:
                break

            self.send_cmd(0.0, self.ANGULAR_SPEED)

        self.stop()

        final_degrees = math.degrees(abs(self.total_yaw))
        self.get_logger().info(f'Остановка. Одометрия: {final_degrees:.1f}°')
        self.get_logger().info(f'Робот вернулся точно к отметке?')
        self.get_logger().info(f'Введи реальный угол поворота (например 350 если недокрутил, 370 если перекрутил):')

        try:
            actual_deg = float(input('Реальный угол (градусы): '))
        except (ValueError, EOFError):
            self.get_logger().warn('Неверный ввод, пропускаю расчёт')
            return

        if actual_deg > 0 and final_degrees > 0:
            # Если одометрия показала 360° но реально было 350° — WHEEL_BASE слишком маленький
            # new_WHEEL_BASE = current * (odom_angle / actual_angle)
            ratio = final_degrees / actual_deg
            new_wheel_base = self.WHEEL_BASE * ratio
            self.get_logger().info(f'')
            self.get_logger().info(f'=== РЕЗУЛЬТАТ ===')
            self.get_logger().info(f'Одометрия: {final_degrees:.1f}°, Реальность: {actual_deg:.1f}°')
            self.get_logger().info(f'Коэффициент: {ratio:.4f}')
            self.get_logger().info(f'Текущий WHEEL_BASE: {self.WHEEL_BASE}')
            self.get_logger().info(f'Новый WHEEL_BASE:   {new_wheel_base:.6f}')
            self.get_logger().info(f'')
            self.get_logger().info(f'Измени WHEEL_BASE в odometry_node.py и esp32_chassis_microros.ino')

    # =========================================================================
    # ТЕСТ 3: КВАДРАТ
    # =========================================================================
    def drive_distance(self, distance):
        """Проехать заданное расстояние по одометрии."""
        rclpy.spin_once(self, timeout_sec=0.1)
        self.save_start()

        rate_period = 1.0 / self.CMD_RATE
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=rate_period)
            if self.distance_from_start() >= distance:
                break
            self.send_cmd(self.LINEAR_SPEED, 0.0)

        self.stop()

    def rotate_angle(self, degrees):
        """Повернуть на заданный угол по одометрии."""
        target_rad = math.radians(abs(degrees))
        direction = 1.0 if degrees > 0 else -1.0

        rclpy.spin_once(self, timeout_sec=0.1)
        self.save_start()

        rate_period = 1.0 / self.CMD_RATE
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=rate_period)
            self.update_total_yaw()

            if abs(self.total_yaw) >= target_rad:
                break

            self.send_cmd(0.0, direction * self.ANGULAR_SPEED)

        self.stop()

    def test_square(self, side_length=1.0):
        self.get_logger().info(f'=== ТЕСТ: КВАДРАТ ({side_length}м x {side_length}м) ===')
        self.get_logger().info('Отметь стартовую позицию и направление робота. Нажми Enter для старта.')
        input()

        rclpy.spin_once(self, timeout_sec=0.1)
        square_start_x = self.x
        square_start_y = self.y
        square_start_yaw = self.yaw

        for i in range(4):
            self.get_logger().info(f'Сторона {i+1}/4: вперёд {side_length}м...')
            self.drive_distance(side_length)

            # Пауза между действиями
            rclpy.spin_once(self, timeout_sec=0.5)

            self.get_logger().info(f'Поворот {i+1}/4: 90° влево...')
            self.rotate_angle(90.0)

            rclpy.spin_once(self, timeout_sec=0.5)

        # Результат
        dx = self.x - square_start_x
        dy = self.y - square_start_y
        drift = math.sqrt(dx * dx + dy * dy)

        dyaw = self.yaw - square_start_yaw
        if dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        self.get_logger().info(f'')
        self.get_logger().info(f'=== РЕЗУЛЬТАТ КВАДРАТА ===')
        self.get_logger().info(f'Дрифт позиции: {drift:.4f}м (dx={dx:.4f}, dy={dy:.4f})')
        self.get_logger().info(f'Дрифт угла: {math.degrees(dyaw):.1f}°')
        self.get_logger().info(f'Робот вернулся на стартовую позицию? Замерь расстояние до отметки.')

    # =========================================================================
    # ГЛАВНОЕ МЕНЮ
    # =========================================================================
    def run_menu(self):
        self.wait_for_odom()

        while rclpy.ok():
            print('\n' + '=' * 50)
            print('КАЛИБРОВКА РОБОТА VERTER')
            print('=' * 50)
            print('1 — Тест прямой линии (калибровка WHEEL_CIRCUMFERENCE)')
            print('2 — Тест поворота (калибровка WHEEL_BASE)')
            print('3 — Тест квадрата (проверка обоих параметров)')
            print('q — Выход')
            print('=' * 50)

            try:
                choice = input('Выбери тест: ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == '1':
                self.test_straight(1.0)
            elif choice == '2':
                self.test_rotation(360.0)
            elif choice == '3':
                self.test_square(1.0)
            elif choice == 'q':
                break
            else:
                print('Неверный выбор')

        self.stop()
        self.get_logger().info('Калибровка завершена')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        node.run_menu()
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
