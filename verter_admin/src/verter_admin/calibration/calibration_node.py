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
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class CalibrationNode(Node):

    WHEEL_CIRCUMFERENCE = 0.576
    WHEEL_BASE = 0.374

    LINEAR_SPEED = 0.20
    ANGULAR_SPEED = 0.3

    def __init__(self):
        super().__init__('calibration_node')

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.target_linear = 0.0
        self.target_angular = 0.0

        # Таймер cmd_vel на 50 Гц
        self.cmd_timer = self.create_timer(0.02, self.cmd_timer_callback)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0

        self.total_yaw = 0.0
        self.prev_yaw = 0.0

        self.get_logger().info('Калибровочная нода запущена')

    def cmd_timer_callback(self):
        msg = Twist()
        msg.linear.x = self.target_linear
        msg.angular.z = self.target_angular
        self.cmd_pub.publish(msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        self.odom_received = True

    def set_cmd(self, linear, angular):
        self.target_linear = linear
        self.target_angular = angular

    def stop_cmd(self):
        self.set_cmd(0.0, 0.0)

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
        if delta > math.pi:
            delta -= 2.0 * math.pi
        elif delta < -math.pi:
            delta += 2.0 * math.pi
        self.total_yaw += delta
        self.prev_yaw = self.yaw

    def wait_seconds(self, seconds):
        time.sleep(seconds)

    # =========================================================================
    # ТЕСТ 1: ПРЯМАЯ ЛИНИЯ
    # =========================================================================
    def test_straight(self, target_distance=1.0):
        self.get_logger().info(f'=== ТЕСТ: ПРЯМАЯ ЛИНИЯ ({target_distance}м) ===')
        self.get_logger().info('Поставь робота у отметки на полу. Нажми Enter для старта.')
        input()

        self.wait_seconds(0.3)
        self.save_start()

        self.get_logger().info(f'Старт! Еду вперёд {target_distance}м...')
        self.set_cmd(self.LINEAR_SPEED, 0.0)

        while rclpy.ok():
            time.sleep(0.02)
            if self.distance_from_start() >= target_distance:
                break

        self.stop_cmd()
        self.wait_seconds(0.5)

        final_dist = self.distance_from_start()
        self.get_logger().info(f'Остановка. Одометрия: {final_dist:.4f}м')
        self.get_logger().info('Замерь реальное расстояние рулеткой и введи в метрах (например 0.95):')

        try:
            actual = float(input('Реальное расстояние (м): '))
        except (ValueError, EOFError):
            self.get_logger().warn('Неверный ввод, пропускаю расчёт')
            return

        if actual > 0 and final_dist > 0:
            ratio = actual / final_dist
            new_circumference = self.WHEEL_CIRCUMFERENCE * ratio
            self.get_logger().info('')
            self.get_logger().info('=== РЕЗУЛЬТАТ ===')
            self.get_logger().info(f'Одометрия: {final_dist:.4f}м, Реальность: {actual:.4f}м')
            self.get_logger().info(f'Коэффициент: {ratio:.4f}')
            self.get_logger().info(f'Текущий WHEEL_CIRCUMFERENCE: {self.WHEEL_CIRCUMFERENCE}')
            self.get_logger().info(f'Новый WHEEL_CIRCUMFERENCE:   {new_circumference:.6f}')
            self.get_logger().info('')
            self.get_logger().info('Измени WHEEL_CIRCUMFERENCE в odometry_node.py и esp32_chassis_microros.ino')

    # =========================================================================
    # ТЕСТ 2: ПОВОРОТ НА 360°
    # =========================================================================
    def test_rotation(self, target_degrees=360.0):
        target_rad = math.radians(target_degrees)

        self.get_logger().info(f'=== ТЕСТ: ПОВОРОТ ({target_degrees}°) ===')
        self.get_logger().info('Отметь направление робота (малярный скотч/лазер). Нажми Enter для старта.')
        input()

        self.wait_seconds(0.3)
        self.save_start()

        self.get_logger().info(f'Старт! Поворачиваю на {target_degrees}°...')
        self.set_cmd(0.0, self.ANGULAR_SPEED)

        while rclpy.ok():
            time.sleep(0.02)
            self.update_total_yaw()
            if abs(self.total_yaw) >= target_rad:
                break

        self.stop_cmd()
        self.wait_seconds(0.5)

        final_degrees = math.degrees(abs(self.total_yaw))
        self.get_logger().info(f'Остановка. Одометрия: {final_degrees:.1f}°')
        self.get_logger().info('Робот вернулся точно к отметке?')
        self.get_logger().info('Введи реальный угол поворота (например 350 если недокрутил, 370 если перекрутил):')

        try:
            actual_deg = float(input('Реальный угол (градусы): '))
        except (ValueError, EOFError):
            self.get_logger().warn('Неверный ввод, пропускаю расчёт')
            return

        if actual_deg > 0 and final_degrees > 0:
            ratio = final_degrees / actual_deg
            new_wheel_base = self.WHEEL_BASE * ratio
            self.get_logger().info('')
            self.get_logger().info('=== РЕЗУЛЬТАТ ===')
            self.get_logger().info(f'Одометрия: {final_degrees:.1f}°, Реальность: {actual_deg:.1f}°')
            self.get_logger().info(f'Коэффициент: {ratio:.4f}')
            self.get_logger().info(f'Текущий WHEEL_BASE: {self.WHEEL_BASE}')
            self.get_logger().info(f'Новый WHEEL_BASE:   {new_wheel_base:.6f}')
            self.get_logger().info('')
            self.get_logger().info('Измени WHEEL_BASE в odometry_node.py и esp32_chassis_microros.ino')

    # =========================================================================
    # ТЕСТ 3: КВАДРАТ
    # =========================================================================
    def drive_distance(self, distance):
        self.wait_seconds(0.3)
        self.save_start()
        self.set_cmd(self.LINEAR_SPEED, 0.0)

        while rclpy.ok():
            time.sleep(0.02)
            if self.distance_from_start() >= distance:
                break

        self.stop_cmd()
        self.wait_seconds(0.5)

    def rotate_angle(self, degrees):
        target_rad = math.radians(abs(degrees))
        direction = 1.0 if degrees > 0 else -1.0

        self.wait_seconds(0.3)
        self.save_start()
        self.set_cmd(0.0, direction * self.ANGULAR_SPEED)

        while rclpy.ok():
            time.sleep(0.02)
            self.update_total_yaw()
            if abs(self.total_yaw) >= target_rad:
                break

        self.stop_cmd()
        self.wait_seconds(0.5)

    def test_square(self, side_length=1.0):
        self.get_logger().info(f'=== ТЕСТ: КВАДРАТ ({side_length}м x {side_length}м) ===')
        self.get_logger().info('Отметь стартовую позицию и направление робота. Нажми Enter для старта.')
        input()

        self.wait_seconds(0.3)
        square_start_x = self.x
        square_start_y = self.y
        square_start_yaw = self.yaw

        for i in range(4):
            self.get_logger().info(f'Сторона {i+1}/4: вперёд {side_length}м...')
            self.drive_distance(side_length)

            self.get_logger().info(f'Поворот {i+1}/4: 90° влево...')
            self.rotate_angle(90.0)

        self.wait_seconds(0.3)
        dx = self.x - square_start_x
        dy = self.y - square_start_y
        drift = math.sqrt(dx * dx + dy * dy)

        dyaw = self.yaw - square_start_yaw
        if dyaw > math.pi:
            dyaw -= 2.0 * math.pi
        elif dyaw < -math.pi:
            dyaw += 2.0 * math.pi

        self.get_logger().info('')
        self.get_logger().info('=== РЕЗУЛЬТАТ КВАДРАТА ===')
        self.get_logger().info(f'Дрифт позиции: {drift:.4f}м (dx={dx:.4f}, dy={dy:.4f})')
        self.get_logger().info(f'Дрифт угла: {math.degrees(dyaw):.1f}°')
        self.get_logger().info('Робот вернулся на стартовую позицию? Замерь расстояние до отметки.')

    # =========================================================================
    # ГЛАВНОЕ МЕНЮ
    # =========================================================================
    def run(self):
        # Ждём одометрию
        self.get_logger().info('Ожидание данных одометрии...')
        while rclpy.ok() and not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f'Одометрия получена: x={self.x:.3f}, y={self.y:.3f}, yaw={math.degrees(self.yaw):.1f}°')

        # Фоновый поток для ROS callbacks (таймер cmd_vel + одометрия)
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()

        # Меню в основном потоке
        self._menu_loop()

    def _spin_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)

    def _menu_loop(self):
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

        self.stop_cmd()
        self.get_logger().info('Калибровка завершена')


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.stop_cmd()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
