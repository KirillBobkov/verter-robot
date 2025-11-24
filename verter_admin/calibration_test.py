#!/usr/bin/env python3
"""
Скрипт для калибровки одометрии робота.
Робот проедет заданное расстояние и автоматически остановится.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys


class CalibrationTest(Node):
    def __init__(self, target_distance=1.0, velocity=0.2):
        super().__init__('calibration_test')

        self.target_distance = target_distance  # Целевое расстояние в метрах
        self.velocity = velocity  # Скорость движения в м/с

        # Издатель команд скорости
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Подписчик на одометрию
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Начальная позиция
        self.start_x = None
        self.start_y = None
        self.current_x = 0.0
        self.current_y = 0.0

        # Флаги состояния
        self.test_started = False
        self.test_completed = False

        self.get_logger().info(f'Калибровочный тест инициализирован')
        self.get_logger().info(f'Целевое расстояние: {target_distance} м')
        self.get_logger().info(f'Скорость движения: {velocity} м/с')
        self.get_logger().info('Ожидание данных одометрии...')

    def odom_callback(self, msg: Odometry):
        """Обработчик данных одометрии."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Устанавливаем начальную позицию при первом вызове
        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(f'Начальная позиция установлена: x={self.start_x:.3f}, y={self.start_y:.3f}')
            self.get_logger().info('Начинаем движение...')
            self.start_movement()
            return

        # Если тест уже завершён, ничего не делаем
        if self.test_completed:
            return

        # Вычисляем пройденное расстояние
        distance = math.sqrt(
            (self.current_x - self.start_x)**2 +
            (self.current_y - self.start_y)**2
        )

        # Выводим прогресс каждые 0.1 метра по одометрии
        if self.test_started and int(distance * 10) % 2 == 0:
            self.get_logger().info(
                f'Пройдено: {distance:.3f} м (одометрия) | '
                f'Цель: {self.target_distance:.3f} м'
            )

        # Проверяем, достигли ли целевого расстояния
        if distance >= self.target_distance:
            self.stop_movement()
            self.test_completed = True
            self.get_logger().info('='*60)
            self.get_logger().info('ТЕСТ ЗАВЕРШЁН!')
            self.get_logger().info(f'Одометрия показывает: {distance:.3f} м')
            self.get_logger().info(f'Целевое расстояние: {self.target_distance:.3f} м')
            self.get_logger().info('='*60)
            self.get_logger().info('')
            self.get_logger().info('ТЕПЕРЬ ИЗМЕРЬТЕ РУЛЕТКОЙ РЕАЛЬНОЕ РАССТОЯНИЕ!')
            self.get_logger().info('Измерьте расстояние от начальной метки до текущей позиции робота.')
            self.get_logger().info('')
            self.get_logger().info('Для расчёта коэффициента калибровки используйте формулу:')
            self.get_logger().info('  новый_wheel_radius = старый_wheel_radius × (реальное_расстояние / одометрия)')
            self.get_logger().info('')

    def start_movement(self):
        """Начать движение вперёд."""
        self.test_started = True
        twist = Twist()
        twist.linear.x = self.velocity
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Команда движения отправлена: {self.velocity} м/с')

    def stop_movement(self):
        """Остановить робота."""
        twist = Twist()  # Все значения = 0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Команда остановки отправлена')


def main(args=None):
    if len(sys.argv) < 2:
        print('Использование: python3 calibration_test.py <расстояние_в_метрах> [скорость]')
        print('Пример: python3 calibration_test.py 1.0 0.2')
        print('')
        print('По умолчанию: расстояние=1.0м, скорость=0.2м/с')
        target_distance = 1.0
        velocity = 0.2
    else:
        target_distance = float(sys.argv[1])
        velocity = float(sys.argv[2]) if len(sys.argv) > 2 else 0.2

    rclpy.init(args=args)

    node = CalibrationTest(target_distance=target_distance, velocity=velocity)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прерывание пользователем')
        node.stop_movement()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
