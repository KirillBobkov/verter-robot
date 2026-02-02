#!/usr/bin/env python3

"""
Телеоперирование роботом с помощью клавиатуры.

Особенности:
    - Плавный разгон и торможение
    - Автоматическое торможение при отпускании клавиши

Клавиши управления:
    w/W - вперед
    s/S - назад
    a/A - поворот влево
    d/D - поворот вправо
    space - экстренный стоп
    q - выход
    +/- - увеличить/уменьшить максимальную скорость
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    """Нода телеоперирования через клавиатуру с плавным разгоном/торможением"""

    def __init__(self):
        super().__init__('teleop_keyboard')

        # Параметры скорости
        self.declare_parameter('max_linear_speed', 0.2)  # м/с
        self.declare_parameter('max_angular_speed', 0.5)  # рад/с
        self.declare_parameter('linear_acceleration', 0.1)  # м/с^2
        self.declare_parameter('angular_acceleration', 0.3)  # рад/с^2
        self.declare_parameter('linear_deceleration', 0.15)  # м/с^2 (торможение быстрее)
        self.declare_parameter('angular_deceleration', 0.5)  # рад/с^2
        self.declare_parameter('update_rate', 20.0)  # Гц
        self.declare_parameter('speed_step', 0.05)  # шаг изменения макс скорости

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_acceleration = self.get_parameter('linear_acceleration').value
        self.angular_acceleration = self.get_parameter('angular_acceleration').value
        self.linear_deceleration = self.get_parameter('linear_deceleration').value
        self.angular_deceleration = self.get_parameter('angular_deceleration').value
        self.update_rate = self.get_parameter('update_rate').value
        self.speed_step = self.get_parameter('speed_step').value

        # Publisher для cmd_vel (через twist_mux)
        self.cmd_vel_pub = self.create_publisher(Twist, '/teleop_keyboard/cmd_vel', 10)

        # Целевая скорость (куда стремимся)
        self.target_linear = 0.0
        self.target_angular = 0.0

        # Текущая скорость (что публикуем)
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Сохранение настроек терминала
        self.settings = termios.tcgetattr(sys.stdin)

        # Интервал обновления
        self.dt = 1.0 / self.update_rate

        self.get_logger().info('Teleop Keyboard Node Started (smooth mode)')
        self.get_logger().info('Управление: W/A/S/D, Space - стоп, Q - выход, +/- - скорость')
        self.get_logger().info(f'Макс. скорость: {self.max_linear_speed:.2f} м/с, {self.max_angular_speed:.2f} рад/с')

    def get_key_non_blocking(self, timeout=0.05):
        """Получить нажатую клавишу без блокировки"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Обработка escape-последовательностей (стрелки)
            if key == '\x1b':
                additional = sys.stdin.read(2)
                key = key + additional
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def smooth_velocity(self, current, target, acceleration, deceleration, dt):
        """Плавное изменение скорости к целевой"""
        if abs(target) < 0.001 and abs(current) < 0.001:
            return 0.0

        diff = target - current

        if abs(diff) < 0.001:
            return target

        # Выбираем ускорение или торможение
        if abs(target) < abs(current) or (target * current < 0):
            # Торможение (цель меньше или смена направления)
            rate = deceleration
        else:
            # Разгон
            rate = acceleration

        max_change = rate * dt

        if abs(diff) <= max_change:
            return target
        elif diff > 0:
            return current + max_change
        else:
            return current - max_change

    def run(self):
        """Главный цикл телеоперирования"""
        try:
            last_key_time = self.get_clock().now()

            while rclpy.ok():
                key = self.get_key_non_blocking(timeout=self.dt)
                current_time = self.get_clock().now()

                # Обработка клавиш
                if key is not None:
                    last_key_time = current_time

                    if key == 'w' or key == 'W' or key == '\x1b[A':
                        self.target_linear = self.max_linear_speed
                        self.target_angular = 0.0

                    elif key == 's' or key == 'S' or key == '\x1b[B':
                        self.target_linear = -self.max_linear_speed
                        self.target_angular = 0.0

                    elif key == 'a' or key == 'A' or key == '\x1b[D':
                        self.target_angular = self.max_angular_speed
                        # При повороте можно двигаться вперёд
                        # self.target_linear = 0.0

                    elif key == 'd' or key == 'D' or key == '\x1b[C':
                        self.target_angular = -self.max_angular_speed
                        # При повороте можно двигаться вперёд
                        # self.target_linear = 0.0

                    elif key == ' ':  # Экстренный стоп
                        self.target_linear = 0.0
                        self.target_angular = 0.0
                        self.current_linear = 0.0
                        self.current_angular = 0.0
                        self.get_logger().info('STOP')

                    elif key == '+' or key == '=':
                        self.max_linear_speed = min(1.0, self.max_linear_speed + self.speed_step)
                        self.max_angular_speed = min(2.0, self.max_angular_speed + self.speed_step * 2)
                        self.get_logger().info(f'Max speed: {self.max_linear_speed:.2f} m/s')

                    elif key == '-' or key == '_':
                        self.max_linear_speed = max(0.05, self.max_linear_speed - self.speed_step)
                        self.max_angular_speed = max(0.1, self.max_angular_speed - self.speed_step * 2)
                        self.get_logger().info(f'Max speed: {self.max_linear_speed:.2f} m/s')

                    elif key == 'q' or key == 'Q' or key == '\x03':
                        self.get_logger().info('Exit...')
                        break

                else:
                    # Клавиша не нажата - тормозим
                    self.target_linear = 0.0
                    self.target_angular = 0.0

                # Плавное изменение скорости
                self.current_linear = self.smooth_velocity(
                    self.current_linear,
                    self.target_linear,
                    self.linear_acceleration,
                    self.linear_deceleration,
                    self.dt
                )

                self.current_angular = self.smooth_velocity(
                    self.current_angular,
                    self.target_angular,
                    self.angular_acceleration,
                    self.angular_deceleration,
                    self.dt
                )

                # Публикуем команду
                twist = Twist()
                twist.linear.x = self.current_linear
                twist.angular.z = self.current_angular
                self.cmd_vel_pub.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Error: {e}')

        finally:
            # Останавливаем робота перед выходом
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(args=None):
    rclpy.init(args=args)

    node = TeleopKeyboard()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()