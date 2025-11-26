#!/usr/bin/env python3

"""
Телеоперирование роботом с помощью клавиатуры.

Клавиши управления:
    w/↑ - вперед
    s/↓ - назад
    a/← - поворот влево
    d/→ - поворот вправо
    space - стоп
    q - выход
    +/- - увеличить/уменьшить скорость
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    """Нода телеоперирования через клавиатуру"""

    def __init__(self):
        super().__init__('teleop_keyboard')

        # Параметры скорости
        self.declare_parameter('linear_speed', 0.3)  # м/с
        self.declare_parameter('angular_speed', 0.3)  # рад/с
        self.declare_parameter('speed_step', 0.1)  # шаг изменения скорости

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.speed_step = self.get_parameter('speed_step').value

        # Publisher для cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Текущая команда движения
        self.twist = Twist()

        # Сохранение настроек терминала
        self.settings = termios.tcgetattr(sys.stdin)

        self.get_logger().info('Teleop Keyboard Node Started')
        self.get_logger().info('Управление: W/A/S/D или стрелки, Space - стоп, Q - выход')
        self.get_logger().info(f'Скорость: линейная={self.linear_speed:.2f} м/с, угловая={self.angular_speed:.2f} рад/с')

    def get_key(self):
        """Получить нажатую клавишу"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """Главный цикл телеоперирования"""
        try:
            while rclpy.ok():
                key = self.get_key()

                # Обработка клавиш
                if key == 'w' or key == '\x1b[A':  # W или стрелка вверх
                    self.twist.linear.x = self.linear_speed
                    self.twist.angular.z = 0.0
                    self.get_logger().info('↑ Вперед')

                elif key == 's' or key == '\x1b[B':  # S или стрелка вниз
                    self.twist.linear.x = -self.linear_speed
                    self.twist.angular.z = 0.0
                    self.get_logger().info('↓ Назад')

                elif key == 'a' or key == '\x1b[D':  # A или стрелка влево
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = self.angular_speed
                    self.get_logger().info('← Поворот влево')

                elif key == 'd' or key == '\x1b[C':  # D или стрелка вправо
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = -self.angular_speed
                    self.get_logger().info('→ Поворот вправо')

                elif key == ' ':  # Пробел - стоп
                    self.twist.linear.x = 0.0
                    self.twist.angular.z = 0.0
                    self.get_logger().info('⏸ Стоп')

                elif key == '+' or key == '=':  # Увеличить скорость
                    self.linear_speed += self.speed_step
                    self.angular_speed += self.speed_step
                    self.get_logger().info(f'⚡ Скорость: {self.linear_speed:.2f} м/с')

                elif key == '-' or key == '_':  # Уменьшить скорость
                    self.linear_speed = max(0.1, self.linear_speed - self.speed_step)
                    self.angular_speed = max(0.1, self.angular_speed - self.speed_step)
                    self.get_logger().info(f'🐌 Скорость: {self.linear_speed:.2f} м/с')

                elif key == 'q' or key == '\x03':  # Q или Ctrl+C - выход
                    self.get_logger().info('Выход...')
                    break

                else:
                    # Неизвестная клавиша - игнорируем
                    continue

                # Публикуем команду
                self.cmd_vel_pub.publish(self.twist)

        except Exception as e:
            self.get_logger().error(f'Ошибка: {e}')

        finally:
            # Останавливаем робота перед выходом
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
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