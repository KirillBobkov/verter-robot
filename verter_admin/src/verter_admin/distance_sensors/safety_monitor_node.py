#!/usr/bin/env python3
"""
===================================================================================
SAFETY MONITOR NODE
===================================================================================

Назначение:
-----------
Мониторит данные с ультразвуковых датчиков HC-SR04 и выполняет экстренную
остановку робота при обнаружении препятствий ближе 30см.

Входные данные:
---------------
- /ultrasonic/ranges (sensor_msgs/LaserScan) - данные с 7 HC-SR04 датчиков

Выходные данные:
----------------
- /cmd_vel (geometry_msgs/Twist) - команды остановки при опасности
- /safety_status (std_msgs/Bool) - статус безопасности (True = безопасно)

Логика работы:
--------------
1. Подписывается на топик /ultrasonic/ranges
2. Проверяет все измерения на наличие препятствий < 30см
3. Если обнаружено препятствие:
   - Публикует команду остановки (все скорости = 0)
   - Устанавливает статус безопасности в False
4. Если все чисто:
   - Устанавливает статус безопасности в True
   - НЕ публикует команды (позволяет другим нодам управлять)

Параметры:
----------
- safety_distance: float = 0.30 (м) - минимальное безопасное расстояние
- check_rate: float = 20.0 (Hz) - частота проверки
- emergency_stop_priority: bool = True - высокий приоритет команд

===================================================================================
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time


class SafetyMonitorNode(Node):
    """Нода для мониторинга безопасности и экстренной остановки."""

    # Константы по умолчанию
    DEFAULT_SAFETY_DISTANCE = 0.30  # 30 см
    DEFAULT_CHECK_RATE = 20.0       # 20 Hz

    def __init__(self):
        super().__init__('safety_monitor_node')

        # Параметры
        self.declare_parameter('safety_distance', self.DEFAULT_SAFETY_DISTANCE)
        self.declare_parameter('check_rate', self.DEFAULT_CHECK_RATE)

        self.safety_distance = self.get_parameter('safety_distance').value
        self.check_rate = self.get_parameter('check_rate').value

        # Состояние
        self.is_safe = True
        self.last_scan_time = time.time()
        self.obstacle_detected = False
        self.min_distance = float('inf')

        # Счетчики для логирования
        self.log_counter = 0
        self.log_interval = int(self.check_rate * 2)  # Логировать каждые 2 секунды

        # Издатели
        self._setup_publishers()

        # Подписчики
        self._setup_subscribers()

        self.get_logger().info(
            f'Safety Monitor инициализирован:\n'
            f'  - Безопасное расстояние: {self.safety_distance*100:.0f} см\n'
            f'  - Частота проверки: {self.check_rate} Hz'
        )

    def _setup_publishers(self):
        """Настройка издателей."""
        # Команды остановки (высокий приоритет - QoS 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Статус безопасности
        self.safety_status_publisher = self.create_publisher(
            Bool,
            '/safety_status',
            10
        )

        self.get_logger().info('Издатели для /cmd_vel и /safety_status созданы')

    def _setup_subscribers(self):
        """Настройка подписчиков."""
        # Подписываемся на данные ультразвуковых датчиков
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            '/ultrasonic/ranges',
            self._scan_callback,
            10
        )

        self.get_logger().info('Подписан на топик /ultrasonic/ranges')

    def _scan_callback(self, msg: LaserScan):
        """Обработка данных со сканера."""
        try:
            # Обновляем время последнего скана
            self.last_scan_time = time.time()

            # Проверяем все измерения
            obstacle_detected = False
            min_distance = float('inf')

            for distance in msg.ranges:
                # Игнорируем невалидные измерения
                if distance < msg.range_min or distance > msg.range_max:
                    continue

                # Обновляем минимальное расстояние
                if distance < min_distance:
                    min_distance = distance

                # Проверяем на опасное расстояние
                if distance < self.safety_distance:
                    obstacle_detected = True

            # Сохраняем состояние
            self.obstacle_detected = obstacle_detected
            self.min_distance = min_distance

            # Реагируем на опасность
            if obstacle_detected:
                self._handle_danger()
            else:
                self._handle_safe()

            # Периодическое логирование
            self._log_status()

        except Exception as e:
            self.get_logger().error(f'Ошибка обработки LaserScan: {e}')

    def _handle_danger(self):
        """Обработка опасной ситуации."""
        if self.is_safe:
            # Переходим в режим опасности
            self.is_safe = False
            self.get_logger().warn(
                f'⚠️  ОПАСНОСТЬ! Препятствие обнаружено на расстоянии '
                f'{self.min_distance*100:.1f} см! ЭКСТРЕННАЯ ОСТАНОВКА!'
            )

        # Публикуем команду остановки
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.linear.y = 0.0
        stop_cmd.linear.z = 0.0
        stop_cmd.angular.x = 0.0
        stop_cmd.angular.y = 0.0
        stop_cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(stop_cmd)

        # Публикуем статус безопасности
        safety_msg = Bool()
        safety_msg.data = False
        self.safety_status_publisher.publish(safety_msg)

    def _handle_safe(self):
        """Обработка безопасной ситуации."""
        if not self.is_safe:
            # Переходим в безопасный режим
            self.is_safe = True
            self.get_logger().info(
                f'✅ Безопасно! Минимальное расстояние: {self.min_distance*100:.1f} см'
            )

        # НЕ публикуем команды - позволяем другим нодам управлять

        # Публикуем статус безопасности
        safety_msg = Bool()
        safety_msg.data = True
        self.safety_status_publisher.publish(safety_msg)

    def _log_status(self):
        """Периодическое логирование статуса."""
        self.log_counter += 1

        if self.log_counter >= self.log_interval:
            self.log_counter = 0

            if self.is_safe:
                self.get_logger().info(
                    f'Статус: БЕЗОПАСНО | '
                    f'Мин. расстояние: {self.min_distance*100:.1f} см'
                )
            else:
                self.get_logger().warn(
                    f'Статус: ОПАСНОСТЬ | '
                    f'Мин. расстояние: {self.min_distance*100:.1f} см | '
                    f'РОБОТ ОСТАНОВЛЕН'
                )

    def destroy_node(self):
        """Корректное завершение работы ноды."""
        self.get_logger().info('Завершение работы Safety Monitor ноды')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Прерывание пользователем (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
