#!/usr/bin/env python3
"""
Waypoint Navigator - отправка робота к именованным точкам

Использование:
    ros2 run verter_admin waypoint_navigator --waypoint "Кабинет директора"
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import yaml
import sys
import os
import math


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Action client для Nav2
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.get_logger().info('Waypoint Navigator запущен')

    def send_goal(self, x, y, yaw, frame_id='map'):
        """Отправить цель навигации"""

        # Ждём, пока Nav2 не будет готов
        self.get_logger().info('Ожидание Nav2...')
        self._action_client.wait_for_server()

        # Создаём цель
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = frame_id
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Позиция
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Ориентация (из yaw в quaternion)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Отправка цели: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # Отправляем цель
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

        return send_goal_future

    def goal_response_callback(self, future):
        """Callback при получении ответа от Nav2"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Цель отклонена Nav2!')
            return

        self.get_logger().info('✅ Цель принята Nav2, робот движется...')

        # Ждём результат
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Callback для обновлений прогресса"""
        feedback = feedback_msg.feedback
        # Можно показывать прогресс, но не будем спамить
        # self.get_logger().info(f'Прогресс: {feedback}')
        pass

    def result_callback(self, future):
        """Callback при завершении навигации"""
        result = future.result().result

        if result:
            self.get_logger().info('🎉 Робот прибыл к цели!')
        else:
            self.get_logger().warn('⚠️ Навигация завершена с проблемами')

        # Завершаем ноду
        rclpy.shutdown()


def load_waypoints(waypoints_file):
    """Загрузить waypoints из YAML файла"""
    if not os.path.exists(waypoints_file):
        print(f"❌ Ошибка: Файл не найден: {waypoints_file}")
        return None

    with open(waypoints_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    return data.get('waypoints', [])


def find_waypoint(waypoints, name):
    """Найти waypoint по имени"""
    for wp in waypoints:
        if wp['name'].lower() == name.lower():
            return wp
    return None


def main(args=None):
    # Парсинг аргументов
    if len(sys.argv) < 2:
        print("Использование: waypoint_navigator <название точки>")
        print("Пример: waypoint_navigator 'Кабинет директора'")
        sys.exit(1)

    waypoint_name = ' '.join(sys.argv[1:])

    # Найти файл waypoints.yaml
    # Ищем в директории пакета
    waypoints_file = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        '..',
        '..',
        '..',
        '..',
        'waypoints.yaml'
    )

    # Загрузить waypoints
    waypoints = load_waypoints(waypoints_file)
    if waypoints is None:
        sys.exit(1)

    # Найти нужный waypoint
    waypoint = find_waypoint(waypoints, waypoint_name)

    if waypoint is None:
        print(f"❌ Ошибка: Точка '{waypoint_name}' не найдена!")
        print("\nДоступные точки:")
        for wp in waypoints:
            print(f"  - {wp['name']}")
        sys.exit(1)

    print("=" * 60)
    print(f"🎯 Отправка робота к точке: {waypoint['name']}")
    print(f"   Координаты: x={waypoint['x']:.2f}, y={waypoint['y']:.2f}")
    print(f"   Ориентация: {waypoint['yaw']:.2f} рад ({math.degrees(waypoint['yaw']):.1f}°)")
    if 'description' in waypoint:
        print(f"   Описание: {waypoint['description']}")
    print("=" * 60)

    # Инициализация ROS2
    rclpy.init(args=args)

    # Создать навигатор
    navigator = WaypointNavigator()

    # Отправить цель
    navigator.send_goal(
        waypoint['x'],
        waypoint['y'],
        waypoint['yaw']
    )

    # Spin до завершения
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        print("\n⚠️ Навигация прервана пользователем")
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
