#!/usr/bin/env python3
"""
Waypoint Manager - ROS2 нода для управления waypoints через веб-интерфейс.

Публикует список точек и статус навигации в JSON-формате,
принимает команды (add, add_current, delete, navigate, cancel) через топик.

Использование:
    ros2 run verter_admin waypoint_manager --ros-args -p waypoints_file:=/path/to/waypoints.yaml
"""

import json
import math
import os
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
import yaml


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self._cb_group = ReentrantCallbackGroup()

        # Параметры
        default_waypoints = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            '..', '..', '..', '..', 'waypoints.yaml'
        )
        self.declare_parameter('waypoints_file', default_waypoints)
        self._waypoints_file = self.get_parameter('waypoints_file').get_parameter_value().string_value

        # Загрузка waypoints
        self._waypoints = self._load_waypoints()

        # Текущая позиция робота
        self._current_pose = None

        # Статус навигации
        self._nav_status = {'state': 'idle', 'target': '', 'message': ''}
        self._goal_handle = None

        # Action client для Nav2
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cb_group
        )

        # Публикация списка точек (1 Hz)
        self._waypoints_pub = self.create_publisher(String, '/waypoint_manager/waypoints', 10)
        self._status_pub = self.create_publisher(String, '/waypoint_manager/status', 10)

        self._wp_timer = self.create_timer(1.0, self._publish_waypoints)
        self._status_timer = self.create_timer(0.5, self._publish_status)

        # Подписка на команды
        self.create_subscription(
            String, '/waypoint_manager/command',
            self._command_callback, 10,
            callback_group=self._cb_group
        )

        # Подписка на позицию робота
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self._pose_callback, 10,
            callback_group=self._cb_group
        )

        self.get_logger().info(
            f'Waypoint Manager запущен, файл: {self._waypoints_file}'
        )

    # ------------------------------------------------------------------
    # YAML I/O
    # ------------------------------------------------------------------

    def _load_waypoints(self):
        if not os.path.exists(self._waypoints_file):
            self.get_logger().warn(f'Файл не найден: {self._waypoints_file}')
            return []
        with open(self._waypoints_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data.get('waypoints', []) if data else []

    def _save_waypoints(self):
        data = {'waypoints': self._waypoints}
        dir_name = os.path.dirname(os.path.abspath(self._waypoints_file))
        fd, tmp_path = tempfile.mkstemp(dir=dir_name, suffix='.yaml')
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)
            os.replace(tmp_path, self._waypoints_file)
            self.get_logger().info('Waypoints сохранены')
        except Exception as e:
            self.get_logger().error(f'Ошибка сохранения: {e}')
            if os.path.exists(tmp_path):
                os.unlink(tmp_path)

    # ------------------------------------------------------------------
    # Публикация
    # ------------------------------------------------------------------

    def _publish_waypoints(self):
        msg = String()
        msg.data = json.dumps(self._waypoints, ensure_ascii=False)
        self._waypoints_pub.publish(msg)

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps(self._nav_status, ensure_ascii=False)
        self._status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Позиция робота
    # ------------------------------------------------------------------

    def _pose_callback(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._current_pose = {
            'x': p.position.x,
            'y': p.position.y,
            'yaw': yaw,
        }

    # ------------------------------------------------------------------
    # Обработка команд
    # ------------------------------------------------------------------

    def _command_callback(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Невалидный JSON: {e}')
            return

        action = cmd.get('action', '')
        self.get_logger().info(f'Команда: {action}')

        if action == 'add':
            self._cmd_add(cmd)
        elif action == 'add_current':
            self._cmd_add_current(cmd)
        elif action == 'delete':
            self._cmd_delete(cmd)
        elif action == 'navigate':
            self._cmd_navigate(cmd)
        elif action == 'cancel':
            self._cmd_cancel()
        else:
            self.get_logger().warn(f'Неизвестная команда: {action}')

    def _cmd_add(self, cmd):
        name = cmd.get('name', '').strip()
        if not name:
            self.get_logger().warn('add: пустое имя')
            return
        wp = {
            'name': name,
            'x': float(cmd.get('x', 0.0)),
            'y': float(cmd.get('y', 0.0)),
            'yaw': float(cmd.get('yaw', 0.0)),
        }
        desc = cmd.get('description', '').strip()
        if desc:
            wp['description'] = desc
        # Удаляем дубликат по имени
        self._waypoints = [w for w in self._waypoints if w['name'] != name]
        self._waypoints.append(wp)
        self._save_waypoints()

    def _cmd_add_current(self, cmd):
        if self._current_pose is None:
            self.get_logger().warn('add_current: позиция робота неизвестна')
            self._nav_status = {
                'state': 'error',
                'target': '',
                'message': 'Позиция робота неизвестна (нет данных AMCL)',
            }
            return
        name = cmd.get('name', '').strip()
        if not name:
            self.get_logger().warn('add_current: пустое имя')
            return
        wp = {
            'name': name,
            'x': round(self._current_pose['x'], 3),
            'y': round(self._current_pose['y'], 3),
            'yaw': round(self._current_pose['yaw'], 3),
        }
        desc = cmd.get('description', '').strip()
        if desc:
            wp['description'] = desc
        self._waypoints = [w for w in self._waypoints if w['name'] != name]
        self._waypoints.append(wp)
        self._save_waypoints()

    def _cmd_delete(self, cmd):
        name = cmd.get('name', '')
        before = len(self._waypoints)
        self._waypoints = [w for w in self._waypoints if w['name'] != name]
        if len(self._waypoints) < before:
            self._save_waypoints()
        else:
            self.get_logger().warn(f'delete: точка "{name}" не найдена')

    def _cmd_navigate(self, cmd):
        name = cmd.get('name', '')
        wp = next((w for w in self._waypoints if w['name'] == name), None)
        if wp is None:
            self.get_logger().warn(f'navigate: точка "{name}" не найдена')
            return
        self._send_nav_goal(wp)

    def _cmd_cancel(self):
        if self._goal_handle is not None:
            self.get_logger().info('Отмена навигации...')
            self._goal_handle.cancel_goal_async()
        self._nav_status = {'state': 'idle', 'target': '', 'message': 'Навигация отменена'}

    # ------------------------------------------------------------------
    # Навигация (NavigateToPose action)
    # ------------------------------------------------------------------

    def _send_nav_goal(self, wp):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 недоступен')
            self._nav_status = {
                'state': 'error', 'target': wp['name'],
                'message': 'Nav2 недоступен',
            }
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.position.z = 0.0

        yaw = float(wp.get('yaw', 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._nav_status = {
            'state': 'navigating', 'target': wp['name'],
            'message': f'Движение к "{wp["name"]}"',
        }

        self.get_logger().info(f'Навигация к "{wp["name"]}" x={wp["x"]}, y={wp["y"]}')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav_feedback_cb
        )
        future.add_done_callback(self._nav_goal_response_cb)

    def _nav_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Цель отклонена Nav2')
            self._nav_status['state'] = 'error'
            self._nav_status['message'] = 'Цель отклонена Nav2'
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_cb)

    def _nav_feedback_cb(self, feedback_msg):
        # Обновляем статус с ETA если доступно
        pass

    def _nav_result_cb(self, future):
        self._goal_handle = None
        status = future.result().status
        # 4 = STATUS_SUCCEEDED, 5 = STATUS_CANCELED, 6 = STATUS_ABORTED
        if status == 4:
            self._nav_status = {
                'state': 'succeeded',
                'target': self._nav_status.get('target', ''),
                'message': f'Прибыл к "{self._nav_status.get("target", "")}"',
            }
            self.get_logger().info('Навигация завершена успешно')
        elif status == 5:
            self._nav_status = {
                'state': 'cancelled',
                'target': self._nav_status.get('target', ''),
                'message': 'Навигация отменена',
            }
        else:
            self._nav_status = {
                'state': 'error',
                'target': self._nav_status.get('target', ''),
                'message': f'Навигация не удалась (статус {status})',
            }
            self.get_logger().warn(f'Навигация не удалась, статус: {status}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
