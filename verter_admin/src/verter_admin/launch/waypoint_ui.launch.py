#!/usr/bin/env python3
"""
Launch файл для веб-интерфейса управления waypoints.

Запускает:
1. rosbridge_websocket — WebSocket-мост ROS2 <-> браузер (порт 9090)
2. waypoint_manager — CRUD точек + навигация через NavigateToPose
3. web_server_node — HTTP-сервер для index.html (порт 8080)

Запускается РЯДОМ с navigation.launch.py, не дублирует Nav2.

Использование:
    ros2 launch verter_admin waypoint_ui.launch.py
    ros2 launch verter_admin waypoint_ui.launch.py waypoints_file:=/path/to/waypoints.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('verter_admin')

    default_waypoints = os.path.expanduser('~/verter-robot/verter_admin/waypoints.yaml')

    waypoints_arg = DeclareLaunchArgument(
        'waypoints_file',
        default_value=default_waypoints,
        description='Path to waypoints.yaml file',
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='HTTP port for web UI',
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='WebSocket port for rosbridge',
    )

    # rosbridge_websocket
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
        }],
        output='screen',
    )

    # waypoint_manager
    waypoint_manager_node = Node(
        package='verter_admin',
        executable='waypoint_manager',
        name='waypoint_manager',
        parameters=[{
            'waypoints_file': LaunchConfiguration('waypoints_file'),
        }],
        output='screen',
    )

    # web_server_node
    web_server_node = Node(
        package='verter_admin',
        executable='web_server_node',
        name='web_server_node',
        parameters=[{
            'port': LaunchConfiguration('web_port'),
        }],
        output='screen',
    )

    return LaunchDescription([
        waypoints_arg,
        web_port_arg,
        rosbridge_port_arg,
        rosbridge_node,
        waypoint_manager_node,
        web_server_node,
    ])
