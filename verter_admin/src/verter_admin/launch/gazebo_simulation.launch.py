#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch файл для запуска робота Verter в симуляторе Gazebo.
    """

    # Пути к файлам
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_verter = get_package_share_directory('verter_admin')

    # Путь к URDF файлу
    urdf_file = os.path.join(pkg_verter, 'urdf', 'verter_robot_gazebo.urdf')

    # Путь к world файлу
    world_file = os.path.join(pkg_verter, 'worlds', 'test_room.world')

    # Чтение URDF файла
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Запуск Gazebo с нашим world файлом
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': world_file
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # Спавн робота в Gazebo (позиция в углу комнаты, подальше от препятствий)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'verter_robot',
            '-topic', 'robot_description',
            '-x', '-3.0',
            '-y', '-3.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    pointcloud_to_laserscan_node = Node(
        package='verter_admin',
        executable='pointcloud_to_laserscan',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/verter/camera_tof/points',
            'output_topic': '/scan',
            'min_height': -0.5,
            'max_height': 1.0,
            'angle_min': -1.0472,  # -60° to match 120° camera FOV
            'angle_max': 1.0472,   # +60° to match 120° camera FOV
            'angle_increment': 0.0175,  # ~1°
            'range_min': 0.05,
            'range_max': 4.0,
            'scan_time': 0.2
        }]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
        pointcloud_to_laserscan_node
    ])
