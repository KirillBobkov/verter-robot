#!/usr/bin/env python3
"""
Полный Launch файл для картографирования на реальном роботе Verter.

Включает ВСЕ необходимые компоненты:
1. twist_mux - мультиплексор команд скорости
2. chassis_node - управление моторами
3. odometry_node - одометрия (энкодеры)
4. robot_state_publisher - публикация TF из URDF
5. rplidar_node - драйвер лидара
6. slam_toolbox - построение карты

Архитектура:
                                    ┌─────────────┐
  /teleop_keyboard/cmd_vel ────────>│             │
  /nav2/cmd_vel ───────────────────>│  twist_mux  │──> /cmd_vel ──> chassis_node
  /safety/cmd_vel ─────────────────>│             │                      │
                                    └─────────────┘                      │
                                                                         v
  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
  │   RPLidar   │───>│ /scan_raw   │───>│laser_filter │───>│   /scan     │
  │   A1M8      │    └─────────────┘    │(remove back)│    └──────┬──────┘
  └─────────────┘                       └─────────────┘           │
                                                                  v
                                        ┌─────────────┐    ┌─────────────┐
                                        │ slam_toolbox│<───│  /odom      │
                                        │             │    │ (encoders)  │
                                        └──────┬──────┘    └─────────────┘
                                               │
                                               v
                                        ┌─────────────┐
                                        │    /map     │
                                        └─────────────┘

Фильтрация лидара:
  - Лидар установлен спереди робота (~25см от центра)
  - Задний сектор (180°) блокируется корпусом робота
  - laser_filter отсекает углы от 90° до 270° (задний сектор)
  - Оставляет передние 180° для SLAM

Использование:
    # На роботе:
    ros2 launch verter_admin mapping.launch.py

    # На удалённом ПК (RViz):
    ros2 launch verter_admin mapping.launch.py rviz:=true

    # Управление:
    ros2 run verter_admin teleop_keyboard
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Получаем пути к пакету
    pkg_verter_admin = get_package_share_directory('verter_admin')

    # Пути к конфигурационным файлам
    urdf_file = os.path.join(pkg_verter_admin, 'urdf', 'verter_robot_minimal.urdf')
    slam_params_file = os.path.join(pkg_verter_admin, 'config', 'slam', 'slam_toolbox_params.yaml')
    twist_mux_config = os.path.join(pkg_verter_admin, 'config', 'twist_mux', 'twist_mux.yaml')
    laser_filter_config = os.path.join(pkg_verter_admin, 'config', 'laser_filters', 'laser_filter.yaml')

    # Читаем URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Launch аргументы
    serial_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLiDAR'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz2 (set to true on remote PC)'
    )

    # =========================================================================
    # БАЗОВАЯ СИСТЕМА УПРАВЛЕНИЯ
    # =========================================================================

    # Twist Mux - мультиплексор команд скорости
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[
            ('cmd_vel_out', '/cmd_vel'),
        ],
        output='screen'
    )

    # Chassis Node - управление моторами Arduino
    chassis_node = Node(
        package='verter_admin',
        executable='chassis_node',
        name='chassis_node',
        output='screen'
    )

    # Odometry Node - одометрия на основе энкодеров
    odometry_node = Node(
        package='verter_admin',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # =========================================================================
    # ROBOT STATE PUBLISHER (TF из URDF)
    # =========================================================================

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # =========================================================================
    # СЕНСОРЫ
    # =========================================================================

    # RPLiDAR A1M8 - публикует в /scan_raw (до фильтрации)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('lidar_port'),
            'frame_id': 'lidar_link',
            'scan_mode': 'Standard',
            'serial_baudrate': 115200,
            'inverted': False,
            'angle_compensate': True,
        }],
        remappings=[
            ('scan', '/scan_raw'),  # Публикуем в /scan_raw для последующей фильтрации
        ],
        output='screen'
    )

    # Laser Filter - фильтрует задний сектор (корпус робота)
    # Подписывается на /scan_raw, публикует в /scan
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        parameters=[laser_filter_config],
        remappings=[
            ('scan', '/scan_raw'),       # Вход: сырые данные с лидара
            ('scan_filtered', '/scan'),  # Выход: отфильтрованные данные
        ],
        output='screen'
    )

    # Датчики расстояния (HC-SR04) - опционально
    distance_sensors_node = Node(
        package='verter_admin',
        executable='distance_sensors_node',
        name='distance_sensors_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # =========================================================================
    # SLAM TOOLBOX
    # =========================================================================

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': False}
        ]
        # scan_topic: /scan уже указан в slam_toolbox_params.yaml
    )

    # =========================================================================
    # ВИЗУАЛИЗАЦИЯ (опционально)
    # =========================================================================

#     rviz_node = Node(
#         package='rviz2',
#         executable='rviz2',
#         name='rviz2',
#         output='screen',
#         parameters=[{'use_sim_time': False}],
#         condition=IfCondition(LaunchConfiguration('rviz'))
#     )

    # =========================================================================
    # LAUNCH DESCRIPTION
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        serial_port_arg,
        rviz_arg,

        # Базовая система управления
        twist_mux_node,
        chassis_node,
        odometry_node,

        # TF
        robot_state_publisher,

        # Сенсоры
        rplidar_node,
        laser_filter_node,  # Фильтрация заднего сектора лидара
        distance_sensors_node,

        # SLAM
        slam_toolbox_node,

        # Визуализация
#         rviz_node,
    ])
