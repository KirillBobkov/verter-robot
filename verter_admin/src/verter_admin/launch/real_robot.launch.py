#!/usr/bin/env python3
"""
===================================================================================
REAL ROBOT LAUNCH FILE - Гибридная система сенсоров
===================================================================================

Запускает реального робота с гибридной системой сенсоров:

1. ToF камера - основная для SLAM/Nav2:
   - tof_camera_node - читает данные с ToF камеры
   - pointcloud_to_laserscan - конвертирует PointCloud2 → LaserScan (/scan)

2. HC-SR04 ультразвуковые датчики - безопасность + дополнительные данные:
   - ultrasonic_to_laserscan_node - конвертирует Arduino данные → LaserScan (/ultrasonic/ranges)
   - safety_monitor_node - экстренная остановка при <30см

Архитектура:

ToF Camera (основная навигация):
    ↓ PointCloud2 (/camera_tof/points)
pointcloud_to_laserscan
    ↓ LaserScan (/scan)
SLAM Toolbox + Nav2

HC-SR04 (безопасность):
    ↓ Arduino Serial
ultrasonic_to_laserscan
    ↓ LaserScan (/ultrasonic/ranges)
safety_monitor
    ↓ cmd_vel (если опасность)

Nav2 Costmap:
    ← /scan (ToF, основной)
    ← /ultrasonic/ranges (HC-SR04, дополнительный)

===================================================================================
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 1. ToF Camera Node - читает данные с ToF камеры
    tof_camera_node = Node(
        package='verter_admin',
        executable='tof_camera_node',
        name='tof_camera_node',
        output='screen',
        parameters=[{
            'camera_frame': 'camera_tof_link',
            'publish_rate': 30.0  # 30 FPS
        }]
    )

    # 2. PointCloud2 → LaserScan конвертер для ToF (основной /scan)
    pointcloud_to_laserscan_node = Node(
        package='verter_admin',
        executable='pointcloud_to_laserscan',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'input_topic': '/verter/camera_tof/points',
            'output_topic': '/scan',
            'min_height': -0.5,
            'max_height': 1.0,
            'angle_min': -1.5708,    # -90° (π/2)
            'angle_max': 1.5708,     # +90° (π/2)
            'angle_increment': 0.0175,  # ~1°
            'range_min': 0.02,       # 2 см
            'range_max': 4.0,        # 4
            'scan_time': 0.033       # ~30 Hz
        }]
    )

    # 3. Ultrasonic → LaserScan конвертер для HC-SR04 (доп /ultrasonic/ranges)
    ultrasonic_to_laserscan_node = Node(
        package='verter_admin',
        executable='ultrasonic_to_laserscan_node',
        name='ultrasonic_to_laserscan_node',
        output='screen'
    )

    # 4. Safety Monitor
    safety_monitor_node = Node(
        package='verter_admin',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'safety_distance': 0.30,  # 30 см критическое расстояние
            'check_rate': 20.0        # 20 Hz проверка
        }]
    )

    # 5. Узел Speech-to-Text (низкоуровневый)
    speech_to_text_node = Node(
        package='verter_admin',
        executable='speech_to_text_node',
        name='speech_to_text_node',
        output='screen'
    )

    # 6. Узел обработки распознанной речи (высокоуровневый)
    recognition_node = Node(
        package='verter_admin',
        executable='recognition_node',
        name='recognition_node',
        output='screen'
    )

    # 7. Узел AI ассистента
    ai_assistant_node = Node(
        package='verter_admin',
        executable='ai_assistant_node',
        name='ai_assistant_node',
        output='screen'
    )

    # 8. Узел синтеза речи
    text_to_speech_node = Node(
        package='verter_admin',
        executable='text_to_speech_node',
        name='text_to_speech_node',
        output='screen'
    )

    # 9. Узел воспроизведения звуков
    sound_player_node = Node(
        package='verter_admin',
        executable='sound_player_node',
        name='sound_player_node',
        output='screen'
    )

    # 10. Узел Chassis
    chassis_node = Node(
        package='verter_admin',
        executable='chassis_node',
        name='chassis_node',
        output='screen'
    )

    # 11. Узел Odometry
    odometry_node = Node(
        package='verter_admin',
        executable='odometry_node',
        name='odometry_node',
        output='screen'
    )

    # 12. Узел DOA (Direction of Arrival)
    doa_node = Node(
        package='verter_admin',
        executable='doa_node',
        name='doa_node',
        output='screen'
    )

    return LaunchDescription([
        tof_camera_node,
        pointcloud_to_laserscan_node,
        ultrasonic_to_laserscan_node,
        safety_monitor_node,

        speech_to_text_node,
        recognition_node,
        ai_assistant_node,
        text_to_speech_node,
        sound_player_node,
        chassis_node,
        odometry_node,
        doa_node,
    ])
