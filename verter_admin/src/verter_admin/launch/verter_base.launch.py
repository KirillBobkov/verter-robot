#!/usr/bin/env python3
"""
===================================================================================
VERTER BASE LAUNCH FILE - Базовые компоненты робота
===================================================================================

Назначение:
-----------
Запускает базовые компоненты робота Verter:
1. robot_state_publisher - публикует URDF модель и TF дерево
2. odometry_node - публикует одометрию и TF: odom -> base_link
3. ultrasonic_to_laserscan_node - конвертирует датчики в LaserScan

Это минимальный набор для работы робота.

Как запустить:
--------------
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

ros2 launch verter_admin verter_base.launch.py

Опции:
------
- use_sim_time:=False  (по умолчанию False, для реального робота)
- urdf_file:=path      (путь к URDF файлу, по умолчанию verter_robot_minimal.urdf)

Что будет запущено:
------------------
✓ robot_state_publisher (TF: base_link -> sensors/wheels)
✓ odometry_node (TF: odom -> base_link, топик /odom)
✓ ultrasonic_to_laserscan_node (топик /scan)

Проверка:
---------
ros2 topic list
# Должны быть: /tf, /tf_static, /odom, /scan, /robot_description

ros2 run tf2_tools view_frames
# TF дерево: odom -> base_footprint -> base_link -> {sensors, wheels}

===================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Генерирует описание запуска базовых компонентов робота Verter."""

    # =========================================================================
    # ПУТИ К ФАЙЛАМ
    # =========================================================================

    pkg_dir = get_package_share_directory('verter_admin')
    default_urdf_path = os.path.join(pkg_dir, 'urdf', 'verter_robot_minimal.urdf')

    # Альтернативный путь для разработки
    # Раскомментируйте если запускаете из исходников:
    # default_urdf_path = os.path.join(
    #     os.path.dirname(__file__),
    #     '..', 'urdf', 'verter_robot_minimal.urdf'
    # )

    # =========================================================================
    # LAUNCH АРГУМЕНТЫ
    # =========================================================================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if true'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=default_urdf_path,
        description='Path to robot URDF file'
    )

    # =========================================================================
    # ПЕРЕМЕННЫЕ
    # =========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')

    # =========================================================================
    # НОДЫ
    # =========================================================================

    # 1. robot_state_publisher - публикует URDF и статические TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(Command(['xacro ', urdf_file]), value_type=str),
            'publish_frequency': 50.0
        }]
    )

    # 2. odometry_node - публикует одометрию и TF: odom -> base_link
    odometry_node = Node(
        package='verter_admin',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # 3. ultrasonic_to_laserscan_node - конвертирует датчики в LaserScan
    ultrasonic_to_laserscan_node = Node(
        package='verter_admin',
        executable='ultrasonic_to_laserscan_node',
        name='ultrasonic_to_laserscan_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # =========================================================================
    # ВОЗВРАТ
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        use_sim_time_arg,
        urdf_file_arg,

        # Ноды
        robot_state_publisher_node,
        odometry_node,
        ultrasonic_to_laserscan_node,
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. Запуск с параметрами по умолчанию:
   ros2 launch verter_admin verter_base.launch.py

2. Запуск с пользовательским URDF:
   ros2 launch verter_admin verter_base.launch.py \
     urdf_file:=/path/to/custom.urdf

3. Запуск в симуляции:
   ros2 launch verter_admin verter_base.launch.py use_sim_time:=True

===================================================================================
СЛЕДУЮЩИЕ ШАГИ
===================================================================================

После запуска базовых компонентов, вы можете:

1. Запустить SLAM для создания карты:
   ros2 launch verter_admin slam_toolbox.launch.py

2. Запустить Nav2 для навигации:
   ros2 launch verter_admin nav2_bringup.launch.py

3. Или запустить все вместе:
   ros2 launch verter_admin verter_navigation.launch.py

===================================================================================
"""
