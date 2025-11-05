#!/usr/bin/env python3
"""
===================================================================================
EXPLORE LITE AUTONOMOUS EXPLORATION LAUNCH FILE
===================================================================================

Назначение:
-----------
Запускает полную систему автономного исследования с explore_lite:
1. verter_base (датчики, одометрия, chassis)
2. SLAM Toolbox (построение карты)
3. Nav2 (навигация)
4. explore_lite (frontier-based exploration)

Как запустить:
--------------
source /opt/ros/humble/setup.bash
cd ~/Study_Projects/verder/verter-robot/verter_admin
source ~/Study_Projects/verder/verter-robot/install/setup.bash

ros2 launch verter_admin explore_lite_navigation.launch.py

Остановка:
----------
Ctrl+C для остановки всей системы

===================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Генерирует описание запуска автономного исследования с explore_lite."""

    # =========================================================================
    # ПУТИ К LAUNCH ФАЙЛАМ И КОНФИГУРАЦИЯМ
    # =========================================================================

    pkg_dir = get_package_share_directory('verter_admin')

    verter_base_launch = os.path.join(pkg_dir, 'launch', 'verter_base.launch.py')
    slam_toolbox_launch = os.path.join(pkg_dir, 'launch', 'slam_toolbox.launch.py')
    nav2_bringup_launch = os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')

    # Путь к конфигурации explore_lite
    explore_params_file = os.path.join(pkg_dir, 'config', 'explore', 'explore_lite_params.yaml')

    # =========================================================================
    # LAUNCH АРГУМЕНТЫ
    # =========================================================================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if true'
    )

    # =========================================================================
    # ПЕРЕМЕННЫЕ
    # =========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')

    # =========================================================================
    # ВКЛЮЧАЕМЫЕ LAUNCH ФАЙЛЫ
    # =========================================================================

    # 1. Базовые компоненты робота
    verter_base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(verter_base_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. SLAM Toolbox
    slam_toolbox_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Nav2
    nav2_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # =========================================================================
    # EXPLORE LITE NODE
    # =========================================================================

    explore_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[
            explore_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    # =========================================================================
    # ВОЗВРАТ
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        use_sim_time_arg,

        # Запускаемые системы
        verter_base_launch_include,
        slam_toolbox_launch_include,
        nav2_launch_include,
        explore_node,
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. Запуск с параметрами по умолчанию:
   ros2 launch verter_admin explore_lite_navigation.launch.py

2. Остановка:
   Ctrl+C

===================================================================================
ЧТО ПРОИСХОДИТ
===================================================================================

1. Запускаются базовые компоненты (датчики, одометрия, chassis)
2. Запускается SLAM Toolbox для построения карты
3. Запускается Nav2 для навигации
4. Запускается explore_lite который:
   - Анализирует карту
   - Находит границы (frontiers) между известным и неизвестным
   - Отправляет цели в Nav2
   - Nav2 ведёт робота к целям
   - Робот автоматически исследует всё помещение

===================================================================================
СОХРАНЕНИЕ КАРТЫ
===================================================================================

После завершения исследования сохраните карту:

mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/explore_map

===================================================================================
"""
