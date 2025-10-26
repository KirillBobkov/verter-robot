#!/usr/bin/env python3
"""
===================================================================================
AUTONOMOUS EXPLORATION LAUNCH FILE
===================================================================================

Назначение:
-----------
Запускает полную систему автономного исследования и построения карты:
1. verter_base (датчики, одометрия, chassis)
2. SLAM Toolbox (построение карты)
3. Autonomous Explorer (автономное движение)

Что делает:
-----------
Робот автономно исследует помещение, избегает препятствий и строит карту.

Как запустить:
--------------
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd ~/Study_Projects/verder/verter-robot/verter_admin

ros2 launch verter_admin autonomous_exploration.launch.py

Параметры:
----------
- exploration_time: время исследования в секундах (по умолчанию 300 = 5 минут)
- linear_speed: скорость движения вперёд (по умолчанию 0.15 м/с)
- angular_speed: скорость поворота (по умолчанию 0.3 рад/с)
- obstacle_distance: минимальное расстояние до препятствия (по умолчанию 0.5 м)

Пример с параметрами:
---------------------
ros2 launch verter_admin autonomous_exploration.launch.py \
  exploration_time:=600 \
  linear_speed:=0.2

Остановка:
----------
Ctrl+C для остановки всей системы

Сохранение карты:
-----------------
После завершения исследования сохраните карту:

ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

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
    """Генерирует описание запуска автономного исследования."""

    # =========================================================================
    # ПУТИ К LAUNCH ФАЙЛАМ
    # =========================================================================

    pkg_dir = get_package_share_directory('verter_admin')

    # Пути к другим launch файлам
    verter_base_launch = os.path.join(pkg_dir, 'launch', 'verter_base.launch.py')
    slam_toolbox_launch = os.path.join(pkg_dir, 'launch', 'slam_toolbox.launch.py')

    # =========================================================================
    # LAUNCH АРГУМЕНТЫ
    # =========================================================================

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time if true'
    )

    exploration_time_arg = DeclareLaunchArgument(
        'exploration_time',
        default_value='300.0',
        description='Time for autonomous exploration in seconds (default 300 = 5 minutes)'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.15',
        description='Linear speed for forward movement (m/s)'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.3',
        description='Angular speed for turning (rad/s)'
    )

    obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance',
        default_value='0.5',
        description='Minimum distance to obstacle before turning (m)'
    )

    # =========================================================================
    # ПЕРЕМЕННЫЕ
    # =========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    exploration_time = LaunchConfiguration('exploration_time')
    linear_speed = LaunchConfiguration('linear_speed')
    angular_speed = LaunchConfiguration('angular_speed')
    obstacle_distance = LaunchConfiguration('obstacle_distance')

    # =========================================================================
    # ВКЛЮЧАЕМЫЕ LAUNCH ФАЙЛЫ
    # =========================================================================

    # 1. Базовые компоненты робота (датчики, одометрия, chassis)
    verter_base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(verter_base_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2. SLAM Toolbox для построения карты
    slam_toolbox_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # =========================================================================
    # AUTONOMOUS EXPLORER NODE
    # =========================================================================

    autonomous_explorer_node = Node(
        package='verter_admin',
        executable='autonomous_explorer_node',
        name='autonomous_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'exploration_time': exploration_time,
            'linear_speed': linear_speed,
            'angular_speed': angular_speed,
            'obstacle_distance': obstacle_distance,
        }]
    )

    # =========================================================================
    # ВОЗВРАТ
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        use_sim_time_arg,
        exploration_time_arg,
        linear_speed_arg,
        angular_speed_arg,
        obstacle_distance_arg,

        # Запускаемые системы
        verter_base_launch_include,
        slam_toolbox_launch_include,
        autonomous_explorer_node,
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. Запуск с параметрами по умолчанию (5 минут исследования):
   ros2 launch verter_admin autonomous_exploration.launch.py

2. Запуск с 10 минутами исследования:
   ros2 launch verter_admin autonomous_exploration.launch.py \
     exploration_time:=600

3. Запуск с более быстрым движением:
   ros2 launch verter_admin autonomous_exploration.launch.py \
     linear_speed:=0.25 \
     angular_speed:=0.5

4. Запуск с большей дистанцией до препятствий:
   ros2 launch verter_admin autonomous_exploration.launch.py \
     obstacle_distance:=0.8

===================================================================================
ПРОВЕРКА РАБОТЫ
===================================================================================

1. Проверить что все ноды запущены:
   ros2 node list

   Должны быть:
   - /robot_state_publisher
   - /odometry_node
   - /chassis_node
   - /ultrasonic_to_laserscan_node
   - /tof_camera_node
   - /slam_toolbox
   - /autonomous_explorer

2. Мониторинг карты в реальном времени (на другом компьютере):
   rviz2

   Добавить:
   - Fixed Frame: map
   - Map topic: /map
   - LaserScan topic: /scan
   - PointCloud2 topic: /camera/depth/points
   - TF

3. Проверить скорость робота:
   ros2 topic echo /cmd_vel

4. Посмотреть логи autonomous_explorer:
   ros2 node info /autonomous_explorer

===================================================================================
СОХРАНЕНИЕ КАРТЫ ПОСЛЕ ИССЛЕДОВАНИЯ
===================================================================================

После завершения автономного исследования, сохраните карту:

# Создайте директорию для карт (если её нет)
mkdir -p ~/maps

# Сохраните карту
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_first_map

Это создаст два файла:
- my_first_map.pgm (изображение карты)
- my_first_map.yaml (метаданные карты)

Теперь эту карту можно использовать для автономной навигации с Nav2!

===================================================================================
"""
