#!/usr/bin/env python3
"""
===================================================================================
VERTER NAVIGATION LAUNCH FILE - Полная система навигации
===================================================================================

Назначение:
-----------
Запускает полную систему автономной навигации робота Verter:
1. Базовые компоненты (URDF, одометрия, датчики)
2. SLAM Toolbox (для создания карты) ИЛИ map_server (для загрузки карты)
3. Nav2 navigation stack (для автономной навигации)

Режимы работы:
--------------
1. mapping - Создание новой карты (SLAM включен, Nav2 опционально)
2. navigation - Навигация по готовой карте (SLAM выключен, Nav2 включен)

Как запустить:
--------------
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

# Режим создания карты
ros2 launch verter_admin verter_navigation.launch.py mode:=mapping

# Режим навигации по карте
ros2 launch verter_admin verter_navigation.launch.py \
  mode:=navigation \
  map_file:=/path/to/map.yaml

Опции:
------
- mode:=mapping|navigation  (по умолчанию mapping)
- use_slam:=True|False      (по умолчанию True в режиме mapping)
- use_nav2:=True|False      (по умолчанию True в режиме navigation)
- map_file:=/path/to/map    (обязательно в режиме navigation)
- use_sim_time:=False       (по умолчанию False)

===================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Генерирует описание запуска полной системы навигации."""

    # =========================================================================
    # ПУТИ К ФАЙЛАМ
    # =========================================================================

    pkg_dir = get_package_share_directory('verter_admin')

    # Launch файлы других компонентов
    base_launch = os.path.join(pkg_dir, 'launch', 'verter_base.launch.py')
    slam_launch = os.path.join(pkg_dir, 'launch', 'slam_toolbox.launch.py')
    nav2_launch = os.path.join(pkg_dir, 'launch', 'nav2_bringup.launch.py')

    # Альтернативные пути для разработки
    # base_launch = os.path.join(os.path.dirname(__file__), 'verter_base.launch.py')
    # slam_launch = os.path.join(os.path.dirname(__file__), 'slam_toolbox.launch.py')
    # nav2_launch = os.path.join(os.path.dirname(__file__), 'nav2_bringup.launch.py')

    # =========================================================================
    # LAUNCH АРГУМЕНТЫ
    # =========================================================================

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='mapping',
        description='Operation mode: mapping (SLAM) or navigation (Nav2 with map)',
        choices=['mapping', 'navigation']
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value=PythonExpression(["'", LaunchConfiguration('mode'), "' == 'mapping'"]),
        description='Use SLAM for mapping'
    )

    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value=PythonExpression(["'", LaunchConfiguration('mode'), "' == 'navigation'"]),
        description='Use Nav2 for navigation'
    )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='Path to map file (required for navigation mode)'
    )

    # =========================================================================
    # ПЕРЕМЕННЫЕ
    # =========================================================================

    mode = LaunchConfiguration('mode')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')
    map_file = LaunchConfiguration('map_file')

    # =========================================================================
    # ВКЛЮЧЕНИЕ ДРУГИХ LAUNCH ФАЙЛОВ
    # =========================================================================

    # 1. Базовые компоненты (всегда запускаются)
    base_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # 2. SLAM Toolbox (только в режиме mapping)
    slam_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_slam)
    )

    # 3. Nav2 (только в режиме navigation)
    nav2_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items(),
        condition=IfCondition(use_nav2)
    )

    # 4. Map Server (только в режиме navigation, для загрузки карты)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file
        }],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", map_file, "' != ''"
        ]))
    )

    # Lifecycle manager для map_server
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }],
        condition=IfCondition(PythonExpression([
            "'", mode, "' == 'navigation' and '", map_file, "' != ''"
        ]))
    )

    # =========================================================================
    # ВОЗВРАТ
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        mode_arg,
        use_sim_time_arg,
        use_slam_arg,
        use_nav2_arg,
        map_file_arg,

        # Компоненты
        base_launch_include,          # Базовые компоненты (всегда)
        slam_launch_include,          # SLAM (если mode=mapping)
        nav2_launch_include,          # Nav2 (если mode=navigation)
        map_server_node,              # Map server (если mode=navigation)
        map_lifecycle_manager,        # Lifecycle для map_server
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. РЕЖИМ СОЗДАНИЯ КАРТЫ (Mapping):
   --------------------------------

   a) Запуск SLAM для создания карты:
      ros2 launch verter_admin verter_navigation.launch.py mode:=mapping

   b) Управляйте роботом для построения карты:
      ros2 topic pub /cmd_vel geometry_msgs/Twist \
        "{linear: {x: 0.2}, angular: {z: 0.0}}"

   c) Сохраните карту:
      ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: '/path/to/my_map'}}"

      ИЛИ используйте map_saver:
      ros2 run nav2_map_server map_saver_cli -f /path/to/my_map

2. РЕЖИМ НАВИГАЦИИ (Navigation):
   -------------------------------

   a) Запуск Nav2 с готовой картой:
      ros2 launch verter_admin verter_navigation.launch.py \
        mode:=navigation \
        map_file:=/path/to/my_map.yaml

   b) Отправьте цель навигации:
      ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
      header:
        frame_id: 'map'
      pose:
        position:
          x: 1.0
          y: 0.5
          z: 0.0
        orientation:
          w: 1.0
      "

3. ГИБРИДНЫЙ РЕЖИМ (Mapping + Navigation):
   ----------------------------------------

   Запуск SLAM и Nav2 одновременно:
   ros2 launch verter_admin verter_navigation.launch.py \
     mode:=mapping \
     use_nav2:=True

===================================================================================
ПРОВЕРКА РАБОТЫ
===================================================================================

1. Проверка запущенных нод:
   ros2 node list

   Режим mapping:
   - /robot_state_publisher
   - /odometry_node
   - /ultrasonic_to_laserscan_node
   - /slam_toolbox

   Режим navigation:
   - /robot_state_publisher
   - /odometry_node
   - /ultrasonic_to_laserscan_node
   - /map_server
   - /controller_server
   - /planner_server
   - /bt_navigator
   - ...

2. Проверка топиков:
   ros2 topic list

   Должны быть:
   - /tf, /tf_static
   - /odom
   - /scan
   - /map (в режиме navigation или после SLAM)
   - /cmd_vel (для Nav2)

3. Проверка TF дерева:
   ros2 run tf2_tools view_frames

   Mapping: map -> odom -> base_link -> sensors
   Navigation: map -> odom -> base_link -> sensors

===================================================================================
ОТЛАДКА
===================================================================================

1. Нет карты в режиме navigation:
   Убедитесь что указан правильный map_file:
   ls -la /path/to/my_map.yaml
   ls -la /path/to/my_map.pgm

2. SLAM не создает карту:
   - Убедитесь что робот движется
   - Проверьте топик /scan: ros2 topic hz /scan
   - Проверьте топик /odom: ros2 topic hz /odom

3. Nav2 не планирует путь:
   - Убедитесь что карта загружена: ros2 topic echo /map --once
   - Проверьте что робот локализован на карте
   - Проверьте costmap: ros2 topic echo /local_costmap/costmap --once

===================================================================================
"""
