#!/usr/bin/env python3
"""
===================================================================================
SLAM TOOLBOX LAUNCH FILE FOR VERTER ROBOT
===================================================================================

Что делает этот launch файл:
---------------------------
1. Запускает async_slam_toolbox_node для создания карты в реальном времени
2. Загружает конфигурацию из slam_toolbox_params.yaml
3. Публикует TF трансформацию map -> odom

Интегрируется с:
----------------
- Одометрией (topic: /odom)
- LaserScan данными (topic: /scan от ultrasonic_to_laserscan_node)
- URDF моделью (через robot_state_publisher)

Как запустить:
--------------
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd /путь/к/verter_admin

ros2 launch verter_admin slam_toolbox.launch.py

Требования:
-----------
1. robot_state_publisher должен быть запущен
2. odometry_node должен быть запущен (публикует /odom)
3. ultrasonic_to_laserscan_node должен быть запущен (публикует /scan)

Выходные данные:
----------------
- /map (nav_msgs/OccupancyGrid) - карта окружения
- TF: map -> odom
- /slam_toolbox/* - топики для управления и визуализации

Сохранение карты:
-----------------
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/path/to/map_name'}}"

===================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Генерирует описание запуска SLAM Toolbox для робота Verter.

    Returns:
        LaunchDescription: Описание ноды и параметров
    """

    # =========================================================================
    # ПУТИ К ФАЙЛАМ
    # =========================================================================

    # Получаем путь к пакету
    pkg_dir = get_package_share_directory('verter_admin')

    # Путь к конфигурации SLAM
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam', 'slam_toolbox_params.yaml')

    # Альтернативный путь для разработки
    # Раскомментируйте если запускаете из исходников:
    # slam_params_file = os.path.join(
    #     os.path.dirname(__file__),
    #     '..', 'config', 'slam', 'slam_toolbox_params.yaml'
    # )


    # =========================================================================
    # LAUNCH АРГУМЕНТЫ
    # =========================================================================

    # Использовать ли симулированное время
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation clock if true'
    )

    # Путь к файлу параметров
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=slam_params_file,
        description='Full path to SLAM Toolbox parameters file'
    )


    # =========================================================================
    # ПЕРЕМЕННЫЕ КОНФИГУРАЦИИ
    # =========================================================================

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')


    # =========================================================================
    # SLAM TOOLBOX NODE
    # =========================================================================

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )


    # =========================================================================
    # ВОЗВРАТ ОПИСАНИЯ ЗАПУСКА
    # =========================================================================

    return LaunchDescription([
        # Аргументы
        use_sim_time_arg,
        params_file_arg,

        # SLAM Toolbox нода
        slam_toolbox_node,
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. Запуск SLAM с параметрами по умолчанию:
   ros2 launch verter_admin slam_toolbox.launch.py

2. Запуск с пользовательским файлом параметров:
   ros2 launch verter_admin slam_toolbox.launch.py \
     params_file:=/path/to/custom_params.yaml

3. Запуск в симуляции:
   ros2 launch verter_admin slam_toolbox.launch.py use_sim_time:=True

===================================================================================
ПРОВЕРКА РАБОТЫ
===================================================================================

1. Проверка запущенных нод:
   ros2 node list

   Должна быть:
   - /slam_toolbox

2. Проверка топиков:
   ros2 topic list | grep -E "(map|slam)"

   Должны быть:
   - /map
   - /slam_toolbox/graph_visualization
   - /slam_toolbox/scan_visualization

3. Проверка TF:
   ros2 run tf2_ros tf2_echo map odom

   Должна показывать трансформацию map -> odom

4. Просмотр карты (в RViz):
   rviz2

   Добавить визуализацию:
   - Topic: /map
   - Type: Map

===================================================================================
СОХРАНЕНИЕ КАРТЫ
===================================================================================

1. Сохранить текущую карту:
   ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
     "{name: {data: '/path/to/my_map'}}"

2. Сериализованная карта (для продолжения SLAM):
   ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
     "{filename: '/path/to/my_map.posegraph'}"

3. Экспорт в формат для map_server:
   Используйте nav2_map_server для конвертации:
   ros2 run nav2_map_server map_saver_cli -f /path/to/my_map

===================================================================================
ИЗВЕСТНЫЕ ПРОБЛЕМЫ
===================================================================================

1. **Нет /scan топика**:
   - Убедитесь что запущен ultrasonic_to_laserscan_node
   - Проверьте: ros2 topic echo /scan

2. **Нет /odom топика**:
   - Убедитесь что запущен odometry_node
   - Проверьте: ros2 topic echo /odom

3. **TF ошибки**:
   - Убедитесь что robot_state_publisher запущен
   - Проверьте TF дерево: ros2 run tf2_tools view_frames

4. **Карта не строится**:
   - Робот должен двигаться для построения карты
   - Проверьте что /scan получает данные
   - Убедитесь что одометрия работает

===================================================================================
"""
