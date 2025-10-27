#!/usr/bin/env python3
"""
===================================================================================
NAV2 BRINGUP LAUNCH FILE FOR VERTER ROBOT
===================================================================================

Что делает этот launch файл:
---------------------------
1. Запускает все компоненты Nav2:
   - BT Navigator (Behavior Tree навигатор)
   - Controller Server (локальный планировщик/контроллер)
   - Planner Server (глобальный планировщик пути)
   - Behavior Server (восстановление: spin, backup, wait)
   - Waypoint Follower (следование по точкам)
   - Velocity Smoother (сглаживание скоростей)
   - Lifecycle Manager (управление жизненным циклом нод)

2. Загружает конфигурацию из nav2_params.yaml

3. Интегрируется с:
   - Одометрией (topic: /odom)
   - URDF моделью (через robot_state_publisher)
   - Датчиками расстояния (через topic: /scan)

Как запустить:
--------------
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd /путь/к/verter_admin

ros2 launch verter_admin nav2_bringup.launch.py

Требования:
-----------
1. robot_state_publisher должен быть запущен (публикует TF)
2. odometry_node должен быть запущен (публикует /odom и TF: odom->base_link)
3. Датчики расстояния должны быть преобразованы в /scan (LaserScan)

===================================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """
    Генерирует описание запуска Nav2 для робота Verter.

    Returns:
        LaunchDescription: Описание всех нод и параметров
    """

    # =========================================================================
    # ПУТИ К ФАЙЛАМ
    # =========================================================================

    # Получаем путь к пакету
    pkg_dir = get_package_share_directory('verter_admin')

    # Путь к конфигурации Nav2
    # nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2', 'nav2_params.yaml')

    # Альтернативный путь для разработки
    # Раскомментируйте если запускаете из исходников:
    nav2_params_file = os.path.join(
        os.path.dirname(__file__),
        '..', 'config', 'nav2', 'nav2_params.yaml'
    )


    # =========================================================================
    # LAUNCH АРГУМЕНТЫ
    # =========================================================================

    # Использовать ли симулированное время (для Gazebo)
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )

    # Путь к файлу параметров
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Full path to the ROS2 parameters file to use for Nav2'
    )

    # Автостарт Nav2
    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack'
    )

    # Использовать ли namespace
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    # Использовать ли composition (композиция нод)
    use_composition_arg = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Use composed bringup if True'
    )


    # =========================================================================
    # ПЕРЕМЕННЫЕ КОНФИГУРАЦИИ
    # =========================================================================

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')


    # =========================================================================
    # REMAPPING (переназначение топиков)
    # =========================================================================

    # Переназначения для Nav2
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]


    # =========================================================================
    # ПАРАМЕТРЫ С ПОДСТАНОВКАМИ
    # =========================================================================

    # Параметры с поддержкой подстановок из launch аргументов
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={},
        convert_types=True
    )


    # =========================================================================
    # NAV2 НОДЫ
    # =========================================================================

    # 1. Controller Server (локальный планировщик)
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings + [('cmd_vel', 'cmd_vel')]
    )

    # 2. Planner Server (глобальный планировщик)
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 3. Behavior Server (восстановление)
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 4. BT Navigator (навигатор на основе Behavior Tree)
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 5. Waypoint Follower (следование по путевым точкам)
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params],
        remappings=remappings
    )

    # 6. Velocity Smoother (сглаживание скоростей)
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=remappings +
                   [('cmd_vel', 'cmd_vel_nav'),
                    ('cmd_vel_smoothed', 'cmd_vel')]
    )

    # 7. Lifecycle Manager (управление жизненным циклом)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server',
                        'planner_server',
                        'behavior_server',
                        'bt_navigator',
                        'waypoint_follower',
                        'velocity_smoother'
                    ]}]
    )


    # =========================================================================
    # ВОЗВРАТ ОПИСАНИЯ ЗАПУСКА
    # =========================================================================

    return LaunchDescription([
        # Переменные окружения
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Аргументы
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        namespace_arg,
        use_composition_arg,

        # Ноды Nav2
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. Запуск Nav2 с параметрами по умолчанию:
   ros2 launch verter_admin nav2_bringup.launch.py

2. Запуск с пользовательским файлом параметров:
   ros2 launch verter_admin nav2_bringup.launch.py params_file:=/path/to/params.yaml

3. Запуск без автостарта:
   ros2 launch verter_admin nav2_bringup.launch.py autostart:=False

4. Запуск в симуляции (Gazebo):
   ros2 launch verter_admin nav2_bringup.launch.py use_sim_time:=True

===================================================================================
ПРОВЕРКА РАБОТЫ
===================================================================================

После запуска проверьте:

1. Запущенные ноды:
   ros2 node list

   Должны быть:
   - /controller_server
   - /planner_server
   - /behavior_server
   - /bt_navigator
   - /waypoint_follower
   - /velocity_smoother
   - /lifecycle_manager_navigation

2. Топики:
   ros2 topic list

   Должны быть:
   - /cmd_vel (выходная скорость для робота)
   - /cmd_vel_nav (скорость от навигатора)
   - /local_costmap/costmap
   - /global_costmap/costmap

3. Статус lifecycle нод:
   ros2 lifecycle list

   Все ноды должны быть в состоянии "active"

4. Логи:
   ros2 topic echo /rosout

   Не должно быть критических ошибок

===================================================================================
ИЗВЕСТНЫЕ ПРОБЛЕМЫ
===================================================================================

1. **Нет карты (map)**:
   - Nav2 требует карту для глобального планирования
   - Используйте SLAM для создания карты или загрузите готовую
   - Или используйте только локальное планирование (local costmap)

2. **Нет /scan топика**:
   - Датчики расстояния должны быть преобразованы в LaserScan
   - Создайте ноду-конвертер от ультразвуковых датчиков к /scan

3. **TF ошибки**:
   - Убедитесь что robot_state_publisher запущен
   - Убедитесь что odometry_node публикует TF: odom->base_link

===================================================================================
"""
