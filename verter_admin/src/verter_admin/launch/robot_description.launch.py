#!/usr/bin/env python3
"""
===================================================================================
LAUNCH FILE ДЛЯ ROBOT DESCRIPTION (URDF)
===================================================================================

ЧТО ДЕЛАЕТ ЭТОТ LAUNCH FILE?
-----------------------------
1. Загружает URDF модель робота из файла
2. Запускает robot_state_publisher
3. Публикует TF трансформации между частями робота

ЧТО ТАКОЕ ROBOT_STATE_PUBLISHER?
---------------------------------
robot_state_publisher - это ROS2 нода, которая:
- Читает URDF файл
- Публикует TF трансформации между всеми частями робота
- Например: base_link -> wheel_left, base_link -> sensor_front_center

ЗАЧЕМ ЭТО НУЖНО?
-----------------
1. Nav2 нужно знать:
   - Где находятся колеса относительно центра робота
   - Где расположены сенсоры
   - Размеры робота (footprint)

2. RViz использует это для:
   - Визуализации 3D модели робота
   - Отображения TF дерева
   - Правильного размещения данных сенсоров

3. TF система использует для:
   - Трансформаций между системами координат
   - Например: от sensor_front_center к base_link

КАК ЗАПУСТИТЬ:
--------------
ros2 launch verter_admin robot_description.launch.py

ДЛЯ ВИЗУАЛИЗАЦИИ В RVIZ:
------------------------
ros2 launch verter_admin robot_description.launch.py use_rviz:=true

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
    """
    Функция генерации описания запуска.

    Эта функция ОБЯЗАТЕЛЬНО должна называться именно так!
    ROS2 ищет эту функцию при запуске launch файла.

    Returns:
        LaunchDescription: Описание что и как запускать
    """

    # ============================================================================
    # ПОЛУЧЕНИЕ ПУТЕЙ К ФАЙЛАМ
    # ============================================================================

    # Получаем директорию пакета verter_admin
    pkg_dir = get_package_share_directory('verter_admin')

    # Путь к URDF файлу
    urdf_file = os.path.join(pkg_dir, 'urdf', 'verter_robot.urdf')

    # Альтернативный путь для разработки (если пакет не установлен)
    # Раскомментируйте если запускаете из исходников:
    # urdf_file = os.path.join(
    #     os.path.dirname(__file__),
    #     '..', 'urdf', 'verter_robot.urdf'
    # )


    # ============================================================================
    # LAUNCH АРГУМЕНТЫ (параметры запуска)
    # ============================================================================

    # Аргумент: использовать ли RViz для визуализации
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Запускать ли RViz для визуализации робота'
    )

    # Получаем значение аргумента
    use_rviz = LaunchConfiguration('use_rviz')


    # ============================================================================
    # ЧТЕНИЕ URDF ФАЙЛА
    # ============================================================================

    # Читаем содержимое URDF файла
    # Command() позволяет выполнить команду и получить результат
    # xacro обрабатывает URDF (в нашем случае просто читает файл)
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )


    # ============================================================================
    # ROBOT_STATE_PUBLISHER NODE
    # ============================================================================

    # robot_state_publisher публикует TF трансформации из URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',

        parameters=[{
            # Передаем содержимое URDF файла
            'robot_description': robot_description,

            # Публиковать ли TF (обязательно True!)
            'publish_frequency': 50.0,

            # Использовать ли sim time (для Gazebo симуляции)
            'use_sim_time': False,
        }]
    )


    # ============================================================================
    # JOINT_STATE_PUBLISHER NODE (опционально)
    # ============================================================================

    # joint_state_publisher публикует состояние подвижных соединений
    # Для нашего робота это колеса (continuous joints)
    # Если не планируете симуляцию в Gazebo, можно закомментировать

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',

        parameters=[{
            # Если есть GUI, можно вручную двигать колеса
            'use_gui': False,
        }]
    )


    # ============================================================================
    # RVIZ NODE (для визуализации)
    # ============================================================================

    # RViz - 3D визуализатор ROS2
    # Запускается только если use_rviz:=true

    # Конфигурация RViz (можно создать свою позже)
    rviz_config_file = os.path.join(pkg_dir, 'config', 'robot_description.rviz')

    # Если файла конфига нет, используем дефолтный
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',

        # Загружаем конфиг если есть
        arguments=['-d', rviz_config_file] if rviz_config_file else [],

        # Запускаем только если use_rviz=true
        condition=lambda context: context.launch_configurations['use_rviz'] == 'true'
    )


    # ============================================================================
    # ВОЗВРАТ ОПИСАНИЯ ЗАПУСКА
    # ============================================================================

    return LaunchDescription([
        # Аргументы
        use_rviz_arg,

        # Ноды
        robot_state_publisher_node,
        joint_state_publisher_node,
        # rviz_node,  # Раскомментируйте для автозапуска RViz
    ])


"""
===================================================================================
ПРИМЕРЫ ИСПОЛЬЗОВАНИЯ
===================================================================================

1. Запустить только robot_state_publisher:
   ros2 launch verter_admin robot_description.launch.py

2. Запустить с RViz для визуализации:
   ros2 launch verter_admin robot_description.launch.py use_rviz:=true

3. Запустить вместе с одометрией:
   # В терминале 1:
   ros2 launch verter_admin robot_description.launch.py

   # В терминале 2:
   python3 src/verter_admin/odometry/odometry_node.py

4. Проверить TF дерево:
   # Посмотреть список всех TF:
   ros2 run tf2_tools view_frames

   # Посмотреть конкретную трансформацию:
   ros2 run tf2_ros tf2_echo base_link sensor_front_center

5. Визуализировать модель в RViz:
   rviz2

   Затем в RViz:
   - Add -> RobotModel
   - Fixed Frame: base_link
   - Add -> TF для визуализации координатных систем

===================================================================================
"""
