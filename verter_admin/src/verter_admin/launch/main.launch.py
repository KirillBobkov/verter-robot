from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """
    Launch-файл для запуска полной системы verter_admin:
    - Нода распознавания речи (speech_recognition_node)
    - Нода AI ассистента (ai_assistant_node)
    """
    
    # Аргументы запуска
    use_ai_arg = DeclareLaunchArgument(
        'use_ai',
        default_value='true',
        description='Запускать ли AI ассистента'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Уровень логирования (debug, info, warn, error)'
    )
    
    # Запуск ноды распознавания речи
    speech_recognition_node = Node(
        package='verter_admin',
        executable='speech_recognition_node',
        name='speech_recognition_node',
        output='screen',
        parameters=[
            {'log_level': LaunchConfiguration('log_level')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Запуск ноды AI ассистента
    ai_assistant_node = Node(
        package='verter_admin',
        executable='ai_assistant_node',
        name='ai_assistant_node',
        output='screen',
        parameters=[
            {'log_level': LaunchConfiguration('log_level')},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        condition=IfCondition(LaunchConfiguration('use_ai'))
    )
    
    # Информационные сообщения
    start_msg = LogInfo(
        msg="Запуск системы verter_admin..."
    )

    return LaunchDescription([
        # Аргументы
        use_ai_arg,
        log_level_arg,
        
        # Сообщения
        start_msg,
        
        # Ноды
        speech_recognition_node,
        ai_assistant_node,
    ])