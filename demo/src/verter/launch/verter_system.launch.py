from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Создает описание запуска для системы Verter с тремя узлами."""
    return LaunchDescription([
        # Узел arduino_proxy_node - связь с Arduino
        Node(
            package='verter',
            executable='arduino_moving_navigation',
            name='arduino_moving_navigation_node',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='verter',
            executable='arduino_hands',
            name='arduino_hands_node',
            output='screen',
            emulate_tty=True,
        ),
        
        # Узел distance_sensor_node - анализ данных с датчика
        Node(   
            package='verter',
            executable='distance_controller',
            name='distance_controller_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'distance_threshold': 30.0}  # Порог в сантиметрах
            ]
        ),
        

    ]) 