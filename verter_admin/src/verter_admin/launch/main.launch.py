from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch-файл для запуска системы распознавания речи verter_admin.
    """
    
    # Запуск ноды распознавания речи
    speech_recognition_node = Node(
        package='verter_admin',
        executable='speech_recognition_node',
        name='speech_recognition_node',
        output='screen',
        parameters=[
            # Здесь можно добавить параметры если потребуется
        ]
    )

    return LaunchDescription([
        speech_recognition_node
    ])