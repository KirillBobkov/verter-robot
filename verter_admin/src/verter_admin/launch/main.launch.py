#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch файл для запуска системы verter_admin.
    """
    return LaunchDescription([
        # Узел распознавания речи
        Node(
            package='verter_admin',
            executable='speech_recognition_node',
            name='speech_recognition_node',
            output='screen'
        ),
        
        # Узел AI ассистента
        Node(
            package='verter_admin',
            executable='ai_assistant_node',
            name='ai_assistant_node',
            output='screen'
        ),
        
        # Узел синтеза речи
        Node(
            package='verter_admin',
            executable='text_to_speech_node',
            name='text_to_speech_node',
            output='screen'
        ),
        
        # Узел воспроизведения звуков
        Node(
            package='verter_admin',
            executable='sound_player_node',
            name='sound_player_node',
            output='screen'
        ),
    ])