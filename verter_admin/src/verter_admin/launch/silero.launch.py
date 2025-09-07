#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch файл для запуска системы Silero TTS.
    """
    return LaunchDescription([
        # Узел Silero TTS
        Node(
            package='verter_admin',
            executable='silero_tts_node',
            name='silero_tts_node',
            output='screen',
            parameters=[
                {'language': 'ru'},
                {'speaker': 'aidar'},  # Можно изменить на другой голос
                {'sample_rate': 48000},
                {'device': 'cpu'}  # Или 'cuda' для GPU
            ]
        ),
        
        # Узел распознавания речи (если нужен)
        Node(
            package='verter_admin',
            executable='speech_recognition_node',
            name='speech_recognition_node',
            output='screen'
        ),
        
        # Узел AI ассистента (если нужен)
        Node(
            package='verter_admin', 
            executable='ai_assistant_node',
            name='ai_assistant_node',
            output='screen'
        ),
    ])
