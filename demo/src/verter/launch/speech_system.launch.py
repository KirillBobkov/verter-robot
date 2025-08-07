#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Запуск VoskNode для распознавания речи
        Node(
            package='verter',
            executable='vosk_node',
            name='vosk_node',
            output='screen',
            emulate_tty=True
        ),
        
        # Запуск SoundPlayerNode для воспроизведения звуков
        Node(
            package='verter',
            executable='sound_player',
            name='sound_player_node',
            output='screen'
        ),
    ]) 