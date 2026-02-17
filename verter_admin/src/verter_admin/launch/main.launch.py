#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch файл для запуска системы verter_admin.
    """
    return LaunchDescription([
        # # Узел Speech-to-Text (низкоуровневый)
        Node(
            package='verter_admin',
            executable='speech_to_text_node',
            name='speech_to_text_node',
            output='screen'
        ),

        # Node(
        #     package='verter_admin',
        #     executable='speech_to_text_transducer_node',
        #     name='speech_to_text_transducer_node',
        #     output='screen'
        # ),

        # Node(
        #     package='verter_admin',
        #     executable='speech_to_text_sherpa_node',
        #     name='speech_to_text_sherpa_node',
        #     output='screen'
        # ),

        # Node(
        #     package='verter_admin',
        #     executable='speech_to_text_parakeet_node',
        #     name='speech_to_text_parakeet_node',
        #     output='screen'
        # ),
        
        # Узел обработки распознанной речи (высокоуровневый)
        Node(
            package='verter_admin',
            executable='recognition_node',
            name='recognition_node',
            output='screen'
        ),
        
        # Узел AI ассистента
        Node(
            package='verter_admin',
            executable='ai_assistant_node',
            name='ai_assistant_node',
            output='screen'
        ),
        
        # Узел синтеза речи (Piper)
        Node(
            package='verter_admin',
            executable='text_to_speech_node',
            name='text_to_speech_node',
            output='screen',
            parameters=[{
                'audio_device': 'pulse'  # Использует default sink PulseAudio
            }]
        ),
        
        # Узел синтеза речи (Silero TTS)
        # Node(
        #     package='verter_admin',
        #     executable='silero_tts_node',
        #     name='silero_tts_node',
        #     output='screen'
        # ),
        
        # Узел воспроизведения звуков
        Node(
            package='verter_admin',
            executable='sound_player_node',
            name='sound_player_node',
            output='screen',
            parameters=[{
                'audio_device': 'pulse'  # Использует default sink PulseAudio
            }]
        ),
        
        # chassis_node УДАЛЁН — ESP32 через micro-ROS заменил его
        # Управление моторами и энкодеры теперь в esp32_chassis (mapping.launch.py)

        # Узел датчиков расстояния (Arduino Mega)
        Node(
            package='verter_admin',
            executable='distance_sensors_node',
            name='distance_sensors_node',
            output='screen'
        ),

        # Узел DOA (Direction of Arrival)
        Node(
            package='verter_admin',
            executable='doa_node',
            name='doa_node',
            output='screen'
        ),
    ])