#!/usr/bin/env python3

"""
Launch файл для голосового пайплайна Verter.
Запускает только активные ноды: STT, recognition, AI, TTS, sound player, EKF, web UI.
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """Генерирует LaunchDescription для голосового пайплайна."""

    pkg = get_package_share_directory('verter_admin')

    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8081',
        description='Port for web UI server',
    )

    ekf_config = os.path.join(pkg, 'config', 'robot_localization', 'ekf.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'verter_robot_minimal.urdf')

    with open(urdf_file) as f:
        robot_description = f.read()

    return LaunchDescription([
        web_port_arg,

        # === Переменные окружения для Yandex Cloud ===
        SetEnvironmentVariable('YANDEX_CLOUD_FOLDER', ''),
        SetEnvironmentVariable('YANDEX_CLOUD_API_KEY', ''),
        SetEnvironmentVariable('YANDEX_CLOUD_MODEL', 'yandexgpt'),

        # === Голосовой пайплайн ===

        # Speech-to-Text (GigaAM v3 RNNT с пунктуацией — расставляет пробелы и знаки)
        Node(
            package='verter_admin',
            executable='speech_to_text_gigaam_v3_rnnt_node',
            name='speech_to_text_node',
            output='screen'
        ),

        # # Speech-to-Text (GigaAM v3 CTC INT8 модель)
        # Node(
        #     package='verter_admin',
        #     executable='speech_to_text_gigaam_v3_ctc_node',
        #     name='speech_to_text_node',
        #     output='screen'
        # ),

        # # Speech-to-Text (CTC модель)
        # Node(
        #     package='verter_admin',
        #     executable='speech_to_text_node',
        #     name='speech_to_text_node',
        #     output='screen'
        # ),

        # FSM обработки распознанной речи (триггер-слова, команды)
        Node(
            package='verter_admin',
            executable='recognition_node',
            name='recognition_node',
            output='screen'
        ),

        # AI ассистент (YandexGPT)
        Node(
            package='verter_admin',
            executable='ai_assistant_node',
            name='ai_assistant_node',
            output='screen'
        ),

        # Text-to-Speech (Silero)
        Node(
            package='verter_admin',
            executable='silero_tts_node',
            name='silero_tts_node',
            output='screen'
        ),

        # Воспроизведение звуков
        Node(
            package='verter_admin',
            executable='sound_player_node',
            name='sound_player_node',
            output='screen',
            parameters=[{
                'audio_device': 'pulse'
            }]
        ),

        # === Робототехническая инфраструктура ===

        # Публикация URDF и TF дерева
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen',
        ),

        # EKF фильтр одометрии
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen',
        ),

        # === Web UI ===

        # Rosbridge WebSocket (порт 9090)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                'address': 'localhost',
            }],
            output='screen',
        ),

        # Web-сервер статики
        Node(
            package='verter_admin',
            executable='web_server_node',
            name='web_server_node',
            parameters=[{
                'port': LaunchConfiguration('web_port'),
            }],
            output='screen',
        ),
    ])
