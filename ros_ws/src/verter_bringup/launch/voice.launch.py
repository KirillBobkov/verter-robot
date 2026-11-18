from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(respawn=True, respawn_delay=5.0, package='verter_voice', executable=n, name=name)
            for n, name in [
                ('speech_to_text_gigaam_v3_rnnt_node', 'speech_to_text_node'),
                ('recognition_node', 'recognition_node'),
                ('ai_assistant_node', 'ai_assistant_node'),
                ('silero_tts_node', 'silero_tts_node'),
                ('sound_player_node', 'sound_player_node'),
            ]
        ]
    )
