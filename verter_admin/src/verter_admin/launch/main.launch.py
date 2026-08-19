#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    """
    Launch файл для запуска системы verter_admin.
    """

    pkg = get_package_share_directory('verter_admin')

    ekf_config         = os.path.join(pkg, 'config', 'robot_localization', 'ekf.yaml')
    laser_filter_cfg   = os.path.join(pkg, 'config', 'laser_filters', 'laser_filter.yaml')
    urdf_file = os.path.join(pkg, 'urdf', 'verter_robot_minimal.urdf')

    with open(urdf_file) as f:
        robot_description = f.read()

    esp32_imu_port_arg = DeclareLaunchArgument(
        'esp32_imu', default_value='/dev/esp32_imu',
        description='Serial port symlink for the imu ESP32',
    )

    esp32_ctrl_port_arg = DeclareLaunchArgument(
        'esp32_ctrl', default_value='/dev/esp32_ctrl',
        description='Serial port symlink for the ble and doa ESP32',
    )

    extra_args_arg = DeclareLaunchArgument(
        'micro_ros_agent_extra_args', default_value='',
        description='Extra args for micro_ros_agent (e.g. -v6 for verbose log)',
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8081',
        description='Port for web UI server',
    )

    chassis_config = os.path.join(
        get_package_share_directory("verter_admin"),
        "chassis",
        "params.yaml"
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port', default_value='/dev/rplidar',
        description='Serial port for RPLiDAR',
    )

    micro_ros_sensors = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'trap "kill 0; exit" TERM INT; '
                'PORT=', LaunchConfiguration('esp32_imu'), '; '
                'while true; do '
                '  echo "[micro_ros_agent chassis] resetting ESP32 on $PORT via DTR..."; '
                '  python3 ~/verter-robot/verter_admin/scripts/reset_esp32_dtr.py $PORT || true; '
                '  source ~/microros_ws/install/setup.bash && '
                '  exec ros2 run micro_ros_agent micro_ros_agent serial '
                '    --dev $PORT -b 921600 ',
                LaunchConfiguration('micro_ros_agent_extra_args'),
                '; '
                '  echo "[micro_ros_agent chassis] exited, restarting in 2 s..."; '
                '  sleep 2; '
                'done',
            ],
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    micro_ros_ctrl = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'trap "kill 0; exit" TERM INT; '
                'PORT=', LaunchConfiguration('esp32_ctrl'), '; '
                'while true; do '
                '  echo "[micro_ros_agent chassis] resetting ESP32 on $PORT via DTR..."; '
                '  python3 ~/verter-robot/verter_admin/scripts/reset_esp32_dtr.py $PORT || true; '
                '  source ~/microros_ws/install/setup.bash && '
                '  exec ros2 run micro_ros_agent micro_ros_agent serial '
                '    --dev $PORT -b 921600 ',
                LaunchConfiguration('micro_ros_agent_extra_args'),
                '; '
                '  echo "[micro_ros_agent chassis] exited, restarting in 2 s..."; '
                '  sleep 2; '
                'done',
            ],
        ],
        output='screen',
        sigterm_timeout='5',
        sigkill_timeout='10',
    )

    return LaunchDescription([
        # Переменные окружения для Yandex Cloud
        SetEnvironmentVariable('YANDEX_CLOUD_FOLDER', ''),
        SetEnvironmentVariable('YANDEX_CLOUD_API_KEY', ''),
        SetEnvironmentVariable('YANDEX_CLOUD_MODEL', 'yandexgpt'),

        #Узел esp32_sensors
        esp32_imu_port_arg,
        extra_args_arg,
        web_port_arg,
        micro_ros_sensors,

        #Узел для управления через ble модуль в ESP32
        esp32_ctrl_port_arg,
        micro_ros_ctrl,

        # Speech-to-Text (GigaAM v3 RNNT с пунктуацией — расставляет пробелы и знаки)
        Node(
            package='verter_admin',
            executable='speech_to_text_gigaam_v3_rnnt_node',
            name='speech_to_text_node',
            output='screen'
        ),

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

        # # Узел синтеза речи (Piper)
        # Node(
        #     package='verter_admin',
        #     executable='text_to_speech_node',
        #     name='text_to_speech_node',
        #     output='screen',
        #     parameters=[{
        #         'audio_device': 'pulse'  # Использует default sink PulseAudio
        #     }]
        # ),

        # Узел синтеза речи (Silero TTS)
        Node(
            package='verter_admin',
            executable='silero_tts_node',
            name='silero_tts_node',
            output='screen'
        ),

        # Узел синтеза речи (Vosk TTS)
        # Node(
        #     package='verter_admin',
        #     executable='vosk_tts_node',
        #     name='vosk_tts_node',
        #     output='screen',
        #     parameters=[{
        #         'speaker_id': 3,
        #         'audio_device': 'pulse'
        #     }]
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
        
        # Нода для управления моторами через ZLAC драйвер
        Node(
            package="verter_admin",
            executable="chassis_zlac_node",
            parameters=[chassis_config],
            output="screen"
        ),

        # Узел DOA (Direction of Arrival)
        Node(
           package='verter_admin',
           executable='doa_node',
           name='doa_node',
           output='screen'
        ),

        # Узел для работы лидара
        lidar_port_arg,
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rplidar_ros',
                    executable='rplidar_node',
                    name='rplidar_node',
                    parameters=[{
                        'serial_port':      LaunchConfiguration('lidar_port'),
                        'frame_id':         'lidar_link',
                        'scan_mode':        'Sensitivity',
                        'serial_baudrate':  115200,
                        'inverted':         False,
                        'angle_compensate': False,
                    }],
                    remappings=[('scan', '/scan_raw')],
                    respawn=True,
                    respawn_delay=5.0,
                    output='screen',
                ),
            ],
        ),
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            parameters=[laser_filter_cfg],
            remappings=[('scan', '/scan_raw'), ('scan_filtered', '/scan')],
            output='screen',
        ),

        # Узел для публикации габаритов робота
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen',
        ),


        # Узел для расчета одометрии
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen',
        ),

        # === Kiosk UI ===
        # rosbridge WebSocket server — мост ROS <-> web-фронтенд (порт 9090).
        # Внимание: rosbridge даёт НЕаутентифицированный доступ к ROS-топикам/сервисам.
        # Допустимо только на localhost для kiosk; не экспонировать наружу без защиты.
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': 9090,
                # Слушать только localhost — rosbridge даёт НЕаутентифицированный
                # доступ к ROS-топикам/сервисам; не экспонировать в сеть.
                'address': 'localhost',
            }],
            output='screen',
        ),

        # Web-сервер статики фронтенда.
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