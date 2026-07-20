#!/usr/bin/env python3
"""Minimal chassis bringup — only micro_ros_agent for the new ZLAC chassis ESP32.

Use this for bench-testing the chassis in isolation BEFORE running mapping or
nav2. Brings up just:
  - micro_ros_agent serial → /dev/esp32_chassis @ 921600
    (auto-restart loop so it reconnects after ESP32 cold-boot from E-stop)

Does NOT bring up SLAM, Nav2, lidar, IMU, twist_mux, odometry — those add
moving parts that get in the way of debugging the chassis link itself. Once
/chassis/state reaches ACTIVE (3) and wheel_rpm responds to /cmd_vel, switch
to mapping.launch.py for the full stack.

Usage:
  ros2 launch verter_admin chassis_bringup.launch.py

In other terminals (while this is running):
  ros2 topic echo /chassis/state          # expect 2 (READY) then 3 (ACTIVE)
  ros2 topic echo /chassis/fault          # expect 0
  ros2 topic echo /chassis/cmd_watchdog   # expect false until first /cmd_vel
  ros2 topic echo /chassis/wheel_rpm      # expect [0, 0] idle, real RPM on motion

  # WHEELS IN AIR ONLY for the first motion test:
  ros2 topic pub -r 10 -t 20 /cmd_vel geometry_msgs/msg/Twist \\
      '{linear: {x: 0.05}, angular: {z: 0.0}}'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port', default_value='/dev/esp32_chassis',
        description='Serial port symlink for the chassis ESP32 (Modbus master to ZLAC8015D)',
    )
    extra_args_arg = DeclareLaunchArgument(
        'micro_ros_agent_extra_args', default_value='',
        description='Extra args for micro_ros_agent (e.g. -v6 for verbose log)',
    )

    # Auto-restart loop: ESP32 cold-boots whenever the hardware E-stop in the
    # 12V line is pressed and released (it kills the Buck → ESP32 reboots from
    # scratch). The micro_ros_agent currently in the foreground notices the
    # session dies, the bash loop respawns it, and the agent picks up the new
    # session as soon as the firmware finishes its setup() + ping retries.
    micro_ros_chassis = ExecuteProcess(
        cmd=[
            'bash', '-c',
            [
                'trap "kill 0; exit" TERM INT; '
                'PORT=', LaunchConfiguration('esp32_port'), '; '
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
        esp32_port_arg,
        extra_args_arg,
        micro_ros_chassis,
    ])
