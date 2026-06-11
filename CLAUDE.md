# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Verter Robot** is an autonomous mobile robot for indoor navigation (hospitals, shopping centers, office buildings). Built on ROS2 Humble running on NVIDIA Jetson Orin Nano Super 8GB.

**Tech Stack**: ROS2 Humble, Nav2, SLAM Toolbox, micro-ROS (ESP32), React + TypeScript web UI, Sherpa-ONNX speech recognition, YandexGPT AI assistant, Piper/Silero TTS.

## Architecture Principles

This project follows **Safety-Driven Hexagonal Robotics (SDHR)** principles defined in `.claude/rules/robotics_architecture_rulebook.md`:

### Layered Decomposition by Criticality
- **L0 Safety/Actuation** (MCU): Emergency stop, command timeout — must work without mission stack
- **L1 Control/Localization**: Deterministic control and state estimation
- **L2 Planning/Behavior**: Replaceable without changing L0/L1 contracts
- **L3 Mission/HMI/Cloud**: Never in direct actuator control path

### Key Constraints
- All actuator commands MUST pass through single safety-gated chain
- L2/L3 MUST NOT bypass L1 contracts to reach L0 hardware
- Managed lifecycle for all nodes owning actuator commands, safety state, or sensor fusion
- QoS: Sensor streams use best effort; command paths use reliable delivery

### Hexagonal Boundaries (per package)
- `domain`: invariants, state machine, policies
- `application`: use-cases orchestrating domain
- `adapters`: ROS topics/services/actions, drivers, protocol glue
- `infrastructure`: launch, runtime wiring, config

Dependency rule: domain → (none), application → domain, adapters → application/domain, infrastructure → adapters/application. Forbidden: domain importing adapters/infrastructure.

## Build Commands

```bash
# Main package build
cd ~/verter-robot/verter_admin
colcon build --packages-select verter_admin

# Build with dependencies
colcon build --packages-up-to verter_admin

# Symlink install (for development)
colcon build --symlink-install

# After build, source
source install/setup.bash
```

## Quick Start Commands

### Mapping
```bash
# Manual mapping with teleop
ros2 launch verter_admin mapping.launch.py
ros2 run verter_admin teleop_keyboard

# Autonomous mapping
ros2 launch verter_admin autonomous_mapping_real.launch.py

# Save map
mkdir -p ~/maps
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/maps/my_map'}}"
~/verter-robot/verter_admin/scripts/save_map.sh ~/maps/my_map
```

### Navigation
```bash
# Start navigation (IMPORTANT: set initial pose in RViz after launch)
ros2 launch verter_admin navigation.launch.py map:=~/maps/my_map.yaml
```

### Waypoint Web UI
```bash
# Start navigation first
ros2 launch verter_admin navigation.launch.py map:=~/maps/my_map.yaml

# In another terminal
ros2 launch verter_admin waypoint_ui.launch.py

# Open http://<jetson-ip>:8080
```

### Voice Control
```bash
ros2 launch verter_admin main.launch.py
```

## Project Structure

```
verter_admin/                    # Main ROS2 package
├── src/verter_admin/
│   ├── ai_assistant/           # YandexGPT integration
│   ├── speech_to_text/         # Sherpa-ONNX STT (CTC, Transducer, Parakeet)
│   ├── text_to_speech/         # Piper & Silero TTS
│   ├── sound_player/           # Audio effects
│   ├── recognition/            # Speech processing state machine
│   ├── control/                # Motor control domain
│   │   ├── domain/             # Control logic, invariants
│   │   └── adapters/ros/       # ROS node wrappers
│   ├── waypoints/              # Waypoint management
│   │   ├── domain/
│   │   └── adapters/ros/
│   ├── web/                    # React frontend
│   │   ├── frontend/           # React source (npm install && npm run build)
│   │   └── adapters/ros/
│   ├── launch/                 # Launch files
│   ├── config/                 # Nav2, SLAM, EKF configs
│   └── urdf/                   # Robot model
└── scripts/                    # Utility scripts
```

## Frontend Development

```bash
cd ~/verter-robot/verter_admin/src/verter_admin/web/frontend
npm install
npm run dev          # Dev server on port 3000
npm run build        # Build for production (required before colcon build)
```

## Key ROS Topics

- `/scan` - Laser scan from RPLiDAR A1M8
- `/cmd_vel` - Velocity commands (safety-gated through twist_mux)
- `/amcl_pose` - Estimated robot pose
- `/odometry/filtered` - Fused odometry (EKF)
- `/recognized_text` - Speech recognition output
- `/verter_commands` - Processed voice commands
- `/text_to_speech` - Text for speech synthesis
- `/waypoints/markers` - Waypoint visualization

## Environment Variables

Required for AI assistant:
- `YANDEX_CLOUD_FOLDER` - Yandex Cloud folder ID
- `YANDEX_CLOUD_API_KEY` - API key for YandexGPT
- `YANDEX_CLOUD_MODEL` - Model selection

## Hardware

- **Compute**: NVIDIA Jetson Orin Nano Super 8GB
- **LIDAR**: RPLiDAR A1M8 (180° FOV, 12m range)
- **Ultrasonic**: 7x HC-SR04 sensors
- **IMU**: MPU-6050
- **Voice**: ReSpeaker USB microphone with DOA
- **Motors**: ESP32 with micro-ROS

## Dependencies

Key ROS2 packages: nav2, slam_toolbox, explore_lite, robot_localization, twist_mux, laser_filters, rosserial_arduino.

Python: sherpa-onnx, silero-tts, yandex-cloud-ml-sdk, onnxruntime, torch, opencv-python, sounddevice.

## Diagnostics

```bash
# Check LIDAR
ros2 topic hz /scan

# Check odometry
ros2 topic hz /odometry/filtered

# Check localization
ros2 topic echo /amcl_pose --once

# Check TF tree
ros2 run tf2_tools view_frames

# Check waypoint manager lifecycle
ros2 lifecycle get /waypoint_manager

# Manual waypoint manager activation
ros2 lifecycle set /waypoint_manager configure
ros2 lifecycle set /waypoint_manager activate
```

## Documentation

Comprehensive documentation in `/docs` (Russian, MkDocs-based):
- `docs/project/QUICKSTART_COMMANDS.md` - Detailed launch commands
- `docs/project/FAQ.md` - Common issues and solutions
- `docs/code/api.md` - Interface specifications
- `.claude/rules/robotics_architecture_rulebook.md` - SDHR architecture principles
