# Управление ROS2-нодами

## Окружение

```bash
source /opt/ros/humble/setup.bash
cd ~/verter-robot/verter_admin && source install/setup.bash
```

## Навигационный стек

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar
ros2 run verter_admin odometry_node
ros2 run robot_localization ekf_node
```

## Голосовой интерфейс

```bash
ros2 run verter_admin speech_to_text_node
ros2 run verter_admin recognition_node
ros2 run verter_admin ai_assistant_node
ros2 run verter_admin silero_tts_node
ros2 run verter_admin sound_player_node
ros2 run verter_admin doa_node
```

## Waypoint-менеджер

waypoint_manager_node — обычная нода (plain Node), не LifecycleNode
(преобразован из LifecycleNode для устранения deadlock lifecycle-менеджера).
Команды `ros2 lifecycle set` к ней неприменимы.

```bash
ros2 run verter_admin waypoint_manager_node
```

## Остановка

```bash
# Мягкая (SIGTERM)
pkill -TERM -f "ros2 run verter_admin"

# Принудительная (SIGKILL)
pkill -KILL -f "ros2 run verter_admin"
```

## Проверка

```bash
ps aux | grep verter_admin
ros2 node list
ros2 topic list
ros2 topic info <topic_name>
```
