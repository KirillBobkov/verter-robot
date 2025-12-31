# Первый запуск

## Launch-файлы

Launch-файлы лежат в `verter_admin/src/verter_admin/launch/`:

- `main.launch.py` — основной запуск
- `autonomous_mapping.launch.py` — автономное картирование
- `real_robot_navigation.launch.py` — навигация на реальном роботе
- `lidar_simulation.launch.py` — симуляция лидара
- `test_lidar.launch.py` — тест лидара

## Запуск через ROS2 launch

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch verter_admin main.launch.py
```

## Запуск отдельных нод

Список console scripts см. в `verter_admin/setup.py` (`entry_points.console_scripts`).

Пример:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run verter_admin speech_to_text_node
ros2 run verter_admin recognition_node
ros2 run verter_admin text_to_speech_node
ros2 run verter_admin sound_player_node
```


