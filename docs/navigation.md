# Навигация Verter Robot

Полное руководство по навигационной системе робота Verter.

## Содержание

- [Обзор системы](#обзор-системы)
- [Архитектура](#архитектура)
- [Железо](#железо)
- [Ключевые файлы](#ключевые-файлы)
- [Запуск навигации](#запуск-навигации)
- [Создание карты](#создание-карты)
- [RViz](#rviz)
- [Safety](#safety)
- [Troubleshooting](#troubleshooting)

---

## Обзор системы

Навигационная система Verter Robot основана на **Nav2** (ROS 2 Navigation Framework) и использует:

- **Лидар**: RPLiDAR A1M8 для детектирования препятствий
- **SLAM Toolbox**: для создания карт
- **AMCL**: для локализации на готовой карте
- **DWB**: как локальный планировщик
- **NavFn**: как глобальный планировщик
- **ESP32**: для управления моторами через micro-ROS

**Основные сценарии**:
1. Создание карты помещения (SLAM)
2. Автономная навигация по готовой карте
3. Автономное исследование пространства (Explore Lite)

---

## Архитектура

### Слои системы (SDHR)

```
┌─────────────────────────────────────────────────────────────┐
│                    L3: Mission/HMI                         │
│                  (голосовые команды, AI)                   │
└────────────────────────┬────────────────────────────────────┘
                         │ "поехали на кухню"
                         ↓ /goal_pose
┌─────────────────────────────────────────────────────────────┐
│                     L2: Planning/Behavior                   │
│          Nav2 (AMCL + DWB + NavFn + behaviors)             │
└────────────────────────┬────────────────────────────────────┘
                         │ /nav2/cmd_vel
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                   L1: Control/Localization                  │
│    (odometry_node, EKF, twist_mux, safety_policies)        │
└────────────────────────┬────────────────────────────────────┘
                         │ /cmd_vel
                         ↓
┌─────────────────────────────────────────────────────────────┐
│                    L0: Safety/Actuation                     │
│              ESP32 (micro-ROS, watchdog, PID)               │
└─────────────────────────────────────────────────────────────┘
```

### Цепочка команд

```
bt_navigator / waypoint_follower
    → controller_server (DWB)
        → velocity_smoother
            → /nav2/cmd_vel
                → twist_mux (арбитраж приоритетов)
                    → /cmd_vel
                        → ESP32 (mоторы)
```

### Twist Mux приоритеты

| Приоритет | Источник | Описание |
|-----------|----------|----------|
| 200 | Safety | Экстренная остановка |
| 100 | Teleop keyboard | Ручное управление с клавиатуры |
| 90 | Teleop joystick | Ручное управление с джойстика |
| 10 | Nav2 | Автономная навигация |

**Safety приоритет всегда может остановить робота.**

---

## Железо

### Лидар: RPLiDAR A1M8

- **Подключение**: USB → `/dev/rplidar`
- **Скорость**: 115200 baud
- **Режим**: Express (~10 Hz)
- **Угол обзора**: 360° (фильтруется до передних 180°)
- **Диапазон**: 0.20–6.0 м
- **Позиция**: 0.7 м от пола, развёрнут на 180°

### ESP32 шасси

- **Подключение**: USB → `/dev/esp32_chassis`
- **Скорость**: 921600 baud
- **Watchdog**: 500 мс (без команд → безопасная остановка)
- **Энкодеры**: AS5600 магнитные по I2C

### Моторы

- **Тип**: Дифференциальный привод
- **Max скорость**: 0.3 m/s linear, 0.5 rad/s angular
- **Колёсная база**: 0.374 м
- **Радиус колеса**: 0.1 м

### Дополнительные датчики

- **7× ультразвуковых** (передняя дуга)
- **ToF камера** (перед)
- **IMU BMX055** (временно отключён из-за дрейфа)

---

## Ключевые файлы

### Launch файлы

| Файл | Описание |
|------|----------|
| [`launch/nav2_navigation.launch.py`](../verter_admin/src/verter_admin/launch/nav2_navigation.launch.py) | Навигация по готовой карте |
| [`launch/mapping.launch.py`](../verter_admin/src/verter_admin/launch/mapping.launch.py) | Создание карты (SLAM) + ручное управление |
| [`launch/autonomous_mapping_real.launch.py`](../verter_admin/src/verter_admin/launch/autonomous_mapping_real.launch.py) | Автономное создание карты (Explore Lite) |

### Конфигурация

| Файл | Описание |
|------|----------|
| [`config/nav2/nav2_navigation_params.yaml`](../verter_admin/src/verter_admin/config/nav2/nav2_navigation_params.yaml) | Nav2 настройки (AMCL, DWB, costmaps) |
| [`config/slam/slam_toolbox_params.yaml`](../verter_admin/src/verter_admin/config/slam/slam_toolbox_params.yaml) | SLAM настройки |
| [`config/robot_localization/ekf.yaml`](../verter_admin/src/verter_admin/config/robot_localization/ekf.yaml) | EKF фильтр |
| [`config/laser_filters/laser_filter.yaml`](../verter_admin/src/verter_admin/config/laser_filters/laser_filter.yaml) | Фильтр лидара |
| [`config/explore/explore_lite_real_params.yaml`](../verter_admin/src/verter_admin/config/explore/explore_lite_real_params.yaml) | Explore Lite |
| [`config/rviz/waypoint_navigation.rviz`](../verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz) | RViz конфигурация |

### Контракты

| Файл | Описание |
|------|----------|
| [`contracts/motion.py`](../verter_admin/src/verter_admin/contracts/motion.py) | Topic contracts для движения |
| [`urdf/verter_robot_minimal.urdf`](../verter_admin/src/verter_admin/urdf/verter_robot_minimal.urdf) | Робот модель |

### Карта

- **Пример**: [`maps/hospital_map.yaml`](../maps/hospital_map.yaml)
- **Формат**: `.yaml` + `.pgm` (стандарт ROS)
- **Разрешение**: 0.05 м/пиксель (5 см/ячейка)

---

## Запуск навигации

### Предварительная проверка

```bash
# Проверить устройства
ls /dev/rplidar      # Лидар
ls /dev/esp32_chassis  # ESP32 шасси

# Если устройства не видны, проверить
ls /dev/ttyUSB* /dev/ttyACM*
```

### Шаг 1: Сборка (если нужно)

```bash
cd verter_admin
source /opt/ros/humble/setup.bash
colcon build --packages-select verter_admin --symlink-install
source install/setup.bash
```

### Шаг 2: Запуск навигации по готовой карте

```bash
# Базовый запуск
ros2 launch verter_admin nav2_navigation.launch.py map:=maps/hospital_map.yaml

# С自定义 портами (если нужно)
ros2 launch verter_admin nav2_navigation.launch.py \
    map:=maps/hospital_map.yaml \
    lidar_port:=/dev/ttyUSB0 \
    esp32_port:=/dev/ttyUSB1
```

### Шаг 3: Проверка работы

```bash
# Посмотреть ноды
ros2 node list

# Проверить топики
ros2 topic list | grep -E "(scan|cmd_vel|map|odom)"

# Проверить частоту лидара
ros2 topic hz /scan

# Проверить одометрию
ros2 topic echo /odometry/filtered --once
```

### Отправка цели через RViz

1. **Запустите RViz**:
   ```bash
   rviz2
   ```

2. **Загрузите конфигурацию**:
   - File → Open Config → `config/rviz/waypoint_navigation.rviz`

3. **Установите начальную позицию**:
   - Инструмент "2D Pose Estimate"
   - Кликните на карте где находится робот

4. **Укажите цель**:
   - Инструмент "Nav2 Goal"
   - Кликните куда нужно поехать

5. **Робот начнёт движение!**

### Через CLI (Action)

```bash
# Через ros2 action
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

---

## Создание карты

### Вариант 1: Ручное картирование (teleop)

```bash
# Запуск SLAM
ros2 launch verter_admin mapping.launch.py

# В другом терминале — ручное управление
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/navigation/cmd_vel
```

**Процесс**:
1. Управляйте роботом с клавиатуры
2. Обходите всё помещение медленно (≤0.15 m/s)
3. В RViz смотрите карту в реальном времени
4. После окончания сохраните карту

### Сохранение карты

**Вариант A: Через RViz**
1. Panels → Add New Panel → SlamToolboxPlugin
2. Кнопка "Save Map"
3. Введите имя файла

**Вариант B: Через CLI**
```bash
ros2 run nav2_map_server map_saver_cli -f maps/my_new_map
```

### Вариант 2: Автономное картирование

```bash
# Запуск autonomous mapping
ros2 launch verter_admin autonomous_mapping_real.launch.py

# RViz для визуализации
rviz2
```

**Explore Lite автоматически**:
- Исследует неизвестные области
- Создаёт карту
- Останавливается когда исследовано всё

**Параметры**:
- `stop_distance`: 0.15 м (экстренная остановка)
- `resume_distance`: 0.20 м (возобновление движения)

---

## RViz

### Конфигурация

**Файл**: `config/rviz/waypoint_navigation.rviz`

**Отображаемые элементы**:
- **Map** — карта (`/map`)
- **LaserScan** — лидар (`/scan`)
- **TF** — трансформации
- **Robot Model** — 3D модель робота
- **MarkerArray** — waypoints (`/waypoints/markers`)
- **Grid** — сетка

### Панель Navigation2

**Инструменты**:
- **SetInitialPose** — установка начальной позиции
- **SetGoal** — указание цели навигации
- **Measure** — измерение расстояний
- **Orbit** — орбитальная камера

**Кнопки**:
- Pause/Resume — приостановить/возобновить навигацию
- Clear Goals — очистить цели

---

## Safety

### Safety states

| Состояние | Описание | Действие |
|-----------|----------|----------|
| READY | Всё нормально | Движение разрешено |
| DEGRADED | Датчики устарели | Движение ограничено |
| FAULT | Препятствие | Экстренная остановка |

### Правила

```python
# STOP
if nearest_distance ≤ 0.15 м:
    stop_robot()

# RESUME
if nearest_distance ≥ 0.20 м:
    resume_motion()

# DEGRADED
if sensors_fresh == False AND allow_degraded_motion == False:
    enter_degraded_mode()
```

### Timeouts

- **Command timeout**: 0.5 сек (все источники cmd_vel)
- **Sensor timeout**: 0.8 сек (ультразвуковые датчики)
- **ESP32 watchdog**: 500 мс (safe stop)

### Приоритеты арбитража

```
200 (Safety) — может остановить в любой момент
100 (Teleop) — ручное управление
90 (Joystick) — джойстик
10 (Nav2) — автонавигация (низший приоритет)
```

---

## Troubleshooting

### Лидар не виден

```bash
# Проверить USB
ls /dev/ttyUSB* /dev/ttyACM*

# Переопределить порт
ros2 launch verter_admin nav2_navigation.launch.py lidar_port:=/dev/ttyUSB0
```

### Робот не двигается

```bash
# Проверить топик cmd_vel
ros2 topic echo /cmd_vel

# Проверить ESP32
ros2 node list | grep micro

# Проверить одометрию
ros2 topic echo /odometry/filtered --once
```

### AMCL не находит позицию

**Решение**: В RViz используйте "2D Pose Estimate" чтобы указать примерную позицию робота на карте.

### Карта пустая

```bash
# Проверить лидар
ros2 topic echo /scan --once

# Проверить SLAM
ros2 node list | grep slam

# Проверить топик карты
ros2 topic hz /map
```

### Робот дрожит/вибрирует

**Причина**: Слишком высокие gains в DWB planner

**Решение**: Уменьшите `velocity_gain` в `config/nav2/nav2_navigation_params.yaml`

### Уведомления о "odom timeout"

**Причина**: Энкодеры не публикуются или ESP32 disconnected

**Решение**:
```bash
# Проверить topik
ros2 topic hz /wheel_encoders

# Перезапустить micro_ros_agent
```

---

## Критические топики

### Sensor streams

| Топик | Тип | QoS | Частота |
|-------|-----|-----|---------|
| `/scan` | LaserScan | Best Effort | ~10 Hz |
| `/ultrasonic/ranges` | Float32[] | Best Effort | 10 Hz |
| `/wheel_encoders` | Encoders | Reliable | 10 Hz |

### Command streams

| Топик | Тип | QoS | Описание |
|-------|-----|-----|----------|
| `/nav2/cmd_vel` | Twist | Reliable | Nav2 output |
| `/safety/cmd_vel` | Twist | Reliable | Safety override |
| `/teleop_keyboard/cmd_vel` | Twist | Reliable | Manual control |
| `/cmd_vel` | Twist | Reliable | Final (ESP32) |

### Localization

| Топик | Тип | QoS | Описание |
|-------|-----|-----|----------|
| `/odometry/filtered` | Odometry | Reliable | EKF output |
| `/odom` | Odometry | Reliable | Raw encoders |
| `/map` | OccupancyGrid | Transient Local | SLAM/map server |
| `/initialpose` | PoseWithCovariance | Reliable | Initial pose estimate |
| `/goal_pose` | PoseStamped | Reliable | Navigation goal |

---

## Интеграция с голосовой системой

### Отправка цели через Python

```python
import rclpy
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = rclpy.create_node('voice_navigation')
publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)

# Пример: "поехали на кухню"
goal = PoseStamped()
goal.header.frame_id = "map"
goal.pose.position.x = 2.5  # Координаты кухни
goal.pose.position.y = 1.0
goal.pose.orientation.w = 1.0  # Не вращаться

publisher.publish(goal)
rclpy.spin_once(node, timeout_sec=1)
```

### Feedback от навигации

```python
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

# Подписка на результат навигации
action_client = ActionClient(node, NavigateToPose, '/navigate_to_pose')

# Проверка статуса
goal_handle = action_client.send_goal_async(goal)
```

---

## Параметры Nav2

### Основные параметры

**DWB Local Planner**:
- `max_vel_x`: 0.3 m/s
- `max_vel_theta`: 0.5 rad/s
- `xy_goal_tolerance`: 0.15 m
- `yaw_goal_tolerance`: 0.1 rad

**Costmaps**:
- `resolution`: 0.05 м
- `robot_radius`: 0.25 м
- `inflation_radius`: 0.40 м

**Controller**:
- `controller_frequency`: 5.0 Hz
- `update_frequency`: 5.0 Hz (local), 1.0 Hz (global)

---

## Дополнительные ресурсы

- **Nav2 Documentation**: https://docs.nav2.org/
- **SLAM Toolbox**: https://github.com/SteveMacenski/slam_toolbox
- **RViz User Guide**: https://docs.ros.org/en/humble/tutorials/tools/rviz2/rviz2.html
- **REP-105** (Coordinate Frames): https://www.ros.org/reps/rep-0105.html
- **ROS 2 Navigation Tuning Guide**: https://docs.nav2.org/tuning_guide.html

---

## Известные ограничения

1. **IMU отключён** — BMX055 гироскоп даёт дрейф, временно отключён в EKF
2. **Задний обзор** — Лидар видит только передние 180°
3. **Максимальная скорость** — 0.3 m/s (ограничено Nav2 и ESP32)
4. **Разрешение карты** — 5 см/ячейка (может не видеть мелкие препятствия)
