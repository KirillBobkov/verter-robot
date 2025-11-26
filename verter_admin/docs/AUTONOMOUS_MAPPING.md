# Автономное картографирование и обнаружение низких препятствий

## Обзор

Система автономного картографирования позволяет роботу Verter самостоятельно исследовать неизвестное окружение, строить карту и избегать препятствий на разных высотах.

## Архитектура системы

### Двухуровневое обнаружение препятствий

```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                         │
│  ┌──────────────┐              ┌────────────────────┐       │
│  │ RPLiDAR A1M8 │              │ 7x HC-SR04 Sensors │       │
│  │ Height: 80cm │              │ Height: 5cm        │       │
│  │ Range: 12m   │              │ Range: 4m          │       │
│  │ FOV: 180°    │              │ Coverage: ~150°    │       │
│  └──────┬───────┘              └─────────┬──────────┘       │
└─────────┼──────────────────────────────┼──────────────────┘
          │                               │
          │ /verter/scan                  │ /verter/distance_sensors/*
          │ (LaserScan)                   │ (Range x7)
          │                               │
          │                               ▼
          │                    ┌──────────────────────┐
          │                    │ range_to_laserscan   │
          │                    │ Converter Node       │
          │                    └──────────┬───────────┘
          │                               │
          │                               │ /ultrasonic/ranges
          │                               │ (LaserScan)
          │                               │
          ▼                               ▼
    ┌─────────────────────────────────────────────┐
    │         NAV2 COSTMAP LAYERS                 │
    │  ┌────────────┐      ┌──────────────────┐  │
    │  │ Lidar Layer│      │ Ultrasonic Layer │  │
    │  │ (High obs.)│      │ (Low obstacles)  │  │
    │  └─────┬──────┘      └────────┬─────────┘  │
    │        └──────────┬────────────┘            │
    │                   ▼                         │
    │          ┌─────────────────┐                │
    │          │ Inflation Layer │                │
    │          └────────┬────────┘                │
    └───────────────────┼─────────────────────────┘
                        │
                        ▼
          ┌─────────────────────────────┐
          │     SLAM TOOLBOX            │
          │  (Map Building)             │
          └──────────┬──────────────────┘
                     │
                     │ /map
                     ▼
          ┌─────────────────────────────┐
          │     EXPLORE LITE            │
          │  (Frontier Exploration)     │
          └──────────┬──────────────────┘
                     │
                     │ goal_pose
                     ▼
          ┌─────────────────────────────┐
          │   NAV2 NAVIGATION           │
          │  (Path Planning & Control)  │
          └──────────┬──────────────────┘
                     │
                     │ /cmd_vel
                     ▼
          ┌─────────────────────────────┐
          │   GAZEBO DIFF DRIVE         │
          │  (Robot Movement)           │
          └─────────────────────────────┘
```

## Компоненты системы

### 1. Конвертер сенсоров (range_to_laserscan)

**Файл:** `src/verter_admin/distance_sensors/range_to_laserscan.py`

**Назначение:** Объединяет данные от 7 дистанционных сенсоров в единый LaserScan топик.

**Входные топики:**
- `/verter/distance_sensors/front_center` (sensor_msgs/Range)
- `/verter/distance_sensors/front_left_inner` (sensor_msgs/Range)
- `/verter/distance_sensors/front_left_outer` (sensor_msgs/Range)
- `/verter/distance_sensors/front_right_inner` (sensor_msgs/Range)
- `/verter/distance_sensors/front_right_outer` (sensor_msgs/Range)
- `/verter/distance_sensors/left` (sensor_msgs/Range)
- `/verter/distance_sensors/right` (sensor_msgs/Range)

**Выходной топик:**
- `/ultrasonic/ranges` (sensor_msgs/LaserScan)

**Параметры LaserScan:**
- FOV: 180° (-90° to +90°)
- Разрешение: ~1° (181 луч)
- Частота: 10 Hz
- Диапазон: 0.02m - 4.0m

**Конфигурация сенсоров в URDF:**

| Сенсор | Угол | Позиция в URDF |
|--------|------|----------------|
| front_center | 0° | (0.28, 0, 0.048) |
| front_left_inner | +5.7° | (0.277, 0.074, 0.048) |
| front_left_outer | +13.2° | (0.262, 0.152, 0.048) |
| front_right_inner | -5.7° | (0.277, -0.074, 0.048) |
| front_right_outer | -13.2° | (0.262, -0.152, 0.048) |
| left | +76° | (0.18, 0.28, 0.048) |
| right | -76° | (0.18, -0.28, 0.048) |

### 2. SLAM Toolbox

**Конфигурация:** `config/slam/slam_toolbox_params.yaml`

**Режим:** async (асинхронное картографирование)

**Входной топик:** `/verter/scan` (RPLiDAR)

**Выходные топики:**
- `/map` - карта окружения
- `/map_metadata` - метаданные карты
- TF: `map` → `odom`

### 3. Nav2 Navigation Stack

**Конфигурация:** `config/nav2/nav2_params.yaml`

**Компоненты:**
- **Local Costmap** - динамическая карта препятствий вокруг робота
  - Размер: 3x3 метра
  - Разрешение: 5 см
  - Частота обновления: 5 Hz

- **Global Costmap** - статическая карта всего окружения
  - Размер: 20x20 метров (расширяется автоматически)
  - Разрешение: 5 см
  - Частота обновления: 1 Hz
  - `track_unknown_space: true` - критично для exploration!

**Слои обнаружения препятствий:**

#### Lidar Layer (VoxelLayer)
```yaml
lidar_scan:
  sensor_frame: lidar_link
  data_type: LaserScan
  topic: /verter/scan
  min_obstacle_height: 0.0
  max_obstacle_height: 2.0
  obstacle_max_range: 10.0
  marking: true
  clearing: true
```

#### Ultrasonic Layer (VoxelLayer)
```yaml
ultrasonic:
  sensor_frame: base_link
  data_type: LaserScan
  topic: /ultrasonic/ranges
  min_obstacle_height: 0.0
  max_obstacle_height: 0.8
  obstacle_max_range: 1.0
  marking: true
  clearing: false  # Только маркируем препятствия, не чистим
```

### 4. Explore Lite

**Конфигурация:** `config/explore/explore_lite_params.yaml`

**Алгоритм:** Frontier-based exploration

**Принцип работы:**
1. Находит границы (frontiers) между известным и неизвестным пространством
2. Оценивает границы по расстоянию и размеру
3. Отправляет робота к самой перспективной границе
4. Повторяет до полного исследования

**Параметры:**
- `min_frontier_size: 0.5` - минимальный размер границы (метры)
- `planner_frequency: 1.0` - частота планирования (Hz)
- `progress_timeout: 30.0` - таймаут достижения цели (секунды)

## Сборка проекта

### Первая установка

```bash
# Установить зависимости
pip install catkin_pkg

# Собрать все пакеты
cd /home/oleksandr/verter/verter-robot/verter_admin
colcon build --packages-select verter_admin
colcon build --base-paths src/m-explore-ros2/ --cmake-args -DCMAKE_BUILD_TYPE=Release

# Применить окружение
source install/setup.bash
```

### После изменений кода

```bash
# Пересобрать только изменённый пакет
colcon build --packages-select verter_admin
source install/setup.bash
```

## Launch файлы

### autonomous_mapping.launch.py

**Назначение:** Полная система автономного картографирования

**Запуск:**
```bash
source install/setup.bash
ros2 launch verter_admin autonomous_mapping.launch.py
```

**Что запускается:**
1. Gazebo с миром `hospital_corridor.world`
2. Робот с сенсорами (спаун в позиции -7.0, -1.2)
3. Robot State Publisher (TF трансформации)
4. Range to LaserScan Converter (обработка дистанционных сенсоров)
5. SLAM Toolbox (построение карты)
6. Nav2 (навигация и планирование пути)
7. Explore Lite (автономное исследование)
8. RViz2 (визуализация)

**Результат:** Робот автоматически исследует окружение и строит карту!

### lidar_simulation.launch.py

**Назначение:** Ручное управление + SLAM

**Запуск:**
```bash
source install/setup.bash
ros2 launch verter_admin lidar_simulation.launch.py

# В другом терминале:
source install/setup.bash
ros2 run verter_admin teleop_keyboard
```

**Что запускается:**
1. Gazebo с миром `hospital_corridor.world`
2. Робот с сенсорами
3. Robot State Publisher
4. Range to LaserScan Converter
5. SLAM Toolbox
6. RViz2

**Результат:** Вы управляете роботом вручную, карта строится автоматически.

## Решение проблемы низких препятствий

### Проблема
RPLiDAR A1M8 установлен на высоте 80 см. Он не видит препятствия ниже этой высоты, такие как:
- Столы и скамейки (высота ~45 см)
- Тележки (высота ~85 см, но могут быть ниже)
- Другие низкие объекты

### Решение
7 ультразвуковых сенсоров HC-SR04 установлены на высоте 5 см:
- **Покрывают переднюю полусферу** (~150°)
- **Дальность:** до 4 метров
- **Высота обнаружения:** 0-80 см

Данные от сенсоров объединяются в LaserScan и добавляются в costmap как отдельный слой.

### Преимущества подхода

1. **Двойная защита:**
   - Lidar: дальние объекты, высокие препятствия
   - Ultrasonic: ближние объекты, низкие препятствия

2. **Безопасная навигация:**
   - Робот видит препятствия на всех высотах
   - Nav2 учитывает оба источника данных при планировании

3. **Модульность:**
   - Конвертер работает независимо
   - Легко настроить параметры чувствительности
   - Можно отключить любой слой отдельно

## Тестирование системы

### Проверка топиков

```bash
# Проверить данные лидара
ros2 topic echo /verter/scan --once

# Проверить данные дистанционных сенсоров
ros2 topic echo /verter/distance_sensors/front_center --once

# Проверить объединённый LaserScan
ros2 topic echo /ultrasonic/ranges --once

# Проверить карту
ros2 topic echo /map --once
```

### Проверка работы узлов

```bash
# Список всех узлов
ros2 node list

# Должны быть запущены:
# /range_to_laserscan
# /slam_toolbox
# /explore_node
# /controller_server
# /planner_server
# /bt_navigator
```

### Визуализация в RViz

В RViz должны быть видны:
1. **Map** - построенная карта окружения
2. **LaserScan** - красные точки от лидара
3. **Ultrasonic ranges** - данные от дистанционных сенсоров
4. **Local Costmap** - динамические препятствия вокруг робота
5. **Global Costmap** - статическая карта с препятствиями
6. **Frontiers** - зелёные маркеры (границы исследования)
7. **Global Plan** - запланированный путь
8. **Robot footprint** - контур робота

## Настройка параметров

### Для маленьких помещений (3x3м)

**Explore Lite:**
```yaml
min_frontier_size: 0.3
planner_frequency: 0.5
```

**Global Costmap:**
```yaml
width: 10
height: 10
```

### Для больших помещений (10x10м+)

**Explore Lite:**
```yaml
min_frontier_size: 0.8
planner_frequency: 1.0
```

**Global Costmap:**
```yaml
width: 30
height: 30
```

### Для быстрого исследования

**Explore Lite:**
```yaml
planner_frequency: 2.0
progress_timeout: 15.0
```

**Nav2 Controller:**
```yaml
desired_linear_vel: 0.3  # Увеличить скорость
```

### Для осторожного исследования

**Nav2 Costmap:**
```yaml
inflation_radius: 0.70  # Больше отступ от препятствий
robot_radius: 0.45      # Больший радиус робота
```

**Explore Lite:**
```yaml
goal_dist_tolerance: 0.4  # Точнее достигать целей
```

## Troubleshooting

### Робот не двигается

1. Проверить, что Nav2 запущен:
   ```bash
   ros2 node list | grep controller
   ```

2. Проверить топик команд:
   ```bash
   ros2 topic info /cmd_vel
   ```

3. Проверить лог Explore Lite:
   ```bash
   ros2 node list | grep explore
   ```

### Робот врезается в низкие препятствия

1. Проверить, что конвертер работает:
   ```bash
   ros2 node info /range_to_laserscan
   ```

2. Проверить топик ультразвука:
   ```bash
   ros2 topic hz /ultrasonic/ranges
   ```

3. Проверить в RViz визуализацию ultrasonic layer

### Робот не исследует все помещение

1. Увеличить `min_frontier_size` если границы слишком маленькие
2. Увеличить `progress_timeout` если робот отказывается от целей
3. Проверить `track_unknown_space: true` в global_costmap

### Карта искажена

1. Проверить TF трансформации:
   ```bash
   ros2 run tf2_tools view_frames
   ```

2. Проверить параметры SLAM:
   ```bash
   ros2 param list /slam_toolbox
   ```

3. Снизить скорость движения робота

## Файлы и директории

```
verter_admin/
├── src/verter_admin/
│   ├── launch/
│   │   ├── autonomous_mapping.launch.py   # Автономное картографирование
│   │   └── lidar_simulation.launch.py     # Ручное управление + SLAM
│   ├── distance_sensors/
│   │   └── range_to_laserscan.py          # Конвертер сенсоров
│   ├── config/
│   │   ├── nav2/
│   │   │   └── nav2_params.yaml           # Навигация
│   │   ├── slam/
│   │   │   └── slam_toolbox_params.yaml   # SLAM
│   │   └── explore/
│   │       └── explore_lite_params.yaml   # Exploration
│   ├── urdf/
│   │   └── verter_robot_gazebo.urdf       # Модель робота с сенсорами
│   └── worlds/
│       └── hospital_corridor.world        # Тестовый мир
├── setup.py                                # Регистрация узлов
└── docs/
    └── AUTONOMOUS_MAPPING.md              # Эта документация
```

## Дальнейшее развитие

### Возможные улучшения:

1. **Multi-floor mapping** - картографирование нескольких этажей
2. **Dynamic obstacles** - обнаружение движущихся объектов
3. **Semantic mapping** - классификация объектов на карте
4. **Coverage planning** - полное покрытие области
5. **Return to base** - автоматический возврат на базу

### Интеграция с реальным роботом:

1. Заменить Gazebo на реальные драйверы сенсоров
2. Настроить `use_sim_time: false` во всех узлах
3. Откалибровать параметры PID контроллера
4. Настроить фильтр Калмана для одометрии
5. Добавить recovery behaviors для застревания

## Ссылки

- [Nav2 Documentation](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Explore Lite](https://github.com/robo-friends/m-explore-ros2)
- [RPLiDAR ROS2](https://github.com/Slamtec/sllidar_ros2)
- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
