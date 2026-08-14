# Автономное картографирование

Система позволяет роботу самостоятельно исследовать неизвестное помещение, строить карту и избегать препятствий на разных высотах.

## Архитектура

```
Датчики
  RPLiDAR A1M8          (86 см, ~80° после фильтра, физически 12 м)
  7x HC-SR04             (2 см, ~30° FOV, 2.5 м)
        |
        +--> /scan_raw --> laser_filter --> /scan   (LaserScan, ~10 Hz)
        |                                         |
        |                                         +--> Nav2 Costmap Layers
        |                                               voxel_layer (local, VoxelLayer)       (препятствия, /scan)
        |                                               obstacle_layer (global, ObstacleLayer) (препятствия, /scan)
        |                                               inflation_layer
        |                                               (Ultrasonic Layer — запланировано, не реализован)
        |
        +--> /ultrasonic/distances  (Float32MultiArray, ESP32)
               |
               v
         range_converter_node
               |
               +--> /verter/distance_sensors/*   (Range x7)

  /ultrasonic/ranges (LaserScan) — НЕ публикуется (конвертер Range->LaserScan не реализован)
               |
               +--> proximity_safety_node --> /safety/cmd_vel (приоритет 200, twist_mux)
                    (пока работает только по /scan — /ultrasonic/ranges нет данных)
        |
        v
    SLAM Toolbox         (построение карты -> /map)
        |
        v
    Explore Lite         (frontier exploration -> goal_pose)
        |
        v
    Nav2 Navigation      (планирование пути -> /cmd_vel напрямую, минуя twist_mux)
```

## Порядок запуска компонентов

Автономное картографирование требует строгого порядка инициализации:

| Время  | Компоненты                                                     |
|--------|----------------------------------------------------------------|
| 0 с    | micro-ROS (chassis + IMU), twist_mux, одометрия, EKF, robot_state_publisher, laser_filter, range_converter, proximity_safety, SLAM |
| 1 с    | RPLiDAR (TimerAction 1 с)                                      |
| 18 с   | Nav2-стек (локализация, планирование, TimerAction 18 с)        |
| 60 с   | Explore Lite (автономное исследование, TimerAction 60 с)       |

SLAM должен инициализироваться и начать строить карту до того, как Nav2 попытается локализироваться.

## Поток данных

1. RPLiDAR публикует `/scan_raw`, который фильтруется laser_filter в `/scan` (LaserScan, ~10 Hz).
2. Семь ультразвуковых сенсоров ESP32 публикуют массив `Float32MultiArray` на `/ultrasonic/distances`.
3. Нода `range_converter_node` преобразует `Float32MultiArray` в 7 отдельных `Range`-сообщений на `/verter/distance_sensors/*`.
4. `/scan` поступает в costmap-слои Nav2 (VoxelLayer local / ObstacleLayer global). Ультразвук НЕ входит в costmap. Узел `proximity_safety_node` подписан на `/ultrasonic/ranges` (LaserScan), но этот топик не публикуется — ультразвук пока не поступает в safety-слой (см. ниже).
5. SLAM Toolbox строит карту `/map` на основе данных лидара.
6. Explore Lite находит фронтиры на карте и отправляет цели в Nav2.
7. Nav2 планирует путь и выдаёт команды напрямую в `/cmd_vel` (ремап `cmd_vel`→`/nav2/cmd_vel` и velocity_smoother закомментированы в `nav2_navigation.launch.py` — цепочка безопасности twist_mux при standalone-запуске Nav2 разорвана).

## Конвертер ultrasonic (range_converter_node)

**Файл:** `src/verter_admin/control/adapters/ros/range_converter_node.py`

Нода подписывается на массив `Float32MultiArray` от ESP32 и публикует 7 отдельных `Range`-сообщений. Параметры каждого датчика: `min_range=0.02`, `max_range=2.5`, `field_of_view=30°`.

### Входной топик

| Топик                    | Тип               |
|--------------------------|-------------------|
| `/ultrasonic/distances`  | Float32MultiArray |

### Выходные топики

| Топик                                      | Тип   |
|--------------------------------------------|-------|
| `/verter/distance_sensors/front_center`     | Range |
| `/verter/distance_sensors/front_left_inner` | Range |
| `/verter/distance_sensors/front_left_outer` | Range |
| `/verter/distance_sensors/front_right_inner`| Range |
| `/verter/distance_sensors/front_right_outer`| Range |
| `/verter/distance_sensors/left`             | Range |
| `/verter/distance_sensors/right`            | Range |

### Топик `/ultrasonic/ranges` (LaserScan)

Узел `proximity_safety_node` ожидает `LaserScan` на топике `/ultrasonic/ranges` (см. `contracts/motion.py`, `TopicContract.ULTRASONIC_SCAN`). Конвертер `Range → LaserScan` для этого топика запланирован, но в текущем коде не реализован — ультразвуковые данные пока не поступают в safety-слой через этот канал.

### Расстановка сенсоров

| Сенсор              | Угол   | Позиция в URDF      |
|---------------------|--------|---------------------|
| front_center        | 0      | (0.28, 0, 0.048)    |
| front_left_inner    | +5.7   | (0.277, 0.074, 0.048) |
| front_left_outer    | +13.2  | (0.262, 0.152, 0.048) |
| front_right_inner   | -5.7   | (0.277, -0.074, 0.048) |
| front_right_outer   | -13.2  | (0.262, -0.152, 0.048) |
| left                | +76    | (0.180, 0.228, 0.048) |
| right               | -76    | (0.180, -0.228, 0.048) |

## Costmap-слои Nav2

**Конфигурация:** `config/nav2/nav2_navigation_params.yaml` — эталонная конфигурация. В `autonomous_mapping_real.launch.py` передаётся пустой `nav2_mapping_real_params.yaml` (0 байт) → Nav2 запускается со встроенными дефолтами, а не с параметрами из `nav2_navigation_params.yaml`. Параметры ниже описаны по эталонному файлу.

> **Важно:** ультразвуковый слой ОТСУТСТВУЕТ в costmap. Узел `proximity_safety_node` подписан на `/ultrasonic/ranges` (LaserScan), но этот топик не публикуется — фактически safety работает только по лидару (`/scan`). Слои costmap:

### Local Costmap: voxel_layer (VoxelLayer)

```yaml
voxel_layer:
  plugin: nav2_costmap_2d::VoxelLayer
  observation_sources: scan
  scan:
    topic: /scan
    data_type: LaserScan
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    raytrace_max_range: 5.5
    obstacle_max_range: 5.0
```

### Global Costmap: obstacle_layer (ObstacleLayer)

```yaml
obstacle_layer:
  plugin: nav2_costmap_2d::ObstacleLayer
  observation_sources: scan
  scan:
    topic: /scan
    data_type: LaserScan
    max_obstacle_height: 2.0
    clearing: true
    marking: true
    raytrace_max_range: 5.5
    obstacle_max_range: 5.0
```

### Inflation Layer (local и global)

```yaml
inflation_layer:
  plugin: nav2_costmap_2d::InflationLayer
  cost_scaling_factor: 3.0
  inflation_radius: 0.40
```

### Local / Global Costmap

| Параметр             | Local              | Global            |
|----------------------|--------------------|-------------------|
| Размер               | 4x4 м (rolling)    | авто (static)     |
| Разрешение           | 5 см               | 5 см              |
| Частота обновления   | 10 Hz              | 1 Hz              |
| Плагины              | voxel_layer, inflation_layer | static_layer, obstacle_layer, inflation_layer |
| track_unknown_space  | --                 | true (критично для exploration) |

## SLAM Toolbox

**Конфигурация:** `config/slam/slam_toolbox_params.yaml`

| Параметр                           | Значение     | Описание                           |
|------------------------------------|--------------|------------------------------------|
| `mode`                             | mapping      | Асинхронное картирование           |
| `solver_plugin`                    | CeresSolver  | Оптимизатор pose graph             |
| `resolution`                       | 0.03 м       | 3 см на ячейку                     |
| `min_laser_range`                  | 0.2 м        | Минимальная дальность скана        |
| `max_laser_range`                  | 8.0 м        | Ограничение дальности скана        |
| `minimum_travel_distance`          | 0.08 м       | Мин. расстояние между сканами      |
| `minimum_travel_heading`           | 0.0873 рад   | Мин. поворот между сканами (~5°)   |
| `link_match_minimum_response_fine` | 0.15         | Порог качества пар сканов          |
| `do_loop_closing`                  | true         | Закрытие цикла                     |
| `loop_match_minimum_response_fine` | 0.45         | Порог loop closure                 |
| `scan_buffer_size`                 | 10           | Буфер сканов для linking           |
| `use_multicore`                    | true         | Многопоточное сканирование         |

Рекомендуемая скорость при картировании: не более 0.15 м/с (при 10 Hz лидара — ~1.5 см за скан). Фактическая скорость задаётся в Nav2 (`desired_linear_vel: 0.04` в `nav2_navigation_params.yaml` — при 10 Hz лидара ~0.4 см за скан). Оба значения обеспечивают качество scan matching.

## Explore Lite

**Конфигурация:** `config/explore/explore_lite_real_params.yaml`

Алгоритм frontier-based exploration, 4 шага:
1. Находит границы между известным и неизвестным пространством.
2. Оценивает границы по расстоянию и размеру.
3. Отправляет робота к наиболее перспективной границе через Nav2.
4. Повторяет до полного исследования.

### Параметры

| Параметр               | Значение       | Описание                              |
|------------------------|----------------|---------------------------------------|
| `robot_base_frame`     | base_footprint | Координатная система робота           |
| `costmap_topic`        | /global_costmap/costmap | Карта препятствий            |
| `visualize`            | True           | Маркеры границ в RViz                 |
| `min_frontier_size`    | 0.05 м         | Минимальный размер границы            |
| `planner_frequency`    | 2.0 Hz         | Частота планирования новых целей      |
| `progress_timeout`     | 30.0 сек       | Таймаут достижения цели               |
| `goal_dist_tolerance`  | 0.3 м          | Точность достижения цели              |
| `potential_scale`      | 0.0005         | Штраф за расстояние                   |
| `orientation_scale`    | 0.0            | Учёт ориентации (0 = не учитывать)    |
| `gain_scale`           | 3.0            | Преимущество больших границ           |
| `transform_tolerance`  | 0.5 сек        | Таймаут ожидания TF                   |

**Важное требование:** Global costmap обязан иметь `track_unknown_space: true` -- без этого Explore Lite не находит frontiers.

## Высота лидара: обоснование 86 см

### Что видит лидар на разных высотах

| Высота  | Что видит                       | Оценка        |
|---------|---------------------------------|---------------|
| 200 см  | Головы, верхушки; пропускает столы, детей | Плохо   |
| 150 см  | Плечи, высокие столы; может пропустить низкие   | Допустимо |
| **86 см** | Туловища людей, столы, тележки, стулья | **Оптимально** |
| 40 см   | Ноги, ножки мебели               | Допустимо    |
| 15 см   | Низкие объекты, ножки мебели (ультразвук) | Комплементарно |

### Почему 86 см

1. Покрывает 90 %+ типичных препятствий в больницах и офисных зданиях.
2. Надёжная детекция людей -- туловище на высоте 80-100 см всегда видно.
3. Комплементарность с ультразвуком -- датчики на ~15 см покрывают низкие объекты.
4. Чистые карты SLAM -- минимум шума от мелких деталей на полу.
5. Соответствует индустриальным стандартам мобильных роботов.

### Двухуровневая защита

```
[ЛИДАР 86 см]
  Покрытие:  60--200 см
  Дальность: 0.15--12 м (физически); SLAM max_laser_range=8.0 м; AMCL laser_max_range=5.5 м
  FOV:       ~80° спереди (laser_filter ±0.7 рад)
  Роль:      Основная навигация, SLAM

[7x HC-SR04 ~15 см]
  Покрытие:  0--40 см
  Дальность: 0.02--2.5 м
  FOV:       ~30° на датчик (7 датчиков спереди и по бокам)
  Роль:      Safety layer, низкие объекты (через proximity_safety_node — пока не работает, /ultrasonic/ranges не публикуется)
```

Комбинация обеспечит полное покрытие по высоте от 0 до 200 см (ультразвук в safety пока не активен — `/ultrasonic/ranges` не публикуется). LiDAR -- дальняя точная детекция, ультразвук -- ближняя зона безопасности.

### Риски

- **Маленькие дети (рост <86 см)** -- лидар может не увидеть. Компенсация: ультразвук на близком расстоянии (пока не работает в safety) + низкая скорость (`desired_linear_vel: 0.04` м/с).
- **Прозрачное стекло** -- лидар плохо видит. Ультразвук надёжнее; стекло обычно в рамах.
- **Столы с тонкими ножками** -- лидар видит столешницу, ножки должен покрывать ультразвук (через proximity_safety_node — пока не работает) и inflation radius.

## Рекомендации по настройке

### Маленькие помещения (до 3x3 м)

| Компонент      | Параметр              | Значение |
|----------------|-----------------------|----------|
| Global Costmap | `width`, `height`     | 10       |
| Explore Lite   | `min_frontier_size`   | 0.3      |
| Explore Lite   | `planner_frequency`   | 0.5      |

### Большие помещения (10x10 м и более)

| Компонент      | Параметр              | Значение |
|----------------|-----------------------|----------|
| Global Costmap | `width`, `height`     | 30       |
| Explore Lite   | `min_frontier_size`   | 0.8      |
| Explore Lite   | `planner_frequency`   | 1.0      |

### Быстрое исследование

| Компонент       | Параметр               | Значение |
|-----------------|------------------------|----------|
| Explore Lite    | `planner_frequency`    | 2.0      |
| Explore Lite    | `progress_timeout`     | 15.0     |
| Nav2 Controller | `desired_linear_vel`   | 0.3 (дефолт в коде: 0.04) |

### Осторожное исследование

| Компонент      | Параметр               | Значение |
|----------------|------------------------|----------|
| Nav2 Costmap   | `inflation_radius`     | 0.70 (дефолт в коде: 0.40) |
| Nav2 Costmap   | `robot_radius`         | 0.45 (дефолт в коде: 0.25) |
| Explore Lite   | `goal_dist_tolerance`  | 0.4      |

## Запуск

```bash
# Автономное картографирование
ros2 launch verter_admin autonomous_mapping_real.launch.py

# RViz для мониторинга
rviz2
```

### Параметры запуска

| Параметр             | По умолчанию         | Описание                       |
|----------------------|----------------------|--------------------------------|
| `lidar_port`         | `/dev/rplidar`       | Путь к LiDAR                   |
| `esp32_port`         | `/dev/esp32_chassis` | ESP32 шасси                    |
| `imu_esp32_port`     | `/dev/esp32_imu`     | ESP32 IMU                      |
| `micro_ros_agent_extra_args` | `''`         | Доп. аргументы micro_ros_agent (напр. `-v6`) |
| `stop_distance`      | `0.15` м             | Расстояние экстренной остановки|
| `resume_distance`    | `0.20` м             | Расстояние для возобновления   |

## Диагностика

```bash
# Данные лидара
ros2 topic echo /scan --once
ros2 topic hz /scan

# Данные ультразвука (массив от ESP32)
ros2 topic echo /ultrasonic/distances --once
ros2 topic hz /ultrasonic/distances

# Индивидуальные Range-топики (от range_converter_node)
ros2 topic echo /verter/distance_sensors/front_center --once

# Карта
ros2 topic echo /map --once
ros2 topic hz /map

# Ключевые ноды
ros2 node list
# Ожидаемые: /range_converter_node, /slam_toolbox, /explore_node,
#            /controller_server, /planner_server, /bt_navigator, /proximity_safety_node
```

### Робот не двигается

1. `ros2 node list | grep controller` -- убедиться, что controller_server запущен.
2. `ros2 topic info /cmd_vel` -- проверить наличие публичеров.
3. Проверить лог Explore Lite: `ros2 node list | grep explore`.

### Карта искажена

1. Проверить TF: `ros2 run tf2_tools view_frames`.
2. Проверить параметры SLAM: `ros2 param list /slam_toolbox`.
3. Снизить скорость движения.

### Робот не исследует всё помещение

1. Увеличить `min_frontier_size`, если границы слишком маленькие.
2. Увеличить `progress_timeout`, если робот отказывается от целей.
3. Проверить `track_unknown_space: true` в global_costmap.

## Визуализация в RViz

Добавьте дисплеи:
- **Map** -- построенная карта.
- **LaserScan** (`/scan`) -- данные лидара.
- **Range** (`/verter/distance_sensors/*`) -- данные ультразвуковых датчиков.
- **Local Costmap** -- динамические препятствия вокруг робота.
- **Global Costmap** -- статическая карта с препятствиями.
- **MarkerArray** -- зелёные маркеры frontiers.
- **Path** -- запланированный путь.
- **RobotFootprint** -- контур робота.
