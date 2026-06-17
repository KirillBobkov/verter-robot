# Автономное картографирование

Система позволяет роботу самостоятельно исследовать неизвестное помещение, строить карту и избегать препятствий на разных высотах.

## Архитектура

```
Датчики
  RPLiDAR A1M8          (80 см, 180, 12 м)
  7x HC-SR04             (5 см, ~150, 4 м)
        |
        +--> /scan                   (LaserScan)
        |
        +--> /verter/distance_sensors/*  (Range x7)
               |
               v
         range_to_laserscan Converter
               |
               +--> /ultrasonic/ranges   (LaserScan)
        |
        v
    Nav2 Costmap Layers
      Lidar Layer        (высокие препятствия)
      Ultrasonic Layer   (низкие препятствия)
      Inflation Layer
        |
        v
    SLAM Toolbox         (построение карты -> /map)
        |
        v
    Explore Lite         (frontier exploration -> goal_pose)
        |
        v
    Nav2 Navigation      (планирование пути -> /cmd_vel)
```

## Порядок запуска компонентов

Автономное картографирование требует строгого порядка инициализации:

| Время  | Компоненты                                     |
|--------|------------------------------------------------|
| 0 с    | micro-ROS, twist_mux, одометрия, EKF, LiDAR, SLAM |
| 18 с   | Nav2-стек (локализация, планирование)          |
| 60 с   | Explore Lite (автономное исследование)         |

SLAM должен инициализироваться и начать строить карту до того, как Nav2 попытается локализироваться.

## Поток данных

1. RPLiDAR публикует `/scan` (LaserScan, ~10 Hz).
2. Семь ультразвуковых сенсоров публикуют отдельные Range-сообщения.
3. Нода `range_to_laserscan` объединяет 7 Range в единый LaserScan `/ultrasonic/ranges`.
4. Оба LaserScan поступают в costmap-слои Nav2.
5. SLAM Toolbox строит карту `/map` на основе данных лидара.
6. Explore Lite находит фронтиры на карте и отправляет цели в Nav2.
7. Nav2 планирует путь и выдаёт команды `/cmd_vel` через twist_mux.

## Конвертер ultrasonic (range_to_laserscan)

**Файл:** `src/verter_admin/distance_sensors/range_to_laserscan.py`

### Входные топики

| Топик                                      | Тип   |
|--------------------------------------------|-------|
| `/verter/distance_sensors/front_center`     | Range |
| `/verter/distance_sensors/front_left_inner` | Range |
| `/verter/distance_sensors/front_left_outer` | Range |
| `/verter/distance_sensors/front_right_inner`| Range |
| `/verter/distance_sensors/front_right_outer`| Range |
| `/verter/distance_sensors/left`             | Range |
| `/verter/distance_sensors/right`            | Range |

### Выход

| Топик                | Тип       |
|----------------------|-----------|
| `/ultrasonic/ranges` | LaserScan |

### Параметры выходного LaserScan

| Параметр       | Значение            |
|----------------|---------------------|
| FOV            | 180 (-90..+90)      |
| Разрешение     | ~1 (181 луч)        |
| Частота        | 10 Hz               |
| Диапазон       | 0.02 -- 4.0 м       |

### Расстановка сенсоров

| Сенсор              | Угол   | Позиция в URDF      |
|---------------------|--------|---------------------|
| front_center        | 0      | (0.28, 0, 0.048)    |
| front_left_inner    | +5.7   | (0.277, 0.074, 0.048) |
| front_left_outer    | +13.2  | (0.262, 0.152, 0.048) |
| front_right_inner   | -5.7   | (0.277, -0.074, 0.048) |
| front_right_outer   | -13.2  | (0.262, -0.152, 0.048) |
| left                | +76    | (0.18, 0.28, 0.048) |
| right               | -76    | (0.18, -0.28, 0.048) |

## Costmap-слои Nav2

### Lidar Layer (VoxelLayer)

```yaml
lidar_scan:
  sensor_frame: lidar_link
  data_type: LaserScan
  topic: /scan
  min_obstacle_height: 0.0
  max_obstacle_height: 2.0
  obstacle_max_range: 5.0
  raytrace_max_range: 5.5
  marking: true
  clearing: true
```

### Ultrasonic Layer (VoxelLayer)

```yaml
ultrasonic:
  sensor_frame: base_link
  data_type: LaserScan
  topic: /ultrasonic/ranges
  min_obstacle_height: 0.0
  max_obstacle_height: 0.8
  obstacle_max_range: 1.0
  marking: true
  clearing: false   # Только маркируем, не чистим
```

### Local / Global Costmap

| Параметр             | Local     | Global            |
|----------------------|-----------|-------------------|
| Размер               | 3x3 м     | 20x20 м (авто)    |
| Разрешение           | 5 см      | 5 см              |
| Частота обновления   | 5 Hz      | 1 Hz              |
| track_unknown_space  | --        | true (критично для exploration) |

## SLAM Toolbox

**Конфигурация:** `config/slam/slam_toolbox_params.yaml`

| Параметр                           | Значение     | Описание                           |
|------------------------------------|--------------|------------------------------------|
| `mode`                             | mapping      | Асинхронное картирование           |
| `solver_plugin`                    | CeresSolver  | Оптимизатор pose graph             |
| `resolution`                       | 0.05 м       | 5 см на ячейку                     |
| `minimum_range`                    | 0.2 м        | Совпадает с laser_filter           |
| `max_laser_range`                  | 5.5 м        | Ниже предела фильтра (6.0 м)       |
| `minimum_travel_distance`          | 0.05 м       | Мин. расстояние между сканами      |
| `minimum_travel_heading`           | 0.05 рад     | Мин. поворот между сканами         |
| `minimum_score`                    | 0.5          | Отклонять слабые scan-match        |
| `link_match_minimum_response_fine` | 0.1          | Порог качества пар сканов          |
| `do_loop_closure`                  | true         | Закрытие цикла                     |
| `loop_match_minimum_response_fine` | 0.45         | Порог loop closure                 |
| `scan_buffer_size`                 | 10           | Буфер сканов для linking           |

Рекомендуемая скорость при картировании: не более 0.15 м/с. При 10 Hz лидара робот проходит ~1.5 см за скан, что обеспечивает качество scan matching.

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

## Высота лидара: обоснование 80 см

### Что видит лидар на разных высотах

| Высота  | Что видит                       | Оценка        |
|---------|---------------------------------|---------------|
| 200 см  | Головы, верхушки; пропускает столы, детей | Плохо   |
| 150 см  | Плечи, высокие столы; может пропустить низкие   | Допустимо |
| **80 см** | Туловища людей, столы, тележки, стулья | **Оптимально** |
| 40 см   | Ноги, ножки мебели               | Допустимо    |
| 5 см    | Пороги, кабели, бордюры (ультразвук) | Комплементарно |

### Почему 80 см

1. Покрывает 90 %+ типичных препятствий в больницах и офисных зданиях.
2. Надёжная детекция людей -- туловище на высоте 80-100 см всегда видно.
3. Комплементарность с ультразвуком -- датчики на 5 см покрывают низкие объекты.
4. Чистые карты SLAM -- минимум шума от мелких деталей на полу.
5. Соответствует индустриальным стандартам мобильных роботов.

### Двухуровневая защита

```
[ЛИДАР 80 см]
  Покрытие:  60--200 см
  Дальность: 0.15--12 м
  FOV:       180 спереди
  Роль:      Основная навигация, SLAM

[7x HC-SR04 ~5 см]
  Покрытие:  0--40 см
  Дальность: 0.02--4 м (точно до 1 м)
  FOV:       ~150 спереди
  Роль:      Safety layer, низкие объекты
```

Комбинация обеспечивает полное покрытие по высоте от 0 до 200 см. LiDAR -- дальняя точная детекция, ультразвук -- ближняя зона безопасности.

### Риски

- **Маленькие дети (рост <80 см)** -- лидар может не увидеть. Компенсация: ультразвук на близком расстоянии + низкая скорость 0.3 м/с.
- **Прозрачное стекло** -- лидар плохо видит. Ультразвук надёжнее; стекло обычно в рамах.
- **Столы с тонкими ножками** -- лидар видит столешницу, ножки покрывает ультразвуковой слой и inflation radius.

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
| Nav2 Controller | `desired_linear_vel`   | 0.3      |

### Осторожное исследование

| Компонент      | Параметр               | Значение |
|----------------|------------------------|----------|
| Nav2 Costmap   | `inflation_radius`     | 0.70     |
| Nav2 Costmap   | `robot_radius`         | 0.45     |
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
| `stop_distance`      | `0.15` м             | Расстояние экстренной остановки|
| `resume_distance`    | `0.20` м             | Расстояние для возобновления   |

## Диагностика

```bash
# Данные лидара
ros2 topic echo /scan --once
ros2 topic hz /scan

# Данные ультразвука
ros2 topic echo /ultrasonic/ranges --once
ros2 topic hz /ultrasonic/ranges

# Карта
ros2 topic echo /map --once
ros2 topic hz /map

# Ключевые ноды
ros2 node list
# Ожидаемые: /range_to_laserscan, /slam_toolbox, /explore_node,
#            /controller_server, /planner_server, /bt_navigator
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
- **LaserScan** (`/ultrasonic/ranges`) -- данные ультразвука.
- **Local Costmap** -- динамические препятствия вокруг робота.
- **Global Costmap** -- статическая карта с препятствиями.
- **MarkerArray** -- зелёные маркеры frontiers.
- **Path** -- запланированный путь.
- **RobotFootprint** -- контур робота.
