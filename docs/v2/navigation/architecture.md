# Навигационная архитектура

Навигационная система Verter Robot основана на Nav2 (ROS 2 Navigation Framework) и следует принципам Safety-Driven Hexagonal Robotics (SDHR).

## Слойная архитектура (SDHR)

```
+-----------------------------------------------------+
|              L3: Mission/HMI                        |
|          (голосовые команды, AI, веб-UI)            |
+-----------------------------------------------------+
                         |
                         v
+-----------------------------------------------------+
|             L2: Planning/Behavior                   |
|    Nav2 (AMCL + DWB + NavFn + поведение)            |
+-----------------------------------------------------+
                         |
                         v
+-----------------------------------------------------+
|           L1: Control/Localization                  |
|  (odometry_node, EKF, twist_mux, safety_policies)   |
+-----------------------------------------------------+
                         |
                         v
+-----------------------------------------------------+
|            L0: Safety/Actuation                     |
|        ESP32 (micro-ROS, watchdog, PID)             |
+-----------------------------------------------------+
```

### Обязательности по слоям

- **L0** MUST управлять остановкой и таймаутом команд -- работать без L2/L3.
- **L1** MUST предоставлять детерминированный контроль и оценку состояния.
- **L2** SHOULD быть заменяемой без изменения контрактов L0/L1.
- **L3** MUST NOT находиться в прямом пути управления эффекторами.

Перекрестные зависимости: L2 и L3 общаются с L0/L1 только через контракты L1. L0/MCU не зависит от L2/L3.

## Цепочка команд

```
bt_navigator / waypoint_follower
    -> controller_server (DWB)
        -> velocity_smoother
            -> /nav2/cmd_vel
                -> twist_mux (арбитраж приоритетов)
                    -> /cmd_vel
                        -> ESP32 (моторы)
```

## Twist Mux -- приоритеты

| Приоритет | Источник             | Описание                         |
|-----------|----------------------|----------------------------------|
| 200       | Safety               | Экстренная остановка             |
| 100       | Teleop keyboard      | Ручное управление с клавиатуры   |
| 90        | Teleop joystick      | Ручное управление с джойстика    |
| 10        | Nav2                 | Автономная навигация             |

Safety (200) останавливает робота в любой момент. Nav2 (10) -- низший приоритет.

## Safety-автомат состояний

| Состояние  | Описание                    | Действие                      |
|------------|-----------------------------|-------------------------------|
| READY      | Все датчики в норме         | Движение разрешено            |
| DEGRADED   | Данные датчиков устарели    | Движение ограничено           |
| FAULT      | Препятствие в зоне опасности | Экстренная остановка         |

Переходы:

```
STOP      -- если nearest_distance <= 0.15 м
RESUME    -- если nearest_distance >= 0.20 м
DEGRADED  -- если sensors_fresh == False и allow_degraded_motion == False
```

## Таймауты

| Параметр                      | Значение   | Источник                    |
|-------------------------------|------------|-----------------------------|
| Command timeout               | 0.5 сек    | Все источники cmd_vel       |
| Sensor timeout                | 0.8 сек    | Ультразвуковые датчики      |
| ESP32 watchdog                | 500 мс     | Safe stop на MCU            |
| EKF transform_timeout         | 0.0 сек    | Не публиковать TF без данных|
| AMCL transform_tolerance      | 0.5 сек    | Задержка TF map->odom       |
| Controller transform_tolerance| 0.2 сек    | Задержка TF в DWB           |
| Velocity smoother timeout     | 1.0 сек    | Отсутствие новых команд     |

## QoS-профили по топикам

### Sensor streams

| Топик                     | Тип            | QoS            | Частота  |
|---------------------------|----------------|----------------|----------|
| `/scan`                   | LaserScan      | Best Effort    | ~10 Hz   |
| `/ultrasonic/ranges`      | LaserScan      | Best Effort    | 10 Hz    |
| `/wheel_encoders`         | Encoders       | Reliable       | 10 Hz    |
| `/odom`                   | Odometry       | Reliable       | 50 Hz    |
| `/odometry/filtered`      | Odometry       | Reliable       | 50 Hz    |

### Command streams

| Топик                        | Тип    | QoS      | Описание                  |
|------------------------------|--------|----------|---------------------------|
| `/nav2/cmd_vel`              | Twist  | Reliable | Выход Nav2                |
| `/safety/cmd_vel`            | Twist  | Reliable | Приоритетный оверрайд     |
| `/teleop_keyboard/cmd_vel`   | Twist  | Reliable | Ручное управление         |
| `/cmd_vel`                   | Twist  | Reliable | Финальный (ESP32)         |

### Localization

| Топик                       | Тип            | QoS            | Описание                      |
|-----------------------------|----------------|----------------|-------------------------------|
| `/map`                      | OccupancyGrid  | Transient Local | Карта SLAM / map server      |
| `/initialpose`              | PoseWithCovariance | Reliable   | Оценка начальной позиции      |
| `/goal_pose`                | PoseStamped    | Reliable       | Цель навигации                |

## Координатные системы и TF-дерево

Следует REP-105.

```
map                          -- глобальная карта (SLAM Toolbox)
 └─ odom                     -- одометрия (EKF)
     └─ base_footprint (z=0) -- уровень пола
         └─ base_link (z=0.1 м) -- корпус
             ├─ wheel_left
             ├─ wheel_right
             ├─ imu_link            (z=0.05 м, 15 см над полом)
             ├─ sensor_front_center (z=0.048 м, 14.8 см)
             └─ lidar_link          (z=0.70 м, 80 см над полом)
```

### Владелец преобразований

| Преобразование            | Публикует               |
|---------------------------|-------------------------|
| `map -> odom`             | slam_toolbox            |
| `odom -> base_footprint`  | ekf_filter_node         |
| `base_footprint -> base_link` | robot_state_publisher |
| `base_link -> *`          | robot_state_publisher   |

### Правила frame_id

- Пространственные сообщения MUST содержать корректный `frame_id`.
- `/scan`: `frame_id = lidar_link`.
- `/ultrasonic/ranges`: `frame_id = base_link`.
- `/odom`, `/odometry/filtered`: `header.frame_id = odom`, `child_frame_id = base_footprint`.
- Цели навигации: `header.frame_id = map`.

## Железо

### Лидар: RPLiDAR A1M8

- Подключение: USB -> `/dev/rplidar`
- Скорость: 115200 baud, режим Express (~10 Hz)
- Угол обзора: 360 (фильтруется до передних 180)
- Диапазон: 0.20--6.0 м
- Позиция: 80 см от пола, развёрнут на 180

### ESP32 шасси

- Подключение: USB -> `/dev/esp32_chassis`
- Скорость: 921600 baud
- Watchdog: 500 мс (без команд -- безопасная остановка)
- Энкодеры: AS5600 магнитные по I2C

### Моторы

- Дифференциальный привод
- Максимальная скорость: 0.3 м/с linear, 0.5 рад/с angular
- Колёсная база: 0.374 м, радиус колеса: 0.1 м

## Параметры Nav2

### DWB Local Planner

| Параметр                 | Значение  |
|--------------------------|-----------|
| `max_vel_x`              | 0.3 м/с   |
| `max_vel_theta`          | 0.5 рад/с |
| `xy_goal_tolerance`      | 0.15 м    |
| `yaw_goal_tolerance`     | 0.1 рад   |
| `sim_time`               | 1.5 сек   |
| `controller_frequency`   | 5.0 Hz    |

### Costmaps

| Параметр                 | Значение  |
|--------------------------|-----------|
| `resolution`             | 0.05 м    |
| `robot_radius`           | 0.25 м    |
| `inflation_radius`       | 0.40 м    |
| `cost_scaling_factor`    | 3.0       |

### AMCL

| Параметр                   | Значение                |
|----------------------------|-------------------------|
| `max_particles`            | 2000                    |
| `min_particles`            | 500                     |
| `laser_max_range`          | 5.5 м                   |
| `update_min_d`             | 0.25 м                  |
| `update_min_a`             | 0.2 рад                 |
| `robot_model_type`         | DifferentialMotionModel |

## Ключевые файлы

| Файл | Описание |
|------|----------|
| `launch/nav2_navigation.launch.py` | Навигация по готовой карте |
| `launch/mapping.launch.py` | Создание карты (SLAM) + ручное управление |
| `launch/autonomous_mapping_real.launch.py` | Автономное создание карты |
| `config/nav2/nav2_navigation_params.yaml` | Nav2 (AMCL, DWB, costmaps) |
| `config/slam/slam_toolbox_params.yaml` | SLAM |
| `config/robot_localization/ekf.yaml` | EKF фильтр |
| `config/laser_filters/laser_filter.yaml` | Фильтр лидара |
| `urdf/verter_robot_minimal.urdf` | Модель робота |

## Известные ограничения

1. **IMU отключён** -- BMX055 гироскоп имеет дрейф (~3.4 град/мин), временно отключён в EKF.
2. **Задний обзор** -- лидар видит только передние 180.
3. **Максимальная скорость** -- 0.3 м/с (ограничено Nav2 и ESP32).
4. **Разрешение карты** -- 5 см/ячейка (не фиксирует мелкие препятствия).
