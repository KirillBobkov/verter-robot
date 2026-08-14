# Одометрия

## Что такое одометрия

Одометрия вычисляет положение робота на основе того, как он двигался. Робот знает свою начальную позицию и отслеживает скорость и направление движения — интегрируя эти данные, он определяет текущие координаты.

**Ограничение:** ошибки накапливаются со временем (колёса проскальзывают, поверхность меняет сцепление). Для точной навигации одометрия комбинируется с локализацией (AMCL + карта + лидар).

---

## Архитектура одометрии

Одометрия в Verter Robot **энкодерная** — положение вычисляется по показаниям
колёсных энкодеров, а не по командам скорости. Узел `odometry_node`
(`control/adapters/ros/odometry_node.py`) подписывается на `/wheel_encoders`
и интегрирует дельты шагов энкодеров.

```
ВХОДНЫЕ ДАННЫЕ:
┌──────────────────────┐
│  /wheel_encoders     │ ──► Шаги левого/правого колеса (Int64MultiArray[2])
│  (Int64MultiArray)   │     BEST_EFFORT, 50 Гц от ESP32
└──────────────────────┘

ОБРАБОТКА (odometry_node → ComputeOdometry):
┌────────────────────────────────────────────────────────┐
│  1. Получить шаги энкодеров (left, right)              │
│  2. Вычислить дельты шагов с прошлого сообщения        │
│  3. Перевести дельты в расстояние и поворот            │
│     (meters_per_step, wheel_base)                      │
│  4. ИНТЕГРИРОВАТЬ позу (трапецеидально, по середине)   │
│  5. Опубликовать результат (50 Гц, таймер 0.02 с)      │
└────────────────────────────────────────────────────────┘

ВЫХОДНЫЕ ДАННЫЕ:
┌──────────────────┐
│   /odom          │ ──► Позиция (x, y, theta) + скорость
│  (Odometry)      │     frame_id=odom, child_frame_id=base_footprint
└──────────────────┘

┌──────────────────┐
│   /tf            │ ──► Трансформация odom → base_footprint
│ (TransformStamped)    (опционально, см. publish_tf ниже)
└──────────────────┘
```

### Полный путь одометрии (production)

В `main.launch.py` одометрия идёт через ZLAC-драйвер и EKF, `odometry_node`
не запускается:

```
ZLAC8015D (Modbus RTU, регистры позиций моторов)
       │
       ▼
chassis_zlac_node ──► /odom_raw (энкодерная одометрия по позициям моторов)
                          │
                          ▼
              EKF (robot_localization, ekf.yaml)
                  │
                  ├──► /odometry/filtered
                  └──► TF odom → base_footprint (publish_tf=true)
```

`/odom_raw` публикует `chassis_zlac_node` (`chassis/chassis_zlac_node.py`) —
он читает позиции моторов ZLAC8015D по Modbus RTU и интегрирует дельты
энкодеров (тот же трапецеидальный метод, `child_frame_id=base_footprint`).
EKF (`ekf_node` из пакета `robot_localization`, конфиг
`config/robot_localization/ekf.yaml`) сливает `/odom_raw` и (когда включён)
`/imu/data`, на выходе даёт `/odometry/filtered` и владеет TF
`odom → base_footprint` (`publish_tf: true` в ekf.yaml).

`/wheel_encoders` — отдельный поток от ESP32 (micro-ROS), его потребляет
именно `odometry_node` (вариант mapping/autonomous_mapping), а не
`chassis_zlac_node`.

В `mapping.launch.py` и `autonomous_mapping_real.launch.py` `odometry_node`
запускается с `publish_tf: False` — TF отдаётся EKF, чтобы не было
конкуренции за `odom → base_footprint`.

---

## Интегрирование энкодеров

### Формулы

Интегрирование выполняется в `control/application/compute_odometry.py`
(доменный слой — `control/domain/odometry_policy.py`). Шаги энкодеров
переводятся в пройденное расстояние и изменение курса, затем поза
интегрируется трапецеидально (по середине угла):

```python
# meters_per_step — единственная кинематическая константа одометрии
meters_per_step = wheel_circumference / (encoder_resolution * gear_ratio)
                          # = π×0.196 / (4096×1.0) ≈ 1.503e-4 м

# Пройденное расстояние каждым колесом
dist_left  = delta_left_steps  * meters_per_step
dist_right = delta_right_steps * meters_per_step

# Среднее перемещение и поворот
delta_distance = (dist_left + dist_right) / 2.0
delta_theta    = (dist_right - dist_left) / wheel_base   # wheel_base = 0.3642 м

# Трапецеидальная интеграция (по середине угла — точнее, чем прямой Эйлер)
avg_theta = old_theta + delta_theta / 2.0
new_x     = old_x + delta_distance * cos(avg_theta)
new_y     = old_y + delta_distance * sin(avg_theta)
new_theta = normalize_angle(old_theta + delta_theta)
```

### Почему cos и sin

Робот движется в **своей** системе координат (вперёд), но позиция требуется в **глобальной** системе.

- Робот смотрит на север и едет вперёд: X растёт (cos(0°) = 1), Y не меняется (sin(0°) = 0)
- Робот смотрит на восток и едет вперёд: X не меняется (cos(90°) = 0), Y растёт (sin(90°) = 1)

---

## Ключевые типы сообщений

### Twist (geometry_msgs/Twist)

Команды скорости:

```python
msg = Twist()
msg.linear.x = 0.5   # скорость вперёд, м/с
msg.angular.z = 1.0  # поворот, рад/с
```

### Odometry (nav_msgs/Odometry)

Позиция и скорость робота:

```python
odom = Odometry()
odom.pose.pose.position.x = 1.5      # X, метры
odom.pose.pose.position.y = 2.0      # Y, метры
odom.pose.pose.orientation = quat    # кватернион (roll, pitch, yaw)
odom.twist.twist.linear.x = 0.5      # текущая скорость
```

### TF (Transform Framework)

Система координат. Каждая часть робота имеет свой frame. TF описывает связи между ними:

```
map          ──► глобальная карта (появляется при SLAM)
 └─ odom     ──► начальная точка робота
     └─ base_footprint ──► проекция робота на пол
         └─ base_link ──► центр робота
             ├─ lidar_link ──► LiDAR
             └─ imu_link    ──► IMU
```

Публикация трансформации (из `odometry_node.py`):

```python
transform = TransformStamped()
transform.header.frame_id = 'odom'            # родитель
transform.child_frame_id = 'base_footprint'   # потомок
transform.transform.translation.x = 1.5       # сдвиг
transform.transform.rotation = quaternion     # поворот
```

---

## Запуск и проверка

```bash
# Запуск через launch (odometry_node запускается в mapping/autonomous_mapping;
# в main.launch.py одометрия идёт через chassis_zlac_node → EKF)
ros2 launch verter_admin mapping.launch.py

# Проверка что нода работает (энкодерная одометрия)
ros2 topic echo /odom

# Проверка TF (odom → base_footprint владеет EKF)
ros2 run tf2_ros tf2_echo odom base_footprint

# Проверка потока энкодеров от ESP32 (50 Гц)
ros2 topic hz /wheel_encoders

# Отправка тестовой команды скорости (проверка шасси, не одометрии напрямую)
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

---

## Калибровка одометрии

**Проблема:** робот проехал 1 м, одометрия показывает 1.2 м.

**Решение:**

```python
# scale = реальное / одометрия
self.linear_scale = 1.0 / 1.2  # = 0.833
```

Параметры в `control/domain/odometry_policy.py` (`OdometryParameters`):

```python
wheel_circumference = math.pi * 0.196   # 0.6158 м (нагруженный диаметр 196 мм)
gear_ratio          = 1.0               # direct-drive hub motor
encoder_resolution  = 4096              # подтверждено калибровочными прогонами
wheel_base          = 0.3642            # м, откалибровано 2026-06-02
max_linear_velocity = 0.5               # м/с
max_angular_velocity = 1.0              # рад/с
linear_scale = 1.0                      # калибровка линейной скорости
angular_scale = 1.0                     # калибровка угловой скорости
encoder_timeout = 1.0                   # с — после этого vx/vth обнуляются
```

`meters_per_step` (= `wheel_circumference / (encoder_resolution * gear_ratio)`
≈ 1.503e-4 м) — единственная кинематическая константа, которую одометрия
использует напрямую. Калибровка: `scripts/chassis_calibrate.py encoder`
(линейная) и `... wheelbase` (межосевое расстояние).

---

## TF отладка

```bash
# Визуализация дерева TF
ros2 run tf2_tools view_frames
# Создаёт frames.pdf

# Проверка конкретной трансформации
ros2 run tf2_ros tf2_echo odom base_footprint
```

---

## FAQ

**Зачем одометрия если есть LiDAR?** LiDAR говорит «что вокруг», одометрия — «где робот». Для навигации нужно и то и другое.

**Почему одометрия «дрейфует»?** Интегрирование накапливает ошибки. Для точной навигации нужна локализация (AMCL + карта).

**Что такое кватернион?** Представление поворота в 3D (w, x, y, z). Используется вместо углов Эйлера — нет gimbal lock, эффективнее для интерполяции. Стандарт в робототехнике.

**GPS вместо одометрии?** Нет. GPS работает на улице, точность 3-5 м — недостаточно для навигации в помещении.
