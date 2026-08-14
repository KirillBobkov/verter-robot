# SLAM / Локализация — проблемы и решения

При въезде в узкие места и разворотах робот теряет ориентацию, карта показывает двойные стены, лидар не совпадает с картой.

---

## Статус исправлений

| # | Проблема | Статус |
|---|----------|--------|
| 0 | `odom0_differential: false` — EKF берёт из odom0 только скорости (vx, vy, vyaw), позиция отключена | ✅ Актуально → `false` (соответствует коду) |
| 1 | Laser filter обрезает до ±0.7 рад (~80°) | 🔲 Рекомендуется расширить до ±150° |
| 2 | Wheel slip при вращении на месте | 🔲 Снизить скорость, использовать arc-повороты |
| 3 | `link_match_minimum_response_fine: 0.15` — мягкий порог | 🔲 Поднять до 0.45 |
| 4 | `angular_scale: 1.0` в cmd_vel fallback одометрии | ✅ Уже 1.0 (масштабирования нет) |
| 5 | URDF wheel_base (0.3642) = calibrated (0.3642), но y=±0.178 даёт 0.356 | 🔲 Привести origin к ±0.1821 |
| 6 | Низкая скорость авто-картирования (`desired_linear_vel: 0.04`) | 🔲 Проверить достаточность для mapping |
| 7 | Nav2 провоцирует развороты в тесных местах | 🔲 Настроить RegulatedPurePursuit на forward-only |
| 8 | Агрессивные угловые ограничения в controller | 🔲 Снизить max_angular_accel (сейчас 1.0) |
| 9 | AMCL: `set_initial_pose` не задан (по умолчанию false) | 🔲 Требуется указание через RViz |
| 10 | Odom covariance зависит только от encoder_fresh, не от режима движения | 🔲 Сделать адаптивную covariance |
| 11 | IMU и odom в разных временных базах | 🔲 Синхронизировать источники |
| 12 | IMU — только angular velocity, нет абсолютного yaw | Известное ограничение |
| 13 | Ультразвук: proximity_safety ждёт LaserScan на /ultrasonic/ranges, но нет публикатора | 🔲 Сверить naming-конвенции |

---

## 1. Laser filter: только передние ±0.7 рад (~80°)

**Файл:** `config/laser_filters/laser_filter.yaml`

Лидар видит только узкий передний сектор (±0.7 рад ≈ ±40°, всего ~80°). В узком коридоре при повороте сцена меняется кардинально от скана к скану, стены создают симметричную геометрию — matcher может спутать стороны.

Текущие значения в конфиге:

```yaml
lower_angle: -0.7
upper_angle:  0.7
```

### Рекомендация

Расширить фильтр до ±150° (быстрый тест):

```yaml
lower_angle: -2.618  # -150°
upper_angle:  2.618  # +150°
```

Или оставить полный 360° и вырезать только секторы корпуса точечно.

### Диагностика

```bash
# В RViz добавить /scan_raw рядом с /scan — видно что срезается
ros2 topic echo /scan_raw --once
```

---

## 2. Wheel slip при вращении на месте

При вращении на месте колёса несут полную нагрузку в противоположных направлениях. На паркете/плитке колёса скользят — энкодеры фиксируют вращение колеса, а не пятно контакта.

### Диагностика

```bash
# 1. Маркер на полу, ровно на 360°
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.5}}" --times 1
# 2. Сравнить реальный угол с /odom theta
ros2 topic echo /odom --once | grep -A3 orientation
```

### Рекомендация

В `nav2_navigation_params.yaml` (контроллер — `RegulatedPurePursuitController`, не DWB):

```yaml
# Снизить скорость вращения при rotate_to_heading
rotate_to_heading_angular_vel: 0.15  # текущее значение

# Запретить вращение на месте — использовать arc-повороты
use_rotate_to_heading: false          # отключит spin на месте
min_approach_linear_velocity: 0.02    # текущее значение
```

---

## 3. `link_match_minimum_response_fine: 0.15`

**Файл:** `config/slam/slam_toolbox_params.yaml`

SLAM принимает scan match как валидный при score ≥ 0.15 — очень мягкий порог. Типичные рабочие значения: 0.45–0.55.

### Рекомендация

```yaml
link_match_minimum_response_fine: 0.45
```

Риск: SLAM реже добавляет ноды — карта строится медленнее в малоинформативных коридорах.

---

## 4. `angular_scale: 1.0` в cmd_vel fallback

**Файл:** `src/verter_admin/control/domain/odometry_policy.py`

При fallback на cmd_vel (если энкодеры не приходят >1 сек, `encoder_timeout: 1.0`) угловая скорость используется как есть — масштабирования нет (`angular_scale: 1.0`, `linear_scale: 1.0`). Ранее предполагалось значение 2.0, но в коде оно уже равно 1.0.

### Текущее состояние

```python
angular_scale: float = 1.0  # масштабирования нет
linear_scale: float = 1.0
```

---

## 5. URDF wheel_base: свойство vs origin

`wheels.xacro` определяет `wheel_base = 0.3642` (совпадает с `odometry_policy.py`), но origin-координаты колёс `y=±0.178` дают расстояние 0.356 m — рассогласование между свойством и фактической геометрией в URDF.

### Рекомендация

Привести `origin xyz` в `wheels.xacro` к `y=±0.1821`, чтобы геометрия совпадала со значением `wheel_base = 0.3642`.

---

## 6. Скорость в auto-mapping

**Файлы:** `config/slam/slam_toolbox_params.yaml`, `config/nav2/nav2_navigation_params.yaml`

Текущее значение `desired_linear_vel: 0.04` (в `nav2_navigation_params.yaml`, контроллер `RegulatedPurePursuitController`). Для авто-картирования с RPLidar A1M8 это может быть слишком медленно — скан обновляется ~10 Гц, робот едва движется.

### Рекомендация

Для авто-картирования подобрать скорость в диапазоне 0.10–0.15 м/с:

```yaml
desired_linear_vel: 0.12  # текущее 0.04
```

---

## 7. Nav2 провоцирует развороты в тесных местах

`use_rotate_to_heading: true` + `allow_reversing: false` + recovery behavior `spin` → робот уходит в вращение на месте, колёса проскальзывают, odom и scan matching расходятся.

### Диагностика

```bash
# В RViz — local costmap и траектория контроллера
# Если перед потерей локализации робот регулярно уходит в spin — причина здесь
```

---

## 8. Агрессивные угловые ограничения

Текущие значения: `max_rotational_vel: 0.5`, `rotational_acc_lim: 1.0` (behavior_server), `max_angular_accel: 1.0` (controller). Высокие угловые ускорения усиливают проскальзывание. Ошибка yaw растёт по цепочке: wheel slip → плохой odom → плохой map→odom → лидар не совпадает с картой.

---

## 9. AMCL: начальная поза не задана в конфиге

В `nav2_navigation_params.yaml` параметры `set_initial_pose` и `initial_pose.*` отсутствуют — AMCL использует значения по умолчанию (`set_initial_pose: false`). Начальная поза должна задаваться через RViz (2D Pose Estimate), иначе локализация стартует с (0, 0, 0). В симметричных коридорах AMCL долго не сходится без явного указания.

### Диагностика

```bash
ros2 param get /amcl set_initial_pose   # ожидается false (по умолчанию)
# initial_pose.* параметры отсутствуют в конфиге — поза задаётся через RViz
```

---

## 10. Odom covariance зависит только от encoder_fresh

Covariance переключается между двумя значениями: `_SMALL = 1e-4` (энкодеры свежие) и `_LARGE = 0.5` (энкодеры устарели >1 сек). Covariance не зависит от режима движения (прямой ход / разворот / пробуксовка) — EKF одинаково доверяет odom и при вращении на месте.

### Диагностика

```bash
ros2 topic echo /odom --once
# covariance одинаковая на прямом ходе и во время разворота
```

---

## 11. IMU и odom — разное время

IMU публикует stamp из `rmw_uros_epoch_nanos()` (время синхронизированной сессии micro-ROS на ESP32), odometry_node ставит время по часам хоста (`get_clock().now()`). При рассинхронизации часов на быстрых поворотах gyro и wheel odom описывают разные моменты времени. ESP32 периодически вызывает `rmw_uros_sync_session()` (каждые 60 с) для синхронизации.

### Диагностика

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /odom --once
# Сравнить header.stamp
```

---

## 12. IMU — только angular velocity + acceleration

IMU публикует `angular_velocity` (x, y, z) и `linear_acceleration` (x, y, z). Поля `orientation` и `orientation_covariance` не заполняются (остаются нулевыми по умолчанию) — нет независимого абсолютного yaw-источника.

### Диагностика

```bash
ros2 topic echo /imu/data --once
# orientation: x=0, y=0, z=0, w=0 — ориентация не заполняется
# orientation_covariance: все нули (не -1)
```

---

## 13. Ультразвук — тракт топиков не сходится

`proximity_safety_node` ожидает `LaserScan` на `/ultrasonic/ranges` (параметр `ultrasonic_topic`), но ни один узел не публикует `LaserScan` на этот топик. Цепочка данных:

- ESP32 (`esp32_sensors_refab`) публикует `Float32MultiArray` на `/ultrasonics`
- `range_converter_node` подписывается на `/ultrasonic/distances` и публикует 7 отдельных `Range` на `/verter/distance_sensors/{name}`

Ни одна из цепочек не создаёт `LaserScan` на `/ultrasonic/ranges` — safety-узел не получает ближний обзор.

### Диагностика

```bash
ros2 topic list | grep ultrasonic
ros2 topic info /ultrasonic/ranges
```

---

## Корневые причины (по приоритету)

1. Несогласованная кинематика: `wheel_base` (свойство 0.3642 vs origin 0.356), поведение при slip
2. Бедная геометрия лидара: фильтр ±0.7 рад (~80°)
3. Угловая динамика при картографировании (`max_angular_accel: 1.0`, `rotational_acc_lim: 1.0`)
4. EKF: odom0 только по скоростям; imu0:/imu/data раскомментирован (вопреки заголовку «IMU DISABLED»), bias гироскопа ~-0.001 рад/с по z
5. Слабая наблюдаемость в узких местах + агрессивная логика Nav2 (spin, rotate_to_heading)
