# SLAM / Локализация — проблемы и решения

При въезде в узкие места и разворотах робот теряет ориентацию, карта показывает двойные стены, лидар не совпадает с картой.

---

## Статус исправлений

| # | Проблема | Статус |
|---|----------|--------|
| 0 | `odom0_differential: false` — EKF доверяет абсолютному yaw | ✅ Исправлено → `true` |
| 1 | Laser filter обрезает до 180° | 🔲 Рекомендуется расширить до ±150° |
| 2 | Wheel slip при вращении на месте | 🔲 Снизить скорость, использовать arc-повороты |
| 3 | `link_match_minimum_response_fine: 0.1` — слишком мягкий порог | 🔲 Поднять до 0.45 |
| 4 | `angular_scale: 2.0` в cmd_vel fallback одометрии | 🔲 Убрать масштабирование |
| 5 | URDF wheel_base (0.356) ≠ calibrated (0.374) | 🔲 Измерить реальное расстояние |
| 6 | Высокая скорость auto-mapping — motion distortion | 🔲 Снизить до 0.10-0.15 м/с |
| 7 | Nav2 провоцирует развороты в тесных местах | 🔲 Настроить DWB на forward-only |
| 8 | Агрессивные угловые ограничения в controller | 🔲 Снизить max_angular_accel |
| 9 | AMCL стартует с фиксированной позой (0, 0, 0) | 🔲 Требуется указание через RViz |
| 10 | Odom covariance не зависит от режима движения | 🔲 Сделать адаптивную covariance |
| 11 | IMU и odom в разных временных базах | 🔲 Синхронизировать источники |
| 12 | IMU — только angular velocity, нет абсолютного yaw | Известное ограничение |
| 13 | Ультразвук в costmap, но тракт топиков не сходится | 🔲 Сверить naming-конвенции |

---

## 1. Laser filter: только передние 180°

**Файл:** `config/laser_filters/laser_filter.yaml`

Лидар видит только переднюю полусферу. В узком коридоре при повороте сцена меняется кардинально от скана к скану, стены создают симметричную геометрию — matcher может спутать стороны.

### Рекомендация

Расширить фильтр до ±150° (быстрый тест):

```yaml
angle_min: -2.618  # -150°
angle_max:  2.618  # +150°
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

В `nav2_navigation_params.yaml`:

```yaml
# Снизить скорость вращения
max_vel_theta: 0.4

# Или запретить вращение на месте — использовать arc-повороты
min_vel_x: 0.05  # небольшой forward drift при повороте
```

---

## 3. `link_match_minimum_response_fine: 0.1`

**Файл:** `config/slam/slam_toolbox_params.yaml`

SLAM принимает scan match как валидный при score ≥ 0.1 — практически любое совпадение. Типичные рабочие значения: 0.45–0.55.

### Рекомендация

```yaml
link_match_minimum_response_fine: 0.45
```

Риск: SLAM реже добавляет ноды — карта строится медленнее в малоинформативных коридорах.

---

## 4. `angular_scale: 2.0` в cmd_vel fallback

**Файл:** `src/verter_admin/control/domain/odometry_policy.py`

При fallback на cmd_vel (если энкодеры не приходят >1 сек) угловая скорость умножается на 2. Кратковременный скачок yaw ×2 во время разворота.

### Рекомендация

```python
angular_scale: float = 1.0  # убрать масштабирование
```

---

## 5. URDF wheel_base ≠ calibrated

URDF: `y=±0.178` → wheel_base = **0.356m**, calibrated: **0.374m**. Модель в RViz неточная.

### Рекомендация

Измерить расстояние между центрами точек контакта колёс с полом. Обновить URDF и привести к единому значению.

---

## 6. Слишком высокая скорость в auto-mapping

**Файлы:** `config/slam/slam_toolbox_params.yaml`, `config/nav2/nav2_navigation_params.yaml`

При `desired_linear_vel: 0.25` скан "растягивается" по времени — стены двоются и расходятся параллельно.

### Рекомендация

Для RPLidar A1M8 безопасный диапазон:

```yaml
desired_linear_vel: 0.12  # было 0.25
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

Высокие `max_rotational_vel`, `rotational_acc_lim`, `max_angular_accel` усиливают проскальзывание. Ошибка yaw растёт по цепочке: wheel slip → плохой odom → плохой map→odom → лидар не совпадает с картой.

---

## 9. AMCL стартует с фиксированной позой (0, 0, 0)

`set_initial_pose: true` + фиксированная поза. Если робот стартует не из этой точки — локализация получает ложную гипотезу. В симметричных коридорах AMCL долго не сходится.

### Диагностика

```bash
ros2 param get /amcl set_initial_pose
ros2 param get /amcl initial_pose.x
ros2 param get /amcl initial_pose.y
ros2 param get /amcl initial_pose.yaw
```

---

## 10. Odom covariance фиксирована

Фиксированные covariance — EKF слишком доверяет odom при вращении на месте, пробуксовке или fallback-режиме.

### Диагностика

```bash
ros2 topic echo /odom --once
# covariance одинаковая на прямом ходе и во время разворота
```

---

## 11. IMU и odom — разное время

IMU публикует stamp из `millis()` ESP32, odometry_node ставит время по хосту. На быстрых поворотах gyro и wheel odom описывают разные моменты времени.

### Диагностика

```bash
ros2 topic echo /imu/data --once
ros2 topic echo /odom --once
# Сравнить header.stamp
```

---

## 12. IMU — только angular velocity

IMU публикует только гироскопический `angular_velocity.z`. Ориентация отсутствует — нет независимого абсолютного yaw-источника.

### Диагностика

```bash
ros2 topic echo /imu/data --once
# orientation_covariance: -1 — абсолютная ориентация не используется
```

---

## 13. Ультразвук в costmap — тракт топиков не сходится

Конфиги costmap ждут `LaserScan` на `/ultrasonic/ranges`, но сенсоры публикуют `Float32MultiArray` и набор `Range`. Робот не получает ближний обзор, на который рассчитывает конфигурация.

### Диагностика

```bash
ros2 topic list | grep ultrasonic
ros2 topic info /ultrasonic/ranges
```

---

## Корневые причины (по приоритету)

1. Несогласованная кинематика: `wheel_base`, scale, поведение при slip
2. Бедная геометрия лидара: front-only фильтр
3. Высокая скорость и угловая динамика при картографировании
4. Слишком сильное доверие к wheel odom в EKF
5. Слабая наблюдаемость в узких местах + агрессивная логика Nav2
