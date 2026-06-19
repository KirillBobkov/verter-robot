# План Диагностики Навигации Verter Robot

## Context

Проблема: навигация работает неправильно, карта строится неверно с искажениями (двойные стены, дрейф карты).

Анализ тремя агентами выявил **5 критичных проблем**:

1. **SLAM Scan Matching**: `link_match_minimum_response_fine: 0.1` - СЛИШКОМ НИЗКИЙ (должен быть 0.45-0.5)
2. **Nav2 Safety**: `BaseObstacle.scale: 0.02` - ОПАСНО для навигации (должен быть 5.0-10.0)
3. **Nav2 Inflation**: `inflation_radius: 0.40` - недостаточно (рекомендуется 0.60)
4. **Отключённый IMU**: гироскоп BMX055 имеет drift 3.4°/мин, полностью отключён в EKF
5. **Ультразвук не интегрирован**: 7x HC-SR04 датчиков не используются в costmap

## План Диагностики (5 Фаз)

---

### Phase 1: SLAM Diagnostics - Качество Scan Matching

**Цель**: Проверить качество сопоставления сканов и подтвердить критическую проблему.

**Файлы**:
- `D:\repos\verter-robot\verter_admin\src\verter_admin\config\slam\slam_toolbox_params.yaml`

**Команды диагностики**:
```bash
# Запустить SLAM
ros2 launch verter_admin mapping.launch.py

# Проверить критичные параметры
ros2 param get /slam_toolbox link_match_minimum_response_fine
ros2 param get /slam_toolbox minimum_score
ros2 param get /slam_toolbox loop_match_minimum_response_fine

# Проверить качество сканов
ros2 topic hz /scan
ros2 topic echo /scan --once

# Визуализировать в RViz
rviz2
# Добавить: Map, LaserScan, TF
```

**Ожидаемые результаты**:
- `link_match_minimum_response_fine` = **0.1** (КРИТИЧНО НИЗКИЙ)
- `minimum_score` = 0.5 (нормально)
- `/scan` частота ~10 Hz

**Критерии проблемы**:
- Параметр 0.1 означает, что SLAM принимает сканы с качеством ≥10%
- Это приводит к плохим связям между сканами → накопление ошибок → искажение карты

**Рекомендуемое исправление**:
```yaml
# В slam_toolbox_params.yaml строка 57:
link_match_minimum_response_fine: 0.45  # вместо 0.1
```

---

### Phase 2: TF & Odometry Diagnostics - Дрейф Локализации

**Цель**: Оценить дрейф одометрии из-за отключённого IMU.

**Файлы**:
- `D:\repos\verter-robot\verter_admin\src\verter_admin\config\robot_localization\ekf.yaml`
- `D:\repos\verter-robot\verter_admin\src\verter_admin\urdf\verter_robot_minimal.urdf`

**Команды диагностики**:
```bash
# Проверить TF дерево
ros2 run tf2_tools view_frames

# Тест на дрейф при повороте
ros2 run verter_admin teleop_keyboard
# Выполнить 5 полных оборотов по часовой стрелке (1800°)
# Ожидаемое: yaw изменение = 31.4 радиана
# Фактическое: записать из /odometry/filtered

# Проверить IMU данные (доступны но не используются)
ros2 topic echo /imu/data --once | grep angular_velocity
```

**Ожидаемые результаты**:
- Ошибка поворота >20% = критический дрейф
- IMU публикует данные с drift ~-0.001 рад/с по z

**Критерии проблемы**:
- Нет IMU коррекции → angular velocity только из энкодеров
- Wheel slip на поворотах → накопление ошибок yaw
- Wheel base несоответствие: URDF 0.356м vs калибровка 0.3642м

**Рекомендуемые исправления** (варианты):
1. Откалибровать IMU BMX055 bias
2. Увеличить process noise для yaw в EKF
3. Временно оставить без IMU если карта чистая

---

### Phase 3: Nav2 Safety Diagnostics - Obstacle Avoidance

**Цель**: Проверить безопасность навигации и costmap параметры.

**Файлы**:
- `D:\repos\verter-robot\verter_admin\src\verter_admin\config\nav2\nav2_navigation_params.yaml`

**Команды диагностики**:
```bash
# Запустить навигацию с картой
ros2 launch verter_admin navigation.launch.py map:=~/maps/test_map.yaml

# Проверить критичные параметры
ros2 param get /controller_server FollowPath.BaseObstacle.scale
ros2 param get /local_costmap/local_costmap inflation_radius
ros2 param get /global_costmap/global_costmap inflation_radius

# Проверить observation sources
ros2 param get /local_costmap/local_costmap observation_sources

# Визуализировать costmap в RViz
# Добавить: /local_costmap/costmap, /global_costmap/costmap, RobotFootprint
```

**Ожидаемые результаты**:
- `BaseObstacle.scale` = **0.02** (КРИТИЧНО НИЗКИЙ - опасно!)
- `inflation_radius` = 0.40 (маловато)
- `observation_sources` = только `scan` (ультразвук отсутствует)

**Критерии проблемы**:
- BaseObstacle.scale 0.02 означает, что препятствия почти игнорируются при планировании
- Робот может въехать в стену/объект
- Зазор всего 15см (0.40-0.25) - недостаточно для безопасности

**Рекомендуемые исправления**:
```yaml
# В nav2_navigation_params.yaml:
# Строка 130:
BaseObstacle.scale: 5.0  # вместо 0.02

# Строки 271, 309:
inflation_radius: 0.60  # вместо 0.40
```

---

### Phase 4: Sensor Integration - Ультразвук в Costmap

**Цель**: Проверить возможность интеграции 7x HC-SR04 ультразвуковых датчиков.

**Файлы**:
- `D:\repos\verter-robot\verter_admin\src\verter_admin\urdf\verter_robot_minimal.urdf`
- `D:\repos\verter-robot\verter_admin\src\verter_admin\control\adapters\ros\distance_sensors_node.py`

**Команды диагностики**:
```bash
# Проверить ультразвуковые топики
ros2 topic list | grep ultrasonic
ros2 topic list | grep distance_sensor

# Проверить позиции датчиков в URDF
ros2 run tf2_ros tf2_echo base_footprint sensor_front_center

# Проверить существующие costmap слои
ros2 param get /local_costmap/local_costmap plugins
```

**Ожидаемые результаты**:
- Ультразвуковые топики либо отсутствуют, либо не в формате LaserScan
- URDF содержит 7 сенсоров с позициями
- Costmap имеет только лидар слой

**Критерии проблемы**:
- Документация описывает ультразвук интеграцию (autonomous_mapping.md)
- Но в nav2_navigation_params.yaml ультразвук слой отсутствует
- Слепая зона 0.0-0.20м вокруг робота

**Рекомендуемые исправления** (сложная интеграция, требует разработки):
1. Настроить публикацию `/ultrasonic/scan` как LaserScan
2. Добавить observation source в costmap
3. Или снизить min_range лидара до 0.10м

---

### Phase 5: Integration Test - Полный Цикл

**Цель**: Протестировать систему с исправленными параметрами.

**Команды тестирования**:
```bash
# 1. Сохранить бэкапы конфигов
cp ~/verter-robot/verter_admin/src/verter_admin/config/slam/slam_toolbox_params.yaml ~/slam_backup.yaml
cp ~/verter-robot/verter_admin/src/verter_admin/config/nav2/nav2_navigation_params.yaml ~/nav2_backup.yaml

# 2. Внести исправления вручную (согласно фаз 1-3)

# 3. Пересобрать
cd ~/verter-robot/verter_admin
colcon build --packages-select verter_admin --symlink-install
source install/setup.bash

# 4. Тестовое картографирование комнаты 4x4м
ros2 launch verter_admin mapping.launch.py
# В RViz: пройти периметр, вернуться в起点

# 5. Сохранить карту
mkdir -p ~/maps_test
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '$HOME/maps_test/room_4x4'}}"

# 6. Тест навигации
ros2 launch verter_admin navigation.launch.py map:=~/maps_test/room_4x4.yaml
# Отправить цели: противоположная стена, возврат в起点

# 7. Собрать метрики
ros2 topic hz /scan
ros2 topic hz /odometry/filtered
ros2 topic hz /local_costmap/costmap
```

**Критерии успеха**:
1. **Качество карты**: стены прямые, без двойных контуров, loop closure точный
2. **Безопасность**: робот держит дистанцию ≥0.50м от препятствий
3. **Стабильность**: без дрейфов позиций, TF стабилен
4. **Производительность**: controller freq ≥4 Hz

---

## Критичные Файлы для Изменений

| Файл | Строка | Параметр | Текущее | Рекомендуется |
|------|--------|----------|---------|---------------|
| `config/slam/slam_toolbox_params.yaml` | 57 | `link_match_minimum_response_fine` | 0.1 | **0.45** |
| `config/nav2/nav2_navigation_params.yaml` | 130 | `BaseObstacle.scale` | 0.02 | **5.0** |
| `config/nav2/nav2_navigation_params.yaml` | 271, 309 | `inflation_radius` | 0.40 | **0.60** |

---

## Порядок Выполнения

1. **Сначала диагностика** (фазы 1-4) - подтвердить проблемы
2. **Затем исправления** - изменить 3 критичных параметра
3. **Наконец интеграционный тест** (фаза 5) - проверить результат

**Ожидаемое время**: 2-3 часа на диагностику + 30 мин на исправления + 1 час на тестирование

---

## Дополнительная Диагностика (опционально)

Если проблемы сохраняются после исправления параметров:

**A. Проверить лидар установку**:
- Высота 80см от пола (оптимально)
- Позиция смещена вперёд на 0.12м (parallax error)
- Убедиться что лидар НЕ вращается

**B. Проверить калибровку шасси**:
- Перекалибровать энкодеры: `chassis_calibrate.py encoder`
- Перекалибровать wheelbase: `chassis_calibrate.py wheelbase`

**C. Проверить скорость маппинга**:
- Снизить до 0.12-0.15 м/с для лучшего качества сканов

---

## Верификация Исправлений

После внесения изменений:

```bash
# Проверить что параметры применились
ros2 launch verter_admin mapping.launch.py
ros2 param get /slam_toolbox link_match_minimum_response_fine  # должен быть 0.45

ros2 launch verter_admin navigation.launch.py map:=~/maps/test.yaml
ros2 param get /controller_server FollowPath.BaseObstacle.scale  # должен быть 5.0

# Визуально в RViz:
# - Стены на карте прямые, без двойных контуров
# - Робот держит дистанцию от стен
# - Costmap показывает inflated зоны
```
