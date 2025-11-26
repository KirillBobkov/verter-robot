# План разработки: Переход на лидарную навигацию

## Обзор

Данный план описывает переход от ToF камеры к лидару RPLiDAR A1M8 для улучшения качества картографирования и навигации в крупных помещениях.

## Архитектурные решения

### 1. Сенсорная архитектура

```
┌─────────────────────────────────────────────────────────┐
│                    Sensor Fusion                        │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  Приоритет 1: RPLiDAR A1M8                             │
│  • FOV: 180° (спереди, -90° до +90°)                   │
│  • Дальность: 0.15-12 м                                │
│  • Назначение: SLAM + основная навигация               │
│  • Topic: /scan                                         │
│                                                         │
│  Приоритет 2: Ультразвуковые HC-SR04 (7 шт)           │
│  • Покрытие: ~150° спереди (-76° до +76°)              │
│  • Дальность: 0.02-4 м (точно до ~1м)                  │
│  • Назначение: Safety layer, детекция ближних объектов │
│  • Topic: /ultrasonic/ranges                            │
│  • Датчики: Front Center (0°), L/R Inner (±6°),        │
│    L/R Outer (±13°), Left/Right (±76°)                 │
│                                                         │
│  Приоритет 3: IMU MPU-6050 + Одометрия                │
│  • Назначение: Sensor fusion для локализации           │
│  • EKF для объединения данных                          │
│  • Topics: /imu/data, /odom                            │
│                                                         │
│  ⚠️  СЛЕПАЯ ЗОНА: Задняя часть (~180°)                 │
│  • Стратегия: Движение преимущественно вперед          │
│  • Nav2 настроен на минимизацию движения назад         │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

**Ключевые особенности:**
- **Перекрытие лидара и ультразвуковых:** ~150° спереди имеет двойное покрытие
  - Лидар: точная дальняя детекция (0.15-12м)
  - Ультразвуковые: надежная ближняя детекция (0.02-4м)
- **Боковое покрытие:** Частичное (~30° с каждой стороны от боковых датчиков)
- **Задняя слепая зона:** ~180° без прямой сенсорной информации
  - Компенсация через одометрию и IMU
  - Nav2 конфигурация предотвращает движение задом в неизвестные области

**Стратегия работы со слепой зоной:**
1. **Планирование пути:** Nav2 настроен на forward-only navigation где возможно
2. **Повороты на месте:** Робот поворачивается перед движением вместо движения назад
3. **Параметр `allow_reversing: false`** в Regulated Pure Pursuit Controller
4. **При необходимости движения назад:** Сначала поворот на 180°, затем движение вперед
5. **Опциональное расширение:** Возможность добавить 1-2 задних ультразвуковых датчика

### 2. GUI решение: RViz2 + Waypoint Manager

**Выбранное решение:** RViz2 с Nav2 Panel + пользовательский waypoint manager node

**Альтернативы рассмотрены:**
| Вариант | Плюсы | Минусы | Решение |
|---------|-------|--------|---------|
| RViz2 + Nav2 Panel | ✅ Готовое решение<br>✅ Нулевая настройка<br>✅ Отличная визуализация | ❌ Нет save/load в Humble<br>❌ Технический UI | ✅ **ВЫБРАНО**<br>+ Python waypoint manager |
| Foxglove Studio | ✅ Веб-интерфейс<br>✅ Удаленный доступ | ❌ Нет waypoint UI<br>❌ Требует кастомизации | ⏳ Для будущего мониторинга |
| Robotics UI | ✅ Современный UI<br>✅ Native waypoints | ❌ Только ROS2 Iron<br>❌ Не для Humble | ⏳ При апгрейде до Iron |

**Архитектура waypoint management:**
```
┌──────────────┐         ┌─────────────────────┐
│   RViz2      │         │  Waypoint Manager   │
│              │         │       Node          │
│ • Клик по    │         │                     │
│   карте      │────────▶│ • Load YAML        │
│ • Визуализа- │         │ • Save waypoints   │
│   ция        │         │ • Execute mission  │
│              │         │                     │
└──────────────┘         └──────────┬──────────┘
                                    │
                                    ▼
                          ┌─────────────────────┐
                          │   Nav2 Stack        │
                          │                     │
                          │ • followWaypoints() │
                          │ • goToPose()        │
                          └─────────────────────┘
```

### 3. SLAM vs AMCL - Пояснение

**Вопрос:** Что такое AMCL и нужна ли динамическая локализация?

**Два режима работы Nav2:**

#### Режим 1: SLAM (Mapping Mode)
- **Что делает:** Одновременно строит карту И определяет положение робота
- **Когда использовать:** При первом исследовании помещения
- **Инструмент:** SLAM Toolbox
- **Выход:** Новая карта + локализация

```
Робот не знает карту → SLAM → Создает карту + локализуется
```

#### Режим 2: AMCL (Localization Mode)
- **Что делает:** Определяет положение робота на ГОТОВОЙ карте
- **Когда использовать:** Когда карта уже построена и сохранена
- **Инструмент:** AMCL (Adaptive Monte Carlo Localization)
- **Выход:** Только локализация (карта не меняется)

```
Робот знает карту → AMCL → Локализуется на известной карте
```

**Для вашего проекта нужны ОБА режима:**

1. **Первый запуск в новом помещении:**
   - Запустить SLAM Toolbox (mapping mode)
   - Робот автономно исследует (exploration)
   - Сохранить карту: `ros2 run nav2_map_server map_saver_cli`

2. **Последующие запуски в том же помещении:**
   - Загрузить сохраненную карту
   - Запустить AMCL для локализации
   - Навигация по waypoints на известной карте

**Преимущества разделения:**
- SLAM вычислительно тяжелее - не нужен когда карта известна
- AMCL быстрее и точнее на готовой карте
- Карта остается стабильной (не "плывет" при длительной работе)

### 4. ROS2 Iron vs Humble - Анализ апгрейда

**Текущая версия:** ROS2 Humble (LTS до 2027)

**Что даст переход на Iron?**

| Функция | Humble | Iron | Нужно ли сейчас? |
|---------|--------|------|------------------|
| Save/Load waypoints в RViz | ❌ Нет | ✅ Есть | ⚠️ Решается Python скриптом |
| Robotics UI | ❌ Нет | ✅ Есть | ⏳ Удобно, но не критично |
| Nav2 baseline | ✅ Стабильная | ✅ Новая | ✅ Humble достаточно |
| SLAM Toolbox | ✅ 2.6.x | ✅ 2.7.x | ≈ Минимальные отличия |
| LTS Support | ✅ До 2027 | ❌ EOL май 2024 | ⚠️ Iron уже устарел |

**Рекомендация:** **Оставаться на Humble**

**Почему:**
1. ✅ **Humble - LTS** (Long Term Support до 2027)
2. ❌ **Iron уже EOL** (End of Life в мае 2024) - нет обновлений безопасности
3. ✅ Все нужные функции есть в Humble
4. ✅ Waypoint save/load легко добавить через Python (50 строк кода)
5. ⚠️ Миграция на Iron сломает совместимость с пакетами

**Альтернатива:** Если апгрейд, то сразу на **ROS2 Jazzy** (LTS до 2029)
- Но это в будущем, когда стабилизируется экосистема пакетов
- Сейчас Humble - оптимальный выбор

---

## Фазы разработки

### ФАЗА 1: Симуляция с лидаром (1-2 недели)

**Цель:** Полностью работающая симуляция в Gazebo с лидаром, SLAM и навигацией

#### Задача 1.1: Обновить URDF модель робота
- [ ] Создать файл `verter_robot_lidar.urdf` на основе существующего
- [ ] Добавить link для RPLiDAR A1M8:
  - Позиция: спереди корпуса, высота ~0.30м
  - Размер сенсора: 0.05m x 0.05m x 0.04m
- [ ] Добавить Gazebo плагин `libgazebo_ros_ray_sensor.so`:
  ```xml
  <sensor type="ray" name="lidar_sensor">
    <update_rate>5.5</update_rate>  <!-- Real RPLiDAR A1M8 freq -->
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <min_angle>-1.5708</min_angle>  <!-- -90° -->
          <max_angle>1.5708</max_angle>   <!-- +90° (180° FOV) -->
        </horizontal>
      </scan>
      <range>
        <min>0.15</min>
        <max>12.0</max>
      </range>
    </ray>
  </sensor>
  ```
- [ ] Удалить/закомментировать секцию ToF камеры
- [ ] Сохранить старый URDF как `verter_robot_tof.urdf.backup`

**Файлы:**
- `src/verter_admin/urdf/verter_robot_lidar.urdf` (новый)
- `src/verter_admin/urdf/verter_robot_gazebo.urdf` (обновить)

**Проверка:**
```bash
ros2 launch gazebo_ros spawn_entity.py -entity verter -file verter_robot_lidar.urdf
ros2 topic echo /scan
```

#### Задача 1.2: Создать launch файл для симуляции с лидаром
- [ ] Создать `lidar_simulation.launch.py`:
  - Запуск Gazebo с `test_room.world`
  - Spawn робота с новым URDF
  - Запуск robot_state_publisher
  - Параметр `use_sim_time:=true` для всех нод

**Файл:**
- `src/verter_admin/launch/lidar_simulation.launch.py` (новый)

**Структура:**
```python
# 1. Gazebo
# 2. Spawn robot
# 3. Robot state publisher
# 4. Joint state publisher (если нужен)
```

**Проверка:**
```bash
ros2 launch verter_admin lidar_simulation.launch.py
rviz2  # Добавить display для LaserScan на topic /scan
```

#### Задача 1.3: Обновить конфигурации для лидара

**Файл 1: `slam_toolbox_params.yaml`**
- [ ] Обновить параметры:
  ```yaml
  scan_topic: /scan
  max_laser_range: 12.0    # RPLiDAR A1M8
  minimum_range: 0.15
  use_sim_time: true

  # Оптимизация для лидара (vs ToF)
  minimum_travel_distance: 0.3  # Уменьшить (лидар точнее ToF)
  minimum_travel_heading: 0.3
  link_match_minimum_response_fine: 0.2  # Увеличить порог
  ```

**Файл 2: `nav2_params.yaml`**
- [ ] Обновить `local_costmap` и `global_costmap`:
  ```yaml
  observation_sources: lidar_scan ultrasonic

  lidar_scan:
    sensor_frame: base_link  # или lidar_link
    data_type: LaserScan
    topic: /scan
    marking: true
    clearing: true
    max_obstacle_height: 2.0
    raytrace_max_range: 11.0
    obstacle_max_range: 10.0

  ultrasonic:
    topic: /ultrasonic/ranges
    marking: true
    clearing: false  # Safety-only
    obstacle_max_range: 1.0
  ```

**Файл 3: `ekf.yaml` (проверить)**
- [ ] Убедиться что фьюжен IMU + одометрия настроен

**Файлы:**
- `src/verter_admin/config/slam/slam_toolbox_params.yaml` (обновить)
- `src/verter_admin/config/nav2/nav2_params.yaml` (обновить)
- `src/verter_admin/config/robot_localization/ekf.yaml` (проверить)

**Проверка:**
```bash
# Проверить валидность YAML
ros2 param dump /slam_toolbox --output-dir /tmp
```

#### Задача 1.4: Создать launch для SLAM + Navigation
- [ ] Создать `lidar_slam_nav.launch.py`:
  - Симуляция робота (из 1.2)
  - SLAM Toolbox
  - Nav2 bringup
  - RViz2 с преднастройкой

**Файл:**
- `src/verter_admin/launch/lidar_slam_nav.launch.py` (новый)

**Проверка:**
```bash
ros2 launch verter_admin lidar_slam_nav.launch.py
# В RViz должны быть: карта, costmaps, лазерные лучи, робот
```

#### Задача 1.5: Тестирование в симуляции
- [ ] Запустить полный стек
- [ ] Проверить публикацию `/scan` (частота ~5-6 Hz)
- [ ] Запустить teleop: `ros2 run verter_admin teleop_keyboard`
- [ ] Построить карту комнаты вручную
- [ ] Проверить качество карты (четкость стен, отсутствие шума)
- [ ] Сохранить карту:
  ```bash
  ros2 run nav2_map_server map_saver_cli -f ~/maps/test_room
  ```
- [ ] Проверить TF tree:
  ```bash
  ros2 run tf2_tools view_frames
  # Должна быть цепочка: map -> odom -> base_link -> lidar_link
  ```

**Критерии успеха:**
- ✅ LaserScan публикуется стабильно
- ✅ Карта строится без артефактов
- ✅ Робот локализуется корректно
- ✅ Нет ошибок в TF трансформациях

---

### ФАЗА 2: GUI и Waypoint Management (3-5 дней)

**Цель:** Удобный интерфейс для установки waypoints и управления навигацией

#### Задача 2.1: Создать RViz конфигурацию
- [ ] Создать `lidar_navigation.rviz` с отображением:
  - Map (topic: `/map`)
  - LaserScan (topic: `/scan`)
  - Local Costmap (topic: `/local_costmap/costmap`)
  - Global Costmap (topic: `/global_costmap/costmap`)
  - Robot Model (URDF)
  - TF tree
  - Path (topic: `/plan`)
  - Nav2 Goal tool (для waypoints)
  - Nav2 Panel
  - Current pose (topic: `/amcl_pose` или `/pose`)

**Файл:**
- `lidar_navigation.rviz` (новый, в корне проекта)

**Проверка:**
```bash
rviz2 -d lidar_navigation.rviz
```

#### Задача 2.2: Создать Waypoint Manager Node
- [ ] Создать `waypoint_manager_node.py`:
  ```python
  class WaypointManager(Node):
      def __init__(self):
          # Сервисы
          self.srv_load = self.create_service(LoadWaypoints, 'load_waypoints', ...)
          self.srv_save = self.create_service(SaveWaypoint, 'save_waypoint', ...)
          self.srv_execute = self.create_service(ExecuteMission, 'execute_mission', ...)

          # Nav2 Simple Commander
          self.navigator = BasicNavigator()

      def load_waypoints_from_yaml(self, filepath):
          # Читать YAML с waypoints
          pass

      def execute_waypoint_mission(self, waypoints):
          # Использовать navigator.followWaypoints()
          pass
  ```

**Файл:**
- `src/verter_admin/waypoint_manager/waypoint_manager_node.py` (новый)

**Зависимости:**
```bash
pip install pyyaml
```

#### Задача 2.3: Определить формат YAML для waypoints
```yaml
mission_name: "Hospital Floor 1 Route"
waypoints:
  - name: "Reception"
    position:
      x: 2.5
      y: 1.3
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0  # Quaternion (0° = [0,0,0,1])

  - name: "Room 101"
    position:
      x: 5.1
      y: 3.2
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.707
      w: 0.707  # 90°

  - name: "Elevator"
    position:
      x: 8.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 1.0
      w: 0.0  # 180°
```

**Директория:**
- `src/verter_admin/config/missions/*.yaml`

#### Задача 2.4: Создать helper скрипты
- [ ] `save_current_pose.py` - сохранить текущую позу в YAML
  ```bash
  ros2 run verter_admin save_current_pose --name "Kitchen" --output mission.yaml
  ```
- [ ] `load_mission.py` - загрузить и выполнить миссию
  ```bash
  ros2 run verter_admin load_mission --file mission.yaml
  ```

**Файлы:**
- `src/verter_admin/waypoint_manager/save_current_pose.py` (новый)
- `src/verter_admin/waypoint_manager/load_mission.py` (новый)

**Добавить в setup.py:**
```python
'save_current_pose = verter_admin.waypoint_manager.save_current_pose:main',
'load_mission = verter_admin.waypoint_manager.load_mission:main',
'waypoint_manager_node = verter_admin.waypoint_manager.waypoint_manager_node:main',
```

#### Задача 2.5: Workflow тестирование
**Сценарий:**
1. Запустить симуляцию + SLAM + Nav2
2. Открыть RViz с настроенной конфигурацией
3. Построить карту (teleop)
4. Кликнуть 3-4 waypoints в RViz (Nav2 Goal tool)
5. Для каждого waypoint запустить:
   ```bash
   ros2 run verter_admin save_current_pose --name "Point1"
   ```
6. Сохранить миссию в YAML
7. Перезапустить симуляцию
8. Загрузить миссию:
   ```bash
   ros2 run verter_admin load_mission --file test_mission.yaml
   ```
9. Проверить что робот проходит все waypoints

**Критерии успеха:**
- ✅ Waypoints сохраняются корректно
- ✅ Робот проходит маршрут без застреваний
- ✅ Интерфейс интуитивный и быстрый

---

### ФАЗА 3: Автономное Exploration (2-3 дня)

**Цель:** Робот автоматически исследует неизвестное помещение

#### Задача 3.1: Выбрать exploration пакет
**У вас уже есть m-explore-ros2 в проекте!**

**Альтернативы:**
- `m-explore` (multi-robot, сложнее)
- `explore_lite` (проще, один робот)

**Рекомендация:** Начать с `explore_lite`, если m-explore вызывает проблемы

- [ ] Проверить установку:
  ```bash
  ros2 pkg list | grep explore
  ```
- [ ] Если нужно установить explore_lite:
  ```bash
  cd src/
  git clone https://github.com/robo-friends/m-explore-ros2.git -b humble
  cd ..
  colcon build --packages-select explore_lite
  ```

#### Задача 3.2: Создать конфигурацию exploration
- [ ] Создать `explore_params.yaml`:
  ```yaml
  explore_node:
    ros__parameters:
      robot_base_frame: base_link
      costmap_topic: /global_costmap/costmap
      costmap_updates_topic: /global_costmap/costmap_updates
      visualize: true
      planner_frequency: 0.5
      progress_timeout: 30.0
      potential_scale: 3.0
      orientation_scale: 0.0
      gain_scale: 1.0
      min_frontier_size: 0.5  # Минимальный размер frontier (метры)
  ```

**Файл:**
- `src/verter_admin/config/explore/explore_params.yaml` (обновить существующий)

**Адаптация под 180° лидар:**
- Увеличить `min_frontier_size` для стабильности
- Настроить `planner_frequency` под частоту лидара

#### Задача 3.3: Создать launch файл для exploration
- [ ] Создать `autonomous_exploration.launch.py`:
  ```python
  # 1. Симуляция (опционально)
  # 2. SLAM Toolbox (mapping mode)
  # 3. Nav2 bringup
  # 4. Explore node
  # 5. RViz с визуализацией frontiers
  ```

**Файл:**
- `src/verter_admin/launch/autonomous_exploration.launch.py` (новый)

**Проверка:**
```bash
ros2 launch verter_admin autonomous_exploration.launch.py
```

#### Задача 3.4: Тестирование exploration
**Сценарий:**
1. Запустить exploration launch
2. Наблюдать как робот автономно исследует комнату
3. Проверить frontiers в RViz (зеленые области)
4. Дождаться полного покрытия помещения
5. Сохранить карту:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/maps/explored_room
   ```

**Метрики:**
- Время полного исследования комнаты 10x10м: ~10-15 минут
- Покрытие: >95% доступной площади
- Отсутствие зацикливаний

**Критерии успеха:**
- ✅ Робот исследует все доступные области
- ✅ Не застревает в углах
- ✅ Карта полная и точная

---

### ФАЗА 4: Интеграция с реальным роботом (1 неделя)

**Цель:** Запустить всю систему на реальном железе

#### Задача 4.1: Установить драйвер RPLiDAR
- [ ] Установить пакет:
  ```bash
  sudo apt install ros-humble-rplidar-ros
  ```
- [ ] Подключить лидар к USB
- [ ] Проверить порт:
  ```bash
  ls -l /dev/ttyUSB*
  # Обычно /dev/ttyUSB0
  ```
- [ ] Дать права доступа:
  ```bash
  sudo chmod 666 /dev/ttyUSB0
  # Или добавить правило udev
  ```
- [ ] Тестовый запуск:
  ```bash
  ros2 launch rplidar_ros rplidar_a1_launch.py
  ros2 topic echo /scan
  ```

**Проверка:**
- Топик `/scan` публикуется
- Частота ~5.5 Hz
- Ranges в диапазоне 0.15-12.0м

#### Задача 4.2: Создать launch для реального робота
- [ ] Создать `real_robot_lidar.launch.py`:
  ```python
  # 1. RPLiDAR node
  # 2. Distance sensors node (ультразвуковые)
  # 3. Odometry node
  # 4. IMU node (если отдельный)
  # 5. Chassis node
  # 6. Robot state publisher
  # 7. Robot localization (EKF)
  #
  # use_sim_time = False для всех!
  ```

**Файл:**
- `src/verter_admin/launch/real_robot_lidar.launch.py` (новый)

**Важно:**
```python
parameters=[{
    'use_sim_time': False  # Реальное время!
}]
```

#### Задача 4.3: Создать launch для SLAM на реальном роботе
- [ ] Создать `real_robot_slam.launch.py`:
  ```python
  # 1. real_robot_lidar.launch (include)
  # 2. SLAM Toolbox (use_sim_time=False)
  # 3. Nav2 (use_sim_time=False)
  ```

**Файл:**
- `src/verter_admin/launch/real_robot_slam.launch.py` (новый)

#### Задача 4.4: Калибровка робота
- [ ] **Wheelbase калибровка:**
  ```bash
  # Измерить реальное расстояние между колесами
  # Обновить в URDF и nav2_params.yaml
  ```
- [ ] **Wheel diameter калибровка:**
  - Прокатить робот ровно 1 метр
  - Сравнить с одометрией
  - Скорректировать диаметр в URDF
- [ ] **IMU калибровка:**
  - Запустить IMU calibration tool
  - Сохранить offsets
- [ ] **Lidar position:**
  - Измерить точную позицию лидара относительно base_link
  - Обновить TF в URDF

**Документация:**
- Записать все измеренные параметры в `docs/robot_calibration.md`

#### Задача 4.5: Проверка TF tree
- [ ] Запустить реального робота
- [ ] Проверить TF:
  ```bash
  ros2 run tf2_tools view_frames
  evince frames.pdf
  ```
- [ ] Убедиться в цепочке:
  ```
  map -> odom -> base_footprint -> base_link -> lidar_link
                                              -> imu_link
                                              -> wheel_left
                                              -> wheel_right
  ```
- [ ] Проверить синхронизацию:
  ```bash
  ros2 run tf2_ros tf2_echo map base_link
  # Не должно быть ошибок extrapolation
  ```

#### Задача 4.6: Реальное тестирование
**Тест 1: Базовое движение**
- [ ] Запустить teleop
- [ ] Проверить прямое движение (1м)
- [ ] Проверить поворот на 90°
- [ ] Проверить поворот на 180°
- [ ] Проверить одометрию (реальная vs расчетная)

**Тест 2: Построение карты**
- [ ] Запустить SLAM
- [ ] Вручную объехать комнату
- [ ] Сохранить карту
- [ ] Проверить качество (четкость стен)

**Тест 3: Навигация**
- [ ] Загрузить сохраненную карту
- [ ] Запустить AMCL локализацию
- [ ] Установить goal в RViz
- [ ] Проверить что робот доезжает

**Тест 4: Waypoint миссия**
- [ ] Создать миссию с 3-4 waypoints
- [ ] Загрузить миссию
- [ ] Проверить прохождение всего маршрута

**Тест 5: Exploration**
- [ ] Запустить autonomous exploration
- [ ] Дать роботу исследовать незнакомое помещение
- [ ] Проверить полноту карты

**Критерии успеха:**
- ✅ Все топики публикуются стабильно
- ✅ TF tree без ошибок
- ✅ Одометрия точная (погрешность <5%)
- ✅ Карты качественные
- ✅ Навигация успешная (95%+ waypoints достигнуты)

---

### ФАЗА 5: Оптимизация и улучшения (опционально)

**Приоритет: После успешного запуска основного функционала**

#### Задача 5.1: Оптимизация производительности
- [ ] Профилирование CPU/память:
  ```bash
  ros2 run ros2_performance performance_test
  ```
- [ ] Снизить частоту обновления costmap если нужно
- [ ] Оптимизировать SLAM параметры

#### Задача 5.2: Safety improvements
- [ ] Добавить `lidar_safety_monitor_node.py`:
  - Аварийная остановка при критическом препятствии (<0.3м)
  - Публикация `/emergency_stop`
- [ ] Интегрировать с chassis_node

#### Задача 5.3: Multi-floor поддержка
- [ ] Система именования карт по этажам
- [ ] Загрузка карты в зависимости от этажа
- [ ] Интеграция с лифтами (в далеком будущем)

#### Задача 5.4: Semantic waypoints
- [ ] Расширить YAML формат:
  ```yaml
  waypoints:
    - name: "Kitchen"
      semantic_type: "room"
      ...
  ```
- [ ] Голосовое управление: "Поезжай на кухню"

#### Задача 5.5: Web monitoring (Foxglove)
- [ ] Установить Foxglove Studio
- [ ] Настроить rosbridge
- [ ] Создать layout для мониторинга
- [ ] Доступ с планшета/телефона

#### Задача 5.6: Dock station
- [ ] Разработать AprilTag метки для док-станции
- [ ] Автоматическая парковка для зарядки
- [ ] Интеграция с battery monitoring

---

## Структура файлов проекта

```
verter_admin/
├── docs/
│   ├── PROJECT_GOAL.md              (этот файл)
│   ├── DEVELOPMENT_PLAN.md          (план)
│   ├── robot_calibration.md         (калибровочные данные)
│   └── API_REFERENCE.md             (документация интерфейсов)
│
├── src/verter_admin/
│   ├── urdf/
│   │   ├── verter_robot_lidar.urdf  (новый URDF с лидаром)
│   │   └── verter_robot_tof.urdf.backup
│   │
│   ├── launch/
│   │   ├── lidar_simulation.launch.py
│   │   ├── lidar_slam_nav.launch.py
│   │   ├── autonomous_exploration.launch.py
│   │   ├── real_robot_lidar.launch.py
│   │   └── real_robot_slam.launch.py
│   │
│   ├── config/
│   │   ├── slam/
│   │   │   └── slam_toolbox_params.yaml (обновить)
│   │   ├── nav2/
│   │   │   └── nav2_params.yaml (обновить)
│   │   ├── explore/
│   │   │   └── explore_params.yaml
│   │   ├── missions/
│   │   │   ├── hospital_floor1.yaml
│   │   │   └── test_mission.yaml
│   │   └── robot_localization/
│   │       └── ekf.yaml (проверить)
│   │
│   ├── waypoint_manager/           (новый пакет)
│   │   ├── __init__.py
│   │   ├── waypoint_manager_node.py
│   │   ├── save_current_pose.py
│   │   └── load_mission.py
│   │
│   └── worlds/
│       └── test_room.world (проверить/обновить)
│
├── lidar_navigation.rviz           (новая конфигурация)
└── verter_slam.rviz                (существующая)
```

---

## Оценка времени

| Фаза | Задачи | Оптимистично | Реалистично | Пессимистично |
|------|--------|--------------|-------------|---------------|
| Фаза 1 | Симуляция с лидаром | 5 дней | 7-10 дней | 2 недели |
| Фаза 2 | GUI + Waypoints | 2 дня | 3-5 дней | 1 неделя |
| Фаза 3 | Exploration | 1 день | 2-3 дня | 5 дней |
| Фаза 4 | Реальный робот | 3 дня | 5-7 дней | 2 недели |
| Фаза 5 | Оптимизация | - | по желанию | - |
| **ИТОГО** | | **11 дней** | **3-4 недели** | **5-6 недель** |

**Рекомендация:** Закладывать 4 недели на основной функционал

---

## Критерии успеха проекта

### Минимальный MVP (Minimum Viable Product)
- ✅ Робот в симуляции строит карту комнаты с лидаром
- ✅ Навигация к одному waypoint работает
- ✅ Карта сохраняется и загружается

### Базовый функционал
- ✅ MVP +
- ✅ Waypoint manager с save/load
- ✅ Autonomous exploration работает
- ✅ GUI в RViz удобный для использования

### Продакшн-готовый
- ✅ Базовый функционал +
- ✅ Работает на реальном роботе
- ✅ Калибровка выполнена
- ✅ Надежность навигации >95%
- ✅ Документация полная

### Расширенный
- ✅ Продакшн-готовый +
- ✅ Web monitoring (Foxglove)
- ✅ Safety improvements
- ✅ Голосовое управление навигацией
- ✅ Multi-floor поддержка

---

## NVIDIA Jetson Orin Nano - Оптимизации и возможности

### Характеристики платформы
- **GPU:** 1024-core NVIDIA Ampere с 32 Tensor Cores
- **CPU:** 6-core Arm Cortex-A78AE @ 2.0 GHz
- **Память:** 8GB LPDDR5
- **AI Performance:** 40 TOPS (INT8)
- **Power:** 7-15W
- **Охлаждение:** Требуется активное (вентилятор) для полной производительности

### Преимущества для навигации

#### 1. CUDA-Accelerated Processing

**Да, мы МОЖЕМ использовать CUDA для навигации!** Вот конкретные способы:

##### А. CPU оптимизации (Фаза 1-4)
```yaml
# slam_toolbox_params.yaml - используем все ядра CPU
num_threads: 6  # Все 6 ядер Cortex-A78AE
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY  # Параллельный solver

# nav2_params.yaml - высокие частоты обновления
local_costmap:
  update_frequency: 10.0   # Jetson справится (обычно 5.0)
  publish_frequency: 5.0
  resolution: 0.03         # Выше разрешение (обычно 0.05)
```

##### Б. CUDA для навигации (Фаза 5, опционально)

**1. GPU-Voxels для costmap processing**
```bash
# Библиотека от Karlsruhe Institute of Technology
# CUDA-accelerated voxel grid и costmap operations
git clone https://github.com/fzi-forschungszentrum-informatik/gpu-voxels.git
cd gpu-voxels && mkdir build && cd build
cmake .. -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda
make -j6
```
**Ускорение:** 10-50x быстрее для 3D voxel operations

**2. CUDA-PCL для laser scan processing**
```bash
# Point Cloud Library с CUDA support
sudo apt install ros-humble-perception-pcl
```
Создать custom node для обработки `/scan`:
```cpp
// cuda_scan_filter_node.cpp
#include <pcl/cuda/filters/filter.h>
#include <pcl/cuda/sample_consensus/sac_model_plane.h>

// GPU-accelerated filtering:
// - Statistical outlier removal
// - Voxel grid downsampling
// - Radius outlier removal
// Скорость: 20-100x быстрее CPU
```

**3. GPU-accelerated SLAM alternatives**

**Вариант A: KISS-ICP (рекомендуется)**
```bash
pip install kiss-icp
# Поддерживает CUDA для ICP (Iterative Closest Point)
# Ускорение: 15-30x на scan matching
```

**Вариант B: Fast-GICP**
```bash
git clone https://github.com/SMRT-AIST/fast_gicp.git
# CUDA-accelerated Generalized ICP
# Можно интегрировать с SLAM Toolbox
```

**4. Custom CUDA kernels для real-time processing**

Пример - GPU-accelerated scan preprocessing:
```cuda
// cuda_scan_processor.cu
__global__ void filterScanKernel(float* ranges, int size, float min_range, float max_range) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < size) {
        if (ranges[idx] < min_range || ranges[idx] > max_range) {
            ranges[idx] = NAN;
        }
    }
}

// Запуск: 10000 точек за 0.1ms vs 5ms на CPU
```

**5. GPU-accelerated particle filter (AMCL)**

Существует проект `amcl_gpu`:
```bash
# Адаптация AMCL с CUDA для particle filter
# 1000-10000 частиц в реальном времени
# Стандартный AMCL: максимум 500-1000 частиц
```

##### Практический roadmap CUDA интеграции:

**Фаза 1-4 (сейчас):** CPU only
- 6-core CPU достаточно для базовой навигации
- GPU простаивает (0-5% загрузка)

**Фаза 5.1 (опционально):** CUDA для обработки данных
- Custom scan filter node с CUDA (~1 неделя)
- Ускорение обработки `/scan` в 20-50x
- Возможность увеличить частоту обновления до 20-30 Hz

**Фаза 5.2 (опционально):** GPU-accelerated SLAM
- Интеграция KISS-ICP (~2 недели)
- Или Fast-GICP с SLAM Toolbox (~3 недели)
- Более точные карты, меньше drift

**Фаза 5.3 (опционально):** GPU Voxels costmap
- Замена стандартного costmap на GPU-Voxels (~2-3 недели)
- 3D obstacle avoidance (если добавим depth камеру)
- Real-time обновление больших карт

**Приоритет:**
- Фаза 1-4 сначала (базовая навигация)
- Затем оценить: нужно ли CUDA ускорение?
  - Если SLAM медленный → добавить Fast-GICP
  - Если costmap тормозит → добавить GPU-Voxels
  - Если scan processing узкое место → custom CUDA node

**Вывод:** CUDA можно и нужно использовать, но после того как базовая навигация работает!

#### 2. Расширенные возможности (будущее)

**Семантическое картографирование** - что это:
```
Обычная карта робота:
  Робот видит: "Тут препятствие высотой 0.5м"

Семантическая карта:
  Робот понимает: "Это стул" (временное препятствие)
                  "Это человек" (игнорировать для статической карты)
                  "Это дверь" (проход, может открываться)
                  "Это стена" (постоянное препятствие)

Применение:
- Более умное планирование путей
- Интеллектуальные waypoints ("Перед дверью кабинета")
- Адаптация поведения под тип объекта
```

**Реализация на Jetson:**
- YOLOv8 object detection @ 30-60 FPS (GPU inference)
- Потребует добавления камеры (RGB или depth)
- Интеграция с Nav2 через custom costmap layer

#### 3. Настройка производительности

**Режимы работы:**
```bash
# Максимальная производительность (15W)
sudo nvpmodel -m 0
sudo jetson_clocks

# Режим энергосбережения (7W) - для тестирования
sudo nvpmodel -m 1
```

**Docker оптимизация (рекомендуется):**
```dockerfile
FROM dustynv/ros:humble-pytorch-l4t-r35.4.1
# Включает: CUDA 11.4, cuDNN 8.6, TensorRT 8.5
# Оптимизировано для Jetson Orin
```

#### 4. Сравнение производительности

| Задача | CPU (RPi 4) | Jetson Orin Nano |
|--------|-------------|------------------|
| SLAM scan matching | 5-10 Hz | 15-30 Hz |
| Costmap update | 2-5 Hz | 10-15 Hz |
| Sensor fusion (7 датчиков) | Предел | Легко |
| YOLOv8 inference | Невозможно | 30-60 FPS |
| Semantic mapping | Невозможно | Real-time |
| Одновременная обработка | Ограничено | Параллельно |

#### 5. Roadmap использования Jetson

**Фаза 1-4 (текущий план):**
- Базовая навигация с лидаром
- Использование CPU Jetson (6 ядер)
- GPU пока не критичен
- **Преимущество:** Большой запас производительности, нет узких мест

**Фаза 5+ (будущее):**
- Добавление камеры (RGB-D)
- YOLOv8 для семантической сегментации
- Детекция динамических объектов (люди, тележки)
- Person following / avoidance
- Visual odometry как дополнительный источник

**Опциональные расширения:**
- Multi-robot coordination
- Fleet management сервер на Jetson
- Cloud connectivity для удаленного мониторинга

### Установка на Jetson

**Base setup:**
```bash
# Jetpack SDK (включает CUDA, cuDNN, TensorRT)
sudo apt install nvidia-jetpack

# ROS2 Humble
sudo apt install ros-humble-desktop

# CUDA-accelerated библиотеки
sudo apt install ros-humble-perception-pcl
```

**Проверка CUDA:**
```bash
nvcc --version
nvidia-smi  # Мониторинг GPU
jtop        # Jetson monitoring tool
```

---

## Риски и митигация

| Риск | Вероятность | Влияние | Митигация |
|------|-------------|---------|-----------|
| Качество карт с 180° лидаром хуже ожиданий | Средняя | Высокое | Гибридная система с ультразвуковыми датчиками |
| Слепая зона сзади (~180°) | Высокая | Среднее | Forward-only navigation, поворот перед движением назад |
| TF синхронизация проблемы | Высокая | Среднее | Тщательная настройка временных параметров |
| Exploration застревает в углах | Средняя | Среднее | Тюнинг параметров costmap и exploration |
| Одометрия дрифт на большой площади | Высокая | Высокое | EKF fusion с IMU, loop closure в SLAM |
| Перегрев Jetson под нагрузкой | Средняя | Среднее | Активное охлаждение, nvpmodel power modes |
| Проблемы с прозрачными поверхностями | Низкая | Среднее | Ультразвуковые как backup |

---

## Следующие шаги

1. **Немедленно:**
   - ✅ Создать документацию (этот файл)
   - ⏳ Обсудить с командой/заказчиком
   - ⏳ Утвердить приоритеты

2. **На этой неделе:**
   - Начать Фазу 1: Обновить URDF
   - Создать первый launch для симуляции
   - Протестировать публикацию /scan

3. **В течение месяца:**
   - Завершить Фазы 1-3 (симуляция полностью работает)
   - Начать Фазу 4 (реальный робот)

---

## Контакты и ресурсы

**Документация:**
- Nav2: https://navigation.ros.org/
- SLAM Toolbox: https://github.com/SteveMacenski/slam_toolbox
- RPLiDAR ROS2: https://github.com/Slamtec/rplidar_ros

**Сообщество:**
- ROS Discourse: https://discourse.ros.org/
- Nav2 GitHub Discussions

**Инструменты:**
- RViz2 tutorials: http://wiki.ros.org/rviz/Tutorials
- Gazebo: https://gazebosim.org/

---

**Версия:** 1.0
**Дата создания:** 2025-11-12
**Последнее обновление:** 2025-11-12
**Статус:** Утверждено к реализации
