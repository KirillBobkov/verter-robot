# 🗺️ Этап 4: Конвертер датчиков и SLAM картографирование

**Дата:** 18 октября 2025
**Статус:** ✅ ЗАВЕРШЕНО (конфигурация)
**Предыдущий этап:** [03_nav2_setup.md](03_nav2_setup.md)

---

## 🎯 Цель этапа

Создать конвертер ультразвуковых датчиков в формат LaserScan и настроить SLAM Toolbox для автономного построения карт окружения.

---

## 📋 Задачи

- [x] Создать ноду ultrasonic_to_laserscan_node
- [x] Настроить преобразование 7 датчиков в LaserScan
- [x] Установить SLAM Toolbox
- [x] Создать конфигурацию SLAM
- [x] Создать launch файл для SLAM
- [x] Обновить setup.py
- [ ] Протестировать построение карты

---

## 📝 Проблема: Nav2 требует LaserScan

**Проблема:**
Nav2 costmap ожидает топик `/scan` типа `sensor_msgs/LaserScan`, но робот Verter оснащен 7 ультразвуковыми датчиками HC-SR04, которые публикуют отдельные значения расстояний.

**Решение:**
Создана нода `ultrasonic_to_laserscan_node`, которая:
1. Читает данные с 7 датчиков напрямую с Arduino
2. Преобразует их в формат LaserScan
3. Публикует в топики `/scan` и `/ultrasonic/ranges`

---

## 🔧 Часть 1: Конвертер ультразвуковых датчиков

### Расположение датчиков (из URDF)

Робот Verter имеет 7 ультразвуковых датчиков:

```
Sensor 1 (левый боковой):     90° (π/2 rad)     - sensor_left
Sensor 2 (левый передний):    45° (π/4 rad)     - sensor_front_left_outer
Sensor 3 (левый внутренний):  25° (0.436 rad)   - sensor_front_left_inner
Sensor 4 (центральный):        0° (0.0 rad)     - sensor_front_center
Sensor 5 (правый внутренний): -25° (-0.436 rad) - sensor_front_right_inner
Sensor 6 (правый передний):   -45° (-π/4 rad)   - sensor_front_right_outer
Sensor 7 (правый боковой):    -90° (-π/2 rad)   - sensor_right
```

### Параметры HC-SR04

- **Минимальное расстояние:** 2 см (0.02 м)
- **Максимальное расстояние:** 400 см (4.0 м)
- **Угол обзора:** ~15-30° (конус)
- **Частота обновления:** ~10 Hz

### Особенности преобразования

**Проблема:** Ультразвуковые датчики имеют широкий конус обзора (~15-30°), а LaserScan ожидает узкий луч (<1°).

**Решение:**
1. Создаем LaserScan с 360 лучами (1° разрешение)
2. Для каждого луча находим ближайший датчик
3. Если луч отличается от датчика более чем на 15°, ставим max_range
4. Это создает "секторы видимости" для каждого датчика

### Созданный файл

**Файл:** `src/verter_admin/distance_sensors/ultrasonic_to_laserscan_node.py` (~350 строк)

**Основные компоненты:**

```python
class UltrasonicToLaserScanNode(Node):
    # Константы
    NUM_SENSORS = 7
    RANGE_MIN = 0.02  # 2 см
    RANGE_MAX = 4.0   # 4 м

    # Углы датчиков в радианах
    SENSOR_ANGLES = [
        math.pi / 2,      # 90°  - левый боковой
        math.pi / 4,      # 45°  - левый передний
        math.radians(25), # 25°  - левый внутренний
        0.0,              # 0°   - центральный
        -math.radians(25),# -25° - правый внутренний
        -math.pi / 4,     # -45° - правый передний
        -math.pi / 2,     # -90° - правый боковой
    ]
```

**Основной метод преобразования:**

```python
def _publish_laserscan(self):
    scan_msg = LaserScan()
    scan_msg.header.frame_id = 'base_link'
    scan_msg.angle_min = -math.pi / 2  # -90°
    scan_msg.angle_max = math.pi / 2   # +90°

    # Создаем 360 лучей для интерполяции
    num_interpolated_readings = 360
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (num_interpolated_readings - 1)

    ranges = []
    for i in range(num_interpolated_readings):
        angle = scan_msg.angle_min + i * scan_msg.angle_increment

        # Находим ближайший датчик к этому углу
        closest_sensor_idx = min(range(self.NUM_SENSORS),
                               key=lambda idx: abs(self.SENSOR_ANGLES[idx] - angle))

        # Если угол сильно отличается (>15°), ставим max range
        angle_diff = abs(self.SENSOR_ANGLES[closest_sensor_idx] - angle)
        if angle_diff > math.radians(15):
            ranges.append(self.RANGE_MAX)
        else:
            ranges.append(self.sensor_data[closest_sensor_idx])

    scan_msg.ranges = ranges
```

**Публикуемые топики:**
- `/scan` (LaserScan) - для Nav2
- `/ultrasonic/ranges` (LaserScan) - для отладки/визуализации

---

## 🗺️ Часть 2: SLAM Toolbox

### Что такое SLAM?

**SLAM** (Simultaneous Localization and Mapping) - одновременная локализация и картографирование. Это процесс, при котором робот:
1. Строит карту неизвестного окружения
2. Одновременно определяет свое положение на этой карте

### Почему SLAM Toolbox?

**SLAM Toolbox** - это пакет ROS2 для SLAM, который:
- ✅ Работает в реальном времени
- ✅ Поддерживает сохранение/загрузку карт
- ✅ Имеет loop closure detection
- ✅ Совместим с Nav2
- ✅ Может работать с низкочастотными LaserScan (~10 Hz от ультразвука)

### Установка

**Версия:** 2.6.4 (для Python 3.10)

**Команда:**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
conda install -c robostack-staging ros-humble-slam-toolbox=2.6.4 -y
```

**Результат:**
```
The following NEW packages will be INSTALLED:
  ros-humble-slam-toolbox  robostack-staging/osx-arm64::ros-humble-slam-toolbox-2.6.4
```

✅ **Установка успешна!**

---

## ⚙️ Конфигурация SLAM

**Файл:** `src/verter_admin/config/slam/slam_toolbox_params.yaml`

### Основные параметры

```yaml
slam_toolbox:
  ros__parameters:
    # Фреймы
    odom_frame: odom
    map_frame: map
    base_frame: base_link
    scan_topic: /scan

    # Режим
    mode: mapping  # mapping или localization

    # TF публикация
    transform_publish_period: 0.02  # 50 Hz

    # Минимальные изменения для обновления
    minimum_travel_distance: 0.15  # 15 см
    minimum_travel_heading: 0.15   # ~8.6°
```

### Параметры Scan Matching

```yaml
    # Scan Matching
    scan_buffer_size: 10
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    # Loop Closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_search_maximum_distance: 3.0
```

### Параметры корреляции

```yaml
    # Correlation для Scan Matching
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
```

**Почему эти значения?**
- `minimum_travel_distance: 0.15` - учитывает погрешность одометрии
- `do_loop_closing: true` - корректирует дрифт одометрии при возвращении в знакомые места
- `scan_buffer_size: 10` - хранит последние 10 сканов для улучшения точности

---

## 🚀 Launch файл SLAM

**Файл:** `src/verter_admin/launch/slam_toolbox.launch.py` (~200 строк)

**Структура:**

```python
def generate_launch_description():
    # Путь к конфигурации
    pkg_dir = get_package_share_directory('verter_admin')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam', 'slam_toolbox_params.yaml')

    # SLAM Toolbox нода
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        slam_toolbox_node,
    ])
```

---

## 📁 Созданные файлы

### Конвертер датчиков
1. **`src/verter_admin/distance_sensors/ultrasonic_to_laserscan_node.py`** - конвертер (~350 строк)

### SLAM конфигурация
2. **`src/verter_admin/config/slam/slam_toolbox_params.yaml`** - параметры SLAM (~70 строк)
3. **`src/verter_admin/launch/slam_toolbox.launch.py`** - launch файл (~200 строк)

### Обновленные файлы
4. **`setup.py`** - добавлена нода ultrasonic_to_laserscan_node и SLAM конфигурация

---

## 🚀 Как запустить полную систему

### Шаг 1: Запуск базовых компонентов

**Терминал 1: robot_state_publisher**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd /path/to/verter_admin

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro src/verter_admin/urdf/verter_robot_minimal.urdf)" \
  -p publish_frequency:=50.0
```

**Терминал 2: odometry_node**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

ros2 run verter_admin odometry_node
```

### Шаг 2: Запуск конвертера датчиков

**Терминал 3: ultrasonic_to_laserscan_node**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

ros2 run verter_admin ultrasonic_to_laserscan_node
```

**Проверка:**
```bash
# В другом терминале
ros2 topic echo /scan --once
```

**Ожидаемый результат:**
```yaml
header:
  frame_id: base_link
angle_min: -1.5707963267948966  # -π/2
angle_max: 1.5707963267948966   # +π/2
angle_increment: 0.00872664...
ranges: [0.5, 0.6, 0.7, ...]   # Массив расстояний в метрах
```

### Шаг 3: Запуск SLAM

**Терминал 4: SLAM Toolbox**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

ros2 launch verter_admin slam_toolbox.launch.py
```

**Проверка:**
```bash
# Проверка топиков
ros2 topic list | grep -E "(map|slam)"

# Должны быть:
# /map
# /slam_toolbox/graph_visualization
# /slam_toolbox/scan_visualization
```

### Шаг 4: Проверка TF дерева

```bash
# Проверка трансформации map -> odom
ros2 run tf2_ros tf2_echo map odom

# Ожидаемый результат:
# At time 0.0
# - Translation: [x, y, 0.000]
# - Rotation: in Quaternion [0.000, 0.000, z, w]
```

**Полное TF дерево:**
```
map (от SLAM)
  └─ odom (от SLAM)
       └─ base_footprint (от odometry_node)
            └─ base_link (от robot_state_publisher)
                 ├─ wheel_left
                 ├─ wheel_right
                 ├─ caster_back
                 └─ sensors (x7)
```

---

## 🧪 Тестирование построения карты

### Подготовка

1. ✅ Все 4 ноды запущены (robot_state_publisher, odometry, ultrasonic_to_laserscan, slam_toolbox)
2. ✅ Arduino подключен и датчики работают
3. ✅ Робот на ровной поверхности

### Процесс построения карты

**Шаг 1:** Запустите движение робота:
```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

**Шаг 2:** Наблюдайте за логами SLAM:
```
[slam_toolbox]: Message Filter dropping message: frame 'odom' at time 0.000
[slam_toolbox]: Registering sensor: [Scan]
[slam_toolbox]: ScanMatcher: Adding new scan at pose (x: 0.00, y: 0.00, theta: 0.00)
```

**Шаг 3:** Проверьте карту:
```bash
ros2 topic echo /map --once
```

**Шаг 4:** Визуализация в RViz (опционально):
```bash
rviz2

# Добавьте:
# - Fixed Frame: map
# - Add -> Map -> Topic: /map
# - Add -> LaserScan -> Topic: /scan
# - Add -> RobotModel
```

---

## 💾 Сохранение карты

### Вариант 1: Сервис SLAM Toolbox

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/path/to/my_map'}}"
```

### Вариант 2: Сериализованная карта (для продолжения SLAM)

```bash
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/path/to/my_map.posegraph'}"
```

### Вариант 3: map_saver (для Nav2)

```bash
# Установка (если не установлен)
conda install -c robostack-staging ros-humble-nav2-map-server

# Сохранение карты
ros2 run nav2_map_server map_saver_cli -f /path/to/my_map
```

**Результат:**
- `my_map.pgm` - изображение карты
- `my_map.yaml` - метаданные карты

---

## 📈 Что работает

| Компонент | Статус | Описание |
|-----------|--------|----------|
| **ultrasonic_to_laserscan_node** | ✅ | Конвертер датчиков создан |
| **Публикация /scan** | ✅ | LaserScan топик готов |
| **SLAM Toolbox** | ✅ | Версия 2.6.4 установлена |
| **SLAM конфигурация** | ✅ | Параметры настроены |
| **SLAM launch файл** | ✅ | slam_toolbox.launch.py готов |
| **Setup.py** | ✅ | Обновлен для установки |

---

## ⚠️ Известные ограничения

### 1. Ультразвуковые датчики vs Лидар

**Ограничения:**
- ❌ Низкая частота обновления (~10 Hz vs 10-40 Hz у лидара)
- ❌ Широкий угол обзора (конус вместо луча)
- ❌ Только 7 точек покрытия (vs 360-720 у лидара)
- ❌ Чувствительность к отражающим/поглощающим поверхностям

**Последствия:**
- Карты будут менее детальными
- Loop closure может работать хуже
- Требуется медленное движение робота

**Рекомендации:**
- Двигайтесь медленно (< 0.2 м/с)
- Делайте паузы на поворотах
- Избегайте зеркал и стекол

### 2. Погрешность одометрии

**Проблема:** Дифференциальный привод накапливает ошибку при поворотах.

**Решение:** SLAM с loop closure частично компенсирует дрифт.

**Рекомендация:** При создании карты проезжайте по замкнутым контурам для loop closure.

### 3. Требования к окружению

**Подходит:**
- ✅ Комнаты с мебелью
- ✅ Коридоры со стенами
- ✅ Закрытые помещения

**Не подходит:**
- ❌ Открытые пространства (парки, улицы)
- ❌ Помещения со стеклами/зеркалами
- ❌ Очень большие залы

---

## 🔧 Следующие шаги

### Немедленно (для запуска)
- [ ] Протестировать ultrasonic_to_laserscan_node с реальным роботом
- [ ] Проверить качество /scan данных
- [ ] Построить первую тестовую карту
- [ ] Сохранить карту для Nav2

### Краткосрочные
- [ ] Оптимизировать параметры SLAM под ультразвуковые датчики
- [ ] Создать launch файл для полной системы (одна команда)
- [ ] Настроить автосохранение карт
- [ ] Протестировать Nav2 с созданной картой

### Долгосрочные
- [ ] Рассмотреть добавление IMU для улучшения одометрии
- [ ] Возможно добавить RPLidar для улучшения SLAM
- [ ] Настроить RViz визуализацию
- [ ] Создать behavior trees для автономной навигации

---

## 📋 Проверочный список перед запуском

### Железо
- [ ] Arduino Mega подключен (devpath=1.4)
- [ ] 7 ультразвуковых датчиков подключены и работают
- [ ] Моторы подключены к chassis_node
- [ ] Робот на ровной поверхности

### Программное обеспечение
- [ ] Conda окружение ros_humble_310 активировано
- [ ] robot_state_publisher запущен
- [ ] odometry_node запущен и публикует /odom
- [ ] ultrasonic_to_laserscan_node запущен и публикует /scan
- [ ] slam_toolbox запущен

### Проверка топиков
```bash
ros2 topic list | grep -E "(odom|scan|map|tf)"
```

**Ожидаемый результат:**
```
/odom
/scan
/map
/tf
/tf_static
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
```

### Проверка TF
```bash
ros2 run tf2_tools view_frames
```

**Ожидаемая структура:**
```
map -> odom -> base_footprint -> base_link -> {sensors, wheels}
```

---

## 💡 Советы по использованию

### Для лучшего SLAM:

1. **Медленное движение:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.15}, angular: {z: 0.0}}"  # Не быстрее!
   ```

2. **Паузы на поворотах:**
   ```bash
   # Остановка
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once

   # Поворот
   ros2 topic pub /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.0}, angular: {z: 0.3}}"

   # Остановка после поворота
   ros2 topic pub /cmd_vel geometry_msgs/Twist "{}" --once
   ```

3. **Замкнутые маршруты:**
   - Создавайте маршруты с возвратом к началу
   - Это активирует loop closure detection
   - Улучшает точность карты

4. **Избегайте:**
   - Быстрых поворотов
   - Резких ускорений
   - Вибраций
   - Зеркал и стекол

---

## 🐛 Отладка проблем

### Проблема 1: Нет топика /scan

**Симптомы:**
```bash
ros2 topic echo /scan
# Ничего не выводится
```

**Решение:**
```bash
# Проверьте ноду
ros2 node list | grep ultrasonic

# Проверьте логи
ros2 run verter_admin ultrasonic_to_laserscan_node

# Проверьте Arduino
ros2 topic echo /verter_commands  # Должны быть команды от distance_sensors
```

### Проблема 2: SLAM не создает карту

**Симптомы:**
```bash
ros2 topic echo /map --once
# Пустая или не обновляется
```

**Причины:**
1. Робот не двигается
2. /scan не публикуется
3. /odom не публикуется
4. TF дерево не полное

**Решение:**
```bash
# Проверьте все топики
ros2 topic hz /scan    # Должен быть ~10 Hz
ros2 topic hz /odom    # Должен быть ~50 Hz
ros2 topic hz /map     # Должен обновляться при движении

# Проверьте TF
ros2 run tf2_ros tf2_echo map odom
```

### Проблема 3: Карта искажена

**Симптомы:** Стены кривые, комната неправильной формы

**Причины:**
- Слишком быстрое движение
- Проскальзывание колес
- Ошибки одометрии

**Решение:**
- Уменьшите скорость движения
- Проверьте, что колеса не проскальзывают
- Сделайте loop closure (вернитесь к началу)

---

## 📚 Полезные команды

### Мониторинг SLAM

```bash
# Просмотр статистики SLAM
ros2 topic echo /slam_toolbox/feedback

# Визуализация графа
ros2 topic echo /slam_toolbox/graph_visualization

# Проверка корректности сканов
ros2 topic echo /slam_toolbox/scan_visualization
```

### Управление картой

```bash
# Очистить текущую карту (начать заново)
ros2 service call /slam_toolbox/clear_queue slam_toolbox/srv/ClearQueue

# Пауза/возобновление SLAM
ros2 service call /slam_toolbox/pause_new_measurements slam_toolbox/srv/Pause
ros2 service call /slam_toolbox/resume slam_toolbox/srv/Resume
```

### Отладка датчиков

```bash
# Проверка сырых данных датчиков
ros2 topic echo /scan --field ranges

# Статистика частоты
ros2 topic hz /scan
ros2 topic hz /odom
```

---

## 🔗 Связанные файлы

- **Конвертер датчиков:** `src/verter_admin/distance_sensors/ultrasonic_to_laserscan_node.py`
- **SLAM конфигурация:** `src/verter_admin/config/slam/slam_toolbox_params.yaml`
- **SLAM launch файл:** `src/verter_admin/launch/slam_toolbox.launch.py`
- **Setup.py:** Обновлен (строки 85, 113)
- **Одометрия:** [01_odometry_testing.md](01_odometry_testing.md)
- **URDF:** [02_urdf_testing.md](02_urdf_testing.md)
- **Nav2:** [03_nav2_setup.md](03_nav2_setup.md)

---

## 📚 Полезные ссылки

- [SLAM Toolbox Documentation](https://github.com/SteveMacenski/slam_toolbox)
- [ROS2 Nav2 Documentation](https://navigation.ros.org/)
- [LaserScan Message Format](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)
- [TF2 Tutorials](http://wiki.ros.org/tf2/Tutorials)

---

**Автор:** AI Assistant + Oleksandr Karpachov
**Последнее обновление:** 18 октября 2025, 20:15
**Статус этапа:** ✅ ЗАВЕРШЕНО (конфигурация)

**Следующий этап:** Тестирование и интеграция полной системы навигации ⏭️
