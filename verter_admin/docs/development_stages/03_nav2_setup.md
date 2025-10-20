# 🧭 Этап 3: Настройка Nav2 Navigation Stack

**Дата:** 18 октября 2025
**Статус:** ✅ ЗАВЕРШЕНО (конфигурация)
**Предыдущий этап:** [02_urdf_testing.md](02_urdf_testing.md)

---

## 🎯 Цель этапа

Установить и настроить Nav2 (ROS2 Navigation Stack) для автономной навигации робота Verter, используя одометрию и датчики расстояния.

---

## 📋 Задачи

- [x] Установить Nav2 пакеты в conda окружение
- [x] Создать конфигурационный файл nav2_params.yaml
- [x] Настроить параметры под робот Verter
- [x] Создать launch файл для запуска Nav2
- [x] Обновить setup.py для установки конфигурации
- [ ] Создать конвертер датчиков (ultrasonic → LaserScan)
- [ ] Создать или получить карту окружения
- [ ] Протестировать полную навигацию

---

## 📝 Что такое Nav2?

**Nav2** (Navigation2) - это полный стек автономной навигации для ROS2, который позволяет роботу:
- Планировать путь от точки A к точке B
- Объезжать препятствия
- Следовать по заданной траектории
- Восстанавливаться при застревании

### Основные компоненты Nav2

1. **BT Navigator** - управляет навигацией через поведенческие деревья (Behavior Trees)
2. **Planner Server** - глобальное планирование пути на карте
3. **Controller Server** - локальное управление движением и объезд препятствий
4. **Behavior Server** - поведения восстановления (развороты, откаты)
5. **Costmap 2D** - карты стоимости для планирования (local и global)
6. **Lifecycle Manager** - управление жизненным циклом нод

---

## 🔧 Процесс установки и настройки

### Шаг 1: Установка Nav2 в conda окружение

**Поиск доступных версий:**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

# Поиск Nav2 пакетов
conda search ros-humble-navigation2
```

**Найденные версии:**
```
ros-humble-navigation2  1.1.5  py310h5aa156f_6  robostack-staging
ros-humble-navigation2  1.1.6  py310h5aa156f_6  robostack-staging
```

**Установка (выбрана стабильная версия 1.1.5):**
```bash
conda install -c robostack-staging ros-humble-navigation2=1.1.5
```

**Установленные пакеты (15+ компонентов):**
```
ros-humble-nav2-amcl
ros-humble-nav2-behavior-tree
ros-humble-nav2-behaviors
ros-humble-nav2-bt-navigator
ros-humble-nav2-common
ros-humble-nav2-controller
ros-humble-nav2-core
ros-humble-nav2-costmap-2d
ros-humble-nav2-lifecycle-manager
ros-humble-nav2-map-server
ros-humble-nav2-msgs
ros-humble-nav2-planner
ros-humble-nav2-smoother
ros-humble-nav2-util
ros-humble-nav2-velocity-smoother
ros-humble-nav2-waypoint-follower
ros-humble-navigation2
```

✅ **Установка успешна!**

---

### Шаг 2: Создание конфигурации nav2_params.yaml

**Файл:** `src/verter_admin/config/nav2/nav2_params.yaml`

**Основные разделы конфигурации:**

#### 1. BT Navigator (Поведенческое дерево)
```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: False
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
```

#### 2. Controller Server (Локальный планировщик)
```yaml
controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      max_vel_x: 0.5      # 0.5 м/с линейная скорость
      max_vel_theta: 1.0   # 1.0 рад/с угловая скорость
      acc_lim_x: 1.0
      acc_lim_theta: 3.2
```

**Почему DWB?** Dynamic Window Approach (DWB) идеально подходит для дифференциального привода.

#### 3. Local Costmap (Локальная карта препятствий)
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true    # Двигается вместе с роботом
      width: 3                # 3 метра
      height: 3
      resolution: 0.05        # 5 см точность
      robot_radius: 0.35      # Радиус робота (для 0.40x0.50м)

      plugins: ["voxel_layer", "inflation_layer"]

      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        observation_sources: scan
        scan:
          sensor_frame: base_link
          data_type: LaserScan
          topic: /scan          # ⚠️ Требуется конвертер!
          marking: true
          clearing: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55  # Зона безопасности вокруг препятствий
```

#### 4. Global Costmap (Глобальная карта)
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map       # ⚠️ Требуется карта!
      robot_base_frame: base_link
      robot_radius: 0.35
      resolution: 0.05

      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
```

#### 5. Planner Server (Глобальный планировщик)
```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: false        # Dijkstra по умолчанию
      allow_unknown: true     # Может планировать через неизвестные области
```

#### 6. Behavior Server (Восстановление)
```yaml
behavior_server:
  ros__parameters:
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]

    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"

    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

#### 7. Velocity Smoother (Сглаживание скорости)
```yaml
velocity_smoother:
  ros__parameters:
    smoothing_frequency: 20.0
    max_velocity: [0.5, 0.0, 1.0]    # [vx, vy, omega]
    min_velocity: [-0.5, 0.0, -1.0]
    max_accel: [1.0, 0.0, 3.2]
    max_decel: [-1.0, 0.0, -3.2]
```

---

### Шаг 3: Создание launch файла nav2_bringup.launch.py

**Файл:** `src/verter_admin/launch/nav2_bringup.launch.py`

**Структура launch файла:**

```python
def generate_launch_description():
    # Путь к конфигурации
    pkg_dir = get_package_share_directory('verter_admin')
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2', 'nav2_params.yaml')

    # Альтернативный путь для разработки (если пакет не установлен)
    # nav2_params_file = os.path.join(
    #     os.path.dirname(__file__),
    #     '..', 'config', 'nav2', 'nav2_params.yaml'
    # )

    # Launch аргументы
    use_sim_time = LaunchConfiguration('use_sim_time')  # default: False
    autostart = LaunchConfiguration('autostart')        # default: True

    # Создание нод Nav2
    return LaunchDescription([
        # Аргументы
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='True'),

        # Nav2 ноды
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,
    ])
```

**Запущенные ноды:**
1. `controller_server` - локальный контроллер (DWB)
2. `planner_server` - глобальный планировщик (NavFn)
3. `behavior_server` - поведения восстановления
4. `bt_navigator` - навигатор на основе BT
5. `waypoint_follower` - следование по путевым точкам
6. `velocity_smoother` - сглаживание команд скорости
7. `lifecycle_manager_navigation` - управление жизненным циклом

**Remapping топиков:**
```python
remappings=[
    ('/tf', 'tf'),
    ('/tf_static', 'tf_static'),
    ('cmd_vel', 'cmd_vel')           # Выход контроллера
]

# Для velocity_smoother:
[('cmd_vel', 'cmd_vel_nav'),         # Вход от навигатора
 ('cmd_vel_smoothed', 'cmd_vel')]    # Выход к роботу
```

---

### Шаг 4: Обновление setup.py

Добавлена установка Nav2 конфигурации:

```python
data_files=[
    # ... другие файлы ...

    # Включаем URDF модели робота
    (os.path.join('share', package_name, 'urdf'),
     glob(os.path.join('src/verter_admin/urdf', '*.urdf'))),

    # Включаем конфигурацию Nav2
    (os.path.join('share', package_name, 'config', 'nav2'),
     glob(os.path.join('src/verter_admin/config/nav2', '*.yaml'))),

    # ...
]
```

---

## 📊 Созданные файлы

### Конфигурация
1. **`src/verter_admin/config/nav2/nav2_params.yaml`** - полная конфигурация Nav2 (~350 строк)

### Launch файлы
2. **`src/verter_admin/launch/nav2_bringup.launch.py`** - запуск Nav2 stack (~330 строк)

### Обновленные файлы
3. **`setup.py`** - добавлена установка Nav2 конфигурации
4. **`docs/development_stages/README.md`** - обновлен прогресс (40%)

---

## ⚠️ Важные зависимости и требования

### Для работы Nav2 требуется:

#### 1. TF дерево ✅
```
map → odom → base_link → sensors/wheels
```

**Статус:** ✅ Есть `odom → base_link` (одометрия работает)
**Проблема:** ❌ Нет `map → odom` (требуется SLAM или статическая карта)

#### 2. Топик /odom ✅
**Тип:** `nav_msgs/Odometry`
**Статус:** ✅ Публикуется odometry_node

#### 3. Топик /scan ❌
**Тип:** `sensor_msgs/LaserScan`
**Статус:** ❌ НЕТ - есть только 7 ультразвуковых датчиков
**Решение:** Нужно создать конвертер ultrasonic → LaserScan

#### 4. Карта окружения ❌
**Тип:** `nav_msgs/OccupancyGrid`
**Топик:** `/map`
**Статус:** ❌ НЕТ - нужно либо:
  - Запустить SLAM для создания карты (slam_toolbox)
  - Загрузить готовую карту (map_server)
  - Работать только с local costmap (без глобального планирования)

#### 5. robot_description ✅
**Статус:** ✅ URDF модель готова, robot_state_publisher работает

---

## 🚀 Как запустить Nav2 (когда все готово)

### Вариант 1: Полный запуск

**Требования:**
- ✅ Одометрия работает (`odometry_node`)
- ✅ URDF опубликован (`robot_state_publisher`)
- ⚠️ Есть `/scan` топик (нужен конвертер датчиков)
- ⚠️ Есть карта (`map_server` или SLAM)

**Команда:**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd /path/to/verter_admin

ros2 launch verter_admin nav2_bringup.launch.py
```

### Вариант 2: Только с локальным планированием

Если нет карты, можно работать только с local costmap (объезд препятствий без глобального пути):

**Изменения в nav2_params.yaml:**
```yaml
# Отключить global costmap
global_costmap:
  global_costmap:
    ros__parameters:
      always_send_full_costmap: False
      plugins: []  # Пустой список плагинов
```

---

## 📈 Что уже работает

| Компонент | Статус | Описание |
|-----------|--------|----------|
| **Nav2 установка** | ✅ | Версия 1.1.5 установлена в conda |
| **Конфигурация** | ✅ | nav2_params.yaml создан |
| **Launch файл** | ✅ | nav2_bringup.launch.py готов |
| **Setup.py** | ✅ | Обновлен для установки конфига |
| **Параметры робота** | ✅ | Настроены под Verter (размеры, скорости) |

---

## ⚙️ Что НЕ работает (требует доработки)

| Компонент | Статус | Что нужно сделать |
|-----------|--------|-------------------|
| **Топик /scan** | ❌ | Создать конвертер ultrasonic → LaserScan |
| **Карта** | ❌ | Настроить SLAM или загрузить статическую карту |
| **TF: map→odom** | ❌ | Будет создан SLAM или AMCL |
| **Тестирование** | ❌ | Протестировать с реальным роботом |

---

## 🔧 Следующие шаги

### Краткосрочные (необходимо)

#### 1. Создать конвертер датчиков ультразвука → LaserScan

**Файл:** `src/verter_admin/distance_sensors/ultrasonic_to_laserscan.py`

**Что делает:**
- Подписывается на 7 топиков ультразвуковых датчиков
- Объединяет их в один топик `/scan` (LaserScan)
- Преобразует углы и расстояния

**Пример структуры:**
```python
class UltrasonicToLaserScan(Node):
    def __init__(self):
        super().__init__('ultrasonic_to_laserscan')

        # Подписка на датчики
        self.sensors = {
            'front_center': (-90°, sensor_topic_1),
            'front_left_inner': (-25°, sensor_topic_2),
            # ... и т.д.
        }

        # Публикация LaserScan
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
```

#### 2. Настроить SLAM (slam_toolbox)

**Установка:**
```bash
conda install -c robostack-staging ros-humble-slam-toolbox
```

**Запуск:**
```bash
ros2 launch slam_toolbox online_async_launch.py
```

**Результат:** Создаст топики `/map` и TF `map → odom`

---

### Долгосрочные (по желанию)

- [ ] Настроить AMCL для локализации на готовой карте
- [ ] Добавить визуализацию в RViz
- [ ] Создать behavior trees для сложных сценариев
- [ ] Настроить recovery behaviors (что делать при застревании)
- [ ] Интеграция с IMU (когда будет установлен)
- [ ] Оптимизация параметров DWB для вашего робота

---

## 📋 Проверка конфигурации

### Проверка установленных пакетов
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

# Проверка Nav2 пакетов
conda list | grep nav2
```

**Ожидаемый результат:** 15+ пакетов `ros-humble-nav2-*`

### Проверка синтаксиса launch файла
```bash
python3 -m py_compile src/verter_admin/launch/nav2_bringup.launch.py
```

**Ожидаемый результат:** Без ошибок

### Проверка файлов конфигурации
```bash
ls -la src/verter_admin/config/nav2/
```

**Ожидаемый результат:**
```
nav2_params.yaml
```

---

## 🐛 Известные проблемы

### 1. Пакет не установлен (при запуске launch)
**Ошибка:**
```
Package 'verter_admin' not found
```

**Решение:** Раскомментировать альтернативный путь в launch файле:
```python
# В nav2_bringup.launch.py строки 71-74
nav2_params_file = os.path.join(
    os.path.dirname(__file__),
    '..', 'config', 'nav2', 'nav2_params.yaml'
)
```

### 2. Нет топика /scan
**Ошибка:**
```
[voxel_layer]: No messages received on topic /scan
```

**Решение:** Создать конвертер ultrasonic → LaserScan (см. Следующие шаги)

### 3. Нет карты
**Ошибка:**
```
[global_costmap]: Timed out waiting for transform from base_link to map
```

**Решение:**
- Вариант 1: Запустить SLAM (slam_toolbox)
- Вариант 2: Отключить global_costmap и работать только локально

---

## 📚 Полезные команды

### Проверка запущенных нод Nav2
```bash
ros2 node list | grep -E "(controller|planner|navigator|behavior)"
```

**Должны быть:**
```
/controller_server
/planner_server
/bt_navigator
/behavior_server
/waypoint_follower
/velocity_smoother
/lifecycle_manager_navigation
```

### Проверка топиков Nav2
```bash
ros2 topic list | grep -E "(costmap|cmd_vel|path)"
```

**Должны быть:**
```
/cmd_vel
/cmd_vel_nav
/local_costmap/costmap
/global_costmap/costmap
/plan
/local_plan
```

### Проверка состояния lifecycle нод
```bash
ros2 lifecycle list
```

**Все ноды должны быть:** `active [3]`

### Отправка цели навигации (когда все работает)
```bash
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "
header:
  frame_id: 'map'
pose:
  position:
    x: 1.0
    y: 0.5
    z: 0.0
  orientation:
    w: 1.0
"
```

---

## 💡 Технические детали

### Выбор DWB контроллера

**Почему DWB, а не другие?**

1. **DWB (Dynamic Window Approach):**
   - ✅ Идеально для дифференциального привода
   - ✅ Хорошо работает с низкими скоростями
   - ✅ Много настраиваемых critics (GoalAlign, PathAlign, PathDist, etc.)

2. **Альтернативы:**
   - TEB (Timed Elastic Band) - сложнее настроить
   - RPP (Regulated Pure Pursuit) - для высоких скоростей
   - MPPI - требует больше вычислений

### Параметры скорости

Настроены согласно URDF и реальным возможностям робота:

```yaml
max_vel_x: 0.5        # Максимум 0.5 м/с (36 м/мин)
max_vel_theta: 1.0    # Максимум 1.0 рад/с (~57°/с)
acc_lim_x: 1.0        # Ускорение 1.0 м/с²
acc_lim_theta: 3.2    # Угловое ускорение 3.2 рад/с²
```

### Размеры costmap

**Local costmap:**
- Размер: 3x3 метра
- Rolling window (двигается с роботом)
- Разрешение: 5 см
- Обновление: 5 Hz

**Global costmap:**
- Покрывает всю карту
- Статический
- Разрешение: 5 см
- Обновление: 1 Hz

### Inflation radius

```yaml
inflation_radius: 0.55  # 55 см от препятствий
```

**Расчет:**
- Радиус робота: ~0.35 м (для робота 0.40x0.50 м)
- Зона безопасности: +0.20 м
- Итого: 0.55 м

---

## 📝 Заметки для дальнейшей разработки

### Конвертер датчиков

**Задача:** Создать ноду, которая преобразует 7 ультразвуковых датчиков в `/scan` (LaserScan).

**Особенности:**
- Ультразвук имеет конус обзора (~15-30°)
- LaserScan ожидает узкий луч (<1°)
- Нужно правильно распределить показания по углам

**Углы датчиков (из URDF):**
- Front center: 0°
- Front left inner: ~25°
- Front left outer: ~45°
- Front right inner: ~-25°
- Front right outer: ~-45°
- Left: 90°
- Right: -90°

### SLAM vs Static Map

**SLAM (slam_toolbox):**
- ✅ Создает карту автоматически
- ✅ Обновляется в реальном времени
- ❌ Требует движения робота
- ❌ Больше вычислительных ресурсов

**Static Map (map_server):**
- ✅ Быстрая загрузка
- ✅ Стабильная карта
- ❌ Нужно заранее создать
- ❌ Не адаптируется к изменениям

**Рекомендация:** Начать со SLAM, затем сохранить карту для быстрой загрузки.

---

## 🔗 Связанные файлы

- **Конфигурация Nav2:** `src/verter_admin/config/nav2/nav2_params.yaml`
- **Launch файл:** `src/verter_admin/launch/nav2_bringup.launch.py`
- **Setup.py:** `setup.py` (строки 82-83)
- **URDF модель:** `src/verter_admin/urdf/verter_robot_minimal.urdf`
- **Одометрия:** [01_odometry_testing.md](01_odometry_testing.md)
- **URDF тестирование:** [02_urdf_testing.md](02_urdf_testing.md)

---

## 📚 Полезные ссылки

- [Nav2 Documentation](https://navigation.ros.org/)
- [Nav2 Configuration Guide](https://navigation.ros.org/configuration/index.html)
- [DWB Controller](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html)
- [Costmap 2D](https://navigation.ros.org/configuration/packages/configuring-costmaps.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)

---

**Автор:** AI Assistant + Oleksandr Karpachov
**Последнее обновление:** 18 октября 2025, 19:30
**Статус этапа:** ✅ ЗАВЕРШЕНО (конфигурация)

**Следующий этап:** Создание конвертера датчиков и настройка SLAM ⏭️
