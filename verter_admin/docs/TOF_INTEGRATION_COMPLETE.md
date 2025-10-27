# ✅ ToF камера интегрирована с Nav2

**Дата:** 26 октября 2025
**Статус:** Готово к тестированию

---

## Что сделано

### 1. ✅ ToF камера добавлена в URDF
**Файл:** `src/verter_admin/urdf/verter_robot_minimal.urdf`

Добавлены links:
- `camera_tof` - физический корпус камеры (серый box 30×60×20мм)
- `camera_tof_optical` - оптический фрейм (для данных PointCloud2)

Позиция: `xyz="0.150 0 0.400"` (150мм вперед от центра, 400мм высота)

### 2. ✅ ToF нода публикует PointCloud2
**Файл:** `src/verter_admin/tof_camera/tof_camera_node.py`

**Добавлено:**
- Конвертация depth image → PointCloud2
- Фильтрация по диапазону глубины (0.1-5.0м)
- Субдискретизация (каждый 2-й пиксель) для производительности
- Публикация в топик `/camera/depth/points`
- Frame ID: `camera_tof_optical`

**Параметры:**
- `fps`: 15 (по умолчанию)
- `depth_min`: 0.1м
- `depth_max`: 5.0м
- `fov_h`: 60° (горизонтальный FOV)
- `fov_v`: 45° (вертикальный FOV)
- `downsample`: 2 (каждый 2-й пиксель)

### 3. ✅ Nav2 настроен для PointCloud2
**Файл:** `src/verter_admin/config/nav2/nav2_params.yaml`

**Изменения в `local_costmap/voxel_layer`:**

```yaml
observation_sources: scan pointcloud

# HC-SR04 датчики (LaserScan)
scan:
  sensor_frame: base_link
  data_type: LaserScan
  topic: /scan
  max_obstacle_height: 0.8

# ToF камера (PointCloud2)
pointcloud:
  sensor_frame: camera_tof_optical
  data_type: PointCloud2
  topic: /camera/depth/points
  min_obstacle_height: 0.1
  max_obstacle_height: 0.8
  obstacle_max_range: 2.5
```

### 4. ✅ ToF камера добавлена в базовый launch
**Файл:** `src/verter_admin/launch/verter_base.launch.py`

Добавлена нода `tof_camera_node` с оптимальными параметрами.

---

## Как запустить

### Полная система (робот + ToF + Nav2):

```bash
# Терминал 1: Базовые компоненты (включая ToF)
cd ~/Study_Projects/verder/verter-robot/verter_admin
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
ros2 launch verter_admin verter_base.launch.py
```

### Проверка что ToF работает:

```bash
# Проверить топики
ros2 topic list | grep camera
# Должен быть: /camera/depth/points

# Проверить частоту публикации
ros2 topic hz /camera/depth/points
# Должно быть: ~15 Hz

# Посмотреть данные
ros2 topic echo /camera/depth/points --once
```

### Визуализация в RViz:

```bash
rviz2
```

**Настройка RViz:**
1. Fixed Frame → `base_link`
2. Add → RobotModel (увидите камеру на роботе)
3. Add → TF (увидите `camera_tof` и `camera_tof_optical`)
4. Add → PointCloud2 → Topic: `/camera/depth/points`
   - Size: 0.01
   - Color Transformer: AxisColor или Intensity

---

## Запуск с Nav2

```bash
# Терминал 1: Базовые компоненты + ToF
ros2 launch verter_admin verter_base.launch.py

# Терминал 2: SLAM
ros2 launch verter_admin slam_toolbox.launch.py

# Терминал 3: Nav2
ros2 launch verter_admin nav2_bringup.launch.py

# Терминал 4: RViz для визуализации
rviz2
```

**В RViz добавьте:**
- Map (`/map` - от SLAM)
- LocalCostmap (`/local_costmap/costmap`)
- GlobalCostmap (`/global_costmap/costmap`)
- PointCloud2 (`/camera/depth/points`)
- LaserScan (`/scan`)
- Path (`/plan` - планируемый путь)

---

## Преимущества интеграции

### До (только HC-SR04):
- ❌ 7 лучей горизонтально
- ❌ Не видит объекты на разной высоте (столы, стулья)
- ❌ Пропускает узкие препятствия между датчиками

### После (HC-SR04 + ToF):
- ✅ Тысячи точек от ToF + 7 лучей от датчиков
- ✅ Обнаруживает объекты на высоте 0.1-0.8м
- ✅ Плотное облако точек - не пропускает препятствия
- ✅ Лучше объезжает мебель и людей
- ✅ Более детальная costmap для Nav2

---

## Следующие шаги (тестирование)

### 1. Тест на столе (без движения):
```bash
ros2 launch verter_admin verter_base.launch.py
```
- Проверить что ToF публикует данные
- Визуализировать в RViz
- Поместить препятствие перед камерой - должно появиться в облаке точек

### 2. Тест с SLAM (с движением):
```bash
# Запустить base + SLAM
ros2 launch verter_admin verter_base.launch.py &
ros2 launch verter_admin slam_toolbox.launch.py

# Двигать робота и смотреть как ToF помогает строить карту
```

### 3. Тест с Nav2 (автономная навигация):
```bash
# Запустить всю систему
ros2 launch verter_admin verter_navigation.launch.py

# Дать цель в RViz (2D Nav Goal)
# Проверить что робот объезжает препятствия используя ToF
```

---

## Устранение проблем

### ToF не публикует данные:
```bash
# Проверить что камера подключена
ls /dev/video*

# Проверить логи ноды
ros2 topic echo /rosout | grep tof_camera
```

### PointCloud2 пустой:
- Проверить что `depth_min` и `depth_max` настроены правильно
- Убедиться что перед камерой есть объекты в диапазоне 0.1-5.0м

### Nav2 не использует ToF данные:
- Проверить что TF `base_link` → `camera_tof_optical` публикуется
- Убедиться что в `nav2_params.yaml` правильно указан `sensor_frame`

---

## Параметры для тонкой настройки

### Если ToF потребляет много CPU:
```yaml
fps: 10  # Уменьшить с 15 до 10
downsample: 3  # Увеличить с 2 до 3 (каждый 3-й пиксель)
```

### Если нужна большая дальность:
```yaml
depth_max: 8.0  # Увеличить до 8 метров
```

### Если нужно фильтровать землю:
```yaml
min_obstacle_height: 0.15  # Увеличить с 0.1 до 0.15м
```

---

**Готово к использованию!** 🎉

Все компоненты интегрированы, осталось только протестировать на реальном роботе.
