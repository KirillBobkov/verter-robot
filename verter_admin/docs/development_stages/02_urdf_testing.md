# 📐 Этап 2: URDF Модель робота - Тестирование и настройка

**Дата:** 18 октября 2025
**Статус:** ✅ ЗАВЕРШЕНО
**Предыдущий этап:** [01_odometry_testing.md](01_odometry_testing.md)

---

## 🎯 Цель этапа

Создать и протестировать URDF (Unified Robot Description Format) модель робота Verter для использования в Nav2 и визуализации в RViz.

---

## 📋 Задачи

- [x] Создать URDF модель с реальными параметрами робота
- [x] Настроить запуск robot_state_publisher
- [x] Исправить проблемы conda окружения (sdformat plugin)
- [x] Проверить TF трансформации всех частей робота
- [x] Создать минимальную версию URDF без кириллицы

---

## 📝 Что такое URDF?

**URDF** (Unified Robot Description Format) - это XML формат для описания:
- Физических размеров робота (геометрия)
- Расположения колес, датчиков, камер
- TF трансформаций между частями робота
- Визуальной и collision моделей

### Зачем нужен URDF?

1. **Nav2** использует URDF для:
   - Понимания размеров робота (footprint)
   - Планирования пути с учетом габаритов
   - Collision detection

2. **RViz** использует для:
   - 3D визуализации робота
   - Отображения TF дерева

3. **robot_state_publisher** публикует TF трансформации из URDF

---

## 🤖 Параметры робота Verter

### Конфигурация привода
- **Тип:** Дифференциальный привод
- **Ведущие колеса:** 2 шт., СПЕРЕДИ робота
- **Опорное колесо:** 1 шт., СЗАДИ (крутящееся)
- **Моторы:** Коллекторные с редуктором, 12В

### Размеры колес (реальные)
- **Диаметр:** 200 мм (радиус 0.1 м)
- **Ширина:** 50 мм (0.05 м)
- **Позиция:** X = 0.10 м (спереди от центра)
- **Расстояние между колесами:** ~0.22 м (wheelbase)

### Размеры корпуса (пользовательские)
- **Длина:** 0.40 м (40 см)
- **Ширина:** 0.50 м (50 см)
- **Высота:** 1.50 м (150 см)

### Датчики расстояния (7 штук)
- **5 датчиков СПЕРЕДИ:** разные углы обзора (-45°, -25°, 0°, +25°, +45°)
- **2 датчика ПО БОКАМ:** 90° влево/вправо
- **Тип:** Ультразвуковые датчики

### IMU (планируется)
- **Статус:** На этапе подбора, не установлен
- **Компоненты:** Гироскоп, акселерометр, магнитометр

---

## 🛠️ Процесс создания и тестирования

### Шаг 1: Создание базовой URDF модели

Создан файл `src/verter_admin/urdf/verter_robot.urdf` с:
- Base link (корпус робота)
- 2 ведущих колеса (continuous joints)
- Опорное колесо (fixed joint)
- 7 датчиков расстояния (fixed joints)
- Base footprint для Nav2

### Шаг 2: Обновление размеров

Обновлены размеры корпуса согласно указаниям пользователя:
```xml
<box size="0.40 0.50 1.50"/>  <!-- Длина Ширина Высота -->
```

### Шаг 3: Проблемы при запуске

#### Проблема 1: XML парсинг с кириллицей
**Ошибка:**
```
XML parsing error: not well-formed (invalid token): line 8, column 2
```

**Причина:** Xacro не может корректно обработать длинные комментарии с кириллицей

**Решение:** Создана минимальная версия URDF (`verter_robot_minimal.urdf`) без обширных комментариев

#### Проблема 2: Плагин sdformat_urdf_plugin
**Ошибка:**
```
Could not find library corresponding to plugin sdformat_urdf_plugin/SDFormatURDFParser
```

**Причина:** На macOS pluginlib искал `.dylib`, а файл был `.so`

**Решение:**
```bash
ln -sf ~/miniforge3/envs/ros_humble_310/lib/libsdformat_urdf_plugin.so \
       ~/miniforge3/envs/ros_humble_310/lib/libsdformat_urdf_plugin.dylib
```

### Шаг 4: Запуск robot_state_publisher

**Команда для запуска:**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro src/verter_admin/urdf/verter_robot_minimal.urdf)" \
  -p publish_frequency:=50.0
```

**Результат:**
```
[INFO] [robot_state_publisher]: got segment base_footprint
[INFO] [robot_state_publisher]: got segment base_link
[INFO] [robot_state_publisher]: got segment caster_back
[INFO] [robot_state_publisher]: got segment sensor_front_center
[INFO] [robot_state_publisher]: got segment sensor_front_left_inner
[INFO] [robot_state_publisher]: got segment sensor_front_left_outer
[INFO] [robot_state_publisher]: got segment sensor_front_right_inner
[INFO] [robot_state_publisher]: got segment sensor_front_right_outer
[INFO] [robot_state_publisher]: got segment sensor_left
[INFO] [robot_state_publisher]: got segment sensor_right
[INFO] [robot_state_publisher]: got segment wheel_left
[INFO] [robot_state_publisher]: got segment wheel_right
```

✅ **Все 12 сегментов загружены успешно!**

---

## ✅ Проверка TF трансформаций

### Проверка опубликованных топиков
```bash
ros2 topic list | grep -E '(robot_description|tf)'
```

**Результат:**
```
/robot_description
/tf
/tf_static
```

### Проверка статических трансформаций
```bash
ros2 topic echo /tf_static --once
```

**Пример вывода (sensor_front_center):**
```yaml
header:
  frame_id: base_link
child_frame_id: sensor_front_center
transform:
  translation:
    x: 0.16
    y: 0.0
    z: 0.08
  rotation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### Проверка конкретной трансформации
```bash
ros2 run tf2_ros tf2_echo base_link sensor_front_center
```

**Результат:**
```
At time 0.0
- Translation: [0.160, 0.000, 0.080]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (degree) [0.000, -0.000, 0.000]
- Matrix:
  1.000  0.000  0.000  0.160
  0.000  1.000  0.000  0.000
  0.000  0.000  1.000  0.080
  0.000  0.000  0.000  1.000
```

✅ **TF трансформации работают корректно!**

---

## 📊 Структура TF дерева

```
odom (от odometry_node)
  └─ base_footprint (на уровне земли, z=0)
       └─ base_link (поднят на высоту колеса, z=0.1)
            ├─ wheel_left (X=0.10, Y=0.11)
            ├─ wheel_right (X=0.10, Y=-0.11)
            ├─ caster_back (X=-0.12, Y=0, Z=-0.07)
            ├─ sensor_front_center (X=0.16, Y=0, Z=0.08)
            ├─ sensor_front_left_inner (X=0.15, Y=0.05, Z=0.08)
            ├─ sensor_front_left_outer (X=0.14, Y=0.10, Z=0.08)
            ├─ sensor_front_right_inner (X=0.15, Y=-0.05, Z=0.08)
            ├─ sensor_front_right_outer (X=0.14, Y=-0.10, Z=0.08)
            ├─ sensor_left (X=0, Y=0.11, Z=0.08)
            └─ sensor_right (X=0, Y=-0.11, Z=0.08)
```

---

## 📁 Созданные файлы

### URDF модели
1. **`src/verter_admin/urdf/verter_robot.urdf`** - полная URDF с комментариями (проблемы с xacro)
2. **`src/verter_admin/urdf/verter_robot_minimal.urdf`** - минимальная рабочая версия ✅
3. **`src/verter_admin/urdf/verter_robot_full.urdf`** - бэкап полной версии

### Скрипты запуска
1. **`src/verter_admin/launch/test_urdf.py`** - Python скрипт
2. **`src/verter_admin/launch/start_robot_state_publisher.sh`** - Bash скрипт с xacro
3. **`src/verter_admin/launch/simple_robot_state.sh`** - упрощенный скрипт

### Launch файлы (из предыдущего этапа)
- **`src/verter_admin/launch/robot_description.launch.py`** - стандартный ROS2 launch файл

---

## 🔧 Исправления для conda окружения

### Проблема с плагином sdformat_urdf_plugin

**Файл:** `~/miniforge3/envs/ros_humble_310/lib/libsdformat_urdf_plugin.so`

**Исправление (однократно):**
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310

ln -sf ~/miniforge3/envs/ros_humble_310/lib/libsdformat_urdf_plugin.so \
       ~/miniforge3/envs/ros_humble_310/lib/libsdformat_urdf_plugin.dylib
```

**Проверка:**
```bash
ls -la ~/miniforge3/envs/ros_humble_310/lib/libsdformat_urdf_plugin.*
```

**Ожидаемый результат:**
```
libsdformat_urdf_plugin.dylib -> .../libsdformat_urdf_plugin.so
libsdformat_urdf_plugin.so
```

---

## 📈 Результаты тестирования

| Компонент | Статус | Описание |
|-----------|--------|----------|
| **URDF модель** | ✅ | Создана и загружена |
| **robot_state_publisher** | ✅ | Запущен и работает |
| **TF дерево** | ✅ | 12 фреймов опубликованы |
| **base_footprint** | ✅ | На уровне земли |
| **base_link** | ✅ | Поднят на 0.1м |
| **Колеса** | ✅ | Спереди, правильные размеры |
| **Опорное колесо** | ✅ | Сзади |
| **Датчики (7 шт)** | ✅ | Все опубликованы |
| **/tf_static** | ✅ | Публикуется |
| **/robot_description** | ✅ | Публикуется |

---

## 🚀 Как запустить для тестирования

### Вариант 1: Прямой запуск (рекомендуется)
```bash
# Терминал 1: Активация окружения
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd /путь/к/verter_admin

# Запуск robot_state_publisher
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -p robot_description:="$(xacro src/verter_admin/urdf/verter_robot_minimal.urdf)" \
  -p publish_frequency:=50.0
```

### Вариант 2: Через скрипт
```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate ros_humble_310
cd /путь/к/verter_admin

bash src/verter_admin/launch/start_robot_state_publisher.sh
```

### Проверка работы

**Проверка топиков:**
```bash
ros2 topic list
# Должны быть: /tf, /tf_static, /robot_description
```

**Проверка TF трансформации:**
```bash
ros2 run tf2_ros tf2_echo base_link sensor_front_center
```

**Проверка статических трансформаций:**
```bash
ros2 topic echo /tf_static --once
```

---

## ⚠️ Известные проблемы и ограничения

### 1. Полная URDF не работает с xacro
**Проблема:** `verter_robot.urdf` с обширными комментариями вызывает ошибки парсинга
**Решение:** Использовать `verter_robot_minimal.urdf`

### 2. Плагин требует dylib на macOS
**Проблема:** conda пакет содержит `.so` вместо `.dylib`
**Решение:** Создан симлинк (выполнить один раз)

### 3. Отсутствует inertia для caster_back
**Предупреждение:**
```
Error: Inertial element must have inertia element
```
**Влияние:** Только предупреждение, не критично для robot_state_publisher
**Статус:** Исправлено в minimal версии

### 4. Размеры требуют уточнения
**Статус:** Использованы примерные значения
**TODO:** Измерить реальные размеры корпуса и точные позиции датчиков

---

## 📝 Следующие шаги

### Немедленно
- [x] ✅ Запустить robot_state_publisher
- [x] ✅ Проверить TF трансформации
- [x] ✅ Создать документацию

### Краткосрочные (по желанию)
- [ ] Измерить точные размеры робота
- [ ] Обновить URDF с реальными параметрами
- [ ] Добавить joint_state_publisher для колес
- [ ] Визуализировать в RViz

### Долгосрочные
- [ ] Добавить IMU когда будет установлен
- [ ] Настроить Nav2 с URDF моделью
- [ ] Добавить визуальные mesh модели (опционально)

---

## 🔗 Связанные файлы

- **URDF модели:** `src/verter_admin/urdf/`
- **Launch скрипты:** `src/verter_admin/launch/`
- **Документация параметров:** `src/verter_admin/urdf/PARAMETERS_TO_MEASURE.md`
- **История изменений:** `src/verter_admin/urdf/CHANGES_LOG.md`
- **Setup.py:** Обновлен для установки URDF файлов

---

## 📚 Полезные команды

### Проверка URDF синтаксиса (не работает в conda)
```bash
check_urdf src/verter_admin/urdf/verter_robot_minimal.urdf
```

### Визуализация TF дерева
```bash
ros2 run tf2_tools view_frames
# Создаст файл frames.pdf
```

### Просмотр URDF модели
```bash
ros2 topic echo /robot_description -n 1
```

### Проверка всех фреймов
```bash
ros2 run tf2_ros tf2_echo <source_frame> <target_frame>
# Например:
ros2 run tf2_ros tf2_echo base_footprint sensor_front_center
```

---

## 💡 Заметки

1. **Минимальная URDF vs Полная:** Используйте minimal для запуска, full - для справки
2. **Conda окружение:** Симлинк для плагина нужен только один раз
3. **Координаты:** X - вперед, Y - влево, Z - вверх (REP-103)
4. **Wheelbase:** ~0.22м, нужно измерить точно для одометрии
5. **Footprint:** base_footprint на z=0 (уровень земли), base_link на z=0.1 (радиус колеса)

---

**Автор:** AI Assistant + Oleksandr Karpachov
**Последнее обновление:** 18 октября 2025, 14:45
**Статус этапа:** ✅ ЗАВЕРШЕНО

**Следующий этап:** [03_nav2_setup.md](03_nav2_setup.md) ⏭️
