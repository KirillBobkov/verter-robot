# Прочие проблемы

## URDF — несоответствия параметров

**Симптом:** Визуализация в RViz не совпадает с реальным роботом.

### Исправлено

- Диаметр колёс: 130 мм → 200 мм
- Ширина колёс: 30 мм → 50 мм
- Позиция приводных колёс: сзади → спереди
- Опорное колесо: спереди → сзади
- Конфигурация ультразвуковых датчиков: 7 шт (неверно) → 5 спереди + 2 по бокам
- Base footprint: 0.065 м → 0.1 м

Тип моторов: коллекторные с редуктором, 12В.

### Датчики

**Передние (5):** `sensor_front_center` (0°), `sensor_front_left_inner` (~25°), `sensor_front_left_outer` (~45°), `sensor_front_right_inner` (≈-25°), `sensor_front_right_outer` (≈-45°)

**Боковые (2):** `sensor_left` (90°), `sensor_right` (-90°)

Задних датчиков нет.

### IMU placeholder

В URDF добавлена закомментированная секция IMU (гироскоп, акселерометр, магнитометр). Статус: на этапе подбора, не установлен. Для активации: раскомментировать, измерить позицию, обновить координаты.

### Параметры, требующие точных измерений

1. Размеры корпуса (длина ~0.30 м, ширина ~0.20 м, высота ~0.10 м)
2. Точные позиции колёс (X: ~0.10 м, Y: ±0.11 м)
3. Точная позиция опорного колеса (X: ~-0.12 м, Z: ~-0.07 м)
4. Позиции всех 7 датчиков (X, Y, Z, углы Yaw, высота установки)
5. Масса робота (сейчас ~2.0 кг)

---

## Установка ROS2 — `apt upgrade` перед установкой

**Симптом:** При установке ROS2 на свежей Ubuntu 22.04 без предварительного `apt upgrade` могут удалиться системные пакеты (systemd, udev).

[См. ros2/ros2#1272](https://github.com/ros2/ros2/issues/1272)

**Порядок:**
1. Добавить ROS2 apt-репозиторий
2. `sudo apt update && sudo apt upgrade` — апгрейд уже после добавления репозитория ROS
3. `sudo apt install ros-humble-desktop`

---

## Очистка проекта — загрязнение артефактами ToF-камеры

**Симптом:** В проекте остались неиспользуемые entry points и build-артефакты.

**Решение:**
- Создан `.gitignore`: `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`, `*.egg-info/`, `.venv/`, `venv/`, `.idea/`, `.vscode/`, `*.log`, `journal.log`
- Удалены `__pycache__`, `.pyc`, egg-info (освобождено ~7.1 MB: egg-info ~100KB, __pycache__ ~5MB, .pyc ~2MB)
- Из `setup.py` удалены entry points: `tof_camera_node`, `pointcloud_to_laserscan`, `safety_monitor` (связан с ToF)

**Что НЕ удалено намеренно:**
1. URDF-секция ToF-камеры — закомментирована, может пригодиться как пример
2. Комментарии в конфигах — упоминают ToF для контекста
3. `ultrasonic_to_laserscan_node` — рабочий узел, используется

**После очистки:**
```bash
colcon build --symlink-install
# Или полная очистка:
rm -rf build/ install/ log/
colcon build --symlink-install
```

---

## RViz удалённо — `Cannot open display`

**Симптом:** RViz не открывается при подключении по SSH.

### Однострочная команда

```bash
ssh -X jetson@192.168.0.9 "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz"
```

### Пошагово

1. `ssh -X jetson@192.168.0.9`
2. На роботе: `source /opt/ros/humble/setup.bash && source install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz`

### Настройка X-сервера по ОС

- **macOS:** Установить [XQuartz](https://www.xquartz.org/), перезагрузить сессию терминала
- **Linux:** `echo $DISPLAY` должно быть `:0` или `:1`. Если пусто: `export DISPLAY=:0`
- **Windows:** MobaXterm (X-server встроен) или PuTTY + [Xming](https://sourceforge.net/projects/xming/)

### Диагностика X-forwarding

```bash
echo $DISPLAY
# Ожидаемый вывод: localhost:10.0 или :0
# Если пусто — X-forwarding не включён, проверьте флаг -X при ssh
```

**Примечание:** X-forwarding через внешний адрес может быть медленным — рекомендуется использовать локальную сеть.
