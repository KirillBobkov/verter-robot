# Диагностика LiDAR RPLiDAR A1M8

Пошаговая проверка от USB-порта до данных в ROS2.

## 1. USB-устройство видно

```bash
lsusb | grep -i "slamtec\|rplidar\|cp2102\|ch340"
```

Ожидаемо: `ID 10c4:ea60` (CP2102) или `ID 1a86:7523` (CH340). Если пусто — проверьте подключение.

## 2. brltty не блокирует

```bash
systemctl is-active brltty
```

Если `active`:
```bash
sudo systemctl stop brltty && sudo systemctl disable brltty && sudo apt remove brltty
```

## 3. Serial-порт существует

```bash
ls -l /dev/ttyUSB*       # серийные порты
ls -l /dev/rplidar       # symlink из udev
```

## 4. Права доступа

```bash
groups | grep dialout
```

Если группы нет:
```bash
sudo usermod -aG dialout $USER && newgrp dialout
```

## 5. Порт свободен

```bash
lsof /dev/rplidar
```

Пустой вывод — порт свободен.

## 6. Пакет rplidar_ros установлен

```bash
ros2 pkg list | grep rplidar_ros
```

Если не установлен: на Jetson Orin нужен патченый `rplidar_ros` (исправление зависания `cp210x` на `tegra-xusb`). Используйте скрипт `scripts/setup_rplidar_ros.sh` — он клонирует репозиторий, применяет патч и собирает пакет.

## 7. Запуск драйвера

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar -p frame_id:=lidar_link -p serial_baudrate:=115200
```

> **Примечание:** в launch-файлах проекта (`main.launch.py`, `mapping.launch.py`) нода `rplidar_ros` запускается с `scan_mode:='Sensitivity'` и публикует сырые данные в `/scan_raw`, а не напрямую в `/scan`. Топик `/scan` публикуется отдельным узлом `laser_filter`.

В другом терминале:
```bash
ros2 topic echo /scan --once
```

Данные с углами и дальностями — LiDAR работает.

## 8. Частота сканов

```bash
ros2 topic hz /scan
```

Ожидаемо: около 10 Гц.

---

## Параметры по умолчанию

| Параметр | Значение |
|---|---|
| Порт | `/dev/rplidar` |
| Скорость | 115200 бод |
| Frame ID | `lidar_link` |
| Scan mode | `Sensitivity` (`main.launch.py`, `mapping.launch.py`); не задан в `autonomous_mapping_real.launch.py` |
| Топик сырых данных | `/scan_raw` (remap `scan`→`/scan_raw`) |
| Топик отфильтрованных | `/scan` (laser_filter, remap `scan_filtered`→`/scan`) |

### laser_filter

Узел `laser_filters/scan_to_scan_filter_chain` (`config/laser_filters/laser_filter.yaml`) фильтрует `/scan_raw` → `/scan`:

| Параметр | Значение |
|---|---|
| Тип фильтра | `LaserScanAngularBoundsFilterInPlace` |
| `lower_angle` | -0.7 рад |
| `upper_angle` | 0.7 рад |
| `replace_with_nan` | `true` |

Дуга обзора ≈ 80° (±0.7 рад), не 180°. Углы вне диапазона заменяются на `NaN`.

Автоматическая проверка: `bash ~/verter-robot/verter_admin/diagnostics/lidar_monitor.sh`

Конкретные проблемы с LiDAR: [Навигация — LiDAR не виден](../problems_solved/navigation.md).
