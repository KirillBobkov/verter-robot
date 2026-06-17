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

Если не установлен: `sudo apt install ros-humble-rplidar-ros`.

## 7. Запуск драйвера

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar
```

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
| Топик сырых данных | `/scan_raw` |
| Топик отфильтрованных | `/scan` |

Автоматическая проверка: `bash ~/verter-robot/verter_admin/check_lidar_connection.sh`

Конкретные проблемы с LiDAR: [Навигация — LiDAR не виден](../problems_solved/navigation.md).
