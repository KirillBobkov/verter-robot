# Навигация

## 1. LiDAR не виден

**Диагностика и решение (пошагово):**

1. `lsusb | grep -i "slamtec\|rplidar\|cp2102\|ch340"` — CP2102 = `ID 10c4:ea60`, CH340 = `ID 1a86:7523`
2. Проверить brltty: `systemctl is-active brltty` → если active: `sudo systemctl stop brltty && sudo systemctl disable brltty && sudo apt remove brltty`
3. `ls -l /dev/ttyUSB*` — есть ли serial-порты; `ls -l /dev/rplidar` — есть ли symlink
4. `groups | grep dialout` — права доступа; `sudo usermod -a -G dialout $USER` (перелогин)
5. `lsof /dev/rplidar` — порт не занят
6. `ros2 pkg list | grep rplidar_ros` — установлен ли пакет; `sudo apt install ros-humble-rplidar-ros`
7. `ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar` — ручной запуск драйвера
8. `ros2 topic echo /scan --once` — данные с лидара

Переопределение порта при запуске (параметр `lidar_port` доступен в `mapping.launch.py` и `autonomous_mapping_real.launch.py`, но не в `nav2_navigation.launch.py`):
```bash
ros2 launch verter_admin mapping.launch.py lidar_port:=/dev/ttyUSB0
```

Создание symlink:
```bash
sudo ln -s /dev/ttyUSB0 /dev/rplidar
```

Стабильное имя (udev-правило `/etc/udev/rules.d/99-rplidar.rules`):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Сценарии решения

**Лидар не обнаружен в lsusb:**
1. Проверить физическое подключение к USB-хабу
2. Проверить, что USB-хаб подключён к Jetson
3. Попробовать другой USB-порт

**Serial порт не появляется:**
1. Проверить brltty
2. Проверить права доступа
3. `dmesg | grep -i usb` — логи ядра

**Нет данных с лидара:**
1. Убедиться, что лидар подключён к питанию
2. Проверить, что лидар вращается (LED мигает)
3. Убедиться, что порт не занят: `lsof /dev/rplidar`

**Ошибка доступа к порту:**
```bash
sudo usermod -a -G dialout $USER
# Перелогиниться
```

---

## 2. Робот не двигается

**Диагностика:**
- `ros2 topic echo /cmd_vel` — есть ли команды скорости
- `ros2 topic hz /cmd_vel` — частота команд
- `ros2 node list | grep micro` — подключён ли ESP32
- `ros2 topic echo /odometry/filtered --once` — работает ли одометрия

---

## 3. AMCL не находит позицию

**Решение:** В RViz использовать «2D Pose Estimate» для указания примерной начальной позиции. AMCL стартует с фиксированной позой (0, 0, 0).

---

## 4. Карта пустая

**Диагностика:**
- `ros2 topic echo /scan --once` — данные LiDAR
- `ros2 topic hz /scan` — частота сканов
- `ros2 node list | grep slam` — запущена ли SLAM-нода
- `ros2 topic hz /map` — карта публикуется

---

## 5. Робот дрожит/вибрирует

**Причина:** Слишком агрессивные параметры углового управления в RegulatedPurePursuitController.

**Решение:** Уменьшить `max_angular_accel` и/или `rotate_to_heading_angular_vel` в конфиге Nav2 (`config/nav2/nav2_navigation_params.yaml`, секция `FollowPath`).

---

## 6. Уведомления «odom timeout»

**Причина:** Энкодеры не публикуются или ESP32 отключён.

**Решение:** Проверить `/wheel_encoders`, перезапустить micro_ros_agent.

---

## 7. EKF: `odom0_differential: false`

**Конфигурация:** `odom0_differential: false` в `config/robot_localization/ekf.yaml`. Источник `odom0: /odom_raw` отдаёт в EKF только скорости (vx, vy, vyaw), позиции и ускорения отключены. EKF сам выводит позицию и yaw из скоростей.

---

## 8. IMU в EKF

**Конфигурация:** `imu0: /imu/data` раскомментирован в `config/robot_localization/ekf.yaml` (строка 84). Из IMU используется только угловая скорость vyaw (yaw_rate); ускорения отключены. Заголовок файла отмечает IMU DISABLED, но фактически IMU включён в фьюжн.

---

## Известные ограничения

- **Задний обзор:** LiDAR видит только передние ~80° (AngularBoundsFilterInPlace ±0.7 рад в `config/laser_filters/laser_filter.yaml`)
- **Максимальная скорость:** 0.3 м/с (ограничено Nav2 и ESP32); `desired_linear_vel: 0.04` в RegulatedPurePursuitController
- **Разрешение карты:** 3 см/ячейка (`resolution: 0.03` в `config/slam/slam_toolbox_params.yaml`)

## Топики LiDAR

- `/scan_raw` — сырые данные с лидара
- `/scan` — отфильтрованные (без заднего сектора)
