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

Переопределение порта при запуске:
```bash
ros2 launch verter_admin nav2_navigation.launch.py lidar_port:=/dev/ttyUSB0
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

Скрипт автоматической проверки:
```bash
bash /home/jetson/verter-robot/verter_admin/check_lidar_connection.sh
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

**Причина:** Слишком высокие gains в DWB planner.

**Решение:** Уменьшить `velocity_gain` в конфиге Nav2.

---

## 6. Уведомления «odom timeout»

**Причина:** Энкодеры не публикуются или ESP32 отключён.

**Решение:** Проверить `/wheel_encoders`, перезапустить micro_ros_agent.

---

## 7. EKF: `odom0_differential: false`

**Симптом:** EKF доверяет абсолютному yaw от энкодеров, что накапливает дрейф.

**Решение:** Установлено `odom0_differential: true` — EKF использует только изменения yaw.

---

## 8. IMU отключён (известное ограничение)

**Причина:** BMX055 гироскоп даёт дрейф, временно отключён в EKF.

---

## Известные ограничения

- **Задний обзор:** LiDAR видит только передние 180°
- **Максимальная скорость:** 0.3 м/с (ограничено Nav2 и ESP32)
- **Разрешение карты:** 5 см/ячейка (может не видеть мелкие препятствия)

## Топики LiDAR

- `/scan_raw` — сырые данные с лидара
- `/scan` — отфильтрованные (без заднего сектора)
