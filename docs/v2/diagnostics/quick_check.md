# Быстрая проверка подсистем

Однострочные команды для мгновенной оценки состояния каждого компонента.

## LiDAR

```bash
ros2 topic hz /scan                    # частота сканов (ожидаемо ~10 Гц)
ros2 topic echo /scan --once --no-arr  # один скан без служебного шума
```

## Одометрия

```bash
ros2 topic hz /odometry/filtered       # частота (ожидаемо ~50 Гц)
ros2 topic echo /odometry/filtered --once --no-arr
```

## IMU

```bash
ros2 topic hz /imu/data                # частота данных IMU
```

## ESP32 / Шасси

```bash
ros2 node list | grep micro            # micro_ros_agent подключён
ros2 topic echo /chassis/state --once  # состояние шасси
```

## Аудио

```bash
pactl list short sources               # устройства ввода
pactl list short sinks                 # устройства вывода
arecord -f cd -d 2 - | aplay           # запись 2 сек и сразу воспроизведение
```

## Распознавание речи

```bash
ros2 topic echo /recognized_text --once    # последнее распознанное
ros2 topic echo /verter_commands --once    # последняя обработанная команда
```

## Навигация

```bash
ros2 topic echo /amcl_pose --once          # текущая оценка позы
ros2 node info /waypoint_manager           # состояние ноды (обычный Node, не LifecycleNode)
```

## Web-UI

```bash
curl -s http://localhost:8080 | head -5    # HTTP-сервер отвечает
```

---

## Справочная таблица

| Задача | Команда |
|---|---|
| Статус сервиса | `systemctl --user status verter-admin.service --no-pager` |
| Логи сервиса | `journalctl --user -u verter-admin.service -f` |
| LiDAR работает | `ros2 topic hz /scan` |
| Одометрия работает | `ros2 topic hz /odometry/filtered` |
| ESP32 подключён | `ros2 node list \| grep micro` |
| Аудио устройства | `pactl list short sinks && pactl list short sources` |
| Тест микрофона | `arecord -f cd -d 2 - \| aplay` |
| Список ROS2-нод | `ros2 node list` |
| Список топиков | `ros2 topic list` |
| USB-устройства | `lsusb` |
| Serial-порты | `ls /dev/ttyUSB* /dev/ttyACM*` |
| Ядро USB | `dmesg \| grep -i usb \| tail -20` |
| Lock-файл | `rm -f /tmp/verter_admin.lock` |
| Web-UI доступен | `curl -s http://localhost:8080 \| head -5` |
| Локальный IP | `hostname -I` |
