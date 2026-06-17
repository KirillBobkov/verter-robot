# Диагностика и операции

Справочник по оперативной проверке состояния робота, управлению сервисами и узлами, локализации логов.

Для решения конкретных проблем обратитесь к справочнику проблем в соответствующем разделе документации. Для управления systemd-сервисом (установка, удаление, переустановка) — [Система и сервисы](problems_solved/system_services.md).

---

## 1. Быстрая проверка подсистем

Однострочные команды для мгновенной оценки состояния каждого компонента.

### LiDAR

```bash
ros2 topic hz /scan                    # частота сканов (ожидаемо ~10 Гц)
ros2 topic echo /scan --once --no-arr  # один скан без служебного шума
```

### Одометрия

```bash
ros2 topic hz /odometry/filtered       # частота (ожидаемо ~50 Гц)
ros2 topic echo /odometry/filtered --once --no-arr
```

### IMU

```bash
ros2 topic hz /imu/data                # частота данных IMU
```

### ESP32 / Шасси

```bash
ros2 node list | grep micro            # micro_ros_agent подключён
ros2 topic echo /chassis/state --once  # состояние шасси
```

### Аудио (ввод/вывод)

```bash
pactl list short sources               # устройства ввода
pactl list short sinks                 # устройства вывода
arecord -f cd -d 2 - | aplay           # запись 2 сек и сразу воспроизведение
```

### Распознавание речи

```bash
ros2 topic echo /recognized_text --once    # последнее распознанное
ros2 topic echo /verter_commands --once    # последняя обработанная команда
```

### Навигация / Локализация

```bash
ros2 topic echo /amcl_pose --once          # текущая оценка позы
ros2 lifecycle get /waypoint_manager       # состояние Lifecycle-нodel
```

### Web-UI (путевые точки)

```bash
curl -s http://localhost:8080 | head -5    # HTTP-сервер отвечает
```

---

## 2. Systemd-сервис

Сервис `verter-admin.service` управляет полным запуском системы. Установка, удаление и переустановка описаны в [Система и сервисы](problems_solved/system_services.md).

### Жизненный цикл

```bash
systemctl --user start verter-admin.service    # запустить
systemctl --user stop verter-admin.service     # остановить
systemctl --user restart verter-admin.service  # перезапустить
systemctl --user status verter-admin.service --no-pager
```

### Автозапуск

```bash
systemctl --user enable verter-admin.service   # включить автозапуск
systemctl --user disable verter-admin.service  # отключить
```

После изменения файла сервиса:
```bash
systemctl --user daemon-reload && systemctl --user restart verter-admin.service
```

### Linger (запуск без входа в сессию)

Пользовательский systemd-сервис работает только после входа пользователя. Для запуска в headless-режиме включите linger:

```bash
sudo loginctl enable-linger verter
loginctl show-user verter | grep Linger      # должно показать Linger=yes
```

### Ручной запуск (без systemd)

```bash
cd ~/verter-robot/verter_admin/services && ./start_verter_admin.sh
cd ~/verter-robot/verter_admin/services && ./stop_verter_admin.sh
```

---

## 3. Управление отдельными ROS2-нодами

### Окружение

```bash
source /opt/ros/humble/setup.bash
cd ~/verter-robot/verter_admin && source install/setup.bash
```

### Навигационный стек

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar
ros2 run verter_admin odometry_node
ros2 run verter_admin ekf_node
```

### Голосовой интерфейс

```bash
ros2 run verter_admin speech_to_text_node
ros2 run verter_admin recognition_node
ros2 run verter_admin ai_assistant_node
ros2 run verter_admin text_to_speech_node
ros2 run verter_admin sound_player_node
ros2 run verter_admin doa_node
```

### Живой цикл

```bash
ros2 run verter_admin waypoint_manager_node          # LifecycleNode
ros2 lifecycle set /waypoint_manager configure
ros2 lifecycle set /waypoint_manager activate
```

### Остановка нод

```bash
# Мягкая остановка (SIGTERM)
pkill -TERM -f "ros2 run verter_admin"
pkill -TERM -f "ros2 launch verter_admin"

# Принудительная (SIGKILL) — если SIGTERM не помог
pkill -KILL -f "ros2 run verter_admin"
```

### Проверка процессов

```bash
ps aux | grep verter_admin
ros2 node list
ros2 topic list
ros2 topic info <topic_name>
```

---

## 4. Проверка подключения LiDAR RPLiDAR A1M8

Пошаговая проверка от физического USB-порта до данных в ROS2.

### Шаг 1. USB-устройство видно

```bash
lsusb | grep -i "slamtec\|rplidar\|cp2102\|ch340"
```

Ожидаемо: `ID 10c4:ea60` (CP2102) или `ID 1a86:7523` (CH340). Если пусто — проверьте физическое подключение к USB-хабу и питание Jetson.

### Шаг 2. brltty не блокирует

```bash
systemctl is-active brltty
```

Если `active` — отключите:
```bash
sudo systemctl stop brltty && sudo systemctl disable brltty && sudo apt remove brltty
```

### Шаг 3. Serial-порт существует

```bash
ls -l /dev/ttyUSB*                    # серийные порты
ls -l /dev/rplidar                    # symlink из udev-правил
```

Symlink создаётся udev-правилом (`/etc/udev/rules.d/99-robot-devices.rules`). См. [Настройка udev правил](udev_rules.md).

### Шаг 4. Права доступа

```bash
groups | grep dialout
```

Если группы нет:
```bash
sudo usermod -aG dialout $USER && newgrp dialout
```

### Шаг 5. Порт свободен

```bash
lsof /dev/rplidar
```

Пустой вывод — порт свободен. Если занят — остановите процесс, держащий порт.

### Шаг 6. Пакет rplidar_ros установлен

```bash
ros2 pkg list | grep rplidar_ros
```

Если не установлен: `sudo apt install ros-humble-rplidar-ros`.

### Шаг 7. Запуск драйвера

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar
```

В другом терминале:
```bash
ros2 topic echo /scan --once
```

Данные с углами и дальностями — LiDAR работает.

### Шаг 8. Мониторинг частоты

```bash
ros2 topic hz /scan
```

Ожидаемо: около 10 Гц.

### Параметры по умолчанию

| Параметр | Значение |
|---|---|
| Порт | `/dev/rplidar` |
| Скорость | 115200 бод |
| Frame ID | `lidar_link` |
| Топик сырых данных | `/scan_raw` |
| Топик отфильтрованных | `/scan` |

Автоматическая проверка: `bash ~/verter-robot/verter_admin/check_lidar_connection.sh`

Диагностика конкретных проблем с LiDAR: [Навигация — LiDAR не виден](problems_solved/navigation.md).

---

## 5. USB / Arduino диагностика

### Перечень устройств

```bash
ls /dev/ttyUSB* /dev/ttyACM*
ls -la /dev/rplidar /dev/esp32_chassis /dev/esp32_imu /dev/arduino_chassis /dev/arduino_sensors
```

### Определение физического USB-порта

```bash
# Полный путь устройства в tree ядра
udevadm info -a -n /dev/ttyUSB0 | grep devpath

# Путь в topology
udevadm info -q path /dev/ttyUSB0
# Вывод: /devices/.../1-2.3/1-2.3:1.0/ttyUSB0/...
# Используйте "1-2.3:1.0" как значение KERNELS в udev-правиле
```

### Vendor / Product ID

```bash
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct"
```

### Проверка udev-правил

```bash
# Список всех применённых правил для устройства
udevadm info --query=property --name=/dev/ttyUSB0

# Перезагрузка правил (после редактирования)
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Ядро: последние события USB

```bash
dmesg | grep -i usb | tail -20
```

### Подключение ESP32

```bash
lsof /dev/esp32_chassis                                # порт не занят
ros2 node list | grep micro                             # micro_ros_agent активен
ros2 run verter_admin micro_ros_agent --ros-args -r __node:=micro_ros_agent_chassis -p baudrate:=115200 -p serial_port:=/dev/esp32_chassis
```

Проблемы с USB-serial и brltty: [USB-Serial / CH340](problems_solved/usb_serial.md).

---

## 6. Аудио диагностика

### PulseAudio: устройства

```bash
pactl list short sinks              # вывод
pactl list short sources            # ввод
pactl info | grep "Default Sink"    # текущий default sink
```

### PulseAudio: громкость и mute

```bash
pactl list sinks | grep -A 10 "Name:.*alsa_output"   # состояние sink'ов
pactl list sinks | grep "Mute"                        # замотушены ли
```

### Запись и воспроизведение

```bash
arecord -f cd -d 5 test.wav      # записать 5 секунд
aplay test.wav                   # воспроизвести
arecord -f cd -d 2 - | aplay     # запись + воспроизведение без файла
```

### Тестирование конкретного sink

```bash
aplay -D pulse:0 test.wav        # sink #0
aplay -D pulse:1 test.wav        # sink #1
```

### Сменить default sink

```bash
pactl set-default-sink alsa_output.usb-Generic_USB_Audio_20210726905926-00.analog-stereo
```

### USB-аудио в системе

```bash
lsusb | grep -i audio
python3 -c "import sounddevice as sd; print(sd.query_devices())"
```

### ALSA (обход PulseAudio)

```bash
aplay -l                              # список ALSA-устройств
aplay -D hw:1,0 test.wav              # прямое воспроизведение на устройство 1,0
```

Диагностика конкретных проблем: [Аудио система](problems_solved/audio_system.md).

---

## 7. Логи

### dmesg (ядро, USB, драйверы)

```bash
dmesg | tail -80
dmesg | grep -i usb
dmesg | grep -i tty
```

### systemd journal

```bash
journalctl --user -u verter-admin.service -f          # реальное время
journalctl --user -u verter-admin.service -n 100      # последние 100 строк
journalctl --user -u verter-admin.service --since "1 hour ago"
journalctl --user -u verter-admin.service --since today
```

### Файлы журналов приложения

```bash
tail -f ~/verter-robot/verter_admin/journal.log
tail -f ~/verter-robot/verter_admin/journal-error.log
```

### ROS2 логи (auto-logging)

```bash
ls -lt ~/.ros/log/                         # последние сессии
tail -f ~/.ros/log/latest/*/router.log     # ros2 run в фоне
```

### tegrastats ( Jetson )

```bash
watch -n 1 nvpmodel -q | grep "RAM"
tegrastats
```

---

## 8. SSH и удалённый доступ

### Подключение к роботу

```bash
# Локальная сеть / Ethernet
ssh jetson@192.168.0.9

# Внешний доступ (через NAT)
ssh jetson@109.195.134.20 -p 3333
```

### Узнать IP-адреса

```bash
# На самом роботе
hostname -I          # локальный IP
curl ifconfig.me     # внешний IP
```

### Найти робота в локальной сети

```bash
sudo arp-scan --localnet | grep -i "jetson\|nvidia"
nmap -sn 192.168.0.0/24 | grep -B2 jetson
```

### RViz через X-forwarding

```bash
# Однострочный запуск
ssh -X jetson@192.168.0.9 "source /opt/ros/humble/setup.bash && source ~/verter-robot/verter_admin/install/setup.bash && rviz2 -d ~/verter-robot/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz"

# Интерактивная сессия
ssh -X jetson@192.168.0.9
source /opt/ros/humble/setup.bash
source ~/verter-robot/verter_admin/install/setup.bash
rviz2 -d ~/verter-robot/verter_admin/src/verter_admin/config/rviz/waypoint_navigation.rviz
```

Проверка X-forwarding: `echo $DISPLAY` должно показать `localhost:10.0` или аналогичное.

### Работа с файлами

```bash
# Копирование файлов
scp file.txt jetson@192.168.0.9:~/

# VS Code Remote SSH — подключитесь через расширение Remote-SSH
# Host: jetson@192.168.0.9 или jetson@109.195.134.20 -p 3333
```

Подробности по удалённому доступу: [Удалённое подключение к роботу](remote_access.md).

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
| SSH X-forwarding | `ssh -X jetson@192.168.0.9` |
