# Проверка подключения лидара RPLiDAR A1M8

## Обзор

Документация по проверке подключения лидара RPLiDAR A1M8 к Jetson через USB хаб.

## DOD (Definition of Done)

Данные лидара видны в консоли и их можно читать.

## Автоматическая проверка

Используйте скрипт `check_lidar_connection.sh` для автоматической проверки:

```bash
bash /home/jetson/verter-robot/verter_admin/check_lidar_connection.sh
```

## План проверки

### Шаг 1: Проверка USB устройств

Проверка, что лидар обнаружен в системе:

```bash
lsusb | grep -i "slamtec\|rplidar\|cp2102\|ch340"
```

Ожидаемый результат:
- CP2102 (Silicon Labs): `ID 10c4:ea60`
- CH340: `ID 1a86:7523`

### Шаг 2: Проверка brltty

brltty может блокировать USB-serial устройства на Jetson:

```bash
systemctl is-active brltty
```

Если активен, отключите:
```bash
sudo systemctl stop brltty
sudo systemctl disable brltty
sudo apt remove brltty
```

### Шаг 3: Проверка serial портов

Проверка наличия serial портов:

```bash
ls -l /dev/ttyUSB*
```

Проверка symlink:
```bash
ls -l /dev/rplidar
```

Если symlink не существует, создайте:
```bash
sudo ln -s /dev/ttyUSB0 /dev/rplidar
```

### Шаг 4: Проверка прав доступа

Проверьте, что у пользователя есть права на чтение/запись:

```bash
ls -l /dev/rplidar
```

Если прав недостаточно:
```bash
sudo usermod -a -G dialout $USER
# Требуется перелогин
```

### Шаг 5: Проверка использования порта

Проверьте, не используется ли порт другим процессом:

```bash
lsof /dev/rplidar
```

### Шаг 6: Проверка ROS2

Проверьте, установлен ли rplidar_ros:

```bash
ros2 pkg list | grep rplidar_ros
```

Если не установлен:
```bash
sudo apt install ros-humble-rplidar-ros
```

### Шаг 7: Запуск rplidar_node

Запустите драйвер лидара:

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/rplidar
```

В другом терминале проверьте данные:

```bash
ros2 topic echo /scan --once
```

### Шаг 8: Мониторинг данных

Скрипт `check_lidar_connection.sh` включает режим непрерывного мониторинга данных лидара с интервалом 1 секунда.

## Конфигурация проекта

### Параметры лидара

- **Модель**: RPLiDAR A1M8
- **Драйвер**: rplidar_ros
- **Порт по умолчанию**: `/dev/rplidar`
- **Скорость**: 115200 baud
- **Режим сканирования**: Standard
- **Frame ID**: `lidar_link`

### Топики ROS2

- `/scan_raw` - сырые данные с лидара
- `/scan` - отфильтрованные данные (без заднего сектора)

### Launch файлы

- `mapping.launch.py` - полный запуск с SLAM
- `real_robot_navigation.launch.py` - навигация на реальном роботе

## Решение проблем

### Лидар не обнаружен в lsusb

1. Проверьте физическое подключение к USB хабу
2. Проверьте, что USB хаб подключен к Jetson
3. Попробуйте другой USB порт на хабе или Jetson

### Serial порт не появляется

1. Проверьте brltty (см. Шаг 2)
2. Проверьте права доступа (см. Шаг 4)
3. Проверьте логи ядра: `dmesg | grep -i usb`

### Нет данных с лидара

1. Убедитесь, что лидар подключен к питанию
2. Проверьте, что лидар вращается (LED мигает)
3. Убедитесь, что порт не используется другим процессом
4. Проверьте правильный порт: `/dev/rplidar`

### Ошибка доступа к порту

```bash
sudo usermod -a -G dialout $USER
# Перелогинитесь
```

## Стабильное имя порта (опционально)

Создайте udev rule для стабильного имени порта:

```bash
sudo nano /etc/udev/rules.d/99-rplidar.rules
```

Добавьте:
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
```

Перезагрузите правила:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
