# Настройка udev правил

Каждое USB-устройство при подключении получает случайный номер (`/dev/ttyUSB0`, `/dev/ttyUSB1` и т.д.). После перезагрузки номера меняются — робот не найдёт лидар или Arduino.

**Задача udev:** закрепить за каждым устройством постоянное имя (`/dev/rplidar`, `/dev/arduino_chassis`) — чтобы ROS2-ноды всегда находили своё железо.

---

## 1. Создать файл правил

```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

Пример содержимого (адаптируйте под свои устройства):

```
# RPLiDAR — привязка по vendor:product ID (работает с любого USB-порта)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar", MODE="0666"

# Arduino Chassis — привязка к конкретному USB-порту
SUBSYSTEM=="tty", KERNELS=="1-2.3:1.0", SYMLINK+="arduino_chassis", MODE="0666"

# Arduino Sensors — привязка к конкретному USB-порту
SUBSYSTEM=="tty", KERNELS=="1-2.4:1.0", SYMLINK+="arduino_sensors", MODE="0666"
```

## 2. Узнать параметры устройств

**Vendor/Product ID** — для RPLiDAR и других устройств с чипом CP210x:

```bash
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct"
```

Подходит когда устройство можно опознать по чипу — лидар можно воткнуть в любой USB-порт и он будет найден.

**Физический порт USB** — для Arduino (когда несколько одинаковых плат):

```bash
udevadm info -q path /dev/ttyUSB0
# Вывод: /devices/.../1-2.3/1-2.3:1.0/ttyUSB0/...
# Используйте "1-2.3:1.0" как значение KERNELS
```

Vendor/product ID у одинаковых Arduino совпадают — различить можно только по тому, в какой USB-разъём они воткнуты.

## 3. Применить правила

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 4. Проверить

```bash
ls -la /dev/rplidar /dev/arduino_chassis /dev/arduino_sensors
```

Должны появиться симлинки вида `/dev/rplidar → /dev/ttyUSB0`.

## 5. Добавить пользователя в группу dialout

```bash
sudo usermod -aG dialout $USER
newgrp dialout  # или перелогиниться
```

Без этого у пользователя нет прав на чтение/запись через USB-serial — ноды упадут с `Permission denied`.
