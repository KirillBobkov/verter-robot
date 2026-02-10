# Jetson: не появляется /dev/ttyUSB\* (CH340 / USB-Serial)

## Симптомы

-   lsusb видит CH340 / USB-serial устройство
-   /dev/ttyUSB\* не появляется или появляется и сразу исчезает
-   udevadm info не показывает DEVPATH
-   в dmesg есть attach → immediate disconnect
-   порт пропадает сразу после подключения

Типичная строка в логах: interface claimed by 'brltty' device
disconnected

------------------------------------------------------------------------

## Причина

Сервис brltty перехватывает USB-serial интерфейс и отключает устройство.
Драйвер serial загружается, но устройство тут же отвязывается → ttyUSB
не создаётся.

------------------------------------------------------------------------

## Решение

### 1. Отключить и удалить brltty

sudo systemctl stop brltty sudo systemctl disable brltty sudo apt remove
brltty

------------------------------------------------------------------------

### 2. Переподключить USB устройство

ls /dev/ttyUSB\*

Должно появиться: /dev/ttyUSB0

------------------------------------------------------------------------

## Проверка драйвера

lsusb lsmod \| grep ch34 readlink /sys/class/tty/ttyUSB0/device/driver

------------------------------------------------------------------------

## Если нет прав доступа

sudo usermod -a -G dialout \$USER

Перелогиниться.

------------------------------------------------------------------------

## Получить DEVPATH

udevadm info --query=property --name=/dev/ttyUSB0

------------------------------------------------------------------------

## Сделать стабильное имя порта (опционально)

sudo nano /etc/udev/rules.d/99-serial.rules

SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523",
SYMLINK+="robot_serial"

sudo udevadm control --reload-rules sudo udevadm trigger

Результат: /dev/robot_serial



поиск порта:


#!/bin/bash
# Ипользование
# chmod +x show-devpath.sh

DEV=${1:-/dev/ttyUSB0}

if [ ! -e "$DEV" ]; then
  echo "Device not found: $DEV"
fi

SYS=$(readlink -f /sys/class/tty/$(basename $DEV)/device)

echo "Device: $DEV"
echo "Sysfs path:"
echo "$SYS"
echo

PORT=$(echo "$SYS" | grep -oE '[0-9]+-[0-9]+(\.[0-9]+)*')

echo "USB devpath:"
echo "$PORT"

------------------------------------------------------------------------

## ReSpeaker USB Permissions Fix (DOA Node)

### Симптомы

-   Ошибка в логах: `[Errno 13] Access denied (insufficient permissions)`
-   DOA node не может получить направление звука
-   ReSpeaker устройство видится в `lsusb` но недоступно для приложения

### Причина

ReSpeaker USB устройство (idVendor=0x2886, idProduct=0x0018) требует специальных прав доступа для чтения/записи через USB.

### Решение

#### Автоматическая установка

В проекте есть готовый скрипт для настройки прав доступа:

```bash
cd /home/jetson/verter-robot/verter_admin
sudo bash setup_respeaker_usb.sh
```

После выполнения скрипта:
1. Отключите и снова подключите ReSpeaker устройство
2. Перезапустите DOA node

#### Ручная установка

Создайте файл правил udev:

```bash
sudo nano /etc/udev/rules.d/99-respeaker-usb.rules
```

Добавьте следующие строки:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
SUBSYSTEM=="usb_device", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
```

Перезагрузите правила udev:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Проверка

Проверьте, что устройство видно:

```bash
lsusb | grep 2886:0018
```

Должна быть строка вида:
```
Bus 001 Device 005: ID 2886:0018 ...
```

Проверьте права доступа:

```bash
ls -l /dev/bus/usb/001/005
```

Должны быть права `crw-rw-rw-` (0666).

### Отладка

Если проблема сохраняется:

1. Проверьте, что устройство не занято другим процессом:
   ```bash
   sudo lsof | grep 2886
   ```

2. Проверьте логи ядра:
   ```bash
   dmesg | grep -i usb
   ```

3. Попробуйте добавить пользователя в группу `audio`:
   ```bash
   sudo usermod -a -G audio $USER
   ```
   (требуется перелогин)
