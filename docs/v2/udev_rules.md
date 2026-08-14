# Настройка udev правил

Каждое USB-устройство при подключении получает случайный номер (`/dev/ttyUSB0`, `/dev/ttyUSB1` и т.д.). После перезагрузки номера меняются — робот не найдёт лидар или ESP32.

**Задача udev:** закрепить за каждым устройством постоянное имя (`/dev/rplidar`, `/dev/esp32_chassis`, `/dev/esp32_imu`, `/dev/esp32_ctrl`, `/dev/chassis`, `/dev/arduino_sensors`) и выдать права на USB-аудиоустройство ReSpeaker — чтобы ROS2-ноды всегда находили своё железо. Имена симлинков должны совпадать с теми, что ожидают launch-файлы и `chassis/params.yaml`.

Полный список устройств из кода:

| Симлинк / устройство | Назначение | Где используется |
|---|---|---|
| `/dev/rplidar` | RPLiDAR A1M8 (CP210x) | `main.launch.py`, `mapping.launch.py`, `autonomous_mapping_real.launch.py`, `rplidar_node.py` |
| `/dev/esp32_chassis` | ESP32 chassis (micro-ROS, CH340) | `chassis_bringup.launch.py`, `autonomous_mapping_real.launch.py` |
| `/dev/esp32_imu` | ESP32 IMU/сенсоры (CP210x) | `main.launch.py`, `autonomous_mapping_real.launch.py` |
| `/dev/esp32_ctrl` | ESP32 BLE/DOA | `main.launch.py` |
| `/dev/chassis` | USB-RS485 → ZLAC8015D (Python `chassis_zlac_node`) | `chassis/params.yaml` (`modbus.port`) |
| `/dev/arduino_sensors` | Arduino Mega (ультразвуковые датчики) | `distance_sensors_node.py` |
| ReSpeaker USB mic array | USB-аудиоустройство для DOA (VID `0x2886`, PID `0x0018`) | `doa_node.py` (`RESPEAKER_VENDOR_ID`/`RESPEAKER_PRODUCT_ID`), `diagnostics/setup_respeaker_usb.sh` |

---

## 1. Создать файл правил

```bash
sudo nano /etc/udev/rules.d/99-robot-devices.rules
```

Пример содержимого (адаптируйте под свои устройства — замените `SERIAL_XXX` на реальные serial-номера):

```
# RPLiDAR A1M8 — CP210x USB-UART (VID:PID 10c4:ea60)
# У devices с чипом CP210x VID:PID совпадают — различаем по ATTRS{serial}
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="SERIAL_RPLIDAR", SYMLINK+="rplidar", MODE="0666"

# ESP32 chassis — CH340C (VID:PID 1a86:7523)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32_chassis", MODE="0666"

# ESP32 IMU/сенсоры — CP210x (VID:PID 10c4:ea60, другой serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="SERIAL_ESP32_IMU", SYMLINK+="esp32_imu", MODE="0666"

# ESP32 BLE/DOA — привязка по физическому USB-порту (замените на свой)
SUBSYSTEM=="tty", KERNELS=="1-2.5:1.0", SYMLINK+="esp32_ctrl", MODE="0666"

# USB-RS485 → ZLAC8015D (Python chassis_zlac_node, params.yaml: modbus.port="/dev/chassis")
# Если адаптер на CP210x — добавьте ATTRS{serial} для различения от RPLiDAR и ESP32 IMU
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="SERIAL_CHASSIS", SYMLINK+="chassis", MODE="0666"

# Arduino Mega (ультразвуковые датчики) — привязка по физическому USB-порту
SUBSYSTEM=="tty", KERNELS=="1-2.4:1.0", SYMLINK+="arduino_sensors", MODE="0666"
```

> **Важно:** все три устройства с чипом CP210x (RPLiDAR, ESP32 IMU, USB-RS485) имеют одинаковый VID:PID `10c4:ea60`. Различать их нужно по `ATTRS{serial}` — без этого udev создаст три симлинка на одно устройство.

## 1a. ReSpeaker USB mic array (отдельный файл правил)

ReSpeaker (микрофонная решётка для DOA) — это USB-аудиоустройство, а не serial-порт. Правило задаёт права доступа (`MODE="0666"`), а не симлинк. VID `0x2886`, PID `0x0018` — из `doa_node.py` (`RESPEAKER_VENDOR_ID` / `RESPEAKER_PRODUCT_ID`).

Файл правил — `99-respeaker-usb.rules` (устанавливается скриптом `diagnostics/setup_respeaker_usb.sh`):

```bash
cd verter_admin/diagnostics && sudo bash setup_respeaker_usb.sh
```

Содержимое файла:

```
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
SUBSYSTEM=="usb_device", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
```

Проверка:

```bash
lsusb | grep 2886:0018
ls -l /dev/bus/usb/001/005   # права должны быть crw-rw-rw-
```

## 2. Узнать параметры устройств

**Vendor/Product ID + Serial** — для устройств с чипом CP210x (RPLiDAR, ESP32 IMU, USB-RS485):

```bash
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial"
```

У всех CP210x VID:PID совпадают (`10c4:ea60`) — для различения RPLiDAR, ESP32 IMU и USB-RS485 используйте `ATTRS{serial}`. У CH340 (ESP32 chassis) VID:PID уникальные (`1a86:7523`) — serial не обязателен, если плата одна.

**Физический порт USB** — для устройств, которые нельзя различить по serial (или когда несколько одинаковых плат ESP32/Arduino):

```bash
udevadm info -q path /dev/ttyUSB0
# Вывод: /devices/.../1-2.3/1-2.3:1.0/ttyUSB0/...
# Используйте "1-2.3:1.0" как значение KERNELS
```

Vendor/product ID у одинаковых плат совпадают — различить можно только по тому, в какой USB-разъём они воткнуты.

## 3. Применить правила

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 4. Проверить

```bash
ls -la /dev/rplidar /dev/esp32_chassis /dev/esp32_imu /dev/esp32_ctrl /dev/chassis /dev/arduino_sensors
```

Должны появиться симлинки вида `/dev/rplidar → /dev/ttyUSB0`.

## 5. Добавить пользователя в группу dialout

```bash
sudo usermod -aG dialout $USER
newgrp dialout  # или перелогиниться
```

Без этого у пользователя нет прав на чтение/запись через USB-serial — ноды упадут с `Permission denied`.
