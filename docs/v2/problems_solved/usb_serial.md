# USB-Serial / CH340

## 1. brltty перехватывает USB-serial порт

**Симптомы:**
- `lsusb` видит устройство (CH340), но `/dev/ttyUSB*` не появляется
- В `dmesg`: `interface claimed by 'brltty' device disconnected`
- `udevadm info` не показывает `DEVPATH`
- Порт пропадает сразу после подключения

**Причина:** Сервис `brltty` (Braille display driver, установлен по умолчанию в Ubuntu) перехватывает USB-serial интерфейс и отключает устройство.

**Решение:**
```bash
sudo systemctl stop brltty
sudo systemctl disable brltty
sudo apt remove brltty
```
Переподключить USB-устройство. При необходимости: `sudo usermod -a -G dialout $USER` (требуется перелогин).

### Проверка драйвера

```bash
lsusb | grep -i ch340
lsmod | grep ch34x
readlink /sys/class/tty/ttyUSB0/device/driver
```

### Получить DEVPATH устройства

```bash
udevadm info --query=property --name=/dev/ttyUSB0
```

### Стабильное имя порта (udev-правило)

Файл `/etc/udev/rules.d/99-robot-devices.rules` (CH340 = ESP32 chassis, VID:PID 1a86:7523):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32_chassis", MODE="0666"
```

Применить:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Результат: `/dev/esp32_chassis` как постоянный symlink (используется в `chassis_bringup.launch.py`). Полный набор udev-правил для всех устройств — в `docs/v2/udev_rules.md`.

---

## 2. ReSpeaker USB — отказ в доступе (Access denied)

**Симптомы:**
- `[Errno 13] Access denied (insufficient permissions)` при запуске DOA-ноды
- DOA нода не может получить направление звука
- ReSpeaker видится в `lsusb`, но недоступен для приложения

**Причина:** idVendor=0x2886, idProduct=0x0018 требует специальных прав доступа для чтения/записи через USB.

### Автоматическая установка

```bash
cd /home/jetson/verter-robot/verter_admin/diagnostics && sudo bash setup_respeaker_usb.sh
```
После этого отключить и снова подключить ReSpeaker, перезапустить DOA node.

### Ручная установка

Файл `/etc/udev/rules.d/99-respeaker-usb.rules`:
```
SUBSYSTEM=="usb", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
SUBSYSTEM=="usb_device", ATTR{idVendor}=="2886", ATTR{idProduct}=="0018", MODE="0666"
```

Применить:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### Проверка

```bash
lsusb | grep 2886:0018
# Ожидаемый вывод: Bus 001 Device 005: ID 2886:0018 ...
ls -l /dev/bus/usb/001/005
# Ожидаемые права: crw-rw-rw- (0666)
```

### Отладка

```bash
sudo lsof | grep 2886          # не занято ли устройство
dmesg | grep -i usb            # логи ядра
sudo usermod -a -G audio $USER # добавить в группу audio (требуется перелогин)
```
