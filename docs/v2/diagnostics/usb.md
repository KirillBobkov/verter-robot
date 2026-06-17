# Диагностика USB

## Перечень устройств

```bash
ls /dev/ttyUSB* /dev/ttyACM*
ls -la /dev/rplidar /dev/esp32_chassis /dev/esp32_imu /dev/arduino_chassis /dev/arduino_sensors
```

## Физический USB-порт

```bash
udevadm info -a -n /dev/ttyUSB0 | grep devpath
udevadm info -q path /dev/ttyUSB0
# Вывод: /devices/.../1-2.3/1-2.3:1.0/ttyUSB0/...
# Используйте "1-2.3:1.0" как значение KERNELS в udev-правиле
```

## Vendor / Product ID

```bash
udevadm info -a /dev/ttyUSB0 | grep -E "idVendor|idProduct"
```

## Проверка udev-правил

```bash
udevadm info --query=property --name=/dev/ttyUSB0
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Логи ядра

```bash
dmesg | grep -i usb | tail -20
```

## ESP32

```bash
lsof /dev/esp32_chassis                                # порт не занят
ros2 node list | grep micro                             # micro_ros_agent активен
```

---

Проблемы с brltty и ReSpeaker: [USB-Serial / CH340](../problems_solved/usb_serial.md).
