# Диагностика

## Аудио

```bash
pactl list short sources
pactl list short sinks

arecord -f cd -d 5 test.wav
```

## Arduino / USB

```bash
lsusb
ls /dev/ttyUSB* /dev/ttyACM*

for device in /dev/ttyUSB* /dev/ttyACM*; do [ -e "$device" ] && echo -n "$device: " && udevadm info --query=property --name="$device" | grep DEVPATH | grep -o '1-1\\.[0-9]*\\.[0-9]*' | head -1; done
```

## Логи systemd (если используешь сервис)

```bash
journalctl --user -u verter-admin.service -f
```


