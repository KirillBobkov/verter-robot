# Логи

## dmesg (ядро, USB, драйверы)

```bash
dmesg | tail -80
dmesg | grep -i usb
dmesg | grep -i tty
```

## systemd journal

```bash
journalctl --user -u verter-admin.service -f               # реальное время
journalctl --user -u verter-admin.service -n 100           # последние 100 строк
journalctl --user -u verter-admin.service --since "1 hour ago"
journalctl --user -u verter-admin.service --since today
```

## Файлы журналов приложения

```bash
tail -f ~/verter-robot/verter_admin/journal.log
tail -f ~/verter-robot/verter_admin/journal-error.log
```

## ROS2 логи

```bash
ls -lt ~/.ros/log/                              # последние сессии
tail -f ~/.ros/log/latest/*/router.log          # ros2 run в фоне
```

## tegrastats (Jetson)

```bash
tegrastats
```
