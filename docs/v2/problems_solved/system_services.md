# Система и сервисы

## 1. Lock-файл блокирует запуск

**Симптом:** Сервис `verter-admin.service` не запускается, в логах конфликт с уже запущенным экземпляром.

**Решение:**
```bash
rm -f /tmp/verter_admin.lock
systemctl --user restart verter-admin.service
```

---

## 2. Сервис не запускается

**Диагностика:**
- `systemctl --user status verter-admin.service` — статус
- `journalctl --user -u verter-admin.service -n 100` — последние 100 строк логов
- `ls -l start_verter_admin.sh stop_verter_admin.sh` — проверка прав на выполнение
- `cat ~/.config/systemd/user/verter-admin.service` — проверка путей в файле сервиса

---

## 3. Сервис падает и перезапускается

**Диагностика:**
- `journalctl --user -u verter-admin.service --since "10 minutes ago" | grep -i error`
- `cat /home/verter/verter-robot/verter_admin/journal-error.log` — логи ошибок
- `pactl list sinks short` — проверка доступности аудио

---

## Установка сервиса

```bash
cd /home/verter/verter-robot/verter_admin/services
chmod +x start_verter_admin.sh stop_verter_admin.sh
mkdir -p ~/.config/systemd/user
cp verter-admin.service ~/.config/systemd/user/verter-admin.service
systemctl --user daemon-reload
systemctl --user enable verter-admin.service
systemctl --user start verter-admin.service
```

## Команды управления

```bash
systemctl --user start verter-admin.service
systemctl --user stop verter-admin.service
systemctl --user restart verter-admin.service
systemctl --user disable verter-admin.service
systemctl --user status verter-admin.service --no-pager
```

После изменения файла сервиса: `systemctl --user daemon-reload && systemctl --user restart verter-admin.service`

## Просмотр логов

```bash
journalctl --user -u verter-admin.service -f          # реальное время
journalctl --user -u verter-admin.service -n 100      # последние 100 строк
journalctl --user -u verter-admin.service --since "1 hour ago"
journalctl --user -u verter-admin.service --since today
```

Логи из файлов:
```bash
tail -f /home/verter/verter-robot/verter_admin/journal.log
tail -f /home/verter/verter-robot/verter_admin/journal-error.log
```

## Ручной запуск (без systemd)

```bash
cd /home/verter/verter-robot/verter_admin/services && ./start_verter_admin.sh
cd /home/verter/verter-robot/verter_admin/services && ./stop_verter_admin.sh
```

## Диагностика аудио

```bash
pactl list short sources     # устройства ввода
pactl list short sinks       # устройства вывода
arecord -f cd -d 5 test.wav  # записать 5 секунд
aplay test.wav               # воспроизвести
```

## Диагностика USB/Arduino

```bash
ls /dev/ttyUSB* /dev/ttyACM*

# Найти физический порт USB
for device in /dev/ttyUSB* /dev/ttyACM*; do
  echo "=== $device ==="
  udevadm info --query=property --name=$device | grep ID_SERIAL
done

udevadm info -a -n /dev/ttyUSB3 | grep devpath
```

## Управление отдельными ROS2-нодами

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run verter_admin recognition_node
ros2 run verter_admin ai_assistant_node
ros2 run verter_admin text_to_speech_node
ros2 run verter_admin sound_player_node
```

Остановка:
```bash
pkill -TERM -f "ros2 run verter_admin"
# Если не помогает:
pkill -KILL -f "ros2 run verter_admin"
```

## Проверка процессов

```bash
ps aux | grep verter_admin
ps aux | grep "ros2"
ros2 topic list
ros2 topic info /ai_question
ros2 topic echo /ai_question
```

## Полная переустановка сервиса

```bash
systemctl --user stop verter-admin.service
systemctl --user disable verter-admin.service
rm ~/.config/systemd/user/verter-admin.service
systemctl --user daemon-reload
```

## Автозапуск при входе пользователя

```bash
sudo loginctl enable-linger verter
loginctl show-user verter | grep Linger
```
