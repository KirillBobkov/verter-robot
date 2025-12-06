# Управление сервисами Verter Admin

Эта папка содержит файлы для настройки и управления systemd сервисом Verter Admin.

## Файлы

- `verter-admin.service` - systemd unit файл для автозапуска
- `start_verter_admin.sh` - скрипт запуска системы
- `stop_verter_admin.sh` - скрипт остановки системы

## Установка сервиса

### 1. Подготовка файлов

```bash
# Перейти в директорию services
cd /home/verter/verter-robot/verter_admin/services

# Убедиться, что скрипты имеют права на выполнение
chmod +x start_verter_admin.sh
chmod +x stop_verter_admin.sh
```

### 2. Копирование файла сервиса

```bash
# Создать директорию для пользовательских сервисов (если не существует)
mkdir -p ~/.config/systemd/user

# Скопировать файл сервиса
cp verter-admin.service ~/.config/systemd/user/verter-admin.service
```

### 3. Активация сервиса

```bash
# Перезагрузить конфигурацию systemd
systemctl --user daemon-reload

# Включить автозапуск сервиса
systemctl --user enable verter-admin.service

# Запустить сервис (опционально, если нужно запустить сразу)
systemctl --user start verter-admin.service
```

## Управление сервисом

### Запуск

```bash
systemctl --user start verter-admin.service
```

### Остановка

```bash
systemctl --user stop verter-admin.service
```

### Перезапуск

```bash
systemctl --user restart verter-admin.service
```

### Проверка статуса

```bash
systemctl --user status verter-admin.service
```

Или без пагинатора:

```bash
systemctl --user status verter-admin.service --no-pager
```

### Отключение автозапуска

```bash
systemctl --user disable verter-admin.service
```

### Перезагрузка конфигурации

После изменения файла сервиса:

```bash
# Скопировать обновленный файл
cp verter-admin.service ~/.config/systemd/user/verter-admin.service

# Перезагрузить конфигурацию
systemctl --user daemon-reload

# Перезапустить сервис
systemctl --user restart verter-admin.service
```

## Просмотр логов

### Логи systemd (journalctl)

```bash
# Просмотр логов в реальном времени
journalctl --user -u verter-admin.service -f

# Последние 100 строк
journalctl --user -u verter-admin.service -n 100

# Логи за последний час
journalctl --user -u verter-admin.service --since "1 hour ago"

# Логи с временными метками
journalctl --user -u verter-admin.service --since today
```

### Логи из файлов

```bash
# Стандартный вывод
tail -f /home/verter/verter-robot/verter_admin/journal.log

# Ошибки
tail -f /home/verter/verter-robot/verter_admin/journal-error.log

# Оба файла одновременно
tail -f /home/verter/verter-robot/verter_admin/journal*.log
```

## Ручной запуск/остановка (без systemd)

### Запуск вручную

```bash
cd /home/verter/verter-robot/verter_admin/services
./start_verter_admin.sh
```

### Остановка вручную

```bash
cd /home/verter/verter-robot/verter_admin/services
./stop_verter_admin.sh
```

## Управление отдельными нодами

### Запуск отдельных нод ROS2

```bash
# Настройка окружения
source /opt/ros/humble/setup.bash
cd /home/verter/verter-robot/verter_admin
source install/setup.bash

# Запуск нод по отдельности
ros2 run verter_admin recognition_node
ros2 run verter_admin ai_assistant_node
ros2 run verter_admin text_to_speech_node
ros2 run verter_admin sound_player_node
ros2 run verter_admin arduino_node
```

### Остановка всех ROS2 процессов

```bash
# Мягкая остановка (SIGTERM)
pkill -TERM -f "ros2 launch verter_admin"
pkill -TERM -f "ros2 run verter_admin"

# Принудительная остановка (SIGKILL)
pkill -KILL -f "ros2 launch verter_admin"
pkill -KILL -f "ros2 run verter_admin"
```

## Проверка процессов

### Проверка запущенных процессов

```bash
# Все процессы verter_admin
ps aux | grep verter_admin

# ROS2 процессы
ps aux | grep "ros2"

# Конкретная нода
ps aux | grep recognition_node
```

### Проверка портов и топиков

```bash
# Список активных топиков
ros2 topic list

# Информация о топике
ros2 topic info /ai_question

# Просмотр сообщений в топике
ros2 topic echo /ai_question
```

## Устранение неполадок

### Сервис не запускается

```bash
# Проверить статус
systemctl --user status verter-admin.service

# Проверить логи
journalctl --user -u verter-admin.service -n 50

# Проверить права на выполнение
ls -l start_verter_admin.sh stop_verter_admin.sh

# Проверить пути в service файле
cat ~/.config/systemd/user/verter-admin.service
```

### Сервис падает и перезапускается

```bash
# Посмотреть последние ошибки
journalctl --user -u verter-admin.service --since "10 minutes ago" | grep -i error

# Проверить логи ошибок
cat /home/verter/verter-robot/verter_admin/journal-error.log

# Проверить доступность аудио
pactl list sinks short
```

### Очистка lock файла

Если сервис не запускается из-за lock файла:

```bash
rm -f /tmp/verter_admin.lock
systemctl --user restart verter-admin.service
```

### Полная переустановка сервиса

```bash
# Остановить и отключить
systemctl --user stop verter-admin.service
systemctl --user disable verter-admin.service

# Удалить файл сервиса
rm ~/.config/systemd/user/verter-admin.service

# Перезагрузить конфигурацию
systemctl --user daemon-reload

# Повторить установку (см. раздел "Установка сервиса")
```

## Автозапуск при входе пользователя

Сервис настроен на запуск при входе пользователя в систему (через `default.target`). 

Для запуска без входа в графическую сессию (headless режим), можно использовать systemd user service с `linger`:

```bash
# Включить linger для пользователя
sudo loginctl enable-linger verter

# Проверить статус
loginctl show-user verter | grep Linger
```

