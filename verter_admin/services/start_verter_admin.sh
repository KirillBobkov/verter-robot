#!/bin/bash

echo "=== Запуск Verter Admin (User Session) ==="
date

# Файл блокировки
LOCK_FILE="/tmp/verter_admin.lock"

# Проверка повторного запуска
if [ -f "$LOCK_FILE" ]; then
    PID=$(cat "$LOCK_FILE")
    if ps -p $PID > /dev/null; then
        echo "Verter Admin уже запущен (PID: $PID). Выход."
        exit 1
    else
        echo "Найден старый lock-файл, но процесс мертв. Удаляем."
        rm -f "$LOCK_FILE"
    fi
fi

echo $$ > "$LOCK_FILE"

# Функция очистки
cleanup() {
    echo "Остановка по сигналу..."
    rm -f "$LOCK_FILE"
    # Доверяем внешнему скрипту stop_verter_admin.sh или systemd убивать процессы,
    # но на всякий случай посылаем сигнал группе
    exit 0
}
trap cleanup EXIT SIGTERM SIGINT

# Убиваем старые хвосты перед запуском
pkill -KILL -f "ros2 launch verter_admin" 2>/dev/null || true
pkill -KILL -f "ros2 run verter_admin" 2>/dev/null || true

echo "Ожидание готовности аудио..."
# Ждем пока PulseAudio покажет хоть один Sink (выход)
# Это гарантирует, что звуковой сервер полностью загрузился
MAX_WAIT=30
for ((i=1; i<=MAX_WAIT; i++)); do
    if pactl list sinks short >/dev/null 2>&1; then
        echo "Аудиосистема обнаружена."
        break
    fi
    echo "Ждем аудио... $i/$MAX_WAIT"
    sleep 1
done

echo "Проверка USB аудио устройств..."
MAX_USB_WAIT=20
for ((i=1; i<=MAX_USB_WAIT; i++)); do
    if lsusb | grep -qi "Generic.*USB.*Audio\|ReSpeaker\|ArrayUAC10"; then
        echo "USB аудио устройство обнаружено."
        break
    fi
    echo "Ждем USB аудио... $i/$MAX_USB_WAIT"
    sleep 1
done

# Дополнительная задержка для полной инициализации устройства
echo "Ожидание инициализации аудио устройств..."
sleep 2

echo "=== Диагностика аудио системы ==="
echo "PulseAudio sinks:"
pactl list sinks short 2>/dev/null || echo "Нет sinks"
echo ""
echo "PulseAudio sources:"
pactl list sources short 2>/dev/null || echo "Нет sources"
echo ""
echo "USB аудио устройства:"
lsusb | grep -i "audio\|repeaker\|arrayuac10" || echo "Не найдены"
echo "==================================="

echo "Запуск ROS2 системы..."

# Переходим в рабочую директорию
cd /home/jetson/verter-robot/verter_admin

# Source ROS2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Запуск launch файла
# Используем exec, чтобы shell-скрипт заменился процессом ros2 launch
# Это упрощает управление PID и сигналами
exec ros2 launch verter_admin main.launch.py

