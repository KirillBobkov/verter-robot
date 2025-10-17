#!/bin/bash

echo "Запуск Verter Admin системы..."

# Файл блокировки для предотвращения множественного запуска
LOCK_FILE="/tmp/verter_admin.lock"

# Проверка что не запущен уже
if [ -f "$LOCK_FILE" ]; then
    echo "Verter Admin уже запущен (найден lock файл)"
    exit 1
fi

# Создать lock файл
echo $$ > "$LOCK_FILE"

# Функция очистки при выходе
cleanup() {
    echo "Очистка..."
    rm -f "$LOCK_FILE"
    pkill -TERM -f "ros2 launch verter_admin"
    pkill -TERM -f "ros2 run verter_admin"
    sleep 2
    pkill -KILL -f "ros2 launch verter_admin"
    pkill -KILL -f "ros2 run verter_admin"
    exit 0
}

# Установить обработчик сигналов
trap cleanup EXIT SIGTERM SIGINT

# Убить старые процессы
echo "Остановка старых процессов..."
pkill -KILL -f "ros2 launch verter_admin" 2>/dev/null || true
pkill -KILL -f "ros2 run verter_admin" 2>/dev/null || true
sleep 1

# Настройка переменных окружения для аудио
USER_ID=$(id -u verter)
export XDG_RUNTIME_DIR="/run/user/${USER_ID}"
export DISPLAY=":0"
export LC_ALL="C.UTF-8"
export LANG="C.UTF-8"

# Объединенная проверка всех систем (быстрее чем отдельные циклы)
echo "Проверка готовности системы..."
for i in {1..15}; do
    # Проверяем все сразу
    PULSE_OK=$(pactl info >/dev/null 2>&1 && echo "1" || echo "0")
    RESPEAKER_OK=$(arecord -l | grep -q "ReSpeaker" && echo "1" || echo "0")
    AUDIO_OK=$(aplay -l >/dev/null 2>&1 && echo "1" || echo "0")
    SINK_OK=$(pactl info | grep "Default Sink" | awk '{print $3}' | grep -v "auto_null" >/dev/null && echo "1" || echo "0")
    
    if [[ "$PULSE_OK" == "1" && "$RESPEAKER_OK" == "1" && "$AUDIO_OK" == "1" && "$SINK_OK" == "1" ]]; then
        echo "✓ Все системы готовы"
        break
    fi
    
    echo "Ожидание готовности систем... попытка $i/15"
    sleep 2
done

echo "Запуск Verter Admin системы..."

cd /home/verter/verter-robot/verter_admin
set +u  # Отключаем проверку unbound variables для sourcing
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u  # Включаем обратно

# Запуск основной системы через launch файл
ros2 launch verter_admin main.launch.py &
VERTER_ADMIN_PID=$!

echo "Verter Admin система запущена. PID: $VERTER_ADMIN_PID"

# Ждем завершения
wait $VERTER_ADMIN_PID
