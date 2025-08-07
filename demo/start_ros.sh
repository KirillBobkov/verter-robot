#!/bin/bash

echo "USB устройства проверены systemd, запускаем систему..."
# Файл блокировки для предотвращения множественного запуска
LOCK_FILE="/tmp/verter_demo.lock"

# Проверка что не запущен уже
if [ -f "$LOCK_FILE" ]; then
    echo "ROS 2 App уже запущен (найден lock файл)"
    exit 1
fi

# Проверка что процессы не запущены
if pgrep -f "vosk_node" > /dev/null; then
    echo "vosk_node уже запущен"
    exit 1
fi

# Создать lock файл
echo $$ > "$LOCK_FILE"

# Функция очистки при выходе
cleanup() {
    echo "Очистка..."
    rm -f "$LOCK_FILE"
    pkill -TERM -f "vosk_node"
    pkill -TERM -f "sound_player"
    pkill -TERM -f "arduino_hands"
    pkill -TERM -f "arduino_moving_navigation"
    pkill -TERM -f "motion_controller"
    pkill -TERM -f "distance_controller"
    sleep 2
    pkill -KILL -f "vosk_node"
    pkill -KILL -f "sound_player"
    pkill -KILL -f "arduino_hands"
    pkill -KILL -f "arduino_moving_navigation"
    pkill -KILL -f "motion_controller"
    pkill -KILL -f "distance_controller"
    exit 0
}

# Установить обработчик сигналов
trap cleanup EXIT SIGTERM SIGINT

# Убить старые процессы
echo "Остановка старых процессов..."
pkill -KILL -f "vosk_node" 2>/dev/null || true
pkill -KILL -f "sound_player" 2>/dev/null || true
pkill -KILL -f "arduino_hands" 2>/dev/null || true
pkill -KILL -f "arduino_moving_navigation" 2>/dev/null || true
pkill -KILL -f "motion_controller" 2>/dev/null || true
pkill -KILL -f "distance_controller" 2>/dev/null || true

sleep 2

cd /home/verter/verter-robot/demo
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Запуск ROS 2 системы..."

# Запуск основной системы
ros2 launch verter verter_system.launch.py &
VERTER_PID=$!

echo "Основная система запущена, запускаем речевую систему..."
sleep 3

# Запуск речевой системы
ros2 launch verter speech_system.launch.py &
SPEECH_PID=$!

echo "Системы запущены. Verter PID: $VERTER_PID, Speech PID: $SPEECH_PID"

# Ждем завершения
wait $VERTER_PID $SPEECH_PID
