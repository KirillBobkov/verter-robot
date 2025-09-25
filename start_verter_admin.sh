#!/bin/bash

echo "Системные устройства проверены systemd, запускаем verter_admin..."
# Файл блокировки для предотвращения множественного запуска
LOCK_FILE="/tmp/verter_admin.lock"

# Проверка что не запущен уже
if [ -f "$LOCK_FILE" ]; then
    echo "Verter Admin уже запущен (найден lock файл)"
    exit 1
fi

# Проверка что процессы не запущены
if pgrep -f "speech_recognition_node" > /dev/null; then
    echo "speech_recognition_node уже запущен"
    exit 1
fi

# Создать lock файл
echo $$ > "$LOCK_FILE"

# Функция очистки при выходе
cleanup() {
    echo "Очистка..."
    rm -f "$LOCK_FILE"
    pkill -TERM -f "speech_recognition_node"
    pkill -TERM -f "ai_assistant_node"
    pkill -TERM -f "text_to_speech_node"
    pkill -TERM -f "sound_player_node"
    pkill -TERM -f "arduino_node"
    sleep 2
    pkill -KILL -f "speech_recognition_node"
    pkill -KILL -f "ai_assistant_node"
    pkill -KILL -f "text_to_speech_node"
    pkill -KILL -f "sound_player_node"
    pkill -KILL -f "arduino_node"
    exit 0
}

# Установить обработчик сигналов
trap cleanup EXIT SIGTERM SIGINT

# Убить старые процессы
echo "Остановка старых процессов..."
pkill -KILL -f "speech_recognition_node" 2>/dev/null || true
pkill -KILL -f "ai_assistant_node" 2>/dev/null || true
pkill -KILL -f "text_to_speech_node" 2>/dev/null || true
pkill -KILL -f "sound_player_node" 2>/dev/null || true
pkill -KILL -f "arduino_node" 2>/dev/null || true

sleep 2

# Настройка переменных окружения для аудио
USER_ID=$(id -u verter)
export XDG_RUNTIME_DIR="/run/user/${USER_ID}"
export DISPLAY=":0"
export LC_ALL="C.UTF-8"
export LANG="C.UTF-8"

echo "Проверка аудиосистемы..."
# Ожидание готовности PulseAudio
for i in {1..30}; do
    if pactl info >/dev/null 2>&1; then
        echo "PulseAudio готов"
        break
    fi
    echo "Ожидание PulseAudio... попытка $i/30"
    sleep 1
done

# Ожидание появления ReSpeaker (USB звуковая карта)
echo "Ожидаю появления ReSpeaker..."
for i in {1..20}; do
    if arecord -l | grep -q "ReSpeaker"; then
        echo "ReSpeaker обнаружен"
        break
    fi
    echo "ReSpeaker не найден, попытка $i/20"
    sleep 1
done

# Проверяем, что aplay может открыть PulseAudio (иногда Pulse готов, но sink ещё не создали)
echo "Проверка aplay/Pulse связи..."
for i in {1..10}; do
    if timeout 2 aplay -D pulse -q /usr/share/sounds/alsa/Front_Center.wav 2>/dev/null; then
        echo "aplay успешно воспроизвёл тестовый звук (или вышел без ошибок)"
        break
    fi
    echo "aplay ещё не готов, попытка $i/10"
    sleep 2
done

# Ожидание появления реального PulseAudio sink (не auto_null)
echo "Проверка наличия аппаратного sink..."
for i in {1..20}; do
    DEFAULT_SINK=$(pactl info | grep "Default Sink" | awk '{print $3}')
    if [[ -n "$DEFAULT_SINK" && "$DEFAULT_SINK" != "auto_null" ]]; then
        echo "Обнаружен sink $DEFAULT_SINK"
        break
    fi
    echo "Аппаратный sink не найден, попытка $i/20"
    sleep 1
done

echo "Запускаем систему..."

cd /home/verter/verter-robot/verter_admin
set +u  # Отключаем проверку unbound variables для sourcing
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u  # Включаем обратно

echo "Запуск Verter Admin системы..."

# Запуск основной системы через launch файл
ros2 launch verter_admin main.launch.py &
VERTER_ADMIN_PID=$!

echo "Verter Admin система запущена. PID: $VERTER_ADMIN_PID"

# Ждем завершения
wait $VERTER_ADMIN_PID
