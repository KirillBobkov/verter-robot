#!/bin/bash
# Скрипт диагностики аудио системы для Verter Robot
# Помогает найти причину отсутствия звука

echo "========================================"
echo "  Диагностика аудио системы"
echo "========================================"
echo ""

# Проверка PulseAudio
echo "=== Шаг 1: Проверка PulseAudio ==="
if ! pactl info &>/dev/null; then
    echo "❌ ОШИБКА: PulseAudio не запущен"
    echo "Запустите: pulseaudio --start"
    exit 1
else
    echo "✓ PulseAudio запущен"
fi
echo ""

# Проверка default sink
echo "=== Шаг 2: Default Sink ==="
DEFAULT_SINK=$(pactl info | grep "Default Sink" | awk '{print $3}')
echo "Default Sink: $DEFAULT_SINK"
echo ""

# Список всех sink'ов
echo "=== Шаг 3: Доступные Sinks (выходные устройства) ==="
pactl list sinks short | while read -r line; do
    SINK_NUM=$(echo "$line" | awk '{print $1}')
    SINK_NAME=$(echo "$line" | awk '{print $2}')
    SINK_DESC=$(echo "$line" | awk '{for(i=3;i<=NF;i++) printf $i" "; print ""}')
    echo "  [$SINK_NUM] $SINK_NAME"
    echo "       $SINK_DESC"
done
echo ""

# Проверка громкости и muted состояния
echo "=== Шаг 4: Состояние Sinks ==="
pactl list sinks | while read -r line; do
    if [[ "$line" =~ Name: ]]; then
        SINK_NAME=$(echo "$line" | awk '{print $2}')
        echo "Sink: $SINK_NAME"
    elif [[ "$line" =~ Mute: ]]; then
        MUTE_STATUS=$(echo "$line" | awk '{print $2}')
        if [[ "$MUTE_STATUS" == "yes" ]]; then
            echo "  ⚠️  MUTED (звук отключен)"
        else
            echo "  ✓ Не muted"
        fi
    elif [[ "$line" =~ Volume: ]]; then
        VOLUME=$(echo "$line" | awk '{print $5}' | sed 's/%//')
        if (( $(echo "$VOLUME < 50" | bc -l) )); then
            echo "  ⚠️  Громкость низкая: $VOLUME%"
        else
            echo "  ✓ Громкость: $VOLUME%"
        fi
    fi
done
echo ""

# Тестирование каждого sink
echo "=== Шаг 5: Тестирование каждого Sink ==="
TEST_FILE="/usr/share/sounds/alsa/Front_Center.wav"
if [ ! -f "$TEST_FILE" ]; then
    echo "⚠️  Тестовый файл не найден: $TEST_FILE"
    echo "Пропускаю тест воспроизведения..."
else
    pactl list sinks short | while read -r line; do
        SINK_NUM=$(echo "$line" | awk '{print $1}')
        SINK_NAME=$(echo "$line" | awk '{print $2}')
        SINK_RATE=$(echo "$line" | awk '{print $4}' | sed 's/Hz//')

        echo ""
        echo "Тест Sink #$SINK_NUM: $SINK_NAME ($SINK_RATE Hz)"

        # Определяем формат и частоту для теста
        if [[ "$SINK_RATE" == "16000" ]]; then
            TEST_RATE=16000
            TEST_FORMAT="S16_LE"
        elif [[ "$SINK_RATE" == "48000" ]]; then
            TEST_RATE=48000
            TEST_FORMAT="S16_LE"
        elif [[ "$SINK_RATE" == "44100" ]]; then
            TEST_RATE=44100
            TEST_FORMAT="S16_LE"
        else
            TEST_RATE=48000
            TEST_FORMAT="S16_LE"
        fi

        echo "  Запуск: aplay -D pulse:$SINK_NUM -f $TEST_FORMAT -r $TEST_RATE -c 2 $TEST_FILE"
        if timeout 3 aplay -D "pulse:$SINK_NUM" -f "$TEST_FORMAT" -r "$TEST_RATE" -c 2 "$TEST_FILE" -q 2>&1; then
            echo "  ✓ Sink #$SINK_NUM РАБОТАЕТ"
        else
            echo "  ❌ Sink #$SINK_NUM НЕ РАБОТАЕТ"
        fi
    done
fi
echo ""

# Рекомендации
echo "=== Шаг 6: Рекомендации ==="
DEFAULT_SINK_NUM=$(pactl list sinks short | grep "$DEFAULT_SINK" | awk '{print $1}')
echo "Текущий default sink: #$DEFAULT_SINK_NUM ($DEFAULT_SINK)"
echo ""

# Найти рабочий sink
WORKING_SINK=""
pactl list sinks short | while read -r line; do
    SINK_NUM=$(echo "$line" | awk '{print $1}')
    SINK_NAME=$(echo "$line" | awk '{print $2}')
    SINK_RATE=$(echo "$line" | awk '{print $4}' | sed 's/Hz//')

    TEST_RATE=$SINK_RATE
    if [ -f "$TEST_FILE" ]; then
        if timeout 3 aplay -D "pulse:$SINK_NUM" -f "S16_LE" -r "$TEST_RATE" -c 2 "$TEST_FILE" -q 2>&1; then
            echo "✓ Рабочий sink найден: #$SINK_NUM ($SINK_NAME)"
            echo "  Установите его как default:"
            echo "  pactl set-default-sink $SINK_NAME"
            echo ""
            echo "  Или используйте в коде:"
            echo "  self.audio_device = \"pulse:$SINK_NUM\""
            WORKING_SINK=$SINK_NUM
        fi
    fi
done

if [ -z "$WORKING_SINK" ]; then
    echo "⚠️  Не удалось найти рабочий sink"
    echo ""
    echo "Возможные причины:"
    echo "1. Колонки не подключены"
    echo "2. Громкость установлена на 0 или muted"
    echo "3. Устройство не поддерживает указанный формат"
    echo ""
    echo "Проверьте:"
    echo "- Подключение колонок"
    echo "- Громкость: pactl set-sink-volume <sink_name> 100%"
    echo "- Muted: pactl set-sink-mute <sink_name> 0"
fi

echo ""
echo "========================================"
echo "  Диагностика завершена"
echo "========================================"
