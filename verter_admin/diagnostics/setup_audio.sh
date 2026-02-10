#!/bin/bash
# Скрипт для настройки аудио системы Verter Robot
# Устанавливает USB Audio устройство как default sink для PulseAudio

# Проверяем, запущен ли PulseAudio
if ! pactl info &>/dev/null; then
    echo "Ошибка: PulseAudio не запущен"
    exit 1
fi

# Получаем список доступных sinks
echo "=== Доступные аудио устройства (sinks): ==="
pactl list sinks short

# Находим USB Audio устройство
USB_SINK=$(pactl list sinks short | grep "usb-Generic_USB_Audio" | awk '{print $1}')

if [ -z "$USB_SINK" ]; then
    echo "Предупреждение: USB Audio устройство не найдено"
    echo "Используем текущий default sink"
else
    echo ""
    echo "Устанавливаем USB Audio (sink #$USB_SINK) как default sink..."
    pactl set-default-sink $USB_SINK
    
    if [ $? -eq 0 ]; then
        echo "✓ Default sink изменен успешно"
    else
        echo "✗ Ошибка при изменении default sink"
        exit 1
    fi
fi

# Показываем текущий default sink
echo ""
echo "=== Текущий default sink: ==="
pactl info | grep "Default Sink"

# Показываем уровень громкости
echo ""
echo "=== Уровень громкости: ==="
DEFAULT_SINK=$(pactl info | grep "Default Sink" | awk '{print $3}')
pactl list sinks | grep -A 10 "Name: $DEFAULT_SINK" | grep "Volume:"

echo ""
echo "✓ Настройка аудио завершена"
