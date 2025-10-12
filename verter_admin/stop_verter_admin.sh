#!/bin/bash

echo "Остановка Verter Admin системы..."

# Удаляем lock файл
rm -f /tmp/verter_admin.lock

# Сначала мягко завершаем процессы
pkill -TERM -f "speech_recognition_node"
pkill -TERM -f "ai_assistant_node"
pkill -TERM -f "text_to_speech_node"
pkill -TERM -f "sound_player_node"
pkill -TERM -f "arduino_node"

# Ждем 3 секунды
sleep 3

# Потом жестко завершаем
pkill -KILL -f "speech_recognition_node"
pkill -KILL -f "ai_assistant_node"
pkill -KILL -f "text_to_speech_node"
pkill -KILL -f "sound_player_node"
pkill -KILL -f "arduino_node"

echo "Все процессы Verter Admin остановлены"
