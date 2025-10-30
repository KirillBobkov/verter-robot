#!/bin/bash

echo "Остановка Verter Admin системы..."

# Удаляем lock файл
rm -f /tmp/verter_admin.lock

# Мягко завершаем все ROS2 процессы
echo "Мягкая остановка ROS2 процессов..."
pkill -TERM -f "ros2 launch verter_admin"
pkill -TERM -f "ros2 run verter_admin"

# Ждем 3 секунды для корректного завершения
sleep 3

# Жестко завершаем если что-то осталось
echo "Жесткая остановка оставшихся процессов..."
pkill -KILL -f "ros2 launch verter_admin"
pkill -KILL -f "ros2 run verter_admin"

# Дополнительно убиваем по именам нод (на всякий случай)
pkill -KILL -f "recognition_node"
pkill -KILL -f "ai_assistant_node"
pkill -KILL -f "text_to_speech_node"
pkill -KILL -f "sound_player_node"
pkill -KILL -f "chassis_node"
pkill -KILL -f "distance_sensors_node"
pkill -KILL -f "doa_node"

echo "Все процессы Verter Admin остановлены"
