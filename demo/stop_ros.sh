#!/bin/bash

echo "Остановка ROS 2 системы..."

# Сначала мягко
pkill -TERM -f "vosk_node"
pkill -TERM -f "sound_player"
pkill -TERM -f "arduino_hands"
pkill -TERM -f "arduino_moving_navigation"
pkill -TERM -f "motion_controller"
pkill -TERM -f "distance_controller"

# Ждем 3 секунды
sleep 3

# Потом жестко
pkill -KILL -f "vosk_node"
pkill -KILL -f "sound_player"
pkill -KILL -f "arduino_hands"
pkill -KILL -f "arduino_moving_navigation"
pkill -KILL -f "motion_controller"
pkill -KILL -f "distance_controller"

echo "Все процессы остановлены"
