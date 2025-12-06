#!/bin/bash

# Игнорируем ошибки, наша цель - убить всё
set +e

echo "=== Остановка Verter Admin ==="

# 1. Удаляем lock файл
rm -f /tmp/verter_admin.lock

# 2. Посылаем SIGTERM всем
echo "Отправка SIGTERM..."
# Убиваем родительский launch
pkill -TERM -f "ros2 launch verter_admin"
# Убиваем процессы нод по имени пакета (это самое надежное для ROS2 нод)
pkill -TERM -f "verter_admin"

# Ждем совсем немного
sleep 2

# 3. Убиваем всё что движется (SIGKILL)
echo "Отправка SIGKILL..."
pkill -KILL -f "ros2 launch verter_admin"
pkill -KILL -f "ros2 run verter_admin"
# Убиваем все питоновские процессы запущенные из нашего install
pkill -KILL -f "install/verter_admin"

echo "Очистка завершена."
exit 0

