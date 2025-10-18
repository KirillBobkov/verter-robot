#!/bin/bash
###############################################################################
# ПРОСТОЙ ЗАПУСК robot_state_publisher БЕЗ ПЛАГИНОВ
###############################################################################

set -e

echo "🚀 Запуск robot_state_publisher..."

# Путь к URDF
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
URDF_FILE="${SCRIPT_DIR}/../urdf/verter_robot_minimal.urdf"

# Проверка файла
if [ ! -f "$URDF_FILE" ]; then
    echo "❌ URDF файл не найден: $URDF_FILE"
    exit 1
fi

echo "📁 URDF: $URDF_FILE"

# Читаем URDF как текст
URDF_CONTENT=$(cat "$URDF_FILE")

# Запускаем через ROS2 параметры без xacro
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    --param robot_description:="$URDF_CONTENT" \
    --param publish_frequency:=50.0
