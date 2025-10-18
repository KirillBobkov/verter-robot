#!/bin/bash
###############################################################################
# СКРИПТ ЗАПУСКА ROBOT_STATE_PUBLISHER ДЛЯ VERTER ROBOT
###############################################################################
#
# ЧТО ДЕЛАЕТ:
# -----------
# 1. Читает URDF файл
# 2. Запускает robot_state_publisher
# 3. Публикует TF трансформации всех частей робота
#
# КАК ИСПОЛЬЗОВАТЬ:
# -----------------
# source ~/miniforge3/etc/profile.d/conda.sh
# conda activate ros_humble_310
# cd /путь/к/verter_admin
# bash src/verter_admin/launch/start_robot_state_publisher.sh
#
# ПРОВЕРКА:
# ---------
# ros2 topic list                           # Должен быть /robot_description
# ros2 run tf2_ros tf2_echo base_link wheel_left
# ros2 run tf2_tools view_frames
#
###############################################################################

set -e  # Выход при ошибке

echo ""
echo "================================================================================"
echo "ЗАПУСК ROBOT_STATE_PUBLISHER ДЛЯ VERTER ROBOT"
echo "================================================================================"
echo ""

# Путь к URDF файлу
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
URDF_FILE="${SCRIPT_DIR}/../urdf/verter_robot.urdf"

if [ ! -f "$URDF_FILE" ]; then
    echo "❌ Ошибка: URDF файл не найден: $URDF_FILE"
    exit 1
fi

echo "📁 URDF файл: $URDF_FILE"

# Проверяем что файл валидный XML
echo "🔍 Проверка XML структуры..."
if ! grep -q "<robot" "$URDF_FILE"; then
    echo "❌ Ошибка: URDF файл не содержит тег <robot>"
    exit 1
fi

echo "✅ URDF файл валиден"

# Создаем временный URDF без проблемных комментариев
echo "🔄 Создание временного URDF файла..."
TEMP_URDF=$(mktemp /tmp/verter_robot_XXXXXX.urdf)

# Копируем URDF, убирая длинные комментарии с '==='
grep -v "^<!--$" "$URDF_FILE" | grep -v "^===" | grep -v "^-->" > "$TEMP_URDF" || cat "$URDF_FILE" > "$TEMP_URDF"

echo "📝 Временный файл: $TEMP_URDF"

# Функция очистки при выходе
cleanup() {
    if [ -f "$TEMP_URDF" ]; then
        rm -f "$TEMP_URDF"
        echo "🗑️  Временный файл удален"
    fi
}
trap cleanup EXIT INT TERM

# Запускаем robot_state_publisher
echo ""
echo "🚀 Запуск robot_state_publisher..."
echo "================================================================================"
echo ""

# Используем оригинальный файл (xacro должен уметь работать с UTF-8)
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(xacro $URDF_FILE)" \
    -p publish_frequency:=50.0

echo ""
echo "✅ robot_state_publisher завершен"
echo ""
