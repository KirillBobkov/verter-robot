#!/bin/bash
# Отправить робота к именованной точке
# Использование: ./goto_waypoint.sh <название>

set -e

# Проверка аргументов
if [ $# -eq 0 ]; then
    echo "Использование: $0 <название точки>"
    echo ""
    echo "Доступные точки:"
    ./list_waypoints.sh 2>/dev/null || echo "  (нет сохранённых точек)"
    exit 1
fi

WAYPOINT_NAME="$*"

echo "=================================================="
echo "🚀 Отправка робота к точке"
echo "=================================================="

# Запустить Python ноду для навигации
source install/setup.bash
ros2 run verter_admin waypoint_navigator "$WAYPOINT_NAME"
