#!/bin/bash
# Показать список всех сохранённых точек
# Использование: ./list_waypoints.sh

WAYPOINTS_FILE="./waypoints.yaml"

if [ ! -f "$WAYPOINTS_FILE" ]; then
    echo "❌ Файл waypoints.yaml не найден"
    echo "Создайте первую точку: ./save_waypoint.sh 'Название'"
    exit 1
fi

echo "=================================================="
echo "📍 Сохранённые точки (waypoints)"
echo "=================================================="
echo ""

python3 << 'EOF'
import yaml
import math

with open('./waypoints.yaml', 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

waypoints = data.get('waypoints', [])

if not waypoints:
    print("  (нет сохранённых точек)")
    print("")
    print("Используйте: ./save_waypoint.sh 'Название' для создания")
else:
    for i, wp in enumerate(waypoints, 1):
        print(f"{i}. {wp['name']}")
        print(f"   Координаты: x={wp['x']:.2f}, y={wp['y']:.2f}")
        yaw_deg = math.degrees(wp['yaw'])
        print(f"   Ориентация: {wp['yaw']:.2f} рад ({yaw_deg:.1f}°)")
        if 'description' in wp:
            print(f"   Описание: {wp['description']}")
        print("")

print(f"Всего точек: {len(waypoints)}")
EOF

echo ""
echo "=================================================="
echo "Для отправки робота к точке:"
echo "  ./goto_waypoint.sh '<название>'"
echo "=================================================="
