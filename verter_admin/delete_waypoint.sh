#!/bin/bash
# Удалить точку из списка
# Использование: ./delete_waypoint.sh <название>

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
WAYPOINTS_FILE="./waypoints.yaml"

echo "=================================================="
echo "🗑️  Удаление точки"
echo "=================================================="
echo "Название: $WAYPOINT_NAME"
echo ""

python3 << EOF
import yaml
import os

waypoints_file = "$WAYPOINTS_FILE"

if not os.path.exists(waypoints_file):
    print("❌ Файл waypoints.yaml не найден")
    exit(1)

with open(waypoints_file, 'r', encoding='utf-8') as f:
    data = yaml.safe_load(f)

waypoints = data.get('waypoints', [])

# Найти и удалить точку
found = False
for i, wp in enumerate(waypoints):
    if wp['name'] == "$WAYPOINT_NAME":
        waypoints.pop(i)
        found = True
        print(f"✅ Точка '$WAYPOINT_NAME' удалена")
        break

if not found:
    print(f"❌ Точка '$WAYPOINT_NAME' не найдена!")
    print("\nДоступные точки:")
    for wp in waypoints:
        print(f"  - {wp['name']}")
    exit(1)

# Сохранить обратно
data['waypoints'] = waypoints
with open(waypoints_file, 'w', encoding='utf-8') as f:
    yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

print(f"\n📁 Обновлено в: {waypoints_file}")
print(f"Осталось точек: {len(waypoints)}")
EOF

echo "=================================================="
