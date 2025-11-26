#!/bin/bash
# Сохранить текущую позицию робота как именованную точку
# Использование: ./save_waypoint.sh <название>

set -e

# Проверка аргументов
if [ $# -eq 0 ]; then
    echo "Использование: $0 <название точки> [описание]"
    echo "Пример: $0 'Кабинет директора' 'Второй этаж, слева'"
    exit 1
fi

WAYPOINT_NAME="$1"
DESCRIPTION="${2:-}"
WAYPOINTS_FILE="./waypoints.yaml"

echo "=================================================="
echo "Сохранение текущей позиции робота"
echo "=================================================="
echo "Название: $WAYPOINT_NAME"
[ -n "$DESCRIPTION" ] && echo "Описание: $DESCRIPTION"
echo ""

# Получить текущую позицию робота через TF
echo "Получение текущей позиции робота..."

# Используем ros2 run tf2_ros tf2_echo для получения трансформации
POSE_DATA=$(timeout 5 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null | grep -A 10 "Translation:" || true)

if [ -z "$POSE_DATA" ]; then
    echo "❌ Ошибка: Не удалось получить позицию робота!"
    echo "Убедитесь, что:"
    echo "  1. Робот запущен"
    echo "  2. SLAM/локализация работает"
    echo "  3. TF tree публикуется (map -> base_link)"
    exit 1
fi

# Парсим координаты
X=$(echo "$POSE_DATA" | grep "x:" | head -1 | awk '{print $2}')
Y=$(echo "$POSE_DATA" | grep "y:" | head -1 | awk '{print $2}')
Z_QUAT=$(echo "$POSE_DATA" | grep "z:" | tail -1 | awk '{print $2}')
W_QUAT=$(echo "$POSE_DATA" | grep "w:" | tail -1 | awk '{print $2}')

# Конвертируем quaternion в yaw
YAW=$(python3 -c "import math; print(math.atan2(2*$W_QUAT*$Z_QUAT, 1-2*$Z_QUAT*$Z_QUAT))")

echo "✅ Позиция получена:"
echo "   x = $X"
echo "   y = $Y"
echo "   yaw = $YAW ($(python3 -c "import math; print(f'{math.degrees($YAW):.1f}')") градусов)"
echo ""

# Создать временный Python скрипт для добавления в YAML
python3 << EOF
import yaml
import os

waypoints_file = "$WAYPOINTS_FILE"

# Загрузить существующие waypoints
if os.path.exists(waypoints_file):
    with open(waypoints_file, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
else:
    data = {}

# Инициализировать список если пустой
if 'waypoints' not in data:
    data['waypoints'] = []

# Проверить, существует ли уже точка с таким именем
existing_names = [wp['name'] for wp in data['waypoints']]
if "$WAYPOINT_NAME" in existing_names:
    # Обновить существующую точку
    for wp in data['waypoints']:
        if wp['name'] == "$WAYPOINT_NAME":
            wp['x'] = float("$X")
            wp['y'] = float("$Y")
            wp['yaw'] = float("$YAW")
            if "$DESCRIPTION":
                wp['description'] = "$DESCRIPTION"
            print(f"ℹ️  Точка '$WAYPOINT_NAME' обновлена")
            break
else:
    # Добавить новую точку
    new_waypoint = {
        'name': "$WAYPOINT_NAME",
        'x': float("$X"),
        'y': float("$Y"),
        'yaw': float("$YAW")
    }
    if "$DESCRIPTION":
        new_waypoint['description'] = "$DESCRIPTION"

    data['waypoints'].append(new_waypoint)
    print(f"✅ Точка '$WAYPOINT_NAME' добавлена")

# Сохранить обратно в файл
with open(waypoints_file, 'w', encoding='utf-8') as f:
    yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

print(f"\n📁 Сохранено в: {waypoints_file}")
EOF

echo ""
echo "=================================================="
echo "✅ Точка успешно сохранена!"
echo "=================================================="
echo ""
echo "Для отправки робота к этой точке используйте:"
echo "  ./goto_waypoint.sh '$WAYPOINT_NAME'"
echo ""
echo "Список всех точек:"
echo "  ./list_waypoints.sh"
echo "=================================================="
