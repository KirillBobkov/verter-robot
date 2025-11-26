#!/bin/bash
# Скрипт для загрузки карты в режиме локализации
# Использование: ./load_map.sh <имя_карты>

set -e

# Проверка аргументов
if [ $# -eq 0 ]; then
    echo "Использование: $0 <имя_карты>"
    echo "Пример: $0 hospital_map"
    echo ""
    echo "Доступные карты:"
    ls -1 ~/maps/*.posegraph 2>/dev/null | xargs -n 1 basename | sed 's/.posegraph//' || echo "  (нет сохранённых карт)"
    exit 1
fi

MAP_NAME=$1
MAPS_DIR="$HOME/maps"
MAP_PATH="$MAPS_DIR/$MAP_NAME"

# Проверка существования карты
if [ ! -f "$MAP_PATH.posegraph" ]; then
    echo "❌ Ошибка: Карта не найдена: $MAP_PATH.posegraph"
    echo ""
    echo "Доступные карты:"
    ls -1 "$MAPS_DIR"/*.posegraph 2>/dev/null | xargs -n 1 basename | sed 's/.posegraph//' || echo "  (нет сохранённых карт)"
    exit 1
fi

echo "=================================================="
echo "Загрузка карты в режиме локализации"
echo "=================================================="
echo "Имя карты: $MAP_NAME"
echo "Путь: $MAP_PATH"
echo ""

# Загрузить карту через сервис
echo "Загрузка карты через SLAM Toolbox..."
ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph "{filename: {data: '$MAP_PATH'}}"

echo ""
echo "=================================================="
echo "✅ Карта загружена успешно!"
echo "=================================================="
echo "Робот теперь в режиме локализации на этой карте."
echo "Используйте RViz для установки начальной позиции (2D Pose Estimate)"
echo "=================================================="
