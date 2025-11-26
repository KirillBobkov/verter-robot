#!/bin/bash
# Скрипт для сохранения карты SLAM Toolbox
# Использование: ./save_map.sh <имя_карты>

set -e

# Проверка аргументов
if [ $# -eq 0 ]; then
    echo "Использование: $0 <имя_карты>"
    echo "Пример: $0 hospital_map"
    exit 1
fi

MAP_NAME=$1
MAPS_DIR="../maps"

# Создать директорию для карт если не существует
mkdir -p "$MAPS_DIR"

MAP_PATH="$MAPS_DIR/$MAP_NAME"

echo "=================================================="
echo "Сохранение карты SLAM Toolbox"
echo "=================================================="
echo "Имя карты: $MAP_NAME"
echo "Путь: $MAP_PATH"
echo ""

# Сохранить через SLAM Toolbox
echo "1. Сохранение через SLAM Toolbox (posegraph)..."
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$MAP_PATH'}}"

echo ""
echo "2. Сохранение через map_server (PNG/PGM)..."
ros2 run nav2_map_server map_saver_cli -f "$MAP_PATH"

echo ""
echo "=================================================="
echo "✅ Карта сохранена успешно!"
echo "=================================================="
echo "Файлы:"
ls -lh "$MAP_PATH".*
echo ""
echo "Для использования карты в локализации:"
echo "  1. Измените mode: localization в slam_toolbox_params.yaml"
echo "  2. Укажите map_file_name: $MAP_PATH"
echo "  3. Перезапустите SLAM Toolbox"
echo "=================================================="
