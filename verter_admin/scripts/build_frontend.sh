#!/bin/bash
# Сборка React фронтенда для Verter Robot Web UI

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRONTEND_DIR="$SCRIPT_DIR/src/verter_admin/web/frontend"

echo "Building Verter Robot Web UI..."
echo "Frontend directory: $FRONTEND_DIR"

# Проверяем наличие npm
if ! command -v npm &> /dev/null; then
    echo "Error: npm is not installed"
    exit 1
fi

# Переходим в директорию фронтенда
cd "$FRONTEND_DIR"

# Устанавливаем зависимости если нужно
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm install
fi

# Собираем проект
echo "Building..."
npm run build

# Проверяем результат
if [ -d "dist" ]; then
    echo "Build successful! Files are in: $FRONTEND_DIR/dist"
    echo ""
    echo "To install the package:"
    echo "  cd $SCRIPT_DIR"
    echo "  colcon build --packages-select verter_admin --symlink-install"
else
    echo "Error: Build failed, dist directory not found"
    exit 1
fi
