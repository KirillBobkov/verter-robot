#!/bin/bash
# Firefox kiosk mode для Verter UI
set -u

# Проверка доступности Firefox
if ! command -v firefox >/dev/null 2>&1; then
    echo "[firefox-kiosk] Firefox не найден" >&2
    exit 1
fi

# Ожидание X-сессии
echo "[firefox-kiosk] Ожидание X-сессии..."
for i in $(seq 1 30); do
    if pgrep -x "Xorg" >/dev/null 2>&1 || pgrep -x "Xwayland" >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

# Ожидание web-сервера (порт 8081 — main.launch.py default)
echo "[firefox-kiosk] Ожидание web-сервера :8081..."
for i in $(seq 1 30); do
    if curl -s -o /dev/null http://localhost:8081; then
        break
    fi
    sleep 1
done

# Запуск Firefox в kiosk-режиме
echo "[firefox-kiosk] Запуск Firefox kiosk"
exec firefox --kiosk --display=:0 http://localhost:8081
