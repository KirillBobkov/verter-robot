#!/bin/bash
# Запуск Chromium в kiosk-режиме для Verter dialog UI (10" экран).
# Ждёт X-сессию и доступность web-сервера, затем открывают fullscreen-браузер.

set -u

# Определяем имя браузера (различается на дистрибутивах Jetson)
pick_browser() {
    for b in chromium-browser chromium google-chrome chromium-browser.real; do
        if command -v "$b" >/dev/null 2>&1; then
            echo "$b"
            return 0
        fi
    done
    return 1
}

BROWSER="$(pick_browser)" || {
    echo "[verter-kiosk] Браузер не найден (chromium/chromium-browser/google-chrome)" >&2
    exit 1
}

# Ждём доступности X-сессии
echo "[verter-kiosk] Ожидание X-сессии..."
for i in $(seq 1 30); do
    if pgrep -x "Xorg" >/dev/null 2>&1 || pgrep -x "Xwayland" >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

# Ждём доступности web-сервера (порт 8080)
echo "[verter-kiosk] Ожидание web-сервера :8080..."
for i in $(seq 1 30); do
    if curl -s -o /dev/null http://localhost:8080; then
        break
    fi
    sleep 1
done

echo "[verter-kiosk] Запуск $BROWSER в kiosk-режиме"
exec "$BROWSER" \
    --kiosk \
    --noerrdialogs \
    --disable-translate \
    --disable-features=TranslateUI \
    --disable-session-crashed-bubble \
    --disable-infobars \
    --disable-popup-blocking \
    --no-first-run \
    --window-size=1280,800 \
    --start-fullscreen \
    http://localhost:8080
