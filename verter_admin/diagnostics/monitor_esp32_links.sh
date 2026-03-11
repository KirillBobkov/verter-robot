#!/bin/bash
#
# Сбор диагностики по ESP32 шасси/IMU во время движения робота.
# Помогает отделить:
# - USB disconnect/re-enumeration
# - brownout/EMI на стороне ESP32
# - падение micro_ros_agent
# - высокую нагрузку Jetson без потери USB
#
# Пример:
#   ./diagnostics/monitor_esp32_links.sh
#   ./diagnostics/monitor_esp32_links.sh /home/jetson/esp32-debug
#

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

CHASSIS_DEV="/dev/esp32_chassis"
IMU_DEV="/dev/esp32_imu"

OUT_BASE="${1:-$HOME/esp32-debug}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$OUT_BASE/run_$STAMP"

mkdir -p "$OUT_DIR"

echo -e "${CYAN}========================================${NC}"
echo -e "${CYAN}    MONITOR ESP32 CHASSIS / IMU         ${NC}"
echo -e "${CYAN}========================================${NC}"
echo "Каталог логов: $OUT_DIR"
echo "Остановить: Ctrl+C"
echo ""

cleanup() {
    local code=$?

    echo ""
    echo -e "${YELLOW}Останавливаю мониторинг...${NC}"

    for pid in ${PIDS:-}; do
        kill "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true

    echo "Логи сохранены в: $OUT_DIR"
    exit "$code"
}

trap cleanup INT TERM EXIT

snapshot_file="$OUT_DIR/snapshot.txt"

{
    echo "timestamp=$(date --iso-8601=seconds)"
    echo "hostname=$(hostname)"
    echo "kernel=$(uname -a)"
    echo
    echo "=== symlinks ==="
    ls -l "$CHASSIS_DEV" "$IMU_DEV" 2>&1 || true
    echo
    echo "=== resolved devices ==="
    readlink -f "$CHASSIS_DEV" 2>&1 || true
    readlink -f "$IMU_DEV" 2>&1 || true
    echo
    echo "=== usb list ==="
    lsusb 2>&1 || true
    echo
    echo "=== recent usb/tty kernel messages ==="
    dmesg -T 2>/dev/null | egrep -i "usb|tty|cdc|cp210|ch340|disconnect|reset|over-current|under-voltage|brownout" | tail -n 200 || true
    echo
    echo "=== service status ==="
    systemctl status verter-admin.service --no-pager 2>&1 || true
} > "$snapshot_file"

echo -e "${GREEN}Базовый snapshot записан${NC}: $snapshot_file"

touch "$OUT_DIR/dmesg.log" \
      "$OUT_DIR/service.log" \
      "$OUT_DIR/devices.log" \
      "$OUT_DIR/system.log" \
      "$OUT_DIR/lsof.log"

if command -v tegrastats >/dev/null 2>&1; then
    touch "$OUT_DIR/tegrastats.log"
fi

(
    stdbuf -oL dmesg -wT 2>/dev/null | \
        egrep -i "usb|tty|cdc|cp210|ch340|disconnect|reset|over-current|under-voltage|brownout|xhci" \
        >> "$OUT_DIR/dmesg.log"
) &
PIDS="$PIDS $!"

(
    stdbuf -oL journalctl -f -u verter-admin.service -o short-iso --no-pager 2>/dev/null \
        >> "$OUT_DIR/service.log"
) &
PIDS="$PIDS $!"

(
    while true; do
        {
            echo "===== $(date --iso-8601=seconds) ====="
            ls -l "$CHASSIS_DEV" "$IMU_DEV" 2>&1 || true
            echo "chassis -> $(readlink -f "$CHASSIS_DEV" 2>/dev/null || echo missing)"
            echo "imu     -> $(readlink -f "$IMU_DEV" 2>/dev/null || echo missing)"
            echo
        } >> "$OUT_DIR/devices.log"
        sleep 1
    done
) &
PIDS="$PIDS $!"

(
    while true; do
        {
            echo "===== $(date --iso-8601=seconds) ====="
            free -h 2>/dev/null || true
            echo
            top -bn1 | head -n 20
            echo
        } >> "$OUT_DIR/system.log"
        sleep 2
    done
) &
PIDS="$PIDS $!"

(
    while true; do
        {
            echo "===== $(date --iso-8601=seconds) ====="
            lsof "$CHASSIS_DEV" "$IMU_DEV" 2>/dev/null || true
            echo
        } >> "$OUT_DIR/lsof.log"
        sleep 2
    done
) &
PIDS="$PIDS $!"

if command -v tegrastats >/dev/null 2>&1; then
    (
        stdbuf -oL tegrastats >> "$OUT_DIR/tegrastats.log"
    ) &
    PIDS="$PIDS $!"
fi

echo -e "${BLUE}Идёт сбор:${NC}"
echo "  dmesg      -> $OUT_DIR/dmesg.log"
echo "  service    -> $OUT_DIR/service.log"
echo "  devices    -> $OUT_DIR/devices.log"
echo "  system     -> $OUT_DIR/system.log"
echo "  lsof       -> $OUT_DIR/lsof.log"
if command -v tegrastats >/dev/null 2>&1; then
    echo "  tegrastats -> $OUT_DIR/tegrastats.log"
fi
echo ""
echo -e "${YELLOW}Дальше начните движение робота и дождитесь очередного отвала ESP32.${NC}"
echo -e "${YELLOW}После этого остановите скрипт и проверьте последние строки в dmesg/service log.${NC}"
echo ""

while true; do
    sleep 60
done
