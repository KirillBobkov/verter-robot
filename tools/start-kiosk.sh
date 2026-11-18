#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export XDG_CACHE_HOME="$ROOT/data/cache/kiosk"
export __GL_SHADER_DISK_CACHE_PATH="$ROOT/data/cache/kiosk/nvidia"
export GSETTINGS_BACKEND=memory
mkdir -p "$ROOT/data/kiosk-profile" "$XDG_CACHE_HOME" "$__GL_SHADER_DISK_CACHE_PATH"
for attempt in $(seq 1 90); do
    if { pgrep -x Xorg > /dev/null || pgrep -x Xwayland > /dev/null; } && curl --max-time 2 -fsS -o /dev/null http://127.0.0.1:8081; then
        exec firefox --no-remote --profile "$ROOT/data/kiosk-profile" --kiosk --display=:0 http://127.0.0.1:8081
    fi
    sleep 1
done
echo 'Display or Verter UI :8081 unavailable' >&2
exit 1
