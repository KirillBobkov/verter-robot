#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
exec "$VERTER_ROOT/.venv-nats/bin/python" "$VERTER_ROOT/services/lidar/server.py" --config "$VERTER_ROOT/config/nats/lidar.json"
