#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$VERTER_ROOT"
export PYTHONPATH="$VERTER_ROOT/vendor/pyorbbecsdk-v1/install/lib:$PYTHONPATH"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec "$VERTER_ROOT/.venv-camera/bin/python" "$VERTER_ROOT/services/camera/server.py" --config "$VERTER_ROOT/config/nats/camera.json"
