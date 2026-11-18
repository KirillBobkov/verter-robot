#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
exec ros2 launch verter_bringup perception.launch.py
