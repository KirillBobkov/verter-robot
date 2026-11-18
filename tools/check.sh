#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/env.sh"
cd "$VERTER_ROOT"
python3 -m compileall -q ros_ws/src tools
# Legacy tests replace rclpy in sys.modules; keep real launch tests in a separate process.
python3 -m pytest -q tests/test_startup_isolation.py
python3 -m pytest -q tests/test_runtime.py tests/test_geometry.py tests/legacy/unit
PYTHONPATH="$VERTER_ROOT/ros_ws/src/verter_perception:$PYTHONPATH" python3 -m pytest -q ros_ws/src/verter_perception/tests
for script in tools/*.sh deploy/*.sh tools/verter; do bash -n "$script"; done
