#!/usr/bin/env bash
# Build camera dependencies only. Does not start camera or robot services.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
python3 -m venv --system-site-packages .venv-camera
.venv-camera/bin/pip install -r services/camera/requirements.txt
if [[ ! -d vendor/pyorbbecsdk-v1 ]]; then
    git clone --depth 1 --branch v1.3.4 https://github.com/orbbec/pyorbbecsdk.git vendor/pyorbbecsdk-v1
fi
cmake -S vendor/pyorbbecsdk-v1 -B vendor/pyorbbecsdk-v1/build -Dpybind11_DIR="$(.venv-camera/bin/python -m pybind11 --cmakedir)" -DPython3_EXECUTABLE="$ROOT/.venv-camera/bin/python"
cmake --build vendor/pyorbbecsdk-v1/build -j2
cmake --install vendor/pyorbbecsdk-v1/build
