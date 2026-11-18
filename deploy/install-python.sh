#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-core}"
[[ "$PROFILE" == core || "$PROFILE" == voice ]] || exit 2
python3 -m pip install --target "$ROOT/.python/site" -r "$ROOT/deploy/requirements-$PROFILE.txt"
