#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
mkdir -p "$HOME/.config/systemd/user"
python3 - "$ROOT" "$HOME/.config/systemd/user/verter-supervisor.service" << 'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
if any(c in str(root) for c in ' %\n'):
    raise SystemExit('Service installation requires path without spaces, percent or newline')
Path(sys.argv[2]).write_text((root / 'deploy/verter-supervisor.service.in').read_text().replace('@ROOT@', str(root)))
PY
systemctl --user daemon-reload
echo 'Installed only. Enable/start explicitly after acceptance; starts supervisor in idle.'
