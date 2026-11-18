#!/usr/bin/env bash
set -uo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
for attempt in $(seq 1 60); do
    [[ -S run/control.sock ]] && break
    sleep 1
done
[[ -S run/control.sock ]] || {
    echo "Supervisor socket unavailable" >&2
    exit 1
}
running() {
    bash tools/verter status | python3 -c 'import json,sys; s=json.load(sys.stdin); sys.exit(0 if s.get("ok") and sys.argv[1] in s.get("groups",{}) else 1)' "$1"
}
failed=0
# Attempt every independent group, even when an earlier one cannot start.
if ! running hardware; then bash tools/verter hardware start || failed=1; fi
if ! running voice; then bash tools/verter app voice start || failed=1; fi
if ! running web; then bash tools/verter app web start || failed=1; fi
sleep 5
for group in hardware voice web; do
    running "$group" || {
        echo "Launch group unavailable: $group" >&2
        failed=1
    }
done
# Process presence does not imply sensor readiness. Mode preflight checks data.
bash tools/verter status || failed=1
exit "$failed"
