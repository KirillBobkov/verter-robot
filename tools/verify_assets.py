#!/usr/bin/env python3
from pathlib import Path
import hashlib, json, os

root = Path(__file__).resolve().parent.parent
assets = Path(os.environ.get('VERTER_ASSETS_DIR', root / 'assets'))
manifest = json.loads((root / 'config/assets_manifest.json').read_text())
errors = []
for rel, info in manifest.items():
    p = assets / rel
    if not p.is_file():
        errors.append(rel)
        continue
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            h.update(block)
    if h.hexdigest() != info['sha256']:
        errors.append(rel)
print(json.dumps({'checked': len(manifest), 'failed': errors}, indent=2))
raise SystemExit(bool(errors))
