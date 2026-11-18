#!/usr/bin/env python3
"""Idempotent ACL migration only; does not start or stop any service."""

import json
from pathlib import Path
import subprocess
from robot_nats.access import configure_client

ROOT = Path(__file__).resolve().parents[2]


def main():
    directory = ROOT / 'config/nats'
    config = json.loads((directory / 'bridge.json').read_text())
    path = directory / 'server.conf'
    server = json.loads(path.read_text())
    configure_client(server, config)
    candidate = directory / 'server.contract.candidate.conf'
    candidate.write_text(json.dumps(server, indent=2))
    candidate.chmod(0o600)
    subprocess.run([str(ROOT / 'vendor/nats/nats-server'), '-t', '-c', str(candidate)], check=True)
    candidate.replace(path)
    example = directory / 'server.with-hub.example.conf'
    if example.exists():
        template = json.loads(example.read_text())
        template['authorization'] = server['authorization']
        example.write_text(json.dumps(template, indent=2))
        example.chmod(0o600)
    print('Contract ACL validated and installed. Reload NATS configuration to activate.')


if __name__ == '__main__':
    main()
