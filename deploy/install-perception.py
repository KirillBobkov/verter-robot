#!/usr/bin/env python3
"""Install the observation-only service; activation is an explicit option."""

import argparse
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--activate', action='store_true', help='Enable and start observation adapters')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    if any(character in str(root) for character in ' %\n\r'):
        raise ValueError('Unsupported project path')

    target = Path.home() / '.config/systemd/user/verter-perception.service'
    rendered = (root / 'deploy/verter-perception.service.in').read_text().replace('@ROOT@', str(root))
    if target.exists() and target.read_text() != rendered:
        raise RuntimeError('Existing unit differs; review it before replacing it')

    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(rendered)
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    if args.activate:
        subprocess.run(['systemctl', '--user', 'enable', '--now', 'verter-perception.service'], check=True)
    print('Installed ' + str(target))


if __name__ == '__main__':
    main()
