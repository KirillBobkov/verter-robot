#!/usr/bin/env python3
"""Install only the NATS sidecar under the new project. Never touch robot services."""

import hashlib
import json
import os
from pathlib import Path
import platform
import secrets
import subprocess
import tarfile
import urllib.request
from robot_nats.access import configure_client

ROOT = Path(__file__).resolve().parents[2]
COMPONENT = ROOT / 'services/nats_bridge'
VERSION = '2.15.0'


def run(*args):
    subprocess.run(args, check=True)


def private(path, text):
    path.write_text(text)
    path.chmod(0o600)


def main():
    if platform.machine() not in ('aarch64', 'x86_64'):
        raise RuntimeError('Unsupported architecture')
    arch = 'arm64' if platform.machine() == 'aarch64' else 'amd64'
    vendor = ROOT / 'vendor/nats'
    vendor.mkdir(parents=True, exist_ok=True)
    asset = f'nats-server-v{VERSION}-linux-{arch}.tar.gz'
    base = f'https://github.com/nats-io/nats-server/releases/download/v{VERSION}/'
    checksums = urllib.request.urlopen(base + 'SHA256SUMS', timeout=60).read().decode()
    expected = next(line.split()[0] for line in checksums.splitlines() if line.split()[-1].lstrip('*') == asset)
    payload = urllib.request.urlopen(base + asset, timeout=120).read()
    assert hashlib.sha256(payload).hexdigest() == expected, 'Checksum mismatch'
    archive = vendor / asset
    archive.write_bytes(payload)
    with tarfile.open(archive) as tar:
        member = next(m for m in tar.getmembers() if m.name.endswith('/nats-server') and m.isfile())
        (vendor / 'nats-server').write_bytes(tar.extractfile(member).read())
    (vendor / 'nats-server').chmod(0o755)
    (vendor / 'provenance.json').write_text(
        json.dumps({'version': VERSION, 'url': base + asset, 'sha256': expected}, indent=2)
    )
    env = ROOT / '.venv-nats'
    if not env.exists():
        run('python3', '-m', 'venv', '--system-site-packages', str(env))
    run(str(env / 'bin/python'), '-m', 'pip', 'install', '-r', str(COMPONENT / 'requirements.txt'))
    lock = subprocess.check_output([str(env / 'bin/python'), '-m', 'pip', 'freeze', '--local'], text=True)
    (vendor / 'requirements.lock.txt').write_text(lock)
    cfgdir = ROOT / 'config/nats'
    cfgdir.mkdir(parents=True, exist_ok=True)
    cfgdir.chmod(0o700)
    data = ROOT / 'data/nats-bridge'
    data.mkdir(parents=True, exist_ok=True)
    jsdir = ROOT / 'data/nats'
    jsdir.mkdir(parents=True, exist_ok=True)
    configpath = cfgdir / 'bridge.json'
    if not configpath.exists():
        cfg = {
            'robot_id': 'verter_01',
            'domain': 'verter_01',
            'bucket': 'VERTER_01_STATE',
            'nats_url': 'nats://127.0.0.1:4222',
            'backend_user': 'backend',
            'backend_password': secrets.token_hex(24),
            'client_user': 'client',
            'client_password': secrets.token_hex(24),
            'web_port': 8088,
            'web_enabled': True,
            'ros_enabled': True,
            'allow_mode_changes': False,
            'allow_manual_motion': False,
            'project_root': str(ROOT),
            'data_dir': str(data),
            'maps': {'dtest_map': '/home/jetson/verter-robot/data/maps/dtest_map.yaml'},
        }
        private(configpath, json.dumps(cfg, indent=2))
    cfg = json.loads(configpath.read_text())
    prefix = f'robots.{cfg["robot_id"]}.v1'
    server = cfgdir / 'server.conf'
    if not server.exists():
        # Account/remote credentials for a real hub are supplied separately later.
        conf = {
            'server_name': cfg['robot_id'],
            'listen': '127.0.0.1:4222',
            'http': '127.0.0.1:8222',
            'max_payload': 262144,
            'jetstream': {
                'store_dir': str(jsdir),
                'domain': cfg['domain'],
                'max_memory_store': 33554432,
                'max_file_store': 268435456,
            },
            'authorization': {
                'users': [
                    {'user': cfg['backend_user'], 'password': cfg['backend_password']},
                    {
                        'user': cfg['client_user'],
                        'password': cfg['client_password'],
                        'permissions': {
                            'publish': [
                                prefix + '.request',
                                f'$JS.{cfg["domain"]}.API.INFO',
                                f'$JS.{cfg["domain"]}.API.STREAM.INFO.KV_{cfg["bucket"]}',
                                f'$JS.{cfg["domain"]}.API.STREAM.MSG.GET.KV_{cfg["bucket"]}',
                                '$JS.API.INFO',
                                f'$JS.API.STREAM.INFO.KV_{cfg["bucket"]}',
                                f'$JS.API.STREAM.MSG.GET.KV_{cfg["bucket"]}',
                            ],
                            'subscribe': ['_INBOX.>', prefix + '.state', f'$KV.{cfg["bucket"]}.>'],
                        },
                    },
                ]
            },
            'leafnodes': {'remotes': []},
        }
        private(server, json.dumps(conf, indent=2))
    current = json.loads(server.read_text())
    configure_client(current, cfg)
    private(server, json.dumps(current, indent=2))
    # Full separate example, never used by the running service until configured.
    example = json.loads(server.read_text())
    example['leafnodes'] = {
        'remotes': [
            {
                'urls': ['tls://nats.example.invalid:7422'],
                'credentials': str(cfgdir / 'hub.creds'),
                'tls': {'ca_file': str(cfgdir / 'hub-ca.pem')},
            }
        ]
    }
    private(cfgdir / 'server.with-hub.example.conf', json.dumps(example, indent=2))
    run(str(vendor / 'nats-server'), '-t', '-c', str(server))
    start = COMPONENT / 'start.sh'
    start.write_text(
        '#!/usr/bin/env bash\nset -eo pipefail\nsource "{ROOT}/tools/env.sh"\ncd "'
        + str(COMPONENT)
        + '"\nexec "'
        + str(env / 'bin/python')
        + '" -m robot_nats --config "'
        + str(configpath)
        + '"\n'
    )
    start.chmod(0o755)
    units = Path.home() / '.config/systemd/user'
    units.mkdir(parents=True, exist_ok=True)
    (units / 'verter-nats.service').write_text(f"""[Unit]
Description=Verter local NATS leaf-ready server
After=network-online.target
[Service]
ExecStart={vendor}/nats-server -c {server}
Restart=on-failure
RestartSec=3
UMask=0077
NoNewPrivileges=true
StandardOutput=append:{data}/nats-server.log
StandardError=append:{data}/nats-server.log
[Install]
WantedBy=default.target
""")
    (units / 'verter-nats-bridge.service').write_text(f"""[Unit]
Description=Verter Python NATS API and test web UI
After=verter-nats.service
Wants=verter-nats.service
[Service]
ExecStart={start}
Restart=on-failure
RestartSec=5
KillMode=control-group
TimeoutStopSec=25
UMask=0077
NoNewPrivileges=true
StandardOutput=append:{data}/bridge.log
StandardError=append:{data}/bridge.log
[Install]
WantedBy=default.target
""")
    run('systemctl', '--user', 'daemon-reload')
    print(
        'Installed and validated. No service started. Start with systemctl --user enable --now verter-nats.service verter-nats-bridge.service'
    )


if __name__ == '__main__':
    main()
