#!/usr/bin/env python3
"""Install the read-only BMS publisher; no ROS service or BMS setting changes."""

import json
from pathlib import Path
import secrets
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def main():
    component = ROOT / 'services/nats_bridge'
    cfgdir = ROOT / 'config/nats'
    bridgepath = cfgdir / 'bridge.json'
    bridge = json.loads(bridgepath.read_text())
    env = ROOT / '.venv-bms-ble'
    if not env.exists():
        subprocess.run(['python3', '-m', 'venv', str(env)], check=True)
    subprocess.run(
        [str(env / 'bin/python'), '-m', 'pip', 'install', '-r', str(component / 'requirements-bms.txt')], check=True
    )
    data = ROOT / 'data/bms'
    data.mkdir(parents=True, exist_ok=True)
    path = cfgdir / 'bms.json'
    if not path.exists():
        config = {
            'robot_id': bridge['robot_id'],
            'nats_url': bridge['nats_url'],
            'user': 'bms_reader',
            'password': secrets.token_hex(24),
            'address': 'C8:47:80:55:2F:09',
            'adapter': 'hci0',
            'data_dir': str(data),
            'expected_device': {'model': 'JK-B2A8S20P', 'hardware': '19H', 'software': '19.31'},
        }
        path.write_text(json.dumps(config, indent=2))
        path.chmod(0o600)
    config = json.loads(path.read_text())
    serverpath = cfgdir / 'server.conf'
    server = json.loads(serverpath.read_text())
    users = server['authorization']['users']
    users[:] = [u for u in users if u['user'] != config['user']]
    subject = f'robots.{bridge["robot_id"]}.v1.bms'
    users.append(
        {'user': config['user'], 'password': config['password'], 'permissions': {'publish': [subject], 'subscribe': []}}
    )
    for user in users:
        if user['user'] == bridge['client_user'] and subject not in user['permissions']['subscribe']:
            user['permissions']['subscribe'].append(subject)
    candidate = cfgdir / 'server.candidate.conf'
    candidate.write_text(json.dumps(server, indent=2))
    candidate.chmod(0o600)
    subprocess.run([str(ROOT / 'vendor/nats/nats-server'), '-t', '-c', str(candidate)], check=True)
    candidate.replace(serverpath)
    example = cfgdir / 'server.with-hub.example.conf'
    if example.exists():
        template = json.loads(example.read_text())
        template['authorization'] = server['authorization']
        example.write_text(json.dumps(template, indent=2))
        example.chmod(0o600)
    bridge.pop('web_token', None)
    bridgepath.write_text(json.dumps(bridge, indent=2))
    bridgepath.chmod(0o600)
    unit = Path.home() / '.config/systemd/user/verter-bms.service'
    unit.write_text(f"""[Unit]
Description=Verter read-only JK BMS BLE to NATS
After=verter-nats.service
Wants=verter-nats.service
[Service]
WorkingDirectory={component}
ExecStart={env}/bin/python -m robot_nats.bms_service --config {path}
Restart=on-failure
RestartSec=5
TimeoutStopSec=15
UMask=0077
NoNewPrivileges=true
StandardOutput=append:{data}/service.log
StandardError=append:{data}/service.log
[Install]
WantedBy=default.target
""")
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    print('BMS installed. Reload NATS config, restart bridge and enable verter-bms.service to activate.')


if __name__ == '__main__':
    main()
