"""Install lidar credentials, ACL and inactive unit. Never starts hardware or services."""

import json
import os
from pathlib import Path
import secrets
import subprocess


def main():
    os.umask(0o077)
    root = Path(__file__).resolve().parents[2]
    folder = root / 'config/nats'
    bridge = json.loads((folder / 'bridge.json').read_text())
    path = folder / 'lidar.json'
    config = {
        'robot_id': bridge['robot_id'],
        'nats_url': bridge['nats_url'],
        'user': 'lidar_reader',
        'password': secrets.token_urlsafe(32),
        'urdf': str(root / 'ros_ws/src/verter_description/urdf/verter_robot.urdf'),
        'web_enabled': True,
        'web_host': bridge.get('web_host', '127.0.0.1'),
        'web_port': 8090,
        'web_allowed_hosts': bridge.get('web_allowed_hosts', ['localhost', '127.0.0.1', '::1']),
    }
    if path.exists():
        config.update(json.loads(path.read_text()))
    server_path = folder / 'server.conf'
    server = json.loads(server_path.read_text())
    users = server['authorization']['users']
    users[:] = [u for u in users if u['user'] != config['user']]
    subject = config['robot_id'] + '.lidar.update.*'
    users.append(
        {
            'user': config['user'],
            'password': config['password'],
            'permissions': {'publish': [subject], 'subscribe': [subject]},
        }
    )
    candidate = folder / 'lidar-server.candidate.conf'
    candidate.write_text(json.dumps(server, indent=2))
    candidate.chmod(0o600)
    try:
        subprocess.run([str(root / 'vendor/nats/nats-server'), '-t', '-c', str(candidate)], check=True)
        path.write_text(json.dumps(config, indent=2))
        path.chmod(0o600)
        candidate.replace(server_path)
    finally:
        candidate.unlink(missing_ok=True)
    template = root / 'deploy/verter-lidar.service.in'
    target = Path.home() / '.config/systemd/user/verter-lidar.service'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(template.read_text().replace('@ROOT@', str(root)))
    subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
    print('Installed inactive verter-lidar. Reload NATS ACL, then enable the lidar service.')


if __name__ == '__main__':
    main()
