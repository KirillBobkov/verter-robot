"""Install camera-only permissions and unit; never restart robot groups."""

import json, secrets, subprocess, shutil
from pathlib import Path

root = Path(__file__).resolve().parents[1]
backup = root / 'data/backups/camera-install'
backup.mkdir(parents=True, exist_ok=True, mode=0o700)
server_path = root / 'config/nats/server.conf'
camera_path = root / 'config/nats/camera.json'
if not (backup / 'server.conf.before').exists():
    shutil.copy2(server_path, backup / 'server.conf.before')
bridge = json.loads((root / 'config/nats/bridge.json').read_text())
server = json.loads(server_path.read_text())
robot, bucket, domain = bridge['robot_id'], bridge['bucket'], bridge['domain']
config = json.loads((root / 'services/camera/config.example.json').read_text(encoding='utf-8-sig'))
if camera_path.exists():
    config.update(json.loads(camera_path.read_text()))
config.update(
    robot_id=robot,
    bucket=bucket,
    domain=domain,
    nats_url=bridge['nats_url'],
    user='camera',
    client_user='camera_viewer',
)
config.setdefault('password', secrets.token_urlsafe(32))
config.setdefault('client_password', secrets.token_urlsafe(32))
commands = [f'{robot}.camera.command.{s}' for s in ('status', 'ice', 'offer', 'close')]
write = [f'{robot}.camera.update.>', f'$KV.{bucket}.camera.>', '_INBOX.>']
for prefix in (f'$JS.{domain}.API', '$JS.API'):
    write += [f'{prefix}.STREAM.INFO.KV_{bucket}', f'{prefix}.$KV.{bucket}.camera.>']
users = server['authorization']['users']
for name, password, permissions in [
    ('camera', config['password'], {'publish': write, 'subscribe': commands + ['_INBOX.>']}),
    ('camera_viewer', config['client_password'], {'publish': commands, 'subscribe': ['_INBOX.>']}),
]:
    user = next((u for u in users if u['user'] == name), None)
    if user is None:
        user = {}
        users.append(user)
    user.update(user=name, password=password, permissions=permissions)
client = next(u for u in users if u['user'] == bridge['client_user'])
for subject in commands:
    if subject not in client['permissions']['publish']:
        client['permissions']['publish'].append(subject)
server_path.write_text(json.dumps(server, indent=2))
server_path.chmod(0o600)
camera_path.write_text(json.dumps(config, indent=2))
camera_path.chmod(0o600)
subprocess.run([str(root / 'vendor/nats/nats-server'), '-t', '-c', str(server_path)], check=True)
subprocess.run(['systemctl', '--user', 'kill', '--kill-who=main', '--signal=HUP', 'verter-nats.service'], check=True)
unit = Path.home() / '.config/systemd/user/verter-camera.service'
unit.write_text((root / 'deploy/verter-camera.service.in').read_text(encoding='utf-8-sig').replace('@ROOT@', str(root)))
subprocess.run(['systemctl', '--user', 'daemon-reload'], check=True)
subprocess.run(['systemctl', '--user', 'enable', '--now', 'verter-camera.service'], check=True)
print('Camera unit enabled; NATS permissions reloaded; no robot service restarted')
