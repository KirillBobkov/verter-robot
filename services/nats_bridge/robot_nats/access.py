"""Add public read/command permissions without changing credentials or listeners."""

from .contracts import COMMANDS
from .places import SPATIAL_COMMANDS


def configure_client(server, config):
    user = next(u for u in server['authorization']['users'] if u['user'] == config['client_user'])
    permissions = user['permissions']
    publish = permissions['publish']
    subscribe = permissions['subscribe']
    robot, bucket = config['robot_id'], config['bucket']
    for domain, operation in SPATIAL_COMMANDS.values():
        subject = f'{robot}.{domain}.command.{operation}'
        if subject not in publish:
            publish.append(subject)
        if 'backend_user' in config:
            backend = next(u for u in server['authorization']['users'] if u['user'] == config['backend_user'])
            if 'permissions' in backend and subject not in backend['permissions']['subscribe']:
                backend['permissions']['subscribe'].append(subject)
    for subject in [f'{robot}.service.command.{name}' for name in COMMANDS]:
        if subject not in publish:
            publish.append(subject)
    for operation in ('status', 'set', 'keepalive', 'stop'):
        subject = f'{robot}.cmd_vel.command.{operation}'
        if operation == 'status' or config.get('allow_manual_motion', False):
            if subject not in publish:
                publish.append(subject)
        elif subject in publish:
            publish.remove(subject)
    subject = f'{robot}.battery.command.history'
    if subject not in publish:
        publish.append(subject)
    if 'backend_user' in config:
        backend = next(u for u in server['authorization']['users'] if u['user'] == config['backend_user'])
        if 'permissions' in backend and subject not in backend['permissions']['subscribe']:
            backend['permissions']['subscribe'].append(subject)
    for operation in ('get', 'save', 'delete'):
        subject = f'{robot}.map.command.{operation}'
        if subject not in publish:
            publish.append(subject)
        if 'backend_user' in config:
            backend = next(u for u in server['authorization']['users'] if u['user'] == config['backend_user'])
            if 'permissions' in backend and subject not in backend['permissions']['subscribe']:
                backend['permissions']['subscribe'].append(subject)
    subject = f'{robot}.costmap.command.get'
    if subject not in publish:
        publish.append(subject)
    if 'backend_user' in config:
        backend = next(u for u in server['authorization']['users'] if u['user'] == config['backend_user'])
        if 'permissions' in backend and subject not in backend['permissions']['subscribe']:
            backend['permissions']['subscribe'].append(subject)
    # KV watch uses an ephemeral ordered consumer. No stream/KV write or delete access.
    for prefix in (f'$JS.{config["domain"]}.API', '$JS.API'):
        for suffix in (
            f'CONSUMER.CREATE.KV_{bucket}',
            f'CONSUMER.CREATE.KV_{bucket}.>',
            f'CONSUMER.DELETE.KV_{bucket}.*',
            f'CONSUMER.INFO.KV_{bucket}.*',
            f'CONSUMER.MSG.NEXT.KV_{bucket}.*',
        ):
            subject = prefix + '.' + suffix
            if subject not in publish:
                publish.append(subject)
    subject = f'{robot}.*.update.>'
    if subject not in subscribe:
        subscribe.append(subject)
