"""Bounded Unix-socket client. The supervisor remains the only ROS process owner."""

import json
from pathlib import Path
import socket


class SupervisorClient:
    def __init__(self, root, socket_path=None):
        self.path = Path(socket_path) if socket_path else Path(root) / 'run/control.sock'

    def call(self, request, timeout=3):
        with socket.socket(socket.AF_UNIX) as connection:
            connection.settimeout(timeout)
            connection.connect(str(self.path))
            connection.sendall(json.dumps(request, allow_nan=False).encode() + b'\n')
            data = bytearray()
            while b'\n' not in data:
                chunk = connection.recv(4096)
                if not chunk:
                    raise RuntimeError('supervisor_disconnected')
                data.extend(chunk)
                if len(data) > 65536:
                    raise RuntimeError('supervisor_response_too_large')
        result = json.loads(data)
        if not isinstance(result, dict) or type(result.get('ok')) is not bool:
            raise RuntimeError('invalid_supervisor_response')
        return result
