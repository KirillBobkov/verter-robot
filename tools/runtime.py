#!/usr/bin/env python3
"""Single local owner of Verter process groups. No shell interpolation in RPC."""

import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time


class ModeController:
    """Domain orchestration; backend owns ROS/process details."""

    def __init__(self, backend):
        self.backend = backend
        self.mode = 'idle'
        self.error = None
        self.map_path = None

    def switch(self, mode, map_path=None, discard=False):
        if mode not in ('idle', 'mapping', 'localization', 'navigation'):
            raise ValueError('Unknown mode')
        if mode in ('localization', 'navigation'):
            map_path = self.backend.validate_map(map_path)
        if mode == self.mode and (mode not in ('localization', 'navigation') or map_path == self.map_path):
            return
        if self.mode == 'mapping' and mode != 'mapping' and not discard:
            raise ValueError('Save map first, then explicitly use --discard to leave the SLAM session')
        if mode != 'idle':
            self.backend.preflight()
        self.error = None
        try:
            self.backend.stop_mode()
            self.mode = 'idle'
            self.map_path = None
            if mode != 'idle':
                self.backend.start_mode(mode, map_path)
                self.backend.wait_ready(mode)
            self.mode = mode
            self.map_path = map_path
        except Exception as exc:
            self.mode = 'fault'
            self.error = str(exc)
            try:
                self.backend.stop_mode()
            except Exception as stop_exc:
                self.error += '; cleanup: ' + str(stop_exc)
            raise RuntimeError(self.error) from exc


class Processes:
    def __init__(self, root, run):
        self.root, self.run = root, run
        self.children = {}
        self.last_exit = {}

    def start(self, group, command):
        if group in self.children:
            raise RuntimeError(group + ' already running')
        logfile = open(self.run / (group + '.log'), 'ab', buffering=0)
        try:
            p = subprocess.Popen(command, start_new_session=True, stdout=logfile, stderr=subprocess.STDOUT)
        finally:
            logfile.close()
        self.children[group] = p

    def stop(self, group):
        p = self.children.get(group)
        if p is None:
            return
        # Signal the complete session even if the ros2 launch parent already exited.
        for sig, seconds in [(signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)]:
            try:
                os.killpg(p.pid, sig)
            except ProcessLookupError:
                break
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                p.poll()
                if not self.group_alive(p.pid):
                    break
                time.sleep(0.1)
            if not self.group_alive(p.pid):
                break
        if self.group_alive(p.pid):
            raise RuntimeError('Process group still alive: ' + group)
        p.wait(timeout=1)
        self.children.pop(group, None)

    @staticmethod
    def group_alive(pgid):
        # Zombies cannot publish TF/commands; they may await a host subreaper.
        for stat in Path('/proc').glob('[0-9]*/stat'):
            try:
                fields = stat.read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) == pgid and fields[0] != 'Z':
                    return True
            except (OSError, ValueError, IndexError):
                pass
        return False

    def launch(self, group, name, *args):
        self.start(group, ['ros2', 'launch', 'verter_bringup', name + '.launch.py', *args])

    def probe(self, kind, timeout=15):
        subprocess.run([sys.executable, str(self.root / 'tools/ros_probe.py'), kind], check=True, timeout=timeout)

    def preflight(self):
        if 'hardware' not in self.children or self.children['hardware'].poll() is not None:
            raise RuntimeError('Start hardware first')
        self.probe('hardware', timeout=70)

    def validate_map(self, path):
        import yaml

        if not path:
            raise ValueError('--map is required for localization or navigation')
        p = Path(path).expanduser().resolve()
        data = yaml.safe_load(p.read_text())
        image = Path(data['image'])
        if not image.is_absolute():
            image = p.parent / image
        if not image.is_file() or data.get('resolution', 0) <= 0:
            raise ValueError('Invalid map image/resolution')
        return str(p)

    def stop_mode(self):
        if 'mode' in self.children:
            # Shutdown normally cancels controller execution; process-group exit is mandatory.
            try:
                self.probe('shutdown_nav', timeout=6)
            except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
                pass  # Group termination below is mandatory even if lifecycle services fail.
            self.stop('mode')
            self.probe('zero', timeout=8)

    def start_mode(self, mode, path):
        if mode == 'localization':
            # Run after the old owned mode exited. This check never stops an
            # external process; it rejects conflicting TF/map owners instead.
            self.probe('localization-exclusive', timeout=8)
        arguments = ['map:=' + path] if mode in ('localization', 'navigation') else []
        self.launch('mode', mode, *arguments)

    def localization_status(self):
        result = subprocess.run(
            [sys.executable, str(self.root / 'tools/ros_probe.py'), 'localization-status'],
            check=True,
            timeout=10,
            capture_output=True,
            text=True,
        )
        return json.loads(result.stdout)

    def wait_ready(self, mode):
        try:
            self.probe(mode, timeout=50)
        except Exception:
            raise RuntimeError('Mode not ready; inspect run/mode.log')
        if self.children['mode'].poll() is not None:
            raise RuntimeError('Launch process exited during startup')

    def check_exits(self):
        dead = {k: p.returncode for k, p in self.children.items() if p.poll() is not None}
        self.last_exit.update(dead)
        return dead


def serve(root, run):
    run.mkdir(parents=True, exist_ok=True, mode=0o700)
    lock = open(run / 'supervisor.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    sock_path = run / 'control.sock'
    sock_path.unlink(missing_ok=True)
    server = socket.socket(socket.AF_UNIX)
    server.bind(str(sock_path))
    os.chmod(sock_path, 0o600)
    server.listen(4)
    server.settimeout(0.5)
    backend = Processes(root, run)
    controller = ModeController(backend)
    stopping = False

    def stop_signal(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)
    try:
        while not stopping:
            dead = backend.check_exits()
            if dead:
                if 'hardware' in dead or 'mode' in dead:
                    controller.mode = 'fault'
                    controller.error = 'Unexpected process exit: ' + str(dead)
                    try:
                        backend.stop_mode()
                    except Exception as exc:
                        controller.error += '; ' + str(exc)
                for group in dead:
                    backend.stop(group)
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            with conn:
                conn.settimeout(3)
                try:
                    data = b''
                    while b'\n' not in data:
                        chunk = conn.recv(4096)
                        if not chunk or len(data) > 65536:
                            raise ValueError('Invalid request')
                        data += chunk
                    request = json.loads(data)
                    cmd = request['command']
                    if cmd == 'mode':
                        controller.switch(request['mode'], request.get('map'), request.get('discard', False))
                    elif cmd == 'hardware':
                        if request['action'] == 'start':
                            backend.probe('exclusive')
                            backend.launch('hardware', 'hardware')
                            # Independent nodes retry individually. Sensor readiness gates modes,
                            # not startup of unrelated hardware, voice or web.
                        else:
                            controller.switch('idle', discard=True)
                            backend.stop('hardware')
                    elif cmd == 'app':
                        name = request['name']
                        if name not in ('voice', 'web', 'head'):
                            raise ValueError('Unknown app')
                        if request['action'] == 'start':
                            if name == 'head':
                                raise ValueError('DOA disabled: ESP32 ctrl has priority; head port is not configured')
                            backend.launch(name, name)
                        else:
                            backend.stop(name)
                    elif cmd == 'save-map':
                        if controller.mode != 'mapping':
                            raise ValueError('Save map is available in mapping mode')
                        p = Path(request['path']).expanduser().resolve()
                        if Path(str(p) + '.yaml').exists() or Path(str(p) + '.pgm').exists():
                            raise ValueError('Map already exists; choose a new name')
                        p.parent.mkdir(parents=True, exist_ok=True)
                        subprocess.run(
                            [
                                'ros2',
                                'run',
                                'nav2_map_server',
                                'map_saver_cli',
                                '-f',
                                str(p),
                                '--ros-args',
                                '-p',
                                'save_map_timeout:=10.0',
                                '-p',
                                'map_subscribe_transient_local:=true',
                            ],
                            check=True,
                            timeout=30,
                        )
                    elif cmd == 'health':
                        result = subprocess.run(
                            [sys.executable, str(root / 'tools/ros_probe.py'), 'hardware-status'],
                            check=True,
                            timeout=10,
                            capture_output=True,
                            text=True,
                        )
                        health = json.loads(result.stdout)
                    elif cmd == 'localization-status':
                        localization = backend.localization_status()
                    elif cmd == 'shutdown':
                        stopping = True
                    elif cmd != 'status':
                        raise ValueError('Unknown command')
                    response = {
                        'ok': True,
                        'mode': controller.mode,
                        'error': controller.error,
                        'map': controller.map_path,
                        'groups': {k: p.pid for k, p in backend.children.items()},
                        'last_exit': backend.last_exit,
                    }
                    if cmd == 'health':
                        response['health'] = health
                    if cmd == 'localization-status':
                        response['localization'] = localization
                except Exception as exc:
                    response = {'ok': False, 'error': str(exc), 'mode': controller.mode}
                try:
                    conn.sendall(json.dumps(response).encode() + b'\n')
                except OSError:
                    pass
    finally:
        try:
            backend.stop_mode()
        finally:
            for group in ['head', 'voice', 'web', 'hardware']:
                backend.stop(group)
        server.close()
        sock_path.unlink(missing_ok=True)
        lock.close()


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ['serve', 'status', 'health', 'localization-status', 'shutdown']:
        sub.add_parser(name)
    p = sub.add_parser('mode')
    p.add_argument('mode', choices=['idle', 'mapping', 'localization', 'navigation'])
    p.add_argument('--map')
    p.add_argument('--discard', action='store_true')
    p = sub.add_parser('hardware')
    p.add_argument('action', choices=['start', 'stop'])
    p = sub.add_parser('app')
    p.add_argument('name', choices=['voice', 'web', 'head'])
    p.add_argument('action', choices=['start', 'stop'])
    p = sub.add_parser('save-map')
    p.add_argument('path')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent.parent
    run = Path(os.environ.get('VERTER_RUN_DIR', str(root / 'run')))
    if args.command == 'serve':
        serve(root, run)
    else:
        with socket.socket(socket.AF_UNIX) as s:
            s.settimeout(150)
            s.connect(str(run / 'control.sock'))
            s.sendall(json.dumps(vars(args)).encode() + b'\n')
            data = b''
            while b'\n' not in data:
                chunk = s.recv(4096)
                if not chunk:
                    raise RuntimeError('Supervisor disconnected')
                data += chunk
        response = json.loads(data)
        print(json.dumps(response, ensure_ascii=False, indent=2))
        if not response['ok']:
            raise SystemExit(1)


if __name__ == '__main__':
    main()
