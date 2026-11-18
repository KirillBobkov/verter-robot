"""Offline tests: launch only inert Python children, never robot executables."""

import importlib.util
import os
from pathlib import Path
import socket
import subprocess
import sys
from types import SimpleNamespace as S

import pytest

ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from sensor_health import sample_valid, missing_inputs


def scan(stamp=100.0, ranges=(1.0, 2.0)):
    return S(
        header=S(stamp=S(sec=int(stamp), nanosec=int(stamp % 1 * 1e9)), frame_id='lidar_link'),
        ranges=ranges,
        angle_increment=0.01,
        range_min=0.1,
        range_max=25.0,
    )


def test_dead_or_invalid_lidar_blocks_readiness():
    assert sample_valid('/scan', scan(), 100.2)
    assert not sample_valid('/scan', scan(), 102.0)
    assert not sample_valid('/scan', scan(105.0), 100.0)
    assert not sample_valid('/scan', scan(ranges=(float('inf'), float('nan'))), 100.2)
    assert missing_inputs({'/odometry/filtered': 20.0, '/imu/data': 20.0}, 20.1) == ['/scan']
    assert '/scan' in missing_inputs({'/scan': 10.0}, 20.0)


def test_autostart_attempts_apps_after_hardware_failure(tmp_path):
    (tmp_path / 'tools').mkdir()
    (tmp_path / 'run').mkdir()
    (tmp_path / 'bin').mkdir()
    (tmp_path / 'tools/autostart.sh').write_bytes((ROOT / 'tools/autostart.sh').read_bytes())
    (tmp_path / 'tools/verter').write_text("""#!/bin/bash
echo "$*" >> calls
if [[ "$1" == status ]]; then
  echo '{"ok":true,"groups":{}}'
  exit 0
fi
[[ "$1" == hardware ]] && exit 1
exit 0
""")
    sleeper = tmp_path / 'bin/sleep'
    sleeper.write_text('#!/bin/sh\nexit 0\n')
    sleeper.chmod(0o755)
    with socket.socket(socket.AF_UNIX) as sock:
        sock.bind(str(tmp_path / 'run/control.sock'))
        result = subprocess.run(
            ['bash', 'tools/autostart.sh'],
            cwd=tmp_path,
            env={**os.environ, 'PATH': str(tmp_path / 'bin') + ':' + os.environ['PATH']},
            capture_output=True,
            timeout=10,
        )
    calls = (tmp_path / 'calls').read_text().splitlines()
    assert result.returncode == 1
    assert 'hardware start' in calls and 'app voice start' in calls and 'app web start' in calls


@pytest.mark.parametrize(
    'group,failed', [('hardware', 'lslidar_driver_node'), ('voice', 'speech_to_text_node'), ('web', 'web_server_node')]
)
def test_one_failed_child_does_not_stop_siblings(group, failed, monkeypatch):
    from launch import LaunchService
    from launch.actions import ExecuteProcess, TimerAction, EmitEvent, RegisterEventHandler
    from launch.events import Shutdown
    from launch.event_handlers import OnProcessStart

    path = ROOT / f'ros_ws/src/verter_bringup/launch/{group}.launch.py'
    spec = importlib.util.spec_from_file_location('isolated_' + group, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setenv('VERTER_WEB_DIR', str(ROOT / 'web'))
    if hasattr(module, 'share'):
        monkeypatch.setattr(module, 'share', lambda name: str(ROOT / 'ros_ws/src' / name))
    started = []
    created = []
    names = {}

    def fake_node(**kwargs):
        name = kwargs.get('name', kwargs['executable'])
        created.append(name)
        # Substitute every executable, including driver/agent/voice/web. No ROS I/O.
        action = ExecuteProcess(
            name=name,
            cmd=[
                sys.executable,
                '-c',
                'import time; time.sleep(.1); raise SystemExit(2)' if name == failed else 'import time; time.sleep(30)',
            ],
            respawn=kwargs.get('respawn', False),
            respawn_delay=0.2,
        )
        names[action] = name
        return action

    monkeypatch.setattr(module, 'Node', fake_node)
    description = module.generate_launch_description()
    description.entities.insert(
        0, RegisterEventHandler(OnProcessStart(on_start=lambda event, context: started.append(names[event.action])))
    )
    description.add_action(TimerAction(period=2.0, actions=[EmitEvent(event=Shutdown(reason='offline test done'))]))
    service = LaunchService()
    service.include_launch_description(description)
    service.run()
    assert started.count(failed) >= 2
    assert all(started.count(name) == 1 for name in created if name != failed)
