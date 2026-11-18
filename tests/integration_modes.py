"""Real SLAM/Nav2 in an isolated domain, synthetic sensors, no hardware launch."""

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import tempfile

assert os.environ.get('ROS_DOMAIN_ID') == '177'
assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('runtime', root / 'tools/runtime.py')
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)
diagnostics = root / 'data/diagnostics/integration'
diagnostics.mkdir(parents=True, exist_ok=True)
with tempfile.TemporaryDirectory(prefix='modes-', dir=diagnostics) as d:
    run = Path(d)
    backend = runtime.Processes(root, run)
    controller = runtime.ModeController(backend)
    try:
        backend.start('hardware', [sys.executable, str(root / 'tests/synthetic_sensors.py')])
        controller.switch('mapping')
        print('PASS mapping readiness', flush=True)
        stem = run / 'test_map'
        subprocess.run(
            [
                'ros2',
                'run',
                'nav2_map_server',
                'map_saver_cli',
                '-f',
                str(stem),
                '--ros-args',
                '-p',
                'save_map_timeout:=10.0',
                '-p',
                'map_subscribe_transient_local:=true',
            ],
            check=True,
            timeout=30,
        )
        assert stem.with_suffix('.yaml').is_file()
        print('PASS map save', flush=True)
        controller.switch('navigation', str(stem.with_suffix('.yaml')), discard=True)
        backend.probe('navigation-ready', timeout=50)
        subprocess.run([sys.executable, str(root / 'tests/verify_live_footprints.py')], check=True, timeout=35)
        print('PASS mapping -> navigation, lifecycle active', flush=True)
        controller.switch('mapping')
        print('PASS navigation -> mapping', flush=True)
        controller.switch('idle', discard=True)
        assert 'mode' not in backend.children
        print('PASS idle cleanup; hardware persists', flush=True)
        assert backend.children['hardware'].poll() is None
    finally:
        import shutil

        for log in run.glob('*.log'):
            shutil.copy2(log, diagnostics / ('integration-' + log.name))
        backend.stop('mode')
        backend.stop('hardware')
