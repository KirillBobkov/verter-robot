import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('runtime', Path(__file__).parents[1] / 'tools/runtime.py')
runtime = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runtime)


class Backend:
    def __init__(self):
        self.calls = []
        self.fail = False

    def validate_map(self, p):
        if p != 'valid.yaml':
            raise ValueError('Invalid map')
        return p

    def preflight(self):
        self.calls.append('preflight')

    def stop_mode(self):
        self.calls.append('stop')

    def start_mode(self, m, p):
        self.calls.append(('start', m, p))

    def wait_ready(self, m):
        self.calls.append('ready')
        if self.fail:
            raise RuntimeError('not ready')


def test_stop_before_start():
    b = Backend()
    c = runtime.ModeController(b)
    c.switch('mapping')
    assert b.calls == ['preflight', 'stop', ('start', 'mapping', None), 'ready']
    assert c.mode == 'mapping'


def test_mapping_loss_requires_explicit_choice():
    b = Backend()
    c = runtime.ModeController(b)
    c.switch('mapping')
    b.calls.clear()
    with pytest.raises(ValueError):
        c.switch('navigation', 'valid.yaml')
    assert c.mode == 'mapping' and b.calls == []


def test_invalid_map_keeps_current_mode():
    b = Backend()
    c = runtime.ModeController(b)
    c.switch('mapping')
    b.calls.clear()
    with pytest.raises(ValueError):
        c.switch('navigation', 'missing.yaml', True)
    assert c.mode == 'mapping' and b.calls == []


def test_failure_cleans_partial_group():
    b = Backend()
    b.fail = True
    c = runtime.ModeController(b)
    with pytest.raises(RuntimeError):
        c.switch('mapping')
    assert c.mode == 'fault' and b.calls[-1] == 'stop'


def test_duplicate_switch_does_not_restart():
    b = Backend()
    c = runtime.ModeController(b)
    c.switch('navigation', 'valid.yaml')
    b.calls.clear()
    c.switch('navigation', 'valid.yaml')
    assert b.calls == []


def test_explicit_switch():
    b = Backend()
    c = runtime.ModeController(b)
    c.switch('mapping')
    c.switch('navigation', 'valid.yaml', True)
    assert c.mode == 'navigation'


def test_failed_preflight_keeps_running_mode():
    b = Backend()
    c = runtime.ModeController(b)
    c.switch('navigation', 'valid.yaml')
    b.calls.clear()

    def fail():
        raise RuntimeError('no scan')

    b.preflight = fail
    with pytest.raises(RuntimeError):
        c.switch('mapping')
    assert c.mode == 'navigation' and b.calls == []


def test_process_group_stop(tmp_path):
    import sys

    b = runtime.Processes(tmp_path, tmp_path)
    b.start('test', [sys.executable, '-c', 'import time; time.sleep(60)'])
    b.stop('test')
    assert not b.children
