"""Unit tests for YamlStore. Zero rclpy imports.

Covers SI-10 / C8 atomic write behaviour.
Named test test_atomic_write_exception is the SI-10 scenario required by the brief.
"""
import os
import pytest
from unittest.mock import patch

from verter_admin.waypoints.infrastructure.yaml_store import YamlStore


@pytest.fixture
def tmp_yaml(tmp_path):
    return str(tmp_path / 'waypoints.yaml')


def test_load_nonexistent_returns_empty(tmp_yaml):
    store = YamlStore(tmp_yaml)
    assert store.load() == {}


def test_round_trip(tmp_yaml):
    store = YamlStore(tmp_yaml)
    data = {'home': {'x': 1.0, 'y': 2.0, 'theta': 0.0}}
    store.save(data)
    assert store.load() == data


def test_multiple_waypoints(tmp_yaml):
    store = YamlStore(tmp_yaml)
    data = {
        'a': {'x': 0.0, 'y': 0.0, 'theta': 0.0},
        'b': {'x': 5.0, 'y': 3.0, 'theta': 1.57},
    }
    store.save(data)
    loaded = store.load()
    assert loaded['b']['x'] == 5.0
    assert loaded['a']['theta'] == 0.0


def test_expanduser_applied(tmp_path, monkeypatch):
    """C10: os.path.expanduser must be applied to the path."""
    monkeypatch.setenv('HOME', str(tmp_path))
    store = YamlStore('~/waypoints.yaml')
    store.save({'wp': {'x': 1.0, 'y': 0.0, 'theta': 0.0}})
    assert store.load() == {'wp': {'x': 1.0, 'y': 0.0, 'theta': 0.0}}


def test_atomic_write_exception_leaves_original_unchanged(tmp_yaml):
    """SI-10: if os.replace raises, the original file MUST remain unchanged."""
    store = YamlStore(tmp_yaml)
    original = {'original': {'x': 0.0, 'y': 0.0, 'theta': 0.0}}
    store.save(original)

    with patch('os.replace', side_effect=OSError('simulated failure')):
        with pytest.raises(OSError):
            store.save({'new': {'x': 1.0, 'y': 1.0, 'theta': 1.0}})

    # Original must be readable and unchanged
    assert store.load() == original


def test_save_creates_parent_dirs(tmp_path):
    nested = str(tmp_path / 'deep' / 'nested' / 'waypoints.yaml')
    store = YamlStore(nested)
    store.save({'p': {'x': 1.0, 'y': 1.0, 'theta': 0.0}})
    assert os.path.exists(nested)


def test_overwrite_updates_content(tmp_yaml):
    store = YamlStore(tmp_yaml)
    store.save({'a': {'x': 1.0, 'y': 1.0, 'theta': 0.0}})
    store.save({'b': {'x': 2.0, 'y': 2.0, 'theta': 0.0}})
    loaded = store.load()
    assert 'b' in loaded
    assert 'a' not in loaded
