"""YamlStore — YAML-backed implementation of YamlStorePort.

Implements atomic write via tempfile + os.replace (SI-10 / C8).
Applies os.path.expanduser to support ~/... paths (C10).

R5 note: tempfile.NamedTemporaryFile is always created in the same directory as the
target file to guarantee they reside on the same filesystem, preventing cross-device
rename errors on systems with separate tmpfs mounts.
"""
from __future__ import annotations

import os
import tempfile

import yaml

from verter_admin.waypoints.application.save_waypoint import YamlStorePort


class YamlStore(YamlStorePort):
    """Persist the waypoint dict to a YAML file using atomic rename."""

    def __init__(self, path: str) -> None:
        self._path = os.path.expanduser(path)

    def load(self) -> dict:
        """Load waypoints from file. Returns {} if the file does not exist."""
        path = os.path.expanduser(self._path)
        if not os.path.exists(path):
            return {}
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        return data if isinstance(data, dict) else {}

    def save(self, data: dict) -> None:
        """Atomically write data to the YAML file (tempfile + os.replace).

        If os.replace raises (e.g., permission error), the original file is left
        unchanged. The temp file is cleaned up on failure.
        """
        path = os.path.expanduser(self._path)
        target_dir = os.path.dirname(path) or '.'
        os.makedirs(target_dir, exist_ok=True)

        # Temp file in same directory → same filesystem → POSIX-atomic rename (R5/C8)
        fd, tmp_path = tempfile.mkstemp(dir=target_dir, suffix='.yaml.tmp')
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as f:
                yaml.safe_dump(data, f, default_flow_style=False, allow_unicode=True)
            os.replace(tmp_path, path)
        except Exception:
            # Original file is unchanged; clean up orphaned temp file
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise
