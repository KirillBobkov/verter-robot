"""Waypoint domain model and CRUD store.

Pure Python — zero ROS imports.
Invariants:
- Duplicate names raise ValueError.
- Unknown names raise KeyError.
- Insertion order is preserved for patrol iteration (A7: FIFO).
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List


@dataclass
class Waypoint:
    """A named pose in the map frame. x, y in metres (SI); theta yaw in radians (SI)."""

    name: str
    x: float
    y: float
    theta: float


class WaypointStore:
    """In-memory CRUD store for Waypoint objects.

    Insertion order is preserved via a regular dict (Python 3.7+ guarantees).
    """

    def __init__(self) -> None:
        self._waypoints: Dict[str, Waypoint] = {}

    def save(self, waypoint: Waypoint) -> None:
        """Add a new waypoint. Raises ValueError if the name already exists."""
        if waypoint.name in self._waypoints:
            raise ValueError(f"Waypoint '{waypoint.name}' already exists")
        self._waypoints[waypoint.name] = waypoint

    def get(self, name: str) -> Waypoint:
        """Retrieve a waypoint by name. Raises KeyError if not found."""
        if name not in self._waypoints:
            raise KeyError(f"Waypoint '{name}' not found")
        return self._waypoints[name]

    def delete(self, name: str) -> None:
        """Remove a waypoint by name. Raises KeyError if not found."""
        if name not in self._waypoints:
            raise KeyError(f"Waypoint '{name}' not found")
        del self._waypoints[name]

    def list_all(self) -> List[Waypoint]:
        """Return all waypoints in insertion order."""
        return list(self._waypoints.values())

    def load_from_dict(self, data: dict) -> None:
        """Replace the in-memory store from deserialised YAML.

        Supported schemas:
        - Current: {"name": {"x": ..., "y": ..., "theta": ...}, ...}
        - Legacy: {"waypoints": [{"name": ..., "x": ..., "y": ..., "yaw": ...}, ...]}
        """
        self._waypoints = {}
        for name, fields in self._iter_serialized_waypoints(data):
            self._waypoints[name] = Waypoint(
                name=name,
                x=float(fields['x']),
                y=float(fields['y']),
                theta=float(fields['theta']),
            )

    def _iter_serialized_waypoints(self, data):
        """Yield (name, fields) in a normalized {'x','y','theta'} representation."""
        if not isinstance(data, dict):
            return []

        # Legacy schema: top-level 'waypoints' list with {'name', 'x', 'y', 'yaw'/'theta'}.
        legacy = data.get('waypoints')
        if isinstance(legacy, list):
            items = []
            for entry in legacy:
                if not isinstance(entry, dict):
                    continue
                name = entry.get('name')
                if not name:
                    continue
                theta = entry.get('theta', entry.get('yaw'))
                if theta is None:
                    continue
                if 'x' not in entry or 'y' not in entry:
                    continue
                items.append(
                    (
                        str(name),
                        {'x': entry['x'], 'y': entry['y'], 'theta': theta},
                    )
                )
            return items

        # Current schema: map of waypoint_name -> {'x', 'y', 'theta'}.
        items = []
        for name, fields in data.items():
            if not isinstance(fields, dict):
                continue
            if 'x' not in fields or 'y' not in fields:
                continue
            theta = fields.get('theta', fields.get('yaw'))
            if theta is None:
                continue
            items.append((str(name), {'x': fields['x'], 'y': fields['y'], 'theta': theta}))
        return items

    def to_dict(self) -> dict:
        """Serialise the store to a YAML-compatible dict."""
        return {
            wp.name: {'x': wp.x, 'y': wp.y, 'theta': wp.theta}
            for wp in self._waypoints.values()
        }
