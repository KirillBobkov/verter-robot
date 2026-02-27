"""SaveWaypointUseCase — pure Python, zero ROS imports.

Application layer: orchestrates domain WaypointStore and YamlStorePort.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from verter_admin.waypoints.domain.waypoint_store import Waypoint, WaypointStore


class YamlStorePort(ABC):
    """Port for persisting the waypoint store to durable storage."""

    @abstractmethod
    def save(self, data: dict) -> None:
        """Persist the serialised waypoint dict atomically."""
        ...

    @abstractmethod
    def load(self) -> dict:
        """Load and return the serialised waypoint dict. Returns {} if not found."""
        ...


class SaveWaypointUseCase:
    """Save a new named waypoint to the in-memory store and persist to storage.

    Raises ValueError if a waypoint with the same name already exists.
    """

    def __init__(self, store: WaypointStore, port: YamlStorePort) -> None:
        self._store = store
        self._port = port

    def execute(self, name: str, x: float, y: float, theta: float) -> None:
        waypoint = Waypoint(name=name, x=x, y=y, theta=theta)
        self._store.save(waypoint)          # raises ValueError on duplicate
        self._port.save(self._store.to_dict())
