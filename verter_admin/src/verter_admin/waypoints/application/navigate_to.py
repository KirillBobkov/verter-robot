"""NavigateToUseCase — pure Python, zero ROS imports.

Application layer: resolves a named waypoint and delegates to ActionClientPort.
"""
from __future__ import annotations

from abc import ABC, abstractmethod

from verter_admin.waypoints.domain.waypoint_store import Waypoint, WaypointStore


class ActionClientPort(ABC):
    """Port for sending navigation goals to an action server (C1)."""

    @abstractmethod
    def send_goal(self, waypoint: Waypoint) -> bool:
        """Send a navigation goal. Returns True if the goal was accepted and succeeded."""
        ...

    @abstractmethod
    def cancel_goal(self) -> None:
        """Cancel the currently active navigation goal, if any."""
        ...


class NavigateToUseCase:
    """Navigate the robot to a named waypoint.

    Raises KeyError if the waypoint name is not found in the store.
    Returns True if the action server accepted and completed the goal.
    """

    def __init__(self, store: WaypointStore, action_client: ActionClientPort) -> None:
        self._store = store
        self._action_client = action_client

    def execute(self, name: str) -> bool:
        waypoint = self._store.get(name)        # raises KeyError if unknown
        return self._action_client.send_goal(waypoint)
