"""PatrolUseCase — pure Python, zero ROS imports.

Application layer: drives the patrol loop using PatrolPolicy (SI-6) and ActionClientPort (C1).
The caller drives the loop by calling step() from a dedicated thread.
"""
from __future__ import annotations

from verter_admin.waypoints.domain.patrol_policy import PatrolPolicy, PatrolState
from verter_admin.waypoints.domain.waypoint_store import WaypointStore
from verter_admin.waypoints.application.navigate_to import ActionClientPort


class PatrolUseCase:
    """Iterate waypoints in insertion order (A7: FIFO) and manage PatrolPolicy transitions.

    The use-case contains no ROS calls. All state-machine logic is delegated to
    PatrolPolicy (SI-6). All navigation is delegated to ActionClientPort (C1).
    """

    def __init__(
        self,
        store: WaypointStore,
        policy: PatrolPolicy,
        action_client: ActionClientPort,
    ) -> None:
        self._store = store
        self._policy = policy
        self._action_client = action_client
        self._index = 0

    @property
    def policy(self) -> PatrolPolicy:
        return self._policy

    def start(self) -> None:
        """Begin patrol from the first waypoint in insertion order."""
        self._index = 0
        self._policy.start()

    def step(self) -> PatrolState:
        """Navigate to the next waypoint and return the current policy state.

        Advances the index cyclically over insertion-ordered waypoints.
        Returns PatrolState.FAULT immediately without navigating if policy is in FAULT.
        Returns current state unchanged if the store is empty.
        """
        if self._policy.state == PatrolState.FAULT:
            return PatrolState.FAULT

        waypoints = self._store.list_all()
        if not waypoints:
            return self._policy.state

        wp = waypoints[self._index % len(waypoints)]
        self._index = (self._index + 1) % len(waypoints)

        success = self._action_client.send_goal(wp)
        if success:
            self._policy.on_success()
        else:
            self._policy.on_failure()

        return self._policy.state

    def stop(self) -> None:
        """Stop patrol: cancel the active navigation goal and transition policy to IDLE."""
        self._action_client.cancel_goal()
        self._policy.stop()
