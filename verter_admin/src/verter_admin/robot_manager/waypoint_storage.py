import os
import threading
from typing import Optional

import yaml


class WaypointStorage:

    def __init__(self, path: str):
        self.path = path
        self.lock = threading.RLock()
        self.data = {"waypoints": {}}

        self.load()

    def load(self):
        with self.lock:
            if not os.path.exists(self.path):
                os.makedirs(os.path.dirname(self.path), exist_ok=True)
                self.save()
                return

            with open(self.path, "r", encoding="utf-8") as file:
                data = yaml.safe_load(file)

            if not data:
                data = {}

            self.data = {"waypoints": data.get("waypoints", {})}

    def save(self):
        with self.lock:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)

            temporary_path = self.path + ".tmp"

            with open(temporary_path, "w", encoding="utf-8",) as file:
                yaml.safe_dump(self.data, file, allow_unicode=True, sort_keys=False)

            os.replace(temporary_path, self.path)

    def list(self) -> dict:
        with self.lock:
            return dict(self.data["waypoints"])

    def get(self, waypoint_id: str) -> Optional[dict]:
        with self.lock:
            return self.data["waypoints"].get(waypoint_id)

    def get_base(self) -> Optional[dict]:
        with self.lock:
            for waypoint_id, waypoint in self.data["waypoints"].items():
                if waypoint.get("is_base", False):
                    return {"id": waypoint_id, **waypoint}

        return None

    def create(self, waypoint_id: str, name: str, x: float, y: float, yaw: float, is_base: bool=False) -> dict:
        with self.lock:
            if waypoint_id in self.data["waypoints"]:
                raise ValueError(f"Waypoint '{waypoint_id}' already exists")

            if is_base:
                self._clear_base()

            waypoint = {
                "name": name,
                "is_base": is_base,
                "x": float(x),
                "y": float(y),
                "yaw": float(yaw),
            }

            self.data["waypoints"][waypoint_id] = waypoint

            self.save()

            return {"id": waypoint_id, **waypoint}

    def update(self, waypoint_id: str, values: dict) -> dict:
        with self.lock:
            waypoint = self.data["waypoints"].get(waypoint_id)

            if waypoint is None:
                raise KeyError(f"Waypoint '{waypoint_id}' not found")

            if values.get("is_base") is True:
                self._clear_base()

            for key in ("name", "x", "y", "yaw", "is_base"):
                if key in values:
                    waypoint[key] = values[key]

            self.save()

            return {"id": waypoint_id, **waypoint}

    
    def delete(self, waypoint_id: str):
        with self.lock:
            waypoint = self.data["waypoints"].get(waypoint_id)

            if waypoint is None:
                raise KeyError(f"Waypoint '{waypoint_id}' not found")

            if waypoint.get("is_base"):
                raise ValueError("Base waypoint cannot be deleted. Assign another base first.")

            del self.data["waypoints"][waypoint_id]

            self.save()

    
    def _clear_base(self):
        for waypoint in self.data["waypoints"].values():
            waypoint["is_base"] = False