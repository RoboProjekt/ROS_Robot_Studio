"""XDG application paths and atomic, version-independent user preferences."""

import json
from pathlib import Path

from ros_robot_studio.core.paths import xdg_path as xdg_path
from ros_robot_studio.projects.store import atomic_text


class Settings:
    def __init__(self, path: Path | None = None):
        self.path = path or xdg_path("config") / "settings.json"
        try:
            self.values = json.loads(self.path.read_text())
            if not isinstance(self.values, dict):
                self.values = {}
        except (OSError, ValueError):
            self.values = {}

    def get(self, key: str, default=None):
        return self.values.get(key, default)

    def set(self, key: str, value) -> None:
        self.values[key] = value
        atomic_text(self.path, json.dumps(self.values, indent=2))
