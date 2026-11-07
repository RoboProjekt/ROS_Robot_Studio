"""XDG application paths and atomic, version-independent user preferences."""

import json
import os
from pathlib import Path

from ros_robot_studio.projects.store import atomic_text


def xdg_path(kind: str) -> Path:
    base = {
        "config": ("XDG_CONFIG_HOME", ".config"),
        "cache": ("XDG_CACHE_HOME", ".cache"),
        "state": ("XDG_STATE_HOME", ".local/state"),
    }[kind]
    return Path(os.environ.get(base[0], str(Path.home() / base[1]))) / "ros-robot-studio"


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
