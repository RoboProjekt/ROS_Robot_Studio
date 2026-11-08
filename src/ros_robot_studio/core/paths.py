"""Platform-neutral XDG application paths; kept independent of project persistence."""

import os
from pathlib import Path


def xdg_path(kind: str) -> Path:
    variable, default = {
        "config": ("XDG_CONFIG_HOME", ".config"),
        "cache": ("XDG_CACHE_HOME", ".cache"),
        "state": ("XDG_STATE_HOME", ".local/state"),
        "data": ("XDG_DATA_HOME", ".local/share"),
    }[kind]
    return Path(os.environ.get(variable, str(Path.home() / default))) / "ros-robot-studio"
