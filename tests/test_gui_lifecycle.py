"""Run lifecycle stress in its own interpreter so a native crash fails this test."""

import os
import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    "robot,cycles", [("generic", 12), pytest.param("so101", 20, marks=pytest.mark.model_download)]
)
def test_repeated_gui_destruction_with_pending_ros(robot, cycles, tmp_path):
    root = Path(__file__).resolve().parents[1]
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    if robot == "so101":
        environment["XDG_DATA_HOME"] = str(Path.home() / ".local/share")
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [
            sys.executable,
            str(root / "scripts/verify_gui_lifecycle.py"),
            "--robot",
            robot,
            "--cycles",
            str(cycles),
            "--report",
            str(tmp_path / "lifecycle.json"),
        ],
        cwd=root,
        env=environment,
        capture_output=True,
        text=True,
        timeout=cycles * 20 + 30,
    )
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
