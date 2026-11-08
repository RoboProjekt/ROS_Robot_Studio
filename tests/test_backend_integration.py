import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from ros_robot_studio.backends.isaac import IsaacSimInstallationManager
from ros_robot_studio.runtime.manager import RuntimeManager


def run(script, report, timeout, arguments=()):
    status = RuntimeManager().status()
    if not status.available or "humble" not in status.installed:
        pytest.skip("Current managed Humble image and Docker access required")
    root = Path(__file__).resolve().parents[1]
    environment = dict(
        os.environ, QT_QPA_PLATFORM="offscreen", XDG_DATA_HOME=str(Path.home() / ".local/share")
    )
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, str(root / "scripts" / script), *arguments],
        cwd=root,
        env=environment,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return json.loads((root / "docs" / report).read_text())


@pytest.mark.integration
def test_real_multi_domain_and_read_only_hardware_fixture():
    report = run("verify_ros_domains.py", "backend-domain-verification.json", 100)
    assert report["isolated_jointstates"] and report["separate_ros_graphs"]
    assert report["command_spy_counts"] == [0, 0, 0]


@pytest.mark.integration
def test_two_real_backend_sessions_and_switch():
    report = run("verify_backend_sessions.py", "backend-session-verification.json", 180)
    assert report["simultaneous_connections"] and report["switch_and_cancel"]
    assert report["offline_keeps_background_connections"]


@pytest.mark.integration
@pytest.mark.isaac
def test_real_isaac_so101_task_and_feedback():
    if not IsaacSimInstallationManager().detect():
        pytest.skip("Compatible local Isaac Sim 5.1 installation required")
    report = run("verify_isaac_backend.py", "isaac-backend-verification.json", 280)
    assert report["task_status"] == "succeeded" and report["task_steps"] == 13
    assert report["real_jointstate_samples"] > 20
    assert report["scene"]["urdf_unmodified"]


@pytest.mark.integration
@pytest.mark.parametrize("distro", ["humble", "jazzy", "lyrical"])
def test_motion_failure_and_backend_switch_do_not_resume_old_goal(distro):
    if distro not in RuntimeManager().status().installed:
        pytest.skip("Current managed runtime required")
    report = run(
        "verify_backend_sessions.py",
        f"backend-failure-{distro}-verification.json",
        180,
        ("--distro", distro, "--fault-injection"),
    )
    assert report["reconnect_does_not_resume_task"]
    assert report["other_backend_task_succeeded"]
    assert report["failed_backend_unchanged_while_other_executes"]


@pytest.mark.integration
@pytest.mark.isaac
def test_real_isaac_header_manual_pause_and_explicit_auto_play():
    if not IsaacSimInstallationManager().detect():
        pytest.skip("Compatible local Isaac Sim 5.1 installation required")
    report = run("verify_isaac_header.py", "isaac-header-verification.json", 280)
    assert report["manual_play_default"] and report["paused_without_clock_or_motion"]
    assert report["auto_play_explicit_checkbox"] and report["task_status"] == "succeeded"
    assert report["task_steps"] == 13 and report["source_unmodified"]
    assert report["owned_resources_removed"]
