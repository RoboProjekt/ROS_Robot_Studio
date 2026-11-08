"""Opt-in full GUI/ROS/action integration. Never changes hardware or existing containers."""

import json
import os
import subprocess
import sys
from pathlib import Path

import pytest

from ros_robot_studio.robots.model_manager import RobotModelManager
from ros_robot_studio.runtime.manager import RuntimeManager


@pytest.mark.integration
@pytest.mark.parametrize("distro", ["humble", "jazzy", "lyrical"])
def test_real_ros_pick_place_and_mode_roundtrip(distro):
    status = RuntimeManager().status()
    if not status.available:
        pytest.skip(status.message)
    if distro not in status.installed:
        pytest.skip(distro + " current runtime not installed; no implicit downloads")
    user_data = Path.home() / ".local/share"
    models = RobotModelManager(user_data / "ros-robot-studio/robots")
    if models.status("so101_follower").state != "installed":
        pytest.skip("Pinned original SO-101 not installed")
    root = Path(__file__).resolve().parents[1]
    environment = dict(os.environ, QT_QPA_PLATFORM="offscreen", XDG_DATA_HOME=str(user_data))
    environment.pop("PYTHONPATH", None)
    result = subprocess.run(
        [sys.executable, str(root / "scripts/verify_ros_runtime.py"), "--distro", distro],
        cwd=root,
        env=environment,
        capture_output=True,
        text=True,
        timeout=150,
    )
    assert result.returncode == 0, result.stdout + "\n" + result.stderr
    report = json.loads((root / f"docs/ros-{distro}-verification.json").read_text())
    assert (
        report["native_action"] and report["task_steps"] == 13 and report["jointstate_samples"] > 10
    )
    assert report["mode_roundtrip_twice"] and report["cancel"] and report["old_source_rejected"]


@pytest.mark.integration
def test_setup_wizard_checks_real_action(qtbot, tmp_path, monkeypatch):
    from PySide6.QtWidgets import QMessageBox

    from ros_robot_studio.core.settings import Settings
    from ros_robot_studio.gui.wizard import SetupWizard

    status = RuntimeManager().status()
    if not status.available or "humble" not in status.installed:
        pytest.skip("Current Humble runtime required")
    errors = []
    monkeypatch.setattr(QMessageBox, "warning", lambda *args: errors.append(str(args[-1])))
    monkeypatch.setattr(
        "ros_robot_studio.gui.wizard.system_report",
        lambda: {"Docker daemon / permissions": status.message, "docker_available": True},
    )
    wizard = SetupWizard(RuntimeManager(), Settings(tmp_path / "setup.json"), lambda: None)
    qtbot.addWidget(wizard)
    wizard.show()
    qtbot.waitUntil(lambda: not wizard.runner.busy)
    wizard.verify_runtime()
    qtbot.waitUntil(lambda: not wizard.verifying and not wizard.runner.busy, timeout=60000)
    assert not errors
    assert any(
        line.startswith("PASS:") and "native FollowJointTrajectory" in line
        for line in wizard.report_lines
    ), wizard.components.toPlainText()
    assert wizard.verify_container is None
    wizard.reject()


@pytest.mark.integration
def test_image_healthcheck_uses_configured_internal_port():
    import time

    from ros_robot_studio.runtime.manager import OWNER, VERSION

    manager = RuntimeManager()
    status = manager.status()
    if not status.available or "humble" not in status.installed:
        pytest.skip("Current Humble runtime required")
    client = manager.client_factory()
    container = None
    try:
        image = client.images.get(manager.specs["humble"].image)
        server = "import socket; s=socket.socket(); s.bind(('127.0.0.1',19091)); s.listen();\nwhile True:\n c,a=s.accept(); c.close()"
        container = client.containers.run(
            image.id,
            entrypoint=["python3", "-c", server],
            environment={"STUDIO_BRIDGE_PORT": "19091"},
            labels={OWNER: VERSION, "studio.integration": "true"},
            detach=True,
            cap_drop=["ALL"],
            security_opt=["no-new-privileges:true"],
        )
        check = image.attrs["Config"]["Healthcheck"]["Test"]
        assert check[0] == "CMD-SHELL"
        deadline = time.monotonic() + 5
        while True:
            code, output = manager._owned(client, container.id).exec_run(
                ["/bin/sh", "-c", check[1]]
            )
            if code == 0:
                break
            assert time.monotonic() < deadline, output
            time.sleep(0.05)
    finally:
        if container:
            manager._owned(client, container.id).stop(timeout=1)
            container.remove()
        client.close()


@pytest.mark.integration
def test_two_simulations_same_domain_remain_isolated():
    import socket
    import time

    import websocket

    from ros_robot_studio.core.models import RuntimeConfig
    from ros_robot_studio.projects.store import DATA
    from ros_robot_studio.robots.urdf import parse_urdf
    from ros_robot_studio.runtime.robot_config import simulation_config

    manager = RuntimeManager()
    status = manager.status()
    if not status.available or "humble" not in status.installed:
        pytest.skip("Current Humble runtime required")
    payloads = [
        simulation_config(),
        simulation_config(parse_urdf(DATA / "examples/generic_pick_arm.urdf")),
    ]
    identifiers = []
    connections = []
    try:
        for payload in payloads:
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            identifiers.append(
                manager.start(
                    "humble",
                    "isolation-test",
                    RuntimeConfig(port=port),
                    integration=True,
                    robot_config=payload,
                )
            )
            deadline = time.monotonic() + 30
            while True:
                try:
                    connection = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=1)
                    break
                except (OSError, websocket.WebSocketException):
                    assert time.monotonic() < deadline, manager.logs(identifiers[-1])
                    time.sleep(0.1)
            connections.append(connection)
            for topic, kind in [
                ("/joint_states", "sensor_msgs/msg/JointState"),
                ("/studio/execution/events", "std_msgs/msg/String"),
            ]:
                connection.send(json.dumps(dict(op="subscribe", topic=topic, type=kind)))
        time.sleep(0.5)
        for connection, payload in zip(connections, payloads):
            states = events = 0
            deadline = time.monotonic() + 6
            while states < 10 or events < 2:
                assert time.monotonic() < deadline
                message = json.loads(connection.recv())
                if message.get("topic") == "/joint_states":
                    assert set(message["msg"]["name"]) == set(payload["initial"])
                    states += 1
                elif message.get("topic") == "/studio/execution/events":
                    assert json.loads(message["msg"]["data"])["session"] == payload["session"]
                    events += 1
    finally:
        for connection in connections:
            connection.close()
        for identifier in identifiers:
            manager.stop(identifier)
