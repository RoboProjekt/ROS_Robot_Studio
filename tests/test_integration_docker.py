"""Opt-in real ROS integration. Never touches pre-existing containers."""

import json
import socket
import time
import uuid

import pytest

from ros_robot_studio.core.models import RuntimeConfig
from ros_robot_studio.runtime.manager import RuntimeManager


@pytest.mark.integration
@pytest.mark.parametrize("distro", ["humble", "jazzy", "lyrical"])
def test_real_ros_demo(distro):
    import websocket

    manager = RuntimeManager()
    status = manager.status()
    if not status.available:
        pytest.skip(status.message)
    if distro not in status.installed:
        pytest.skip(f"{distro} Studio image not installed; no implicit multi-GB downloads")
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    identifier, connection = None, None
    try:
        identifier = manager.start(
            distro, "integration-" + uuid.uuid4().hex, RuntimeConfig(port=port), integration=True
        )
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            try:
                connection = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=2)
                break
            except (OSError, websocket.WebSocketException):
                time.sleep(0.5)
        assert connection is not None, manager.logs(identifier)
        connection.settimeout(10)
        for topic, kind in [
            ("/studio/sine", "std_msgs/msg/Float64"),
            ("/joint_states", "sensor_msgs/msg/JointState"),
        ]:
            connection.send(
                json.dumps({"op": "subscribe", "topic": topic, "type": kind, "throttle_rate": 100})
            )
        for name in ("topics", "nodes"):
            connection.send(
                json.dumps(
                    {"op": "call_service", "id": name, "service": "/rosapi/" + name, "args": {}}
                )
            )
        seen, services = set(), {}
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            message = json.loads(connection.recv())
            if message.get("op") == "publish":
                seen.add(message["topic"])
                if message["topic"] == "/studio/sine":
                    assert isinstance(message["msg"]["data"], (int, float))
            if message.get("op") == "service_response":
                services[message["id"]] = message.get("values", {})
            if len(seen) >= 2 and {"topics", "nodes"} <= services.keys():
                break
        assert {"/studio/sine", "/joint_states"} <= seen
        assert "/studio/sine" in services["topics"]["topics"]
        assert any("studio_demo_subscriber" in n for n in services["nodes"]["nodes"])
        # The subscriber log confirms that it has actually received messages.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if "Received " in manager.logs(identifier):
                break
            time.sleep(0.5)
        assert "Received " in manager.logs(identifier)
    finally:
        if connection:
            connection.close()
        if identifier:
            manager.stop(identifier)
