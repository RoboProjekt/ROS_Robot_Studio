"""Real fault injection into owned, read-only bridge containers; no robot controller."""

import json
import socket
import time
from pathlib import Path

import pytest
import websocket

from docker.errors import NotFound
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.runtime.manager import RuntimeManager
from ros_robot_studio.runtime.robot_config import simulation_config


@pytest.mark.integration
@pytest.mark.parametrize("distro", ["humble", "jazzy", "lyrical"])
def test_bridge_recovers_bounded_child_failures_without_starting_controller(distro):
    runtime = RuntimeManager()
    status = runtime.status()
    if not status.available or distro not in status.installed:
        pytest.skip("Current managed runtime and Docker access required")
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    from ros_robot_studio.core.models import RuntimeConfig

    _, model, _ = pick_place_demo(prefer_so101=False)
    payload = simulation_config(model, model.default_configuration())
    payload["profile"] = "read_only"
    identity = runtime.start(
        distro,
        "bridge-recovery-test",
        RuntimeConfig(port=port),
        integration=True,
        robot_config=payload,
    )
    client = runtime.client_factory()
    container = runtime._owned(client, identity)
    network_id = container.labels["studio.network"]
    completed = False

    def child_pid():
        code, output = container.exec_run(
            [
                "python3",
                "-c",
                """import glob,json
found=[]
for p in glob.glob('/proc/[0-9]*/cmdline'):
 try:
  args=open(p,'rb').read().split(b'\\0')
  if any(a.endswith(b'/lib/rosbridge_server/rosbridge_websocket') for a in args[:2]):found.append(int(p.split('/')[2]))
 except OSError:pass
print(json.dumps(found))
""",
            ]
        )
        assert code == 0, output
        pids = json.loads(output)
        return pids[0] if len(pids) == 1 else None

    def ready(previous=None):
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            try:
                pid = child_pid()
                if not pid or pid == previous:
                    time.sleep(0.1)
                    continue
                ws = websocket.create_connection(f"ws://127.0.0.1:{port}", timeout=1)
                try:
                    ws.send(
                        json.dumps(
                            {"op": "call_service", "service": "/rosapi/nodes", "id": "ready"}
                        )
                    )
                    data = json.loads(ws.recv())
                    values = data.get("values") if isinstance(data, dict) else None
                    if (
                        isinstance(values, dict)
                        and data.get("id") == "ready"
                        and "/rosapi" in values.get("nodes", [])
                    ):
                        return pid
                finally:
                    ws.close()
            except (OSError, ValueError, websocket.WebSocketException):
                pass
            time.sleep(0.1)
        raise AssertionError(container.logs(tail=40).decode(errors="replace"))

    injected = 0
    try:
        pid = ready()
        while True:
            log = container.logs().decode(errors="replace")
            used = log.count("STUDIO_ROS_CHILD_RESTART rosbridge_websocket")
            code, output = container.exec_run(
                ["python3", "-c", f"import os,signal;os.kill({pid},signal.SIGKILL)"]
            )
            assert code == 0, output
            injected += 1
            if used < 2:
                pid = ready(pid)
            else:
                break
            assert injected <= 3
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            container.reload()
            if container.status == "exited":
                break
            time.sleep(0.1)
        assert container.status == "exited"
        log = container.logs().decode(errors="replace")
        assert log.count("STUDIO_ROS_CHILD_RESTART rosbridge_websocket") == 2
        assert "STUDIO_ROS_CHILD_FAILED rosbridge_websocket: restart budget exhausted" in log
        assert "controller_nodes.py" not in log and "demo_nodes.py" not in log
        path = Path("docs/bridge-recovery-verification.json")
        reports = json.loads(path.read_text()) if path.exists() else {}
        reports[distro] = dict(
            intentional_child_kills=injected,
            successful_restarts=2,
            budget_exhaustion_stops_runtime=True,
            read_only=True,
            service_responds_after_restart=True,
            hardware_access=False,
        )
        path.write_text(json.dumps(reports, indent=2) + "\n")
        completed = True
    finally:
        runtime.stop(identity)
        with pytest.raises(NotFound):
            client.containers.get(identity)
        with pytest.raises(NotFound):
            client.networks.get(network_id)
        with socket.socket() as released:
            released.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            released.bind(("127.0.0.1", port))
        client.close()
        if completed:
            reports[distro].update(
                owned_container_removed=True,
                owned_network_removed=True,
                websocket_port_released=True,
            )
            path.write_text(json.dumps(reports, indent=2) + "\n")
