import json

import pytest
from PySide6.QtNetwork import QHostAddress
from PySide6.QtWebSockets import QWebSocketServer

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.wizard import SetupWizard
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport
from ros_robot_studio.runtime.manager import RuntimeStatus


def test_real_websocket_protocol_graph_and_reconnect(qtbot):
    server = QWebSocketServer("Studio protocol test", QWebSocketServer.SslMode.NonSecureMode)
    if not server.listen(QHostAddress(QHostAddress.SpecialAddress.LocalHost), 0):
        pytest.skip("Loopback sockets unavailable in sandbox: " + server.errorString())
    sockets, operations = [], []

    def accept():
        socket = server.nextPendingConnection()
        sockets.append(socket)

        def request(raw):
            data = json.loads(raw)
            operations.append(data)
            if data["op"] == "call_service":
                values = (
                    {"topics": ["/test"], "types": ["std_msgs/msg/Float64"]}
                    if data["service"].endswith("topics")
                    else {"nodes": ["/real_protocol_fixture"]}
                )
                socket.sendTextMessage(
                    json.dumps(
                        {
                            "op": "service_response",
                            "id": data["id"],
                            "result": True,
                            "values": values,
                        }
                    )
                )
            elif data["op"] == "subscribe":
                socket.sendTextMessage(
                    json.dumps({"op": "publish", "topic": data["topic"], "msg": {"data": 0.75}})
                )

        socket.textMessageReceived.connect(request)

    server.newConnection.connect(accept)
    transport = RosbridgeTransport(f"ws://127.0.0.1:{server.serverPort()}")
    messages, graphs = [], []
    transport.message.connect(lambda topic, message: messages.append((topic, message)))
    transport.graph.connect(graphs.append)
    transport.subscribe("/test", "std_msgs/msg/Float64")
    transport.connect_transport()
    try:
        qtbot.waitUntil(
            lambda: bool(messages) and any(g.get("nodes") for g in graphs), timeout=5000
        )
        assert messages[0] == ("/test", {"data": 0.75})
        assert graphs[-1]["topics"] == ["/test"]
        sockets[-1].close()
        qtbot.waitUntil(lambda: len(sockets) == 2, timeout=5000)
        qtbot.waitUntil(lambda: len(messages) >= 2, timeout=5000)
        assert sum(operation["op"] == "subscribe" for operation in operations) == 2
        transport.unsubscribe("/test")
        qtbot.waitUntil(lambda: any(o["op"] == "unsubscribe" for o in operations))
    finally:
        transport.disconnect_transport()
        for socket in sockets:
            socket.close()
            socket.deleteLater()
        server.close()
        transport.deleteLater()


def test_setup_without_docker_and_persisted_completion(qtbot, tmp_path, monkeypatch):
    monkeypatch.setattr(
        "ros_robot_studio.gui.wizard.system_report",
        lambda: {
            "OS": "Ubuntu 22.04",
            "Architecture": "x86_64",
            "Python": "3.10",
            "Docker daemon / permissions": "Not installed",
            "docker_available": False,
        },
    )

    class Manager:
        def status(self):
            return RuntimeStatus(False, "Not installed")

    settings = Settings(tmp_path / "settings.json")
    completed = []
    wizard = SetupWizard(Manager(), settings, lambda: completed.append(True))
    qtbot.addWidget(wizard)
    wizard.show()
    qtbot.waitUntil(lambda: not wizard.runner.busy)
    assert len(wizard.pageIds()) == 5
    assert not wizard.install_button.isEnabled()
    assert not wizard.verify_button.isEnabled()
    assert all(checkbox.isChecked() for checkbox in wizard.checked.values())
    for _ in range(4):
        wizard.next()
    assert wizard.currentId() == 4
    wizard.accept()
    assert Settings(settings.path).get("setup_complete")
    assert completed == [True]


def test_service_errors_and_malformed_success_do_not_become_graph_data(qtbot):
    import time

    transport = RosbridgeTransport("ws://127.0.0.1:1")
    transport.active = True
    errors = []
    received = []
    transport.error.connect(errors.append)
    transport.pending["startup"] = (time.monotonic(), received.append)
    transport._receive(
        json.dumps(
            dict(
                op="service_response",
                id="startup",
                result=False,
                values="Service /rosapi/nodes not available",
            )
        )
    )
    assert not received and errors and not transport.graph_data
    transport.pending["malformed"] = (time.monotonic(), received.append)
    transport._receive(
        json.dumps(
            dict(op="service_response", id="malformed", result=True, values="not a result object")
        )
    )
    assert not received and "must be an object" in errors[-1]
    transport.pending["ready"] = (time.monotonic(), received.append)
    transport._receive(
        json.dumps(
            dict(op="service_response", id="ready", result=True, values={"nodes": ["/actual_node"]})
        )
    )
    assert received == [{"nodes": ["/actual_node"]}]
    transport.disconnect_transport()
    transport.deleteLater()
