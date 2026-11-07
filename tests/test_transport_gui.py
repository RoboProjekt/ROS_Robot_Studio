import json
from unittest.mock import MagicMock

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.gui.sensor_dialog import SensorDialog
from ros_robot_studio.ros_bridge.transport import SampleBuffer, numeric_fields
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.manager import RuntimeStatus


def fake_manager():
    manager = MagicMock()
    manager.specs = catalog()
    manager.status.return_value = RuntimeStatus(False, "Docker not installed")
    return manager


def window(qtbot, tmp_path):
    instance = MainWindow(Settings(tmp_path / "settings.json"), fake_manager(), show_wizard=False)
    qtbot.addWidget(instance)
    instance.show()
    qtbot.waitUntil(lambda: not instance.runner.busy)
    return instance


def test_gui_without_docker_and_live_demo(qtbot, tmp_path):
    instance = window(qtbot, tmp_path)
    assert instance.tabs.count() == 4
    assert instance.distro.currentData() == "humble"
    qtbot.waitUntil(lambda: len(instance.monitor.buffer.samples) > 3)
    assert "simulated" in instance.ros_status
    assert instance.monitor.fields["data"] <= 1
    assert not instance.dirty
    assert instance.monitor.curves
    instance.monitor.toggle_pause()
    frozen = list(instance.monitor.buffer.samples)
    qtbot.wait(150)
    assert list(instance.monitor.buffer.samples) == frozen
    instance.monitor.toggle_pause()
    instance.monitor.reset()
    qtbot.waitUntil(lambda: bool(instance.monitor.buffer.samples))
    # Reset must rebuild selected curves for the existing fields.
    instance.dirty = False
    instance.close()


def test_version_switch_releases_subscription_and_keeps_project(qtbot, tmp_path):
    instance = window(qtbot, tmp_path)
    old = instance.transport
    instance.monitor.choose_topic("/joint_states")
    instance.distro.setCurrentIndex(1)
    assert not old.timer.isActive()
    assert old.subscriptions == set()
    assert instance.project.default_ros == "humble"
    assert instance.transport != old
    qtbot.waitUntil(lambda: bool(instance.monitor.buffer.samples))
    instance.distro.setCurrentIndex(2)
    assert instance.project.default_ros == "humble"
    instance.dirty = False
    instance.close()


def test_gui_project_save_open_and_sensor_dialog(qtbot, tmp_path, monkeypatch):
    instance = window(qtbot, tmp_path)
    instance.robot_panel.name.setText("Saved research robot")
    instance.project_path = tmp_path / "saved.yaml"
    instance.save()
    qtbot.waitUntil(lambda: not instance.runner.busy)
    assert instance.project_path.exists()
    instance.project.name = "changed"
    instance.open_path(instance.project_path, confirmed=True)
    qtbot.waitUntil(lambda: not instance.runner.busy)
    assert instance.project.name == "Saved research robot"
    dialog = SensorDialog(instance.project.sensors[0], instance.model.links)
    qtbot.addWidget(dialog)
    dialog.offsets[0].setValue(0.12)
    assert dialog.result_sensor().transform.translation[0] == 0.12
    instance.dirty = False
    instance.close()


def test_invalid_connection_reconnects_and_stops(qtbot):
    transport = RosbridgeTransport("ws://127.0.0.1:1")
    errors = []
    transport.error.connect(errors.append)
    transport.connect_transport()
    qtbot.waitUntil(lambda: bool(errors), timeout=7000)
    assert transport.active
    assert transport.retry.isActive()
    transport.disconnect_transport()
    assert not transport.retry.isActive()
    assert not transport.subscriptions
    transport.deleteLater()


def test_bridge_bounded_latest_queue_and_graph_response(qtbot):
    transport = RosbridgeTransport("ws://127.0.0.1:1")
    transport.active = True
    transport.subscribe("/test")
    received = []
    transport.message.connect(lambda topic, message: received.append(message))
    for index in range(1000):
        transport._receive(json.dumps({"op": "publish", "topic": "/test", "msg": {"data": index}}))
    assert len(transport.latest) == 1
    transport._flush()
    assert received == [{"data": 999}]
    transport._receive("not-json")
    transport.disconnect_transport()
    transport.deleteLater()


def test_numeric_fields_and_buffer_limits():
    assert numeric_fields({"x": [1, 2], "flag": True, "bad": float("nan")}) == {
        "x[0]": 1.0,
        "x[1]": 2.0,
    }
    assert len(numeric_fields({"large": list(range(10000))})) <= 32
    buffer = SampleBuffer(3)
    for i in range(10):
        buffer.append(i, {"data": i})
    assert len(buffer.samples) == 3
    assert buffer.series("data", 9, 1) == ([-1, 0], [8, 9])
