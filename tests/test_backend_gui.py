import json
import sys
import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest
from pydantic import ValidationError
from PySide6.QtCore import QPoint, Qt
from PySide6.QtWidgets import QCheckBox

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.backends.so101_usb import JOINT_NAMES
from ros_robot_studio.core.models import Project
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.backend_connections import BackendProfileDialog
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.manager import RuntimeStatus
from ros_robot_studio.vision.profiles import CameraProfile
from ros_robot_studio.vision.stereo import StereoResult
from ros_robot_studio.vision.stream import RGBFrame


@pytest.fixture
def window(qtbot, tmp_path, monkeypatch):
    runtime = MagicMock()
    runtime.specs = catalog()
    runtime.status.return_value = RuntimeStatus(False, "Fixture: no Docker")
    w = MainWindow(Settings(tmp_path / "settings.json"), runtime, show_wizard=False)
    errors = []
    monkeypatch.setattr(w, "show_error", errors.append)
    qtbot.addWidget(w, before_close_func=lambda obj: setattr(obj, "dirty", False))
    w.show()
    qtbot.waitUntil(
        lambda: not w.runner.busy and not w.motion.runner.busy and not w.initial_model_check_pending
    )
    project, model, _ = pick_place_demo(prefer_so101=False)
    w.project, w.model = project, model
    w.apply_project()
    qtbot.waitUntil(lambda: not w.motion.runner.busy)
    yield w
    assert not errors
    w.dirty = False
    w.close()


def test_backend_profile_editor_binds_model_and_keeps_machine_paths_local(window, qtbot):
    editor = BackendProfileDialog(window, new_profile("isaac_sim"), is_new=True)
    qtbot.addWidget(editor)
    editor.installation.setText("/another/computer/isaacsim")
    profile = editor.values()
    assert profile.model_signature and profile.joint_mapping
    window.backend_manager.add(profile, bind_current=False)
    serialized = window.collect().model_dump_json()
    assert "/another/computer" not in serialized
    assert editor.local_values()["installation_path"] == "/another/computer/isaacsim"
    loaded = Project.model_validate_json(serialized)
    assert loaded.backends[0].id == profile.id


def test_hardware_editor_cannot_enable_motion_even_programmatically(window, qtbot):
    editor = BackendProfileDialog(window, new_profile("real_robot"), is_new=True)
    qtbot.addWidget(editor)
    assert not editor.execution.isEnabled() and editor.read_only.isChecked()
    editor.execution.setChecked(True)
    with pytest.raises(ValidationError, match="read only"):
        editor.values()


def test_real_robot_shows_usb_selector_and_preserves_a_disconnected_choice(
    window, tmp_path, monkeypatch
):
    port = str(tmp_path / "ttyACM0")
    ports = [(port, "ttyACM0 · USB Single Serial")]
    monkeypatch.setattr("ros_robot_studio.gui.simulation_header.usb_ports", lambda: ports)
    header = window.simulation_header
    assert header.robot_port.isHidden()
    header.select_runtime(header.runtime.findData("new:real_robot"))
    assert header.profile().backend_type == "real_robot"
    assert header.profile().read_only
    assert not header.robot_port.isHidden() and header.scene.isHidden()
    index = header.robot_port.findData(port)
    header.robot_port.setCurrentIndex(index)
    header.select_port(index)
    assert header.start.text() == "Read USB Joints…"
    assert not header.connect_button.isEnabled()
    assert port not in window.collect().model_dump_json()
    ports.clear()
    header.refresh_ports_button.click()
    assert header.robot_port.currentData() == port
    assert not header.start.isEnabled()
    ports.append((port, "ttyACM0 · USB Single Serial"))
    header.refresh_ports_button.click()
    assert header.start.isEnabled()
    header.select_runtime(0)
    assert header.robot_port.isHidden() and not header.scene.isHidden()


def test_usb_runtime_connects_its_camera_bridge_without_activating_ros(
    window, tmp_path, monkeypatch
):
    port = tmp_path / "ttyACM0"
    port.touch()
    monkeypatch.setattr(
        "ros_robot_studio.gui.simulation_header.usb_ports", lambda: [(str(port), port.name)]
    )
    manager = window.backend_manager
    profile = manager.add(
        new_profile("real_robot", domain_id=23, network="host", network_confirmed=True)
    )
    window.project.cameras = [CameraProfile(name="Camera", backend_id=profile.id)]
    header = window.simulation_header
    header._select_runtime(profile.id)
    header.robot_port.setCurrentIndex(header.robot_port.findData(str(port)))
    header.select_port(header.robot_port.currentIndex())
    start_usb, connect = MagicMock(), MagicMock()
    monkeypatch.setattr(window.local_runtime, "start_usb", start_usb)
    monkeypatch.setattr(window.backend_workspace, "connect", connect)

    assert header.connect_button.isEnabled()
    assert header.connect_button.text() == "Connect Cameras"
    header.start.click()
    start_usb.assert_called_once_with(profile, str(port), False)
    connect.assert_not_called()

    window.docker_status = RuntimeStatus(True, "Fixture: Humble installed", ["humble"])
    header.start.click()
    connect.assert_called_once_with(profile.id)
    connect.reset_mock()
    header.connect_button.click()
    connect.assert_called_once_with(profile.id, activate=False)


def test_connection_manager_crud_and_prepared_backends(window, qtbot):
    manager = window.backend_manager
    first = manager.add(new_profile("ros_demo"))
    second = manager.add(new_profile("mujoco"))
    window.open_backend_connections()
    dialog = window.backend_workspace.dialog
    assert dialog.table.rowCount() == 2
    dialog.table.selectRow(1)
    dialog.inspect()
    assert not dialog.connect_button.isEnabled()
    assert dialog.table.item(1, 4).text() == "Not Implemented"
    dialog.table.selectRow(0)
    dialog.duplicate()
    assert len(manager.profiles) == 3 and len({p.id for p in manager.profiles.values()}) == 3
    dialog.table.selectRow(1)
    dialog.remove()
    assert second.id not in manager.profiles and first.id in manager.profiles
    assert window.dirty


def test_stereo_preview_frames_do_not_grow_the_scroll_page(window, qtbot):
    pytest.importorskip("cv2")
    vision = window.vision
    stereo = vision.stereo
    page = window.tabs.pages[vision.panel]
    window.resize(1480, 1250)
    vision.open_view()
    stereo.parentWidget().parentWidget().setCurrentWidget(stereo)
    qtbot.wait(30)
    assert page.verticalScrollBar().maximum() == 0

    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    depth = np.zeros((480, 640), dtype=np.uint8)
    result = StereoResult(rgb, rgb, None, (), time.time_ns(), relative_depth=depth)
    stereo.view.addItem("Relative Depth")
    for index in (0, 2, 3) * 12:
        stereo.view.setCurrentIndex(index)
        stereo.accept_frame(stereo.generation, (result, None), time.time_ns())
        qtbot.wait(10)

    assert not stereo.image.pixmap().isNull()
    assert page.verticalScrollBar().maximum() == 0
    assert vision.panel.size() == page.viewport().size()


def test_live_qwen_uses_fresh_frames_without_queuing_and_invalidates_old_queries(window):
    stereo = window.vision.stereo
    real_runner = stereo.requests
    pending = []
    stereo.requests = SimpleNamespace(busy=False, run=lambda *job: pending.append(job))
    stereo.live_stream = SimpleNamespace()
    first = RGBFrame(np.zeros((4, 4, 3), np.uint8), {"stamp_ns": 1})
    second = RGBFrame(first.rgb, {"stamp_ns": 2})
    try:
        stereo.task.setText("Green cube")
        stereo.request_live(first, 100)
        assert len(pending) == 1
        stereo.requests.busy = True
        stereo.request_live(second, 102)
        assert len(pending) == 1
        stereo.requests.busy = False
        pending[0][1](({"id": "green_cube_1"},))
        assert stereo.live_result[1][0]["id"] == "green_cube_1"
        stereo.request_live(first, 102)
        stereo.request_live(second, 100.99)
        assert len(pending) == 1
        stereo.request_live(second, 101)
        assert len(pending) == 2
        stereo.task.setText("Blue bowl")
        pending[1][1](({"id": "green_cube_1"},))
        assert stereo.live_result is None
    finally:
        stereo.requests = real_runner
        stereo.live_stream = None


def test_monitor_resets_when_switching_backend_identity_of_same_type(window, qtbot):
    window.timer.stop()
    states = window.state_manager
    m = window.model
    a = states.register_backend("a", 1, "humble")
    b = states.register_backend("b", 4, "humble")
    q = m.default_configuration()
    states.update(a, q, source_timestamp=1)
    states.activate_backend("a")
    states.update(a, q, source_timestamp=1.1)
    assert len(window.monitor.robot_state.buffer.samples) == 2
    states.update(b, q, source_timestamp=1.2)
    states.activate_backend("b")
    assert len(window.monitor.robot_state.buffer.samples) == 1
    assert window.monitor.robot_state.state.ros_domain == 4


def test_mujoco_slider_and_both_views_share_the_measured_state(window, qtbot):
    pytest.importorskip("mujoco")
    profile = window.backend_manager.add(new_profile("mujoco"))
    window.backend_manager.select_profile(profile.id)
    window.simulation_header.start_profile(profile.id)
    controls = window.robot_panel.joint_controls
    name = next(iter(controls.controls))
    slider = controls.controls[name][0]
    assert slider.isEnabled()
    state = window.state_manager.snapshot
    initial = state().positions[name]
    slider.setValue(slider.value() + 1000)
    qtbot.waitUntil(lambda: abs(state().positions[name] - initial) > 0.002)
    assert state().backend == "mujoco"
    assert controls.values[name] == state().positions[name]
    assert window.monitor.robot_state.state.positions[name] == state().positions[name]
    window.simulation_header.disconnect_selected()
    assert state().backend == "offline"


def test_ros_slider_uses_a_model_matched_trajectory_and_drops_old_targets(window, monkeypatch):
    execution = window.ros_execution
    monkeypatch.setattr(execution, "ready", lambda: True)
    current = window.model.default_configuration()
    monkeypatch.setattr(execution, "read_joint_positions", lambda: current)
    execute = MagicMock()
    monkeypatch.setattr(execution, "execute", execute)
    window.mode = "ros"
    lease = window.state_manager.activate_source("external_simulation", "test")
    window.state_manager.update(lease, current)
    name = next(iter(current))
    window.update_joint_controls()
    slider, numeric, _, _, factor = window.robot_panel.joint_controls.controls[name]
    assert slider.isEnabled()
    numeric.setValue((current[name] + 0.01) * factor)
    assert window.state_manager.snapshot().positions[name] == current[name]
    window.dispatch_live_ros_target()
    plan = execute.call_args.args[0]
    assert plan.valid and plan.chain.model is window.model
    assert plan.steps[0].trajectory.end[name] == pytest.approx(current[name] + 0.01)

    window.live_joint_target(name, current[name] + 0.02)
    window.state_manager.activate_source("external_simulation", "new")
    window.dispatch_live_ros_target()
    execute.assert_called_once()
    window.start_offline()


def load_usb_test_project(window, tmp_path, qtbot):
    from ros_robot_studio.projects.store import PROJECTS_DIR, load_project
    from ros_robot_studio.robots.urdf import parse_urdf

    project = load_project(PROJECTS_DIR / "SO-101-Real.studio.yaml")
    # Use a minimal six-joint URDF so the tests run without a catalog download.
    urdf = tmp_path / "arm.urdf"
    links = "".join(f'<link name="l{i}"/>' for i in range(7))
    joints = "".join(
        f'<joint name="{name}" type="revolute"><parent link="l{i}"/><child link="l{i + 1}"/>'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
        for i, name in enumerate(JOINT_NAMES)
    )
    urdf.write_text(f'<robot name="arm">{links}{joints}</robot>')
    window.project, window.model = project, parse_urdf(urdf)
    project.robot.end_effector = "l6"
    window.apply_project()
    qtbot.waitUntil(lambda: not window.motion.runner.busy)
    return window.backend_manager.profiles[project.active_backend]


def test_usb_measurement_preserves_angles_outside_urdf_limits_and_offline_recovers(
    window, tmp_path, qtbot
):
    from types import SimpleNamespace

    profile = load_usb_test_project(window, tmp_path, qtbot)
    runtime = window.local_runtime
    runtime.enter(profile, "so101_usb", "USB test")
    dialog = SimpleNamespace(can_command=lambda: True, request_target=MagicMock())
    runtime.dialog = dialog
    try:
        runtime.usb_feedback(
            dialog,
            {
                "ranges": dict.fromkeys(JOINT_NAMES, (0, 4095)),
                "encoders": dict.fromkeys(JOINT_NAMES, 2048),
                "positions": dict.fromkeys(JOINT_NAMES, 90),
            },
        )
        assert window.state_manager.snapshot().positions["shoulder_pan"] == pytest.approx(
            1.57079632679
        )
        assert window.robot_panel.joint_controls.values["shoulder_pan"] == pytest.approx(
            1.57079632679
        )
        assert window.monitor.robot_state.state.positions["shoulder_pan"] == pytest.approx(
            1.57079632679
        )
        slider = window.robot_panel.joint_controls.controls["shoulder_pan"][0]
        assert slider.isEnabled()
        slider.setValue(5000)
        dialog.request_target.assert_called_once_with("shoulder_pan", 2048)
        assert window.state_manager.snapshot().positions["shoulder_pan"] == pytest.approx(
            1.57079632679
        )
    finally:
        runtime.dialog = None
        window.start_offline()
    assert window.state_manager.snapshot().positions["shoulder_pan"] == 1


def test_usb_motor_control_and_sliders_work_with_setup_open_or_hidden(
    window, qtbot, tmp_path, monkeypatch
):
    port = tmp_path / "ttyACM0"
    port.touch()
    helper = tmp_path / "stream.py"
    log = tmp_path / "commands.jsonl"
    helper.write_text(
        "import sys, json\nfrom pathlib import Path\n"
        f"names = {JOINT_NAMES!r}\n"
        "calibrated = len(sys.argv) == 4\n"
        "angles = dict.fromkeys(names, 0)\nencoders = dict.fromkeys(names, 2048)\n"
        "armed = False\nmotion = 'idle'\nack = None\n"
        "def emit():\n"
        " print(json.dumps(dict(calibrated=calibrated, positions=angles, encoders=encoders, "
        "ranges=dict.fromkeys(names, [0,4095]), armed=armed, motion=motion, motion_ack=ack)), "
        "flush=True)\n"
        "emit()\nfor line in sys.stdin:\n"
        f" with Path({str(log)!r}).open('a') as target: target.write(line)\n"
        " command=json.loads(line)\n"
        " if command['operation']=='close': break\n"
        " if command['operation']=='arm': armed=True\n"
        " if command['operation']=='disarm': armed=False\n"
        " if command['operation']=='target': encoders[command['joint']]=command['position']\n"
        " if command['operation']=='record': Path(command['path']).touch()\n"
        " if command['operation'] in ('record','replay','stop_motion'):\n"
        "  motion = 'idle' if command['operation']=='stop_motion' else command['operation']\n"
        "  ack = command['request_id']\n"
        " emit()\n"
    )
    monkeypatch.setattr("ros_robot_studio.backends.so101_usb.__file__", str(helper))
    monkeypatch.setattr("ros_robot_studio.gui.usb_robot.calibration_files", lambda: [])
    monkeypatch.setattr("ros_robot_studio.gui.usb_motion_panel.REPLAYS_DIR", tmp_path / "Replays")
    profile = load_usb_test_project(window, tmp_path, qtbot)
    window.settings.set("lerobot_python", sys.executable)
    controls = window.robot_panel.joint_controls
    motion = window.motion.usb_motion
    toggle = controls.usb_motor_control
    slider = controls.controls["shoulder_pan"][0]
    runtime = window.local_runtime
    assert toggle.isHidden()
    assert controls.usb_limits.isHidden()
    assert motion.isHidden()

    runtime.start_usb(profile, str(port), allow_jog=True)
    dialog = runtime.dialog
    assert dialog.isVisible() and not dialog.isModal()
    assert not dialog.findChildren(QCheckBox)
    qtbot.waitUntil(lambda: dialog.positions["shoulder_pan"].text().endswith("counts"))
    assert toggle.isVisible() and not toggle.isEnabled() and not slider.isEnabled()
    dialog.add_calibration(tmp_path / "follower.json")
    dialog.calibration.setCurrentIndex(1)
    qtbot.waitUntil(toggle.isEnabled)
    assert not slider.isEnabled()
    assert dialog.positions["shoulder_pan"].text() == "0.00°"

    def commands():
        return [json.loads(line) for line in log.read_text().splitlines()]

    def operations():
        return [command["operation"] for command in commands()]

    assert controls.usb_limits.isVisible() and controls.usb_limits.isEnabled()
    controls.usb_speed.setValue(60)
    controls.usb_torque.setValue(35)
    qtbot.mouseClick(toggle, Qt.MouseButton.LeftButton, pos=QPoint(8, toggle.height() // 2))
    qtbot.waitUntil(slider.isEnabled)
    assert {"operation": "arm", "speed": 60, "torque": 35} in commands()
    assert "limits" not in operations()
    assert toggle.isChecked() and not dialog.calibration.isEnabled()
    slider.setValue(5500)
    qtbot.waitUntil(lambda: operations().count("target") == 1)
    dialog.close()
    assert not dialog.isVisible()
    assert toggle.isChecked() and slider.isEnabled()
    controls.usb_speed.setValue(80)
    qtbot.waitUntil(lambda: {"operation": "limits", "speed": 80, "torque": 35} in commands())
    controls.usb_torque.setValue(40)
    qtbot.waitUntil(lambda: {"operation": "limits", "speed": 80, "torque": 40} in commands())
    window.robot_panel.detail_tabs.setCurrentWidget(window.motion.program_panel)
    window.motion.program_tabs.setCurrentIndex(0)
    assert motion.isVisible()
    motion.title.setText("Test run")
    motion.record_torque.setValue(8)
    motion.record.click()
    qtbot.waitUntil(lambda: motion.record.text() == "Stop recording")
    record = next(command for command in commands() if command["operation"] == "record")
    assert record["record_torque"] == 8 and record["torque"] == 40
    assert not slider.isEnabled() and not controls.usb_limits.isEnabled()
    assert controls.usb_torque.value() == 40
    motion.record.click()
    qtbot.waitUntil(motion.replay.isEnabled)
    assert motion.replays.currentText() == "Test run"
    motion.replay.click()
    qtbot.waitUntil(lambda: motion.replay.text() == "Stop replay")
    replay = next(command for command in commands() if command["operation"] == "replay")
    assert replay["speed"] == 80 and replay["torque"] == 40 and "record_torque" not in replay
    motion.replay.click()
    qtbot.waitUntil(slider.isEnabled)
    ping_count = operations().count("ping")
    qtbot.waitUntil(lambda: operations().count("ping") > ping_count)
    slider.setValue(4500)
    qtbot.waitUntil(lambda: operations().count("target") == 2)

    qtbot.mouseClick(toggle, Qt.MouseButton.LeftButton, pos=QPoint(8, toggle.height() // 2))
    qtbot.waitUntil(toggle.isEnabled)
    assert not slider.isEnabled() and not dialog.armed
    assert operations().count("disarm") == 1
    motion.title.setText("Passive run")
    motion.record_torque.setValue(0)
    assert motion.record.isEnabled()
    motion.record.click()
    qtbot.waitUntil(lambda: motion.record.text() == "Stop recording")
    assert not toggle.isEnabled() and not dialog.armed
    motion.record.click()
    qtbot.waitUntil(toggle.isEnabled)
    assert motion.replays.currentText() == "Passive run" and not toggle.isChecked()
    runtime.start_usb(profile, str(port), allow_jog=True)
    assert runtime.dialog is dialog and dialog.isVisible()
    qtbot.mouseClick(toggle, Qt.MouseButton.LeftButton, pos=QPoint(8, toggle.height() // 2))
    qtbot.waitUntil(slider.isEnabled)
    window.simulation_header.disconnect_selected()
    qtbot.waitUntil(lambda: runtime.retiring_usb is None)
    assert operations()[-1] == "close"
    assert window.mode == "offline" and toggle.isHidden() and not toggle.isChecked()
    assert controls.usb_limits.isHidden()


def test_cancelled_close_preserves_unsaved_project_and_connected_backends(window, monkeypatch):
    from unittest.mock import Mock

    pending = Mock(return_value=True)
    original_confirm = window.confirm_discard
    monkeypatch.setattr(window.backend_workspace, "close_pending", pending)
    monkeypatch.setattr(window, "confirm_discard", lambda: False)
    window.backend_manager.sessions["pending"] = object()
    window.dirty = True
    try:
        assert not window.close()
        assert window.dirty and not window.closed
        assert "pending" in window.backend_manager.sessions
        pending.assert_not_called()
    finally:
        window.backend_manager.sessions.clear()
        window.dirty = False
        monkeypatch.setattr(window, "confirm_discard", original_confirm)
