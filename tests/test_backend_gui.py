from unittest.mock import MagicMock

import pytest
from pydantic import ValidationError

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.models import Project
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.backend_connections import BackendProfileDialog
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.manager import RuntimeStatus


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


def test_usb_measurement_preserves_angles_outside_urdf_limits_and_offline_recovers(window, qtbot):
    from types import SimpleNamespace

    from ros_robot_studio.backends.so101_usb import JOINT_NAMES
    from ros_robot_studio.projects.store import PROJECTS_DIR, load_project
    from ros_robot_studio.robots.urdf import parse_urdf

    project = load_project(PROJECTS_DIR / "SO-101-Real.studio.yaml")
    # Reuse a tiny local six-joint model, avoiding a catalog download.
    from tempfile import TemporaryDirectory

    with TemporaryDirectory() as temp:
        from pathlib import Path

        urdf = Path(temp) / "arm.urdf"
        links = "".join(f'<link name="l{i}"/>' for i in range(7))
        joints = "".join(
            f'<joint name="{name}" type="revolute"><parent link="l{i}"/><child link="l{i + 1}"/>'
            '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>'
            for i, name in enumerate(JOINT_NAMES)
        )
        urdf.write_text(f'<robot name="arm">{links}{joints}</robot>')
        model = parse_urdf(urdf)
        window.project, window.model = project, model
        window.project.robot.end_effector = "l6"
        window.apply_project()
        qtbot.waitUntil(lambda: not window.motion.runner.busy)
        runtime = window.local_runtime
        profile = window.backend_manager.profiles[project.active_backend]
        runtime.enter(profile, "so101_usb", "USB test")
        dialog = SimpleNamespace(target=None, can_command=lambda: True, request_target=MagicMock())
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
