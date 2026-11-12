"""Exercise user input, identity, persistence and truthful capability presentation."""

from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest
from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QDialog, QDialogButtonBox, QSplitter

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.gui.motion_dialogs import StepDialog
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.program import TaskCompiler
from ros_robot_studio.motion.schema import MotionTask, TaskStep
from ros_robot_studio.projects.store import load_project, save_project
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.docker_diagnostics import classify, endpoint
from ros_robot_studio.runtime.manager import RuntimeStatus


@pytest.fixture
def studio(qtbot, tmp_path):
    manager = MagicMock()
    manager.specs = catalog()
    manager.status.return_value = RuntimeStatus(False, "Test: Docker unavailable")
    w = MainWindow(Settings(tmp_path / "settings.json"), manager, show_wizard=False)
    qtbot.addWidget(w, before_close_func=lambda widget: setattr(widget, "dirty", False))
    w.show()
    qtbot.waitUntil(
        lambda: not w.runner.busy and not w.motion.runner.busy and not w.initial_model_check_pending
    )
    w.project, w.model, _ = pick_place_demo(prefer_so101=False)
    w.apply_project()
    qtbot.waitUntil(lambda: not w.motion.runner.busy)
    yield w
    w.motion.stop()
    qtbot.waitUntil(lambda: not w.runner.busy and not w.motion.runner.busy)
    w.dirty = False
    w.close()


def choose(combo, index):
    """Click the actual popup row, rather than setting currentIndex."""
    QTest.qWait(30)
    QTest.mouseClick(combo, Qt.MouseButton.LeftButton)
    QTest.qWait(450)
    view = combo.view()
    view.scrollTo(combo.model().index(index, 0))
    QTest.qWait(10)
    # Narrow comboboxes can expose only part of a row wider than their viewport.
    rect = view.visualRect(combo.model().index(index, 0)).intersected(view.viewport().rect())
    assert not rect.isEmpty(), "Popup row is not visible"
    QTest.mouseMove(view.viewport(), rect.center(), delay=15)
    QTest.mouseClick(view.viewport(), Qt.MouseButton.LeftButton, pos=rect.center())
    QTest.qWait(15)
    assert combo.currentIndex() == index, repr(
        (
            index,
            combo.currentIndex(),
            combo.itemText(index),
            combo.model().flags(combo.model().index(index, 0)),
            combo.isVisible(),
            combo.isEnabled(),
            view.isVisible(),
            rect,
            view.indexAt(rect.center()).row(),
            view.viewport().rect(),
            combo.width(),
            combo.height(),
            combo.view().size(),
            QApplication.doubleClickInterval(),
        )
    )


def type_number(spin, text):
    QTest.mouseClick(spin.lineEdit(), Qt.MouseButton.LeftButton)
    QTest.keyClick(spin, Qt.Key.Key_A, Qt.KeyboardModifier.ControlModifier)
    QTest.keyClicks(spin, text)
    QTest.keyClick(spin, Qt.Key.Key_Tab)


def drag_tab(workspace, panel, target, edge):
    source = workspace.group_for(panel)
    bar = source.tabBar()
    origin = bar.tabRect(source.indexOf(panel)).center()
    points = {
        "right": QPoint(target.width() - 12, target.height() // 2),
        "left": QPoint(12, target.height() // 2),
        "bottom": QPoint(target.width() // 2, target.height() - 12),
        "top": QPoint(target.width() // 2, 45),
        "center": QPoint(target.width() // 2, target.height() // 2),
    }
    end = bar.mapFromGlobal(target.mapToGlobal(points[edge]))
    QTest.mousePress(bar, Qt.MouseButton.LeftButton, pos=origin)
    QTest.mouseMove(bar, end, delay=30)
    QTest.mouseRelease(bar, Qt.MouseButton.LeftButton, pos=end, delay=30)
    QTest.qWait(30)


@pytest.mark.parametrize(
    "reason,data,error",
    [
        (
            "process_groups_inactive",
            dict(account_groups=["docker"], effective_groups=["local-user"], socket_group="docker"),
            PermissionError("permission denied"),
        ),
        (
            "permission_denied",
            dict(account_groups=[], effective_groups=[]),
            PermissionError("permission denied"),
        ),
        (
            "permission_denied",
            dict(account_groups=["docker"], effective_groups=["docker"]),
            PermissionError("permission denied"),
        ),
        ("not_installed", dict(docker_cli="missing"), RuntimeError("unreachable")),
        ("socket_missing", dict(socket_mode="missing"), RuntimeError("unreachable")),
        ("daemon_stopped", dict(engine="inactive"), RuntimeError("unreachable")),
        ("api_unreachable", dict(engine="active"), RuntimeError("API failed")),
        ("invalid_context", dict(context_error="unknown context"), ValueError("context")),
    ],
)
def test_docker_causes_are_distinct(reason, data, error):
    assert classify(data, error)[0] == reason
    if reason == "process_groups_inactive":
        assert "already belongs" in classify(data, error)[1]


def test_docker_context_environment_precedence(tmp_path, monkeypatch):
    monkeypatch.setenv("DOCKER_CONFIG", str(tmp_path))
    monkeypatch.delenv("DOCKER_CONTEXT", raising=False)
    monkeypatch.setenv("DOCKER_HOST", "unix:///tmp/rootless-test.sock")
    assert endpoint()[:2] == ("environment", "unix:///tmp/rootless-test.sock")
    monkeypatch.setenv("DOCKER_CONTEXT", "default")
    assert endpoint()[:2] == ("default", "unix:///var/run/docker.sock")


def test_repeated_failed_start_reuses_diagnostics_and_offline(studio, qtbot):
    w = studio
    w.start_ros()
    first = w.docker_dialog
    w.start_ros()
    assert w.docker_dialog is first and first.isVisible()
    assert not w.wizard and w.mode == "offline" and w.state_manager.snapshot().valid
    w.manager.start.assert_not_called()
    first.close()


@pytest.mark.parametrize("action", [a for _, a in StepDialog.ACTIONS])
def test_task_action_mouse_keyboard_save_reopen_and_project(studio, qtbot, tmp_path, action):
    w, motion = studio, studio.motion
    dialog = StepDialog(motion.chain, motion.positions, motion.current(), parent=w)
    qtbot.addWidget(dialog)
    dialog.show()
    choose(dialog.action, dialog.action.findData(action))
    QTest.mouseClick(dialog.name, Qt.MouseButton.LeftButton)
    QTest.keyClicks(dialog.name, "Typed " + action)
    type_number(dialog.duration, "1.25")
    assert dialog.relative_panel.isVisible() == (action == "move_relative")
    assert dialog.cartesian.isVisible() == (action == "move_cartesian")
    assert dialog.joints.isVisible() == (action == "move_joint")
    if action == "move_cartesian":
        type_number(dialog.cartesian.values[0], "80.25")
        type_number(dialog.cartesian.values[2], "150")
        choose(dialog.cartesian.frame, dialog.cartesian.frame.findText(motion.chain.base_link))
    elif action == "move_relative":
        type_number(dialog.relative[0], "1.5")
        type_number(dialog.relative[5], "0")
        choose(dialog.frame, dialog.frame.findText(motion.chain.base_link))
    elif action in ("open_gripper", "close_gripper"):
        assert dialog.gripper_values
        spin, _ = next(iter(dialog.gripper_values.values()))
        type_number(spin, str(spin.minimum()))
    elif action == "move_joint":
        _, spin, *_ = next(iter(dialog.joints.controls.values()))
        type_number(spin, "5")
    buttons = dialog.findChild(QDialogButtonBox)
    QTest.mouseClick(
        buttons.button(QDialogButtonBox.StandardButton.Save), Qt.MouseButton.LeftButton
    )
    assert dialog.result() == QDialog.DialogCode.Accepted
    step = dialog.step()
    assert step.duration == 1.25
    reopened = StepDialog(motion.chain, motion.positions, motion.current(), step, w)
    qtbot.addWidget(reopened)
    reopened.show()
    assert reopened.step() == step
    task = MotionTask(name="Input regression", steps=[step])
    assert TaskCompiler(motion.chain, motion.positions).compile(task, motion.current()).valid
    project = w.collect()
    project.tasks.append(task)
    path = tmp_path / "user-input.yaml"
    save_project(project, path)
    assert load_project(path).tasks[-1].steps == [step]


def test_direct_joint_target_cannot_cross_model_binding(studio):
    m = studio.motion
    step = TaskStep(action="move_joint", joint_values=m.current(), model_signature="another robot")
    compiled = TaskCompiler(m.chain, m.positions).compile(
        MotionTask(name="Bad binding", steps=[step]), m.current()
    )
    assert not compiled.valid and compiled.errors


def test_all_solver_entries_mouse_and_keyboard_keep_active_solver(studio, qtbot):
    m = studio.motion
    studio.robot_panel.detail_tabs.setCurrentWidget(m.kin_panel)
    # Chain setup is the second inner kinematics tab.
    m.solver.parentWidget().parentWidget().setCurrentWidget(m.solver.parentWidget())
    active = m.config.solver
    for index in range(m.solver.count()):
        choose(m.solver, index)
        assert m.config.solver == active and m.kinematics_ready
        assert "Active solver:" in m.solver_info.text()
    QTest.keyClick(m.solver, Qt.Key.Key_Home)
    for index in range(m.solver.count()):
        assert m.solver.currentIndex() == index
        QTest.keyClick(m.solver, Qt.Key.Key_Down)
    m.confirm_chain()
    assert not m.runner.busy and m.config.solver == active
    choose(m.solver, m.solver.findData("scipy_trf"))
    QTest.mouseClick(m.confirm, Qt.MouseButton.LeftButton)
    qtbot.waitUntil(lambda: not m.runner.busy)
    assert m.config.solver == "scipy_trf" and m.kinematics_ready


def test_drag_split_merge_restore_identity_and_corrupt_layout(studio, qtbot):
    w, workspace = studio, studio.tabs
    panels = tuple(workspace.panels)
    state_manager = w.state_manager
    drag_tab(workspace, w.monitor, workspace.groups[0], "right")
    assert isinstance(workspace.root, QSplitter) and len(workspace.groups) == 2
    assert workspace.root.orientation() == Qt.Orientation.Horizontal
    assert w.robot_panel.isVisible() and w.monitor.isVisible()
    saved = workspace.state()
    w.save_workspace()
    drag_tab(workspace, w.monitor, workspace.group_for(w.robot_panel), "center")
    assert len(workspace.groups) == 1
    assert workspace.restore(saved)
    assert workspace.state() == saved
    workspace.reset()
    drag_tab(workspace, w.monitor, workspace.groups[0], "bottom")
    assert workspace.root.orientation() == Qt.Orientation.Vertical
    broken = deepcopy(saved)
    broken["tree"]["children"][0]["tabs"] = [2]
    assert not workspace.restore(broken)
    assert tuple(workspace.panels) == panels and w.state_manager is state_manager
    w.restore_workspace()
    assert workspace.root.orientation() == Qt.Orientation.Horizontal
    workspace.reset()
    assert len(workspace.groups) == 1 and workspace.count() == 4


def test_header_selection_scene_history_and_missing_files_are_local(studio, tmp_path):
    w = studio
    header = w.simulation_header
    p = w.backend_manager.add(new_profile("isaac_sim"))
    w.backend_manager.select_profile(p.id)
    assert header.runtime.currentData() == p.id
    assert not header.auto_play.isChecked()
    scene = tmp_path / "arm.usda"
    scene.write_text("#usda 1.0")
    header.assign_scene(str(scene))
    assert header.scene.currentData() == str(scene)
    assert str(scene) not in w.collect().model_dump_json()
    scene.unlink()
    header.refresh()
    assert header.scene.currentText().startswith("Missing")
    header.remove_recent()
    assert not header.local().get("scene_path")
    hardware = w.backend_manager.add(new_profile("real_robot"))
    w.backend_manager.select_profile(hardware.id)
    assert header.file_label.text() == "Hardware Profile"
    assert not header.scene.isEnabled()
    assert "Read Only" in header.status.text()
    w.manager.start.assert_not_called()


def test_unsupported_or_missing_solver_cannot_replace_working_configuration(studio, monkeypatch):
    from ros_robot_studio.kinematics.solvers import KinematicsSolverManager, SolverDescriptor

    m = studio.motion
    active = m.config.model_copy(deep=True)
    monkeypatch.setattr(
        KinematicsSolverManager,
        "catalog",
        lambda self: [SolverDescriptor("scipy_trf", "SciPy", False, True)],
    )
    m.solver.setCurrentIndex(m.solver.findData("scipy_trf"))
    m.confirm_chain()
    assert not m.runner.busy and m.config == active and m.kinematics_ready
    assert "Dependency missing" in m.solver_info.text()


def test_scene_change_requires_confirmation_before_stopping_or_persisting(
    studio, tmp_path, monkeypatch
):
    w, header = studio, studio.simulation_header
    p = w.backend_manager.add(new_profile("isaac_sim"))
    w.backend_manager.select_profile(p.id)
    a, b = tmp_path / "first.usd", tmp_path / "second.usd"
    a.write_text("scene a")
    b.write_text("scene b")
    header.assign_scene(str(a))
    stop = MagicMock()
    monkeypatch.setattr(w.isaac_processes, "stop", stop)
    w.motion.backend.status = "playing"
    monkeypatch.setattr(header, "confirm_change", lambda *args: False)
    header.assign_scene(str(b))
    assert header.local()["scene_path"] == str(a)
    stop.assert_not_called()
    monkeypatch.setattr(header, "confirm_change", lambda *args: True)
    header.assign_scene(str(b))
    assert header.local()["scene_path"] == str(b)
    stop.assert_called_once_with(p.id)
    assert w.motion.backend.status != "playing"


def test_banner_and_advanced_manager_share_selection(studio):
    w = studio
    a = w.backend_manager.add(new_profile("ros_demo"))
    b = w.backend_manager.add(new_profile("real_robot"))
    w.open_backend_connections()
    dialog = w.backend_workspace.dialog
    w.backend_manager.select_profile(a.id)
    assert dialog.selected_id() == a.id
    w.backend_manager.select_profile(b.id)
    assert dialog.selected_id() == b.id
    assert w.simulation_header.runtime.currentData() == b.id
    dialog.close()


def test_invalid_docker_context_is_diagnostic_not_worker_exception(tmp_path, monkeypatch):
    monkeypatch.delenv("DOCKER_CONTEXT", raising=False)
    monkeypatch.delenv("DOCKER_HOST", raising=False)
    monkeypatch.setenv("DOCKER_CONFIG", str(tmp_path))
    (tmp_path / "config.json").write_text('{"currentContext": 77}')
    from ros_robot_studio.runtime.manager import RuntimeManager

    status = RuntimeManager().status()
    assert not status.available and status.reason == "invalid_context"


def test_disabled_profile_is_rejected_before_simulator_start(studio):
    w = studio
    p = w.backend_manager.add(
        new_profile("isaac_sim", enabled=False, connection_mode="local_installation")
    )
    w.docker_status = RuntimeStatus(True, "Test API")
    with pytest.raises(ValueError, match="disabled"):
        w.simulation_header.start_profile(p.id)
    assert not w.isaac_processes.processes


def test_isaac_other_owned_or_external_instance_requires_choice(studio, tmp_path, monkeypatch):
    from ros_robot_studio.backends.isaac import IsaacSimInstallationManager

    w, header = studio, studio.simulation_header
    p = w.backend_manager.add(
        new_profile(
            "isaac_sim",
            connection_mode="local_installation",
            network_confirmed=True,
            simulation_confirmed=True,
        )
    )
    w.backend_manager.select_profile(p.id)
    scene = tmp_path / "scene.usd"
    scene.write_text("test")
    header.assign_scene(str(scene))
    w.docker_status = RuntimeStatus(True, "Test API", installed=["humble"])
    installation = MagicMock(bridge_distributions=["humble"])
    monkeypatch.setattr(IsaacSimInstallationManager, "validate", lambda *args: installation)
    monkeypatch.setattr(w.isaac_processes, "external_instances", lambda *args: [{"pid": 123}])
    choice = MagicMock(return_value="cancel")
    monkeypatch.setattr(header, "existing_instance_action", choice)
    header.start_profile(p.id)
    choice.assert_called_once_with([], [{"pid": 123}])
    assert not w.isaac_processes.processes
    # Also prompt for an already owned process belonging to a different profile.
    monkeypatch.setattr(w.isaac_processes, "external_instances", lambda *args: [])
    w.isaac_processes.processes["other-backend"] = MagicMock()
    try:
        choice.reset_mock()
        header.start_profile(p.id)
        choice.assert_called_once_with(["other-backend"], [])
    finally:
        w.isaac_processes.processes.clear()


def test_scene_status_does_not_reuse_validation_for_new_selection(studio):
    w, header = studio, studio.simulation_header
    p = w.backend_manager.add(new_profile("isaac_sim"))
    w.backend_manager.select_profile(p.id)
    w.isaac_processes.processes[p.id] = MagicMock()
    w.isaac_processes.ready_reports[p.id] = {"valid": True}
    try:
        header.scene_ready_keys[p.id] = ("old-scene.usd", p.model_signature, p.domain_id)
        header.refresh_status()
        assert "Scene: Unverified" in header.status.text()
    finally:
        w.isaac_processes.processes.clear()
        w.isaac_processes.ready_reports.clear()


def test_isaac_banner_requires_reconnect_after_execution_failure(studio, monkeypatch):
    w, header = studio, studio.simulation_header
    manager = w.backend_manager
    profile = manager.add(new_profile("isaac_sim"))
    manager.select_profile(profile.id)
    manager.active_id = profile.id
    manager.sessions[profile.id] = SimpleNamespace(
        status="Execution Ready",
        transport=SimpleNamespace(connected=True),
        clock_seen=True,
        last_clock_received=manager.clock(),
        capabilities=SimpleNamespace(available={"joint_state_feedback"}, execution_permitted=True),
        requires_reconnect=False,
    )
    monkeypatch.setattr(w.ros_execution, "ready", lambda: not w.ros_execution.reconnect_required)
    try:
        header.refresh_status()
        assert "Simulation Ready" in header.status.text()
        w.ros_execution.reconnect_required = True
        header.refresh_status()
        assert (
            "Play: Playing" in header.status.text() and "State: Available" in header.status.text()
        )
        assert "Reconnect required" in header.status.text()
        assert "Simulation Ready" not in header.status.text()
    finally:
        manager.active_id = None
        manager.sessions.clear()


def test_setup_access_help_does_not_recommend_adding_an_existing_group(studio):
    w = studio
    w.open_wizard()
    wizard = w.wizard
    wizard.system_diagnostics = dict(account_groups=["docker"], effective_groups=["local-user"])
    wizard.docker_status_text = "Current process lacks effective socket group"
    wizard.docker_access_help()
    text = wizard.docker_output.toPlainText()
    assert "already belongs" in text and "Do not add the group again" in text
    assert "usermod" not in text
    wizard.reject()


def test_header_error_remains_visible_and_context_controls_follow_worker(studio, qtbot):
    w, header = studio, studio.simulation_header
    p = w.backend_manager.add(new_profile("isaac_sim"))
    w.backend_manager.select_profile(p.id)

    def fail():
        raise ValueError("Select the scene first")

    header.perform(fail)
    header.refresh_status()
    assert "Select the scene first" in header.status.text()
    from threading import Event

    release = Event()
    try:
        w.runner.run(lambda _: release.wait(5))
        assert not header.runtime.isEnabled() and not header.browse.isEnabled()
    finally:
        release.set()
    qtbot.waitUntil(lambda: not w.runner.busy)
    assert header.runtime.isEnabled() and header.browse.isEnabled()


def test_docker_api_failure_after_cached_success_uses_central_dialog(studio, qtbot):
    w = studio
    w.docker_status = RuntimeStatus(True, "Previously reachable")
    w.show_error(
        "Error while fetching server API version: PermissionError(13, 'Permission denied')"
    )
    assert w.docker_dialog and w.docker_dialog.isVisible()
    assert not w.docker_status.available
    qtbot.waitUntil(lambda: not w.runner.busy)
    w.docker_dialog.close()


@pytest.mark.parametrize(
    "kind,extension",
    [
        ("isaac_sim", ".usd"),
        ("isaac_sim", ".usda"),
        ("isaac_sim", ".usdc"),
        ("mujoco", ".xml"),
        ("mujoco", ".mjcf"),
        ("gazebo", ".sdf"),
        ("gazebo", ".world"),
    ],
)
def test_scene_file_filter_formats_and_cancel_preserve_selection(
    studio, tmp_path, monkeypatch, kind, extension
):
    # This checks the file-selector contract, not SDK parsing/articulation validity.
    from PySide6.QtWidgets import QFileDialog

    w, header = studio, studio.simulation_header
    p = w.backend_manager.add(new_profile(kind))
    w.backend_manager.select_profile(p.id)
    selected = tmp_path / ("selected" + extension)
    selected.write_text("selector fixture; SDK validation is a separate real integration test")
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *args: (str(selected), ""))
    QTest.mouseClick(header.browse, Qt.MouseButton.LeftButton)
    assert header.scene.currentData() == str(selected)
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *args: ("", ""))
    QTest.mouseClick(header.browse, Qt.MouseButton.LeftButton)
    assert header.scene.currentData() == str(selected)
    invalid = tmp_path / "wrong.txt"
    invalid.write_text("wrong extension")
    with pytest.raises(ValueError, match="supported type"):
        header.assign_scene(str(invalid))
    assert header.local()["scene_path"] == str(selected)
    assert str(selected) not in w.collect().model_dump_json()


def test_all_relative_components_and_frame_roundtrip_with_real_input(studio, qtbot):
    import math

    m = studio.motion
    dialog = StepDialog(m.chain, m.positions, m.current(), TaskStep(action="move_relative"), studio)
    qtbot.addWidget(dialog)
    dialog.show()
    values = (1.25, -2.5, 3.75, 4, -5, 6)
    for spin, value in zip(dialog.relative, values):
        assert spin.isEnabled() and not spin.isReadOnly()
        type_number(spin, str(value))
    choose(dialog.frame, dialog.frame.findText(m.chain.end_effector))
    step = dialog.step()
    assert step.relative_xyz == pytest.approx(tuple(v / 1000 for v in values[:3]))
    assert step.relative_rpy == pytest.approx(tuple(math.radians(v) for v in values[3:]))
    assert step.reference_frame == m.chain.end_effector
    reopened = StepDialog(m.chain, m.positions, m.current(), step, studio)
    qtbot.addWidget(reopened)
    reopened.show()
    assert reopened.step() == step
    # The editor preserves rotation input; the existing limited-rank solver must
    # still refuse unsupported full-pose execution rather than pretend success.
    if not m.chain.analyze().full_pose_available:
        compiled = TaskCompiler(m.chain, m.positions).compile(
            MotionTask(name="Relative orientation check", steps=[step]), m.current()
        )
        assert not compiled.valid and "six" in compiled.errors[0]


def test_cartesian_orientation_mode_and_units_survive_editing(studio, qtbot):
    import math

    m = studio.motion
    dialog = StepDialog(
        m.chain, m.positions, m.current(), TaskStep(action="move_cartesian"), studio
    )
    qtbot.addWidget(dialog)
    dialog.show()
    choose(dialog.cartesian.mode, dialog.cartesian.mode.findData("axis"))
    choose(dialog.cartesian.axis, 0)
    for spin, value in zip(dialog.cartesian.values, (80.25, 1.5, 150, 10, -20, 30)):
        assert spin.isEnabled() and not spin.isReadOnly()
        type_number(spin, str(value))
    step = dialog.step()
    assert step.target_pose.position == pytest.approx((0.08025, 0.0015, 0.15))
    assert step.target_pose.rpy == pytest.approx(tuple(math.radians(v) for v in (10, -20, 30)))
    assert step.target_pose.mode == "axis" and step.target_pose.axis == "X"
    reopened = StepDialog(m.chain, m.positions, m.current(), step, studio)
    qtbot.addWidget(reopened)
    reopened.show()
    assert reopened.step() == step


def test_vertical_groups_scroll_without_compressing_existing_controls(studio, qtbot):
    w = studio
    drag_tab(w.tabs, w.monitor, w.tabs.groups[0], "bottom")
    qtbot.wait(50)
    for panel in (w.robot_panel, w.monitor):
        page = w.tabs.pages[panel]
        assert page.verticalScrollBar().maximum() > 0
        assert panel.height() >= panel.minimumSizeHint().height()
    assert not w.robot_panel.view.geometry().intersects(w.robot_panel.joint_controls.geometry())
    assert not w.monitor.robot_state.table.geometry().intersects(
        w.monitor.robot_state.plot.geometry()
    )


def test_main_close_waits_for_setup_worker_before_accepting(studio, qtbot, monkeypatch):
    from threading import Event

    release = Event()

    def report():
        release.wait(5)
        return {
            "docker_available": False,
            "Docker daemon / permissions": "Fixture: no Docker",
            "Docker process diagnostics": {},
        }

    monkeypatch.setattr("ros_robot_studio.gui.wizard.system_report", report)
    studio.open_wizard()
    qtbot.waitUntil(lambda: studio.wizard.runner.busy)
    studio.dirty = False
    try:
        assert not studio.close() and not studio.closed
    finally:
        release.set()
    qtbot.waitUntil(lambda: studio.closed, timeout=7000)
    assert not studio.wizard.runner.busy
