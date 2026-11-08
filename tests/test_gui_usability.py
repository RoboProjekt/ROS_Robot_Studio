"""Exercise user input, identity, persistence and truthful capability presentation."""

from copy import deepcopy
from unittest.mock import MagicMock

import pytest
from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QDialog, QDialogButtonBox, QSplitter

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
    rect = view.visualRect(combo.model().index(index, 0))
    QTest.mouseMove(view.viewport(), rect.center(), delay=15)
    QTest.mouseClick(view.viewport(), Qt.MouseButton.LeftButton, pos=rect.center())
    QTest.qWait(15)
    assert combo.currentIndex() == index, (
        index,
        combo.currentIndex(),
        combo.itemText(index),
        combo.model().flags(combo.model().index(index, 0)),
        combo.isVisible(),
        combo.isEnabled(),
        view.isVisible(),
        rect,
        view.indexAt(rect.center()).row(),
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
