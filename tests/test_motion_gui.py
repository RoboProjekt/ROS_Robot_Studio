from unittest.mock import MagicMock

import numpy as np
import pytest
from PySide6.QtWidgets import QFileDialog

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.gui.motion_dialogs import PositionDialog, StepDialog
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.schema import TaskStep
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.manager import RuntimeStatus


@pytest.fixture
def window(qtbot, tmp_path, monkeypatch):
    errors = []
    monkeypatch.setattr(MainWindow, "show_error", lambda self, msg: errors.append(msg))
    manager = MagicMock()
    manager.specs = catalog()
    manager.status.return_value = RuntimeStatus(False, "No Docker")
    w = MainWindow(Settings(tmp_path / "settings.json"), manager, show_wizard=False)
    qtbot.addWidget(w, before_close_func=lambda widget: setattr(widget, "dirty", False))
    w.show()
    qtbot.waitUntil(
        lambda: not w.runner.busy and not w.motion.runner.busy and not w.initial_model_check_pending
    )
    p, m, _ = pick_place_demo(prefer_so101=False)
    w.project = p
    w.model = m
    w.apply_project()
    qtbot.waitUntil(lambda: not w.motion.runner.busy)
    yield w
    assert not errors
    w.motion.stop()
    w.motion.cancel_work()
    qtbot.waitUntil(lambda: not w.motion.runner.busy)
    w.dirty = False
    w.close()
    manager.start.assert_not_called()


def test_cartesian_controls_success_failure_and_preview(window, qtbot):
    w = window
    motion = w.motion
    assert motion.kinematics_ready
    w.robot_panel.detail_tabs.setCurrentWidget(motion.kin_panel)
    motion.pose_editor.values[0].setValue(80)
    motion.pose_editor.values[2].setValue(150)
    old = motion.current()
    motion.calculate.click()
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert motion.result.success
    motion.preview.click()
    assert motion.current() == old and w.robot_panel.view.values != old
    motion.apply.click()
    assert motion.current() != old
    expected = motion.chain.forward_kinematics(motion.current())[:3, 3]
    np.testing.assert_allclose(expected, [0.08, 0, 0.15], atol=0.0001)
    old = motion.current()
    motion.pose_editor.values[0].setValue(10000)
    motion.calculate.click()
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert not motion.result.success and not motion.apply.isEnabled() and motion.current() == old
    assert not motion.pose_editor.values[3].isEnabled()
    assert not motion.pose_editor.mode.model().item(2).isEnabled()


def test_task_editor_order_validation_play_stop_and_bad_step(window, qtbot):
    motion = window.motion
    window.robot_panel.detail_tabs.setCurrentWidget(motion.program_panel)
    motion.program_tabs.setCurrentIndex(1)
    task = motion.selected_task()
    first = task.steps[2].id
    motion.steps.selectRow(2)
    motion.move_step(1)
    assert task.steps[3].id == first
    motion.move_step(-1)
    assert task.steps[2].id == first
    motion.compile_task(True)
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert motion.backend.status == "playing"
    motion.pause()
    assert motion.backend.status == "paused"
    motion.pause()
    assert motion.backend.status == "playing"
    motion.stop()
    assert motion.backend.status == "stopped"
    motion.reset()
    assert motion.backend.elapsed == 0
    task.steps[2].position_id = "missing"
    motion.task_selected()
    motion.compile_task(True)
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert not motion.plan.valid and 2 in motion.plan.errors
    assert motion.steps.item(2, 3).text().startswith("ERROR") and motion.backend.status != "playing"


def test_teach_dialogs_and_export_import(window, qtbot, tmp_path, monkeypatch):
    motion = window.motion
    chain = motion.chain
    dialog = PositionDialog(chain, motion.current(), kind="cartesian")
    qtbot.addWidget(dialog)
    dialog.name.setText("Taught")
    position = dialog.position()
    motion.positions.add(position)
    dialog = PositionDialog(chain, motion.current(), position)
    qtbot.addWidget(dialog)
    dialog.name.setText("Edited")
    motion.positions.add(dialog.position())
    assert motion.positions.get(position.id).name == "Edited"
    step_dialog = StepDialog(
        chain, motion.positions, motion.current(), TaskStep(action="wait", duration=0.2)
    )
    qtbot.addWidget(step_dialog)
    assert step_dialog.step().duration == 0.2
    path = tmp_path / "library.yaml"
    monkeypatch.setattr(QFileDialog, "getSaveFileName", lambda *args: (str(path), ""))
    motion.export_library()
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert path.exists()
    motion.positions.positions = []
    motion.tasks = []
    monkeypatch.setattr(QFileDialog, "getOpenFileName", lambda *args: (str(path), ""))
    motion.import_library()
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert len(motion.tasks) == 1 and motion.positions.get(position.id).name == "Edited"


def test_wizard_validates_and_invalidates_edited_inputs(window, qtbot):
    motion = window.motion
    motion.open_wizard()
    wizard = motion.wizard
    wizard.suggest_gripper()
    wizard.confirm.setChecked(True)
    wizard.generate()
    qtbot.waitUntil(lambda: not wizard.runner.busy, timeout=10000)
    assert wizard.result_data and wizard.result_data[2].valid
    wizard.lift.setValue(wizard.lift.value() + 1)
    assert wizard.result_data is None
    wizard.generate()
    qtbot.waitUntil(lambda: not wizard.runner.busy, timeout=10000)
    assert wizard.result_data
    wizard.finish_generation()
    assert len(motion.tasks) == 2
    wizard.reject()


def test_stopping_or_editing_pending_compile_never_plays(window, qtbot):
    motion = window.motion
    motion.compile_task(True)
    motion.stop()
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert motion.backend.status != "playing"
    motion.compile_task(True)
    motion.speed.setValue(30)
    qtbot.waitUntil(lambda: not motion.runner.busy)
    assert motion.backend.status != "playing"


def test_desktop_starts_without_optional_scipy(qtbot, tmp_path, monkeypatch):
    import ros_robot_studio.kinematics.solvers as solvers

    real_find = solvers.importlib.util.find_spec
    monkeypatch.setattr(
        solvers.importlib.util,
        "find_spec",
        lambda name, *a, **kw: None if name == "scipy" else real_find(name, *a, **kw),
    )
    manager = MagicMock()
    manager.specs = catalog()
    manager.status.return_value = RuntimeStatus(False, "No Docker")
    w = MainWindow(Settings(tmp_path / "optional.json"), manager, show_wizard=False)
    qtbot.addWidget(w, before_close_func=lambda widget: setattr(widget, "dirty", False))
    w.show()
    qtbot.waitUntil(
        lambda: not w.motion.runner.busy and not w.initial_model_check_pending and not w.runner.busy
    )
    assert w.motion.chain and not w.motion.kinematics_ready and not w.motion.calculate.isEnabled()
    w.robot_panel.detail_tabs.setCurrentWidget(w.motion.kin_panel)
    assert not w.motion.install_button.isHidden()
    w.dirty = False
    w.close()
