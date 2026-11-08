from unittest.mock import MagicMock

import numpy as np
from PySide6.QtWidgets import QMessageBox

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.manager import RuntimeStatus


def make_window(qtbot, tmp_path, tiny_catalog, monkeypatch):
    models, _, _, _ = tiny_catalog
    errors = []
    monkeypatch.setattr(MainWindow, "show_error", lambda self, text: errors.append(text))
    monkeypatch.setattr(QMessageBox, "question", lambda *args: QMessageBox.StandardButton.Yes)
    runtime = MagicMock()
    runtime.specs = catalog()
    runtime.status.return_value = RuntimeStatus(False, "No Docker")
    window = MainWindow(
        Settings(tmp_path / "preferences.json"), runtime, show_wizard=False, model_manager=models
    )
    qtbot.addWidget(window)
    window.show()
    qtbot.waitUntil(lambda: not window.initial_model_check_pending and not window.runner.busy)
    return window, models, runtime, errors


def test_catalog_selection_install_joint_motion_and_project_reload(
    qtbot, tmp_path, tiny_catalog, monkeypatch
):
    window, models, runtime, errors = make_window(qtbot, tmp_path, tiny_catalog, monkeypatch)
    panel = window.robot_panel
    original_model = window.model
    panel.profile.setCurrentIndex(panel.profile.findData("test_arm"))
    qtbot.waitUntil(lambda: not window.runner.busy)
    assert window.model is original_model
    assert panel.install_model_button.isVisible()
    assert "not loaded" in panel.structure_note.text()
    # Selection does not silently download or assign a generic model to the chosen ID.
    assert models.status("test_arm").state == "not_installed"
    panel.install_model_button.click()
    qtbot.waitUntil(lambda: window.project.robot.model_id == "test_arm", timeout=5000)
    assert not errors
    assert window.model_progress is None
    assert not panel.follow_live.isChecked()
    assert panel.renderer_status.text().startswith("Kinematic structure")
    assert "OpenGL unavailable" in panel.renderer_status.text()
    before = window.model.transforms(panel.joint_controls.values)["hand"].copy()
    slider, spin, _, _, _ = panel.joint_controls.controls["hinge"]
    slider.setValue(8000)
    after = window.model.transforms(panel.joint_controls.values)["hand"]
    assert not np.allclose(before, after)
    assert window.robot_panel.view.values["hinge"] > 0
    assert "R " in panel.pose.text()
    value = panel.joint_controls.values["hinge"]
    window.distro.setCurrentIndex(2)
    assert window.project.default_ros == "humble"
    assert panel.joint_controls.values["hinge"] == value
    # Generic demo messages must not overwrite the virtual catalog pose.
    window.receive_message("/joint_states", {"name": ["hinge"], "position": [-0.5]})
    assert panel.joint_controls.values["hinge"] == value
    window.project_path = tmp_path / "model.studio.yaml"
    window.save()
    qtbot.waitUntil(lambda: not window.runner.busy)
    assert not errors and window.project_path.exists()
    path = window.project_path
    window.dirty = False
    window.new_project()
    window.dirty = False
    window.open_path(path, confirmed=True)
    qtbot.waitUntil(lambda: not window.runner.busy)
    assert not errors
    assert window.project.robot.model_id == "test_arm"
    assert panel.joint_controls.values["hinge"] == value
    assert window.project.robot.source_revision == "a" * 40
    assert window.project.robot.hardware is None
    runtime.start.assert_not_called()
    runtime.stop.assert_not_called()
    window.dirty = False
    window.close()


def test_missing_model_on_open_offers_exact_reinstallation(
    qtbot, tmp_path, tiny_catalog, monkeypatch
):
    window, models, _, errors = make_window(qtbot, tmp_path, tiny_catalog, monkeypatch)
    models.install("test_arm")
    window.select_robot_model("test_arm")
    qtbot.waitUntil(lambda: not window.runner.busy)
    window.robot_panel.joint_controls.controls["hinge"][1].setValue(30)
    window.project_path = tmp_path / "pose.yaml"
    window.save()
    qtbot.waitUntil(lambda: not window.runner.busy)
    saved_path = window.project_path
    saved_value = window.robot_panel.joint_controls.values["hinge"]
    spec = models.spec("test_arm")
    (models.directory(spec) / "assets/part.stl").unlink()
    window.dirty = False
    window.new_project()
    window.dirty = False
    window.open_path(saved_path, confirmed=True)
    qtbot.waitUntil(
        lambda: window.project.robot.model_id == "test_arm" and not window.runner.busy, timeout=5000
    )
    assert not errors
    assert models.status("test_arm").state == "installed"
    assert window.robot_panel.joint_controls.values["hinge"] == saved_value
    window.dirty = False
    window.close()
