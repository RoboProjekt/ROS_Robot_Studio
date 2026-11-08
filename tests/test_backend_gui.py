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
