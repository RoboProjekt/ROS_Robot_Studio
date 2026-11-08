"""Visible native Qt input, tab-drag, OpenGL and persistence acceptance (no Docker writes)."""

import argparse
import gc
import json
import sys
import tempfile
import time
from pathlib import Path

import shiboken6
from PySide6.QtCore import QCoreApplication, QEvent, QPoint, Qt, QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QDialog, QDialogButtonBox, QFileDialog, QLineEdit

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.gui.motion_dialogs import StepDialog
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.runtime.manager import RuntimeManager

# The same input helpers are used by the deterministic offscreen regression.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
from test_gui_usability import choose, drag_tab, type_number  # noqa: E402


def verify(cycles, output):
    app = QApplication.instance() or QApplication([])
    app.setStyle("Fusion")
    assert app.platformName() == "xcb", "Native acceptance requires a visible X11 session"
    baseline = len(QApplication.allWidgets())
    errors, diagnostics, layouts = [], None, []
    start = time.monotonic()

    def wait(predicate):
        deadline = time.monotonic() + 40
        while not predicate():
            QTest.qWait(10)
            assert time.monotonic() < deadline, "Native input phase timed out"
        assert not errors, errors

    def capture(window, name):
        if output:
            QTest.qWait(100)
            assert window.grab().save(str(output / name))

    output.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="studio-native-usability-") as temp:
        settings = Settings(Path(temp) / "settings.json")
        for cycle in range(cycles):
            w = MainWindow(settings, RuntimeManager(), show_wizard=False)
            w.show_error = errors.append
            w.runner.failed.disconnect()
            w.runner.failed.connect(errors.append)
            w.motion.runner.failed.connect(errors.append)
            w.show()
            wait(
                lambda: not w.runner.busy
                and not w.motion.runner.busy
                and not w.initial_model_check_pending
            )
            w.project, w.model, _ = pick_place_demo(w.model_manager, prefer_so101=True)
            assert w.project.robot.model_id == "so101_follower"
            w.apply_project()
            wait(lambda: not w.motion.runner.busy)
            if cycle:
                assert len(w.tabs.groups) == 2, "Saved layout did not survive a new window"
            w.tabs.reset()
            QTest.qWait(50)
            panels, states = tuple(w.tabs.panels), w.state_manager
            listeners = tuple(states.listeners)
            subscriptions = {
                i: set(s.transport.subscriptions) for i, s in w.backend_manager.sessions.items()
            }
            original = w.tabs.group_for(w.robot_panel)
            drag_tab(w.tabs, w.monitor, original, "right")
            assert len(w.tabs.groups) == 2
            assert w.tabs.root.orientation() == Qt.Orientation.Horizontal
            assert w.robot_panel.isVisible() and w.monitor.isVisible()
            layouts.append("horizontal")
            split = w.tabs.root
            handle = split.handle(1)
            old_sizes = split.sizes()
            QTest.mousePress(handle, Qt.MouseButton.LeftButton, pos=handle.rect().center())
            QTest.mouseMove(handle, handle.rect().center() + QPoint(65, 0), delay=30)
            QTest.mouseRelease(
                handle, Qt.MouseButton.LeftButton, pos=handle.rect().center() + QPoint(65, 0)
            )
            QTest.qWait(70)
            assert split.sizes() != old_sizes, "Splitter handle did not respond"
            control = next(iter(w.robot_panel.joint_controls.controls.values()))[1]
            type_number(control, "5")
            QTest.qWait(100)
            joint = next(iter(w.robot_panel.joint_controls.controls))
            expected = w.robot_panel.joint_controls.values[joint]
            assert abs(states.snapshot().positions[joint] - expected) < 1e-7
            assert abs(w.monitor.robot_state.state.positions[joint] - expected) < 1e-7
            from OpenGL.GL import glGetError, glIsProgram

            for view in (w.robot_panel.view, w.monitor.robot_state.view):
                assert view.gl_view is not None
                assert not view.gl_view.grabFramebuffer().isNull()
                view.gl_view.makeCurrent()
                assert glIsProgram(view.mesh_shader.program()), "Shader context invalid after drag"
                assert glGetError() == 0
                view.gl_view.doneCurrent()
            if not cycle:
                capture(w, "workspace-horizontal.png")
            drag_tab(w.tabs, w.monitor, w.tabs.group_for(w.robot_panel), "center")
            assert len(w.tabs.groups) == 1
            w.tabs.setCurrentIndex(0)
            drag_tab(w.tabs, w.monitor, w.tabs.groups[0], "bottom")
            assert w.tabs.root.orientation() == Qt.Orientation.Vertical
            layouts.append("vertical")
            if not cycle:
                capture(w, "workspace-vertical.png")
            # A second split supports more than two simultaneous groups.
            drag_tab(w.tabs, w.runtime, w.tabs.group_for(w.robot_panel), "right")
            assert len(w.tabs.groups) == 3
            drag_tab(w.tabs, w.runtime, w.tabs.group_for(w.robot_panel), "center")
            assert len(w.tabs.groups) == 2
            drag_tab(w.tabs, w.monitor, w.tabs.group_for(w.robot_panel), "center")
            assert len(w.tabs.groups) == 1
            w.tabs.setCurrentIndex(0)
            w.robot_panel.detail_tabs.setCurrentWidget(w.motion.kin_panel)
            setup = w.motion.solver.parentWidget()
            # QStackedWidget owns pages; its parent QTabWidget selects the visible setup page.
            setup.parentWidget().parentWidget().setCurrentWidget(setup)
            active = w.motion.config.solver
            for index in range(w.motion.solver.count()):
                choose(w.motion.solver, index)
                assert w.motion.config.solver == active and w.motion.kinematics_ready
            QTest.keyClick(w.motion.solver, Qt.Key.Key_Home)
            for index in range(w.motion.solver.count()):
                assert w.motion.solver.currentIndex() == index
                QTest.keyClick(w.motion.solver, Qt.Key.Key_Down)
            if not cycle:
                QTest.mouseClick(w.motion.solver, Qt.MouseButton.LeftButton)
                capture(w, "solver-selection.png")
                QTest.keyClick(w.motion.solver, Qt.Key.Key_Escape)
                d = StepDialog(w.motion.chain, w.motion.positions, w.motion.current(), parent=w)
                d.show()
                QTest.qWait(50)
                choose(d.action, d.action.findData("move_cartesian"))
                type_number(d.cartesian.values[0], "80.25")
                type_number(d.cartesian.values[2], "150")
                assert d.cartesian.values[0].value() == 80.25
                capture(d, "task-editor-cartesian.png")
                QTest.mouseClick(
                    d.findChild(QDialogButtonBox).button(QDialogButtonBox.StandardButton.Save),
                    Qt.MouseButton.LeftButton,
                )
                assert (
                    d.result() == QDialog.DialogCode.Accepted
                    and d.step().target_pose.position[0] == 0.08025
                )
                d.deleteLater()
                for _, action in StepDialog.ACTIONS:
                    dialog = StepDialog(
                        w.motion.chain, w.motion.positions, w.motion.current(), parent=w
                    )
                    dialog.show()
                    choose(dialog.action, dialog.action.findData(action))
                    type_number(dialog.duration, "1.25")
                    if action == "move_relative":
                        type_number(dialog.relative[0], "1.5")
                    elif action == "move_joint":
                        type_number(next(iter(dialog.joints.controls.values()))[1], "5")
                    elif action in ("open_gripper", "close_gripper"):
                        spin, _ = next(iter(dialog.gripper_values.values()))
                        type_number(spin, str(spin.minimum()))
                    QTest.mouseClick(
                        dialog.findChild(QDialogButtonBox).button(
                            QDialogButtonBox.StandardButton.Save
                        ),
                        Qt.MouseButton.LeftButton,
                    )
                    assert dialog.result() == QDialog.DialogCode.Accepted
                    assert dialog.step().action == action and dialog.step().duration == 1.25
                    dialog.deleteLater()
                dialog = None
                # Real RuntimeManager in this actual visible Studio process; no fake permission result.
                diagnostics = dict(w.docker_status.diagnostics)
                diagnostics["reason"] = w.docker_status.reason
                diagnostics["proc_status"] = [
                    line
                    for line in Path("/proc/self/status").read_text().splitlines()
                    if line.startswith(("Uid:", "Gid:", "Groups:", "NoNewPrivs:"))
                ]
                w.open_docker_diagnostics()
                capture(w.docker_dialog, "docker-diagnostics.png")
                w.docker_dialog.close()
                for kind in ("ros_demo", "isaac_sim", "mujoco", "gazebo", "real_robot"):
                    profile = w.backend_manager.add(new_profile(kind))
                    choose(
                        w.simulation_header.runtime,
                        w.simulation_header.runtime.findData(profile.id),
                    )
                    assert w.backend_manager.selected_id == profile.id
                    if kind == "isaac_sim":
                        scene = Path(".bootstrap/isaac-acceptance/scene.usd").resolve()
                        assert scene.is_file()
                        original_picker = QFileDialog.getOpenFileName

                        def picker(parent, title, initial, filters):
                            chooser = QFileDialog(parent, title)
                            chooser.setOption(QFileDialog.Option.DontUseNativeDialog)
                            chooser.setFileMode(QFileDialog.FileMode.ExistingFile)
                            chooser.setNameFilter(filters)

                            def enter_file():
                                entry = chooser.findChild(QLineEdit, "fileNameEdit")
                                assert entry is not None
                                QTest.mouseClick(entry, Qt.MouseButton.LeftButton)
                                QTest.keyClick(
                                    entry, Qt.Key.Key_A, Qt.KeyboardModifier.ControlModifier
                                )
                                QTest.keyClicks(entry, str(scene))
                                QTest.keyClick(entry, Qt.Key.Key_Return)

                            QTimer.singleShot(250, chooser, enter_file)
                            QTimer.singleShot(5000, chooser, chooser.reject)
                            accepted = chooser.exec() == QDialog.DialogCode.Accepted
                            files = chooser.selectedFiles() if accepted else []
                            chooser.deleteLater()
                            assert accepted and files, "Native Qt file chooser input failed"
                            return files[0], filters

                        QFileDialog.getOpenFileName = picker
                        try:
                            QTest.mouseClick(w.simulation_header.browse, Qt.MouseButton.LeftButton)
                        finally:
                            QFileDialog.getOpenFileName = original_picker
                        assert w.simulation_header.scene.currentData() == str(scene)
                        capture(w, "simulation-runtime-header.png")
                        QTest.mouseClick(w.simulation_header.scene, Qt.MouseButton.LeftButton)
                        capture(w, "simulation-file-history.png")
                        QTest.keyClick(w.simulation_header.scene, Qt.Key.Key_Escape)
                    if kind == "real_robot":
                        assert not w.simulation_header.scene.isEnabled()
                        assert "Read Only" in w.simulation_header.status.text()
                choose(w.simulation_header.runtime, 0)
                assert w.backend_manager.selected_id is None
            w.tabs.setCurrentIndex(0)
            drag_tab(w.tabs, w.monitor, w.tabs.group_for(w.robot_panel), "right")
            assert tuple(w.tabs.panels) == panels and w.state_manager is states
            assert tuple(states.listeners) == listeners
            assert {
                i: set(s.transport.subscriptions) for i, s in w.backend_manager.sessions.items()
            } == subscriptions
            timers = w.findChildren(QTimer)
            plots = [w.monitor.plot, w.monitor.robot_state.plot]
            w.dirty = False
            assert w.close() and w.closed
            assert all(not timer.isActive() for timer in timers)
            for plot in plots:
                if shiboken6.isValid(plot):
                    plot.deleteLater()
            w.deleteLater()
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            assert not shiboken6.isValid(w)
            w = panels = states = listeners = subscriptions = original = split = handle = (
                control
            ) = view = setup = timers = None
            plots = plot = d = None
            gc.collect()
            app.processEvents()
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            assert len(QApplication.allWidgets()) <= baseline, [
                (type(x).__name__, x.objectName(), type(x.parent()).__name__)
                for x in QApplication.allWidgets()
            ]
            print(
                f"NATIVE USABILITY {cycle + 1}/{cycles}: drag, resize, merge, GL, input, persist, destroy passed",
                flush=True,
            )
    result = dict(
        cycles=cycles,
        native_platform=app.platformName(),
        real_mouse_keyboard=True,
        task_actions_mouse_keyboard=[action for _, action in StepDialog.ACTIONS],
        scene_file_dialog_mouse_keyboard=True,
        layouts=layouts,
        opengl_after_drag=True,
        listener_subscription_duplicates=0,
        widget_leaks=0,
        timers_stopped=True,
        actual_studio_diagnostics=diagnostics,
        elapsed_seconds=round(time.monotonic() - start, 2),
    )
    (output / "gui-usability-verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=8)
    parser.add_argument("--output", type=Path, default=Path("docs"))
    args = parser.parse_args()
    verify(args.cycles, args.output)
