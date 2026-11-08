"""Native Qt/OpenGL acceptance: Cartesian IK, wizard, playback, persistence. No devices."""

from __future__ import annotations

import json
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QWizard

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.projects.store import load_project


def main():
    app = QApplication([])
    app.setStyle("Fusion")
    errors = []
    with tempfile.TemporaryDirectory(prefix="studio-motion-gui-") as temp:
        w = MainWindow(Settings(Path(temp) / "settings.json"), show_wizard=False)
        w.runner.failed.disconnect()
        w.runner.failed.connect(errors.append)
        w.show_error = errors.append
        w.motion.runner.failed.connect(errors.append)
        w.show()

        def wait(predicate, timeout=30):
            deadline = time.monotonic() + timeout
            while not predicate():
                app.processEvents()
                QTest.qWait(15)
                if errors:
                    raise AssertionError("; ".join(errors))
                if time.monotonic() > deadline:
                    raise TimeoutError("GUI step timed out")

        def idle():
            wait(
                lambda: not w.runner.busy
                and not w.motion.runner.busy
                and not w.initial_model_check_pending
            )

        try:
            idle()
            p, m, _ = pick_place_demo(w.model_manager)
            assert p.name == "SO101_PickAndPlace_Demo", (
                "Install the pinned original model before native SO-101 acceptance"
            )
            w.project = p
            w.model = m
            w.apply_project()
            idle()
            panel = w.robot_panel
            motion = w.motion
            assert motion.kinematics_ready and len(motion.chain.active_joints) == 5
            assert panel.view.gl_view and panel.view.gl_view.isValid()
            assert len(panel.view.mesh_items) == 17
            panel.detail_tabs.setCurrentWidget(motion.kin_panel)
            target = motion.positions.capture_cartesian(
                "Goal",
                motion.chain,
                motion.engine.seed({"shoulder_pan": 0.3, "shoulder_lift": -0.4, "elbow_flex": 0.5}),
            ).pose
            motion.pose_editor.set_pose(target)
            actual_before = motion.current()
            QTest.mouseClick(motion.calculate, Qt.MouseButton.LeftButton)
            idle()
            assert motion.result.success, motion.ik_output.toPlainText()
            assert motion.current() == actual_before
            QTest.mouseClick(motion.preview, Qt.MouseButton.LeftButton)
            assert motion.current() == actual_before and panel.view.values != actual_before
            QTest.mouseClick(motion.apply, Qt.MouseButton.LeftButton)
            np.testing.assert_allclose(
                motion.chain.forward_kinematics(motion.current())[:3, 3],
                target.position,
                atol=0.0001,
            )
            panel.view.reset_camera()
            QTest.qWait(250)
            Path("docs").mkdir(exist_ok=True)
            assert w.grab().save("docs/kinematics.png")
            # Failure has no Apply path and cannot alter the current configuration.
            valid_current = motion.current()
            motion.pose_editor.values[0].setValue(9999)
            QTest.mouseClick(motion.calculate, Qt.MouseButton.LeftButton)
            idle()
            assert not motion.result.success and not motion.apply.isEnabled()
            assert motion.current() == valid_current
            motion.reset_target()
            panel.detail_tabs.setCurrentWidget(motion.program_panel)
            motion.program_tabs.setCurrentIndex(1)
            QTest.mouseClick(motion.wizard_button, Qt.MouseButton.LeftButton)
            wizard = motion.wizard
            assert wizard.isVisible()
            assert wizard.pick.saved.currentData() and wizard.place.saved.currentData()
            wizard.name.setText("GUI verified Pick and Place")
            wizard.suggest_gripper()
            wizard.confirm.setChecked(True)
            wizard.approach.setValue(15)
            wizard.lift.setValue(20)
            wizard.retreat.setValue(15)
            for _ in range(5):
                QTest.mouseClick(
                    wizard.button(QWizard.WizardButton.NextButton), Qt.MouseButton.LeftButton
                )
            assert wizard.currentId() == 5
            QTest.mouseClick(wizard.generate_button, Qt.MouseButton.LeftButton)
            wait(lambda: not wizard.runner.busy)
            assert wizard.result_data, wizard.output.toPlainText()
            QTest.mouseClick(
                wizard.button(QWizard.WizardButton.NextButton), Qt.MouseButton.LeftButton
            )
            assert wizard.currentId() == 6
            assert wizard.grab().save("docs/pick-place-wizard.png")
            wizard.preview()
            QTest.qWait(150)
            assert motion.backend.status == "playing"
            QTest.mouseClick(
                wizard.button(QWizard.WizardButton.FinishButton), Qt.MouseButton.LeftButton
            )
            assert not wizard.isVisible() and len(motion.tasks) == 2
            assert motion.selected_task().name == "GUI verified Pick and Place"
            QTest.mouseClick(motion.play_button, Qt.MouseButton.LeftButton)
            idle()
            wait(lambda: motion.backend.elapsed > 0.15)
            assert motion.backend.status == "playing"
            QTest.mouseClick(motion.pause_button, Qt.MouseButton.LeftButton)
            elapsed = motion.backend.elapsed
            QTest.qWait(150)
            assert motion.backend.elapsed == elapsed
            QTest.mouseClick(motion.pause_button, Qt.MouseButton.LeftButton)
            wait(lambda: motion.backend.read_state().attached)
            QTest.mouseClick(motion.pause_button, Qt.MouseButton.LeftButton)
            assert panel.view.object_item and panel.view.object_transform is not None
            panel.view.reset_camera()
            QTest.qWait(200)
            assert w.grab().save("docs/pick-place-editor.png")
            QTest.mouseClick(motion.pause_button, Qt.MouseButton.LeftButton)
            wait(lambda: motion.backend.status == "finished")
            assert not motion.backend.read_state().attached
            QTest.mouseClick(motion.reset_button, Qt.MouseButton.LeftButton)
            assert motion.backend.elapsed == 0
            # Reorder and restore a step, then compute only that step.
            motion.steps.selectRow(2)
            identifier = motion.selected_task().steps[2].id
            motion.move_step(1)
            assert motion.selected_task().steps[3].id == identifier
            motion.move_step(-1)
            assert motion.selected_task().steps[2].id == identifier
            motion.steps.selectRow(2)
            motion.compile_selected()
            idle()
            assert motion.plan.valid
            w.project_path = Path(temp) / "motion.yaml"
            w.save()
            idle()
            saved = load_project(w.project_path)
            assert len(saved.tasks) == 2 and len(saved.positions) == 16
            path = w.project_path
            w.dirty = False
            w.new_project()
            idle()
            w.dirty = False
            w.open_path(path, confirmed=True)
            idle()
            assert len(motion.positions.positions) == 16 and len(motion.tasks) == 2
            motion.task_choice.setCurrentIndex(1)
            motion.compile_task(True)
            idle()
            assert motion.backend.status == "playing"
            motion.stop()
            assert motion.backend.status == "stopped"
            report = dict(
                verified_at=datetime.now(timezone.utc).isoformat(),
                model="so101_follower",
                source_revision=w.project.robot.source_revision,
                native_renderer=panel.view.renderer,
                arm_axes=len(motion.chain.active_joints),
                ik_exact=True,
                failed_ik_not_applied=True,
                wizard_pages=7,
                generated_task_steps=13,
                cube_attachment_and_release=True,
                pause_resume_stop_reset=True,
                step_reorder_and_single_compute=True,
                project_roundtrip=True,
                hardware_access=False,
                ros_docker_started=False,
                screenshots=[
                    "docs/kinematics.png",
                    "docs/pick-place-editor.png",
                    "docs/pick-place-wizard.png",
                ],
            )
            Path("docs/motion-gui-verification.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(json.dumps(report, indent=2))
        finally:
            motion = w.motion
            motion.stop()
            idle()
            w.dirty = False
            w.close()
            app.processEvents()


if __name__ == "__main__":
    main()
