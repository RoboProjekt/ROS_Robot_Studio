"""Native OpenGL acceptance test: installed SO-101, sliders, pose, save/reopen, screenshots.

Run from the repository with the same Qt environment as start.sh. No downloads or devices.
"""

from __future__ import annotations

import json
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from PySide6.QtCore import QPoint, QPointF, Qt
from PySide6.QtGui import QVector3D, QWheelEvent
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.projects.store import load_project
from ros_robot_studio.robots.urdf import pose_rpy


def main():
    app = QApplication([])
    app.setStyle("Fusion")
    errors = []
    with tempfile.TemporaryDirectory(prefix="studio-so101-gui-") as temporary:
        window = MainWindow(Settings(Path(temporary) / "settings.json"), show_wizard=False)
        window.runner.failed.disconnect()
        window.runner.failed.connect(errors.append)
        window.show_error = errors.append
        window.show()

        def wait(predicate, timeout=15000):
            deadline = time.monotonic() + timeout / 1000
            while not predicate():
                app.processEvents()
                if errors:
                    raise AssertionError("; ".join(errors))
                if time.monotonic() > deadline:
                    raise TimeoutError("Native GUI acceptance step timed out")
                QTest.qWait(20)

        try:
            wait(lambda: not window.initial_model_check_pending and not window.runner.busy)
            panel = window.robot_panel
            panel.name.setText("SO-101 · Offline workbench")
            panel.description.setPlainText(
                "Original new_calib follower · virtual joint control · no hardware connected"
            )
            panel.profile.setCurrentIndex(panel.profile.findData("so101_follower"))
            wait(
                lambda: window.project.robot.model_id == "so101_follower" and not window.runner.busy
            )
            QTest.qWait(600)
            assert panel.view.gl_view is not None and panel.view.gl_view.isValid()
            assert len(panel.view.mesh_items) == 17
            assert panel.view.renderer.startswith("OpenGL meshes")
            assert not panel.follow_live.isChecked() and not panel.follow_live.isEnabled()
            gl_view = panel.view.gl_view
            azimuth = gl_view.opts["azimuth"]
            center = QPoint(gl_view.width() // 2, gl_view.height() // 2)
            QTest.mousePress(gl_view, Qt.MouseButton.LeftButton, pos=center)
            QTest.mouseMove(gl_view, center + QPoint(45, 20), delay=30)
            QTest.mouseRelease(gl_view, Qt.MouseButton.LeftButton, pos=center + QPoint(45, 20))
            assert gl_view.opts["azimuth"] != azimuth
            old_center = gl_view.opts["center"]
            QTest.mousePress(gl_view, Qt.MouseButton.RightButton, pos=center)
            QTest.mouseMove(gl_view, center + QPoint(30, 15), delay=30)
            QTest.mouseRelease(gl_view, Qt.MouseButton.RightButton, pos=center + QPoint(30, 15))
            assert gl_view.opts["center"] != old_center
            distance = gl_view.opts["distance"]
            wheel = QWheelEvent(
                QPointF(center),
                QPointF(gl_view.mapToGlobal(center)),
                QPoint(0, 0),
                QPoint(0, 120),
                Qt.MouseButton.NoButton,
                Qt.KeyboardModifier.NoModifier,
                Qt.ScrollPhase.ScrollUpdate,
                False,
            )
            app.sendEvent(gl_view, wheel)
            assert gl_view.opts["distance"] != distance
            panel.axes_toggle.setChecked(False)
            assert not panel.view.root_axes.visible()
            panel.axes_toggle.setChecked(True)
            panel.meshes_toggle.setChecked(False)
            assert panel.view.renderer.startswith("Kinematic structure")
            panel.meshes_toggle.setChecked(True)
            assert len(panel.view.mesh_items) == 17
            assert window.container_id is None
            before = window.model.transforms(panel.joint_controls.values)[
                "gripper_frame_link"
            ].copy()
            panel.view.reset_camera()
            QTest.qWait(150)
            Path("docs").mkdir(exist_ok=True)
            assert window.grab().save("docs/so101.png")
            # Real Qt slider and numeric entry changes, each constrained by the source limits.
            panel.joint_controls.controls["shoulder_pan"][0].setValue(6200)
            for name, degrees in [
                ("shoulder_lift", -25),
                ("elbow_flex", 35),
                ("wrist_flex", 15),
                ("gripper", 25),
            ]:
                panel.joint_controls.controls[name][1].setValue(degrees)
            after = window.model.transforms(panel.joint_controls.values)[
                "gripper_frame_link"
            ].copy()
            assert not np.allclose(before, after)
            assert not np.allclose(before[:3, 3], after[:3, 3])
            assert panel.view.values == panel.joint_controls.values
            assert "gripper_frame_link" in panel.pose.text()
            # Verify that the GL item uses the exact same world visual transform as FK.
            item, visual = panel.view.mesh_items[-1]
            geometry = window.model.meshes[visual.mesh]
            point = geometry.vertices[0]
            expected = (
                window.model.visual_transform(
                    visual, window.model.transforms(panel.joint_controls.values)
                )
                @ np.r_[point, 1]
            )
            actual = item.transform().map(QVector3D(*point))
            np.testing.assert_allclose(
                [actual.x(), actual.y(), actual.z()], expected[:3], atol=1e-6
            )
            pose = dict(panel.joint_controls.values)
            window.project_path = Path(temporary) / "so101.studio.yaml"
            window.save()
            wait(lambda: not window.runner.busy)
            saved = window.project_path
            parsed = load_project(saved)
            assert parsed.robot.joint_positions == pose
            assert parsed.robot.urdf.startswith("model://so101_follower/")
            window.dirty = False
            window.new_project()
            window.dirty = False
            window.open_path(saved, confirmed=True)
            wait(
                lambda: window.project.robot.model_id == "so101_follower" and not window.runner.busy
            )
            assert panel.joint_controls.values == pose
            np.testing.assert_allclose(
                window.model.transforms(pose)["gripper_frame_link"], after, atol=1e-12
            )
            assert len(panel.view.mesh_items) == 17
            panel.view.reset_camera()
            QTest.qWait(250)
            assert window.grab().save("docs/so101-pose.png")
            report = dict(
                verified_at=datetime.now(timezone.utc).isoformat(),
                model_id=parsed.robot.model_id,
                source_revision=parsed.robot.source_revision,
                renderer=panel.view.renderer,
                links=len(window.model.links),
                joints=len(window.model.joints),
                movable_joints=len(panel.joint_controls.controls),
                visuals=len(panel.view.mesh_items),
                zero_pose_xyz_rpy=list(pose_rpy(before)),
                edited_pose_xyz_rpy=list(pose_rpy(after)),
                joint_positions_si=pose,
                mesh_transform_verified=True,
                camera_orbit_pan_zoom_verified=True,
                axes_and_mesh_toggle_verified=True,
                project_roundtrip_verified=True,
                real_hardware_access=False,
                docker_runtime_started=False,
                screenshots=["docs/so101.png", "docs/so101-pose.png"],
            )
            Path("docs/so101-gui-verification.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps(report, indent=2))
        finally:
            window.dirty = False
            wait(lambda: not window.runner.busy)
            window.close()
            app.processEvents()


if __name__ == "__main__":
    main()
