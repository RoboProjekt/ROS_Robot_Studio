"""Real end-to-end ROS simulation acceptance, with optional native screenshots. No hardware."""

from __future__ import annotations

import argparse
import json
import socket
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distro", choices=["humble", "jazzy", "lyrical"], default="humble")
    parser.add_argument("--screenshots", action="store_true")
    args = parser.parse_args()
    app = QApplication([])
    app.setStyle("Fusion")
    errors = []
    observed = []
    with tempfile.TemporaryDirectory(prefix="studio-ros-acceptance-") as temp:
        w = MainWindow(Settings(Path(temp) / "settings.json"), show_wizard=False)
        w.show_error = errors.append
        w.runner.failed.disconnect()
        w.runner.failed.connect(errors.append)
        w.ros_execution.error.connect(errors.append)
        w.show()

        def wait(predicate, timeout=40):
            deadline = time.monotonic() + timeout
            while not predicate():
                app.processEvents()
                QTest.qWait(10)
                if errors:
                    raise AssertionError("; ".join(errors))
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"{args.distro}: {w.ros_execution.status}; {w.ros_status}; "
                        + w.runtime.output.toPlainText()[-2500:]
                    )

        def idle():
            wait(
                lambda: not w.runner.busy
                and not w.motion.runner.busy
                and not w.initial_model_check_pending
            )

        def capture(name):
            if args.screenshots:
                QTest.qWait(180)
                assert w.grab().save("docs/" + name + ".png")
                if name.startswith("ros-live-monitoring"):
                    assert w.monitor.robot_state.plot.grab().save(
                        "docs/joint-position-velocity-graphs.png"
                    )

        try:
            idle()
            project, model, _ = pick_place_demo(w.model_manager)
            assert project.robot.model_id == "so101_follower"
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                project.runtime.port = probe.getsockname()[1]
            w.project = project
            w.model = model
            w.apply_project()
            idle()
            w.distro.setCurrentIndex(w.distro.findData(args.distro))
            idle()
            # Offline controls and real cubic derivatives drive the same monitoring snapshot.
            w.robot_panel.joint_controls.controls["shoulder_pan"][1].setValue(10)
            state = w.state_manager.snapshot()
            assert state.positions["shoulder_pan"] != 0 and state.velocities is None
            assert w.monitor.robot_state.state.positions == state.positions
            w.motion.compile_task(True)
            idle()
            wait(lambda: w.motion.backend.elapsed > 0.2)
            assert w.state_manager.snapshot().velocities is not None
            w.tabs.setCurrentIndex(2)
            from PySide6.QtCore import Qt

            for i in range(w.monitor.robot_state.fields.count()):
                item = w.monitor.robot_state.fields.item(i)
                item.setCheckState(
                    Qt.CheckState.Checked
                    if item.text()
                    in (
                        "position.shoulder_pan",
                        "position.shoulder_lift",
                        "velocity.shoulder_pan",
                        "velocity.shoulder_lift",
                        "tool.z",
                    )
                    else Qt.CheckState.Unchecked
                )
            capture("offline-robot-monitoring")
            w.motion.stop()
            w.motion.reset()
            assert args.distro in w.docker_status.installed, (
                "Install the current runtime before acceptance"
            )
            print("START", args.distro, flush=True)
            w.start_ros()
            idle()
            wait(lambda: w.ros_execution.ready())
            assert w.mode == "ros" and w.state_manager.snapshot().backend == "ros_simulation"
            assert not w.robot_panel.joint_controls.isEnabled()
            w.state_manager.subscribe(
                lambda state: observed.append(state)
                if state.backend == "ros_simulation" and state.valid
                else None
            )
            w.transport.refresh_graph()
            wait(
                lambda: any(
                    "studio_simulated_robot_controller" in w.monitor.nodes.item(i).text()
                    for i in range(w.monitor.nodes.count())
                )
            )
            w.tabs.setCurrentIndex(3)
            print("EXECUTE", args.distro, flush=True)
            w.motion.compile_task(True)
            idle()
            assert w.motion.plan.valid
            wait(lambda: w.ros_execution.status == "executing")
            w.tabs.setCurrentIndex(2)
            wait(lambda: w.ros_execution.index >= 3)
            fields = w.monitor.robot_state.fields
            for i in range(fields.count()):
                name = fields.item(i).text()

                fields.item(i).setCheckState(
                    Qt.CheckState.Checked
                    if name
                    in [
                        "position.shoulder_pan",
                        "position.shoulder_lift",
                        "velocity.shoulder_pan",
                        "velocity.shoulder_lift",
                        "tool.z",
                    ]
                    else Qt.CheckState.Unchecked
                )
            capture("ros-live-monitoring-" + args.distro)
            w.tabs.setCurrentIndex(0)
            w.robot_panel.detail_tabs.setCurrentWidget(w.motion.program_panel)
            w.motion.program_tabs.setCurrentIndex(1)
            capture("ros-pick-place-" + args.distro)
            wait(lambda: w.ros_execution.status == "succeeded")
            assert w.motion.backend.status != "playing", (
                "Offline backend must never animate a ROS task"
            )
            end = w.ros_execution.plan.steps[-1].trajectory.end
            actual = w.state_manager.snapshot()
            for n, value in end.items():
                assert abs(actual.positions[n] - value) <= w.project.execution.goal_tolerance
            assert len(observed) > 10 and any(
                state.velocities and any(abs(v) > 0.005 for v in state.velocities.values())
                for state in observed
            )
            assert w.monitor.robot_state.state.positions == w.state_manager.snapshot().positions
            for n, value in w.robot_panel.view.values.items():
                assert value == w.state_manager.snapshot().positions[n]
            # Exercise the user-facing diagnostic action; Verified is earned by action and feedback.
            wait(lambda: not w.ros_execution.remote_busy)
            w.verify_demo()
            wait(lambda: w.verification_stage == "succeeded")
            idle()
            w.tabs.setCurrentIndex(3)
            capture("ros-runtime-" + args.distro)
            # Cancel during a real action, then reconnect explicitly and run again.
            w.motion.compile_task(True)
            idle()
            wait(lambda: w.ros_execution.status == "executing")
            w.motion.stop()
            assert w.ros_execution.status == "cancelled"
            old_transport = w.transport
            old_lease = w.ros_lease
            w.request_offline()
            idle()
            wait(lambda: w.mode == "offline")
            assert not old_transport.active and not w.state_manager.update(
                old_lease, actual.positions
            )
            w.start_ros()
            idle()
            wait(lambda: w.ros_execution.ready())
            w.request_offline()
            idle()
            assert w.mode == "offline"
            report = dict(
                verified_at=datetime.now(timezone.utc).isoformat(),
                distribution=args.distro,
                robot="so101_follower",
                runtime_image=w.manager.specs[args.distro].image,
                image_id=w.docker_status.image_ids.get(args.distro),
                docker_access=True,
                ros_graph=True,
                native_action=True,
                task_steps=13,
                jointstate_samples=len(observed),
                positions_velocities_consistent=True,
                offline_monitoring=True,
                mode_roundtrip_twice=True,
                cancel=True,
                runtime_diagnostics=True,
                old_source_rejected=True,
                hardware_access=False,
                isaac_sim="NOT TESTED: no running compatible scene",
            )
            Path("docs").mkdir(exist_ok=True)
            Path(f"docs/ros-{args.distro}-verification.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(json.dumps(report, indent=2), flush=True)
        finally:
            errors.clear()
            if w.container_id:
                w.ros_execution.abort()
                while w.runner.busy:
                    app.processEvents()
                    QTest.qWait(10)
                w.stop_ros()
                wait(lambda: not w.runner.busy)
            while w.motion.runner.busy:
                w.motion.cancel_work()
                app.processEvents()
                QTest.qWait(10)
            w.dirty = False
            w.close()
            app.processEvents()


if __name__ == "__main__":
    main()
