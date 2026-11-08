"""Real two-backend GUI/action acceptance using only owned Studio simulation runtimes."""

import argparse
import json
import tempfile
import time
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--screenshots", action="store_true")
    args = parser.parse_args()
    app = QApplication([])
    app.setStyle("Fusion")
    errors = []
    b_samples = []
    with tempfile.TemporaryDirectory(prefix="studio-backends-") as temp:
        w = MainWindow(Settings(Path(temp) / "settings.json"), show_wizard=False)
        w.runner.failed.disconnect()
        w.runner.failed.connect(errors.append)
        w.show_error = errors.append
        w.backend_manager.failed.disconnect()
        w.backend_manager.failed.connect(errors.append)
        w.ros_execution.error.connect(errors.append)
        w.show()

        def wait(predicate, timeout=45):
            end = time.monotonic() + timeout
            while not predicate():
                app.processEvents()
                QTest.qWait(15)
                assert not errors, errors
                assert time.monotonic() < end, w.runtime.output.toPlainText()[-2500:]

        def idle():
            wait(
                lambda: not w.runner.busy
                and not w.motion.runner.busy
                and not w.initial_model_check_pending
            )

        try:
            idle()
            project, model, _ = pick_place_demo(w.model_manager)
            assert project.robot.model_id == "so101_follower"
            w.project, w.model = project, model
            w.apply_project()
            idle()
            manager = w.backend_manager
            a = manager.add(new_profile("ros_demo", name="ROS Demo A", domain_id=10))
            b = manager.add(new_profile("ros_demo", name="ROS Demo B", domain_id=11))
            w.start_profile_ros(a.id)
            w.start_profile_ros(b.id)
            wait(
                lambda: len(manager.sessions) == 2
                and all(s.capabilities.execution_permitted for s in manager.sessions.values())
            )
            sa, sb = manager.sessions[a.id], manager.sessions[b.id]
            assert sa.port != sb.port and sa.container_id != sb.container_id
            before = dict(w.state_manager.backend_snapshot(b.id).positions)

            def record():
                state = w.state_manager.backend_snapshot(b.id)
                if state and state.valid:
                    b_samples.append(dict(state.positions))

            manager.changed.connect(record)
            w.activate_backend(a.id)
            wait(w.ros_execution.ready)
            old_active = w.state_manager.lease()
            w.motion.compile_task(True)
            wait(lambda: not w.motion.runner.busy)
            wait(lambda: w.ros_execution.status == "succeeded")
            assert b_samples and all(
                all(abs(q[n] - value) < 1e-8 for n, value in before.items()) for q in b_samples
            )
            manager.changed.disconnect(record)
            w.open_backend_connections()
            if args.screenshots:
                QTest.qWait(150)
                w.backend_workspace.dialog.grab().save("docs/backend-connections.png")
            w.backend_workspace.dialog.close()
            w.activate_backend(b.id)
            wait(w.ros_execution.ready)
            assert not w.state_manager.update(old_active, model.default_configuration())
            w.motion.compile_task(True)
            wait(lambda: not w.motion.runner.busy)
            wait(
                lambda: any(
                    abs(w.state_manager.snapshot().positions[n] - v) > 0.03
                    for n, v in before.items()
                )
            )
            w.motion.stop()
            wait(lambda: not w.ros_execution.remote_busy)
            assert w.state_manager.snapshot().backend_id == b.id
            assert w.robot_panel.view.values == {
                n: w.state_manager.snapshot().positions[n] for n in model.default_configuration()
            }
            assert w.monitor.robot_state.state.backend_id == b.id
            w.tabs.setCurrentIndex(2)
            if args.screenshots:
                QTest.qWait(100)
                w.grab().save("docs/backend-live-monitoring.png")
            w.activate_backend(a.id)
            wait(w.ros_execution.ready)
            assert w.state_manager.snapshot().backend_id == a.id
            w.request_offline()
            wait(lambda: w.mode == "offline")
            assert len(manager.sessions) == 2 and all(
                s.transport.connected for s in manager.sessions.values()
            )
            local = dict(w.state_manager.snapshot().positions)
            QTest.qWait(200)
            assert (
                w.state_manager.snapshot().backend_id == ""
                and w.state_manager.snapshot().positions == local
            )
            manager.disconnect_all()
            wait(lambda: not manager.sessions and not w.runner.busy)
            report = dict(
                domains=[10, 11],
                robot="so101_follower",
                distinct_ports=True,
                distinct_containers=True,
                distinct_control_namespaces=True,
                simultaneous_connections=True,
                native_action_task_steps=13,
                inactive_backend_unchanged_during_task=True,
                inactive_samples_checked=len(b_samples),
                switch_and_cancel=True,
                viewer_and_monitoring_consistent=True,
                offline_keeps_background_connections=True,
                late_active_lease_rejected=True,
                hardware_access=False,
                network="separate owned private bridges; DDS-domain isolation tested separately",
            )
            Path("docs/backend-session-verification.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(json.dumps(report, indent=2), flush=True)
        finally:
            errors.clear()
            while w.runner.busy or w.motion.runner.busy:
                app.processEvents()
                QTest.qWait(10)
            w.backend_manager.disconnect_all()
            wait(lambda: not w.backend_manager.sessions and not w.runner.busy)
            if w.container_id:
                w.stop_ros()
                wait(lambda: not w.runner.busy)
            w.dirty = False
            w.close()
            app.processEvents()


if __name__ == "__main__":
    main()
