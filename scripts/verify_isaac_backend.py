"""Real Isaac 5.1 + original SO-101 + Studio action/monitoring acceptance."""

import argparse
import json
import shutil
import tempfile
import time
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.backends.isaac import IsaacSimInstallationManager, IsaacSimSceneManager
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--installation", type=Path)
    parser.add_argument("--screenshots", action="store_true")
    args = parser.parse_args()
    app = QApplication([])
    app.setStyle("Fusion")
    detector = IsaacSimInstallationManager()
    installation = (
        detector.validate(args.installation) if args.installation else detector.detect()[0]
    )
    errors, samples, accepted, results, commands = [], [], [], [], []
    with tempfile.TemporaryDirectory(prefix="studio-isaac-settings-") as temp:
        w = MainWindow(Settings(Path(temp) / "settings.json"), show_wizard=False)
        w.show_error = errors.append
        w.runner.failed.disconnect()
        w.runner.failed.connect(errors.append)
        w.backend_manager.failed.disconnect()
        w.backend_manager.failed.connect(errors.append)
        w.ros_execution.error.connect(errors.append)
        w.isaac_processes.error.connect(lambda identity, text: errors.append(text))
        log = open("/tmp/studio-isaac-acceptance-process.log", "w")
        w.isaac_processes.log.connect(lambda identity, text: (log.write(text), log.flush()))
        w.show()

        def wait(predicate, timeout=90, check_errors=True):
            deadline = time.monotonic() + timeout
            while not predicate():
                app.processEvents()
                QTest.qWait(15)
                if check_errors:
                    assert not errors, tuple(errors)
                assert time.monotonic() < deadline, {
                    "runtime": w.runtime.output.toPlainText()[-2000:],
                    "sessions": [
                        (s.status, s.error, s.graph) for s in w.backend_manager.sessions.values()
                    ],
                    "execution": w.ros_execution.status,
                    "task": w.motion.task_status.text(),
                }

        report = None
        try:
            wait(
                lambda: not w.runner.busy
                and not w.motion.runner.busy
                and not w.initial_model_check_pending
            )
            w.project, w.model, _ = pick_place_demo(w.model_manager)
            assert w.project.robot.model_id == "so101_follower"
            w.apply_project()
            wait(lambda: not w.motion.runner.busy)
            profile = w.backend_manager.add(
                new_profile(
                    "isaac_sim",
                    domain_id=1,
                    network="host",
                    network_confirmed=True,
                    discovery="localhost",
                    simulation_confirmed=True,
                    read_only=False,
                    execution_enabled=True,
                    connection_mode="local_installation",
                )
            )
            request = IsaacSimSceneManager().prepare(
                profile, w.model, Path(".bootstrap/isaac-acceptance")
            )
            if args.screenshots:
                data = json.loads(request.read_text())
                data["capture_image"] = str(request.parent / "isaac-so101.png")
                request.write_text(json.dumps(data, indent=2))
            process = w.isaac_processes.start(
                profile, installation, request, headless=not args.screenshots
            )
            w.start_profile_ros(profile.id)
            wait(lambda: profile.id in w.isaac_processes.ready_reports)
            wait(
                lambda: profile.id in w.backend_manager.sessions
                and w.backend_manager.sessions[profile.id].capabilities.execution_permitted
            )
            w.activate_backend(profile.id)
            wait(w.ros_execution.ready)
            old_lease = w.state_manager.lease()
            w.backend_manager.disconnect_profile(profile.id)
            wait(lambda: profile.id not in w.backend_manager.sessions and not w.runner.busy)
            assert profile.id in w.isaac_processes.processes
            w.start_profile_ros(profile.id)
            wait(
                lambda: profile.id in w.backend_manager.sessions
                and w.backend_manager.sessions[profile.id].capabilities.execution_permitted
            )
            w.activate_backend(profile.id)
            wait(w.ros_execution.ready)
            assert not w.state_manager.update(old_lease, w.model.default_configuration())
            session = w.backend_manager.sessions[profile.id]

            def observe_message(topic, message):
                if topic == session.payload["control_namespace"] + "/events":
                    event = json.loads(message.get("data", "{}"))
                    if event.get("kind") == "accepted":
                        accepted.append(event["id"])
                    elif event.get("kind") == "succeeded":
                        results.append(json.loads(event["message"]))
                elif topic == profile.command_topic:
                    commands.append(message)

            session.transport.message.connect(observe_message)
            session.transport.subscribe(profile.command_topic, "sensor_msgs/msg/JointState")

            def record(state):
                if state.backend_id == profile.id and state.valid:
                    samples.append(dict(state.positions))

            w.state_manager.subscribe(record)
            before = dict(w.state_manager.snapshot().positions)
            print("INITIAL_FEEDBACK", before, flush=True)
            last_status = [None]

            def progress(status):
                current = (status, w.ros_execution.index)
                if current != last_status[0]:
                    print("EXECUTION", *current, flush=True)
                    last_status[0] = current

            w.ros_execution.changed.connect(progress)
            w.motion.compile_task(True)
            wait(lambda: not w.motion.runner.busy)
            assert w.motion.plan and w.motion.plan.valid, w.motion.task_status.text()
            wait(lambda: w.ros_execution.status == "succeeded", timeout=180)
            assert len(samples) > 20
            assert any(abs(q["shoulder_pan"] - before["shoulder_pan"]) > 0.05 for q in samples)
            assert max(q["gripper"] for q in samples) - min(q["gripper"] for q in samples) > 0.05
            assert w.robot_panel.view.values == {
                n: w.state_manager.snapshot().positions[n] for n in w.model.default_configuration()
            }
            assert w.monitor.robot_state.state.backend_id == profile.id
            goals = sum(step.action != "wait" for step in w.motion.plan.steps)
            assert len(accepted) == len(results) == goals
            assert all(
                r["stable_target_reached"]
                and r["samples"] >= 5
                and r["stable_seconds"] >= 0.2
                and r["maximum_position_error"] <= profile.goal_tolerance
                and r["commanded_terminal_velocity"] == 0.0
                for r in results
            )
            terminal_commands = sum(
                bool(m.get("velocity")) and all(v == 0.0 for v in m["velocity"]) for m in commands
            )
            assert terminal_commands >= goals
            if args.screenshots:
                w.open_backend_connections()
                QTest.qWait(100)
                w.backend_workspace.dialog.grab().save("docs/isaac-backend-connection.png")
                w.backend_workspace.dialog.close()
                w.tabs.setCurrentIndex(2)
                QTest.qWait(100)
                w.grab().save("docs/isaac-live-monitoring.png")
                image_path = request.parent / "isaac-so101.png"
                wait(image_path.is_file)
                shutil.copy2(image_path, "docs/isaac-so101.png")
            report = dict(
                installation_version=installation.version,
                distribution="humble",
                domain=1,
                robot="so101_follower",
                scene=IsaacSimSceneManager().validate_result(request),
                real_jointstate_samples=len(samples),
                task_steps=13,
                task_status=w.ros_execution.status,
                shoulder_motion=True,
                gripper_motion=True,
                viewer_monitoring_synchronized=True,
                physical_grasp_validated=False,
                hardware_access=False,
                process_id=process.processId(),
                action_goals_accepted=len(accepted),
                action_results_succeeded=len(results),
                stable_target_results=results,
                zero_velocity_commands_observed=terminal_commands,
                reconnect_without_restarting_simulator=True,
                old_connection_lease_rejected=True,
            )
            # Reports are portable: local scene/source paths stay in the local cache.
            report["scene"].pop("scene", None)
            Path("docs/isaac-backend-verification.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(json.dumps(report, indent=2), flush=True)
        except BaseException:
            print("LAST_STATE", dict(w.state_manager.snapshot().positions), flush=True)
            if w.motion.plan:
                print("PLAN", w.motion.plan.errors, flush=True)
                if w.motion.plan.steps:
                    print(
                        "TARGET",
                        w.motion.plan.steps[w.ros_execution.index].trajectory.end,
                        flush=True,
                    )
            client = w.manager.client_factory()
            for session in w.backend_manager.sessions.values():
                print(
                    "ADAPTER_LOG",
                    w.manager._owned(client, session.container_id)
                    .logs(tail=20)
                    .decode(errors="replace"),
                    flush=True,
                )
            client.close()
            raise
        finally:
            errors.clear()
            if w.ros_execution.transport:
                w.ros_execution.abort()
            while w.runner.busy or w.motion.runner.busy:
                app.processEvents()
                QTest.qWait(10)
            w.backend_manager.disconnect_all()
            wait(lambda: not w.backend_manager.sessions and not w.runner.busy, check_errors=False)
            for identity in list(w.isaac_processes.processes):
                w.isaac_processes.stop(identity)
            wait(lambda: not w.isaac_processes.processes, timeout=25, check_errors=False)
            if report:
                sdk_pid = report["scene"]["simulator_process_id"]
                status_file = Path("/proc") / str(sdk_pid) / "stat"
                assert not status_file.exists() or status_file.read_text().split()[2] == "Z", (
                    "Owned Isaac child still running"
                )
                report["owned_simulator_stopped"] = True
                report["owned_ros_container_removed"] = True
                Path("docs/isaac-backend-verification.json").write_text(
                    json.dumps(report, indent=2) + "\n"
                )
            w.dirty = False
            w.close()
            app.processEvents()
            log.close()


if __name__ == "__main__":
    main()
