"""Real header-driven existing-USD startup, manual pause and explicit Auto Play task."""

import json
import sys
import tempfile
import time
from pathlib import Path

from PySide6.QtCore import QPoint, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.backends.isaac import IsaacSimInstallationManager
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tests"))
from test_gui_usability import choose  # noqa: E402


def main():
    app = QApplication([])
    app.setStyle("Fusion")
    errors, report = [], {}
    installation = IsaacSimInstallationManager().detect()[0]
    source = Path(".bootstrap/isaac-acceptance/scene.usd").resolve()
    assert source.is_file(), (
        "Run the existing Isaac acceptance first to generate a compatible source USD"
    )
    with tempfile.TemporaryDirectory(prefix="studio-isaac-header-") as temp:
        w = MainWindow(Settings(Path(temp) / "settings.json"), show_wizard=False)
        w.runner.failed.disconnect()
        w.runner.failed.connect(errors.append)
        w.show_error = errors.append
        w.backend_manager.failed.disconnect()
        w.backend_manager.failed.connect(errors.append)
        w.isaac_processes.error.connect(lambda identity, text: errors.append(text))
        log = open(".bootstrap/isaac-header-process.log", "w")
        w.isaac_processes.log.connect(lambda identity, text: (log.write(text), log.flush()))
        w.show()

        def wait(predicate, timeout=100, check_errors=True):
            deadline = time.monotonic() + timeout
            while not predicate():
                QTest.qWait(15)
                if check_errors:
                    assert not errors, errors
                assert time.monotonic() < deadline, (
                    w.simulation_header.status.text(),
                    w.motion.task_status.text(),
                    w.runtime.output.toPlainText()[-1600:],
                )

        try:
            wait(
                lambda: not w.runner.busy
                and not w.motion.runner.busy
                and not w.initial_model_check_pending
            )
            assert w.docker_status.available and "humble" in w.docker_status.installed
            w.project, w.model, _ = pick_place_demo(w.model_manager)
            assert w.project.robot.model_id == "so101_follower"
            w.apply_project()
            wait(lambda: not w.motion.runner.busy)
            p = w.backend_manager.add(
                new_profile(
                    "isaac_sim",
                    network_confirmed=True,
                    simulation_confirmed=True,
                    read_only=False,
                    execution_enabled=True,
                    connection_mode="local_installation",
                    discovery="localhost",
                )
            )
            header = w.simulation_header
            choose(header.runtime, header.runtime.findData(p.id))
            header.save_local(p.id, dict(installation_path=str(installation.path), headless=True))
            header.assign_scene(str(source))
            assert not header.auto_play.isChecked()
            QTest.mouseClick(header.start, Qt.MouseButton.LeftButton)
            wait(lambda: p.id in w.isaac_processes.ready_reports)
            wait(
                lambda: p.id in w.backend_manager.sessions
                and w.backend_manager.sessions[p.id].transport is not None
            )
            QTest.qWait(2200)
            session = w.backend_manager.sessions[p.id]
            assert "--auto-play" not in w.isaac_processes.processes[p.id].arguments()
            assert not session.clock_seen and not session.capabilities.execution_permitted
            assert "Scene: Loaded & validated" in header.status.text()
            assert "Play: Paused / waiting" in header.status.text()
            assert "Simulation Ready" not in header.status.text()
            report.update(
                manual_play_default=True,
                paused_without_clock_or_motion=True,
                manual_header_status=header.status.text(),
                domain_id=p.domain_id,
                source_scene=str(source),
                existing_usd_validated=True,
            )
            # Explicitly stop this own paused process, then opt in through the actual checkbox.
            header.disconnect_selected()
            w.isaac_processes.stop(p.id)
            wait(
                lambda: p.id not in w.backend_manager.sessions
                and p.id not in w.isaac_processes.processes
                and not w.runner.busy
            )
            QTest.mouseClick(
                header.auto_play,
                Qt.MouseButton.LeftButton,
                pos=QPoint(8, header.auto_play.height() // 2),
            )
            assert header.local()["auto_play"] is True
            QTest.mouseClick(header.start, Qt.MouseButton.LeftButton)
            wait(
                lambda: p.id in w.isaac_processes.ready_reports
                and p.id in w.isaac_processes.processes
            )
            assert "--auto-play" in w.isaac_processes.processes[p.id].arguments()
            wait(lambda: w.backend_manager.active_id == p.id and w.ros_execution.ready())
            header.refresh_status()
            assert "Play: Playing" in header.status.text()
            assert "Simulation Ready" in header.status.text()
            samples = []
            w.state_manager.subscribe(
                lambda state: samples.append(dict(state.positions))
                if state.backend_id == p.id and state.valid
                else None
            )
            w.motion.compile_task(True)
            wait(lambda: not w.motion.runner.busy)
            wait(lambda: w.ros_execution.status in ("succeeded", "failed", "cancelled"), 120)
            assert w.ros_execution.status == "succeeded", w.motion.task_status.text()
            assert len(samples) > 20
            report.update(
                auto_play_explicit_checkbox=True,
                task_status=w.ros_execution.status,
                task_steps=len(w.motion.plan.steps),
                real_jointstate_samples=len(samples),
                running_header_status=header.status.text(),
                source_unmodified=w.isaac_processes.ready_reports[p.id]["urdf_unmodified"],
            )
        finally:
            w.motion.stop()
            w.backend_manager.disconnect_all()
            for identity in list(w.isaac_processes.processes):
                w.isaac_processes.stop(identity)
            wait(
                lambda: not w.backend_manager.sessions
                and not w.isaac_processes.processes
                and not w.runner.busy
                and not w.motion.runner.busy,
                30,
                check_errors=False,
            )
            w.dirty = False
            assert w.close()
            app.processEvents()
            log.close()
        report.update(owned_resources_removed=True)
        Path("docs/isaac-header-verification.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
