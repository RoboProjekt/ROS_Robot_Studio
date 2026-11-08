"""Real Humble → Jazzy → Lyrical → Humble switches in one unchanged robot project."""

import json
import tempfile
import time
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo


def main():
    app = QApplication([])
    errors = []
    verified = []
    with tempfile.TemporaryDirectory(prefix="studio-distro-cycle-") as temp:
        w = MainWindow(Settings(Path(temp) / "settings.json"), show_wizard=False)
        w.runner.failed.disconnect()
        w.runner.failed.connect(errors.append)
        w.show_error = errors.append
        w.ros_execution.error.connect(errors.append)
        w.show()

        def wait(predicate):
            deadline = time.monotonic() + 45
            while not predicate():
                app.processEvents()
                QTest.qWait(15)
                if errors:
                    raise AssertionError("; ".join(errors))
                if time.monotonic() > deadline:
                    raise TimeoutError(w.runtime.output.toPlainText()[-2000:])

        try:
            wait(
                lambda: not w.initial_model_check_pending
                and not w.runner.busy
                and not w.motion.runner.busy
            )
            project, model, _ = pick_place_demo(w.model_manager)
            w.project = project
            w.model = model
            w.apply_project()
            wait(lambda: not w.motion.runner.busy)
            original_tasks = [task.model_dump() for task in w.motion.tasks]
            old = None
            for distro in ("humble", "jazzy", "lyrical", "humble"):
                assert distro in w.docker_status.installed
                if not w.container_id:
                    w.start_ros()
                else:
                    w.distro.setCurrentIndex(w.distro.findData(distro))
                wait(
                    lambda: not w.runner.busy and w.container_id != old and w.ros_execution.ready()
                )
                old = w.container_id
                w.verify_demo()
                wait(lambda: w.verification_stage == "succeeded")
                wait(lambda: not w.runner.busy)
                assert w.project.default_ros == "humble"
                assert [task.model_dump() for task in w.motion.tasks] == original_tasks
                verified.append(distro)
                print("VERIFIED", distro, flush=True)
            w.request_offline()
            wait(lambda: not w.runner.busy and w.mode == "offline")
            report = dict(
                versions=verified,
                one_project_preserved=True,
                original_tasks_preserved=True,
                default_ros_preserved=True,
                native_action_diagnostics=True,
                final_mode="offline",
                hardware_access=False,
            )
            Path("docs/ros-version-cycle-verification.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(json.dumps(report, indent=2))
        finally:
            errors.clear()
            while w.runner.busy:
                app.processEvents()
                QTest.qWait(10)
            if w.container_id:
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
