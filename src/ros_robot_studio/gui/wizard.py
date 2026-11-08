"""First-run setup: all downloads are explicit; offline use is always available."""

import importlib.util
import shutil
import socket
import subprocess
import sys
import uuid
from pathlib import Path

from PySide6.QtCore import QProcess, QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QHBoxLayout,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QVBoxLayout,
    QWizard,
    QWizardPage,
)

from ros_robot_studio.core.models import RuntimeConfig
from ros_robot_studio.gui.widgets import button, label
from ros_robot_studio.gui.workers import TaskRunner
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport
from ros_robot_studio.runtime.catalog import catalog, inspect_registry
from ros_robot_studio.runtime.system import system_report


class SetupWizard(QWizard):
    def __init__(self, manager, settings, verify_callback, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ROS Robot Studio · Setup")
        self.setWizardStyle(QWizard.WizardStyle.ModernStyle)
        self.resize(840, 700)
        self.manager, self.settings = manager, settings
        self.runner = TaskRunner(self)
        self.runner.failed.connect(self.failed)
        self.runner.progress.connect(self.append)
        self.runner.busy_changed.connect(self.busy_changed)
        self.process = None
        self.verifying = False
        self.verify_container = None
        self.verify_transport = None
        self.verify_deadline = QTimer(self)
        self.verify_deadline.setSingleShot(True)
        self.verify_deadline.timeout.connect(
            lambda: self.finish_verification("FAILED: ROS verification timed out")
        )
        self.report_lines = []
        self.checked = {}
        self.available = False
        self.local_installed = []
        self.verify_callback = verify_callback

        system, layout = self.page(
            "01 / System check", "The desktop works without Docker or a local ROS installation."
        )
        self.system_text = QPlainTextEdit("Checking your system…")
        self.system_text.setReadOnly(True)
        layout.addWidget(self.system_text)
        layout.addWidget(button("Run checks again", self.check_system))
        self.offline_button = button(
            "Open offline demo · postpone all runtime downloads", self.skip_to_demo, True
        )
        layout.addWidget(self.offline_button)
        layout.addWidget(
            label(
                "You may continue with the offline demo at any time. No system settings change during these checks.",
                "subtitle",
                True,
            )
        )
        self.addPage(system)

        docker, layout = self.page(
            "02 / Docker Engine", "Use an existing daemon or review the optional Ubuntu installer."
        )
        layout.addWidget(
            label(
                "Docker is needed only for real ROS container data. The offline UI demo requires no daemon.",
                "",
                True,
            )
        )
        layout.addWidget(
            label(
                "The installer adds Docker’s signed apt repository and requests administrator authorization through PolicyKit. It refuses conflicting packages. Membership in the docker group grants extensive host access; Studio never adds you automatically. Rootless Docker is an alternative.",
                "warning",
                True,
            )
        )
        row = QHBoxLayout()
        row.addWidget(button("Review installer", self.review_installer))
        self.install_docker = button("Authorize Docker installation…", self.authorize_docker)
        row.addWidget(self.install_docker)
        layout.addLayout(row)
        self.docker_output = QPlainTextEdit("Docker installation has not been requested.")
        self.docker_output.setReadOnly(True)
        layout.addWidget(self.docker_output)
        self.addPage(docker)

        runtimes, layout = self.page(
            "03 / ROS runtimes",
            "All three runtimes are selected. Downloads begin only when you choose Install.",
        )
        for identifier, spec in catalog().items():
            check = QCheckBox(
                f"{spec.name} · Ubuntu {spec.ubuntu} · compressed size not yet checked"
            )
            check.setChecked(spec.supported)
            check.setEnabled(spec.supported)
            self.checked[identifier] = check
            layout.addWidget(check)
        row = QHBoxLayout()
        self.inspect_button = button("Check image availability & size", self.inspect_images)
        self.install_button = button("Install selected runtimes", self.install_runtimes, True)
        row.addWidget(self.inspect_button)
        row.addWidget(self.install_button)
        layout.addLayout(row)
        layout.addWidget(
            label(
                "Base image sizes exclude additional ROS packages and unpacked disk usage. Plan several GiB per runtime. Uncheck versions to postpone downloads. Upstream catalog validation: 2026-11-07; local builds are verified during installation.",
                "subtitle",
                True,
            )
        )
        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        layout.addWidget(self.progress)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(500)
        layout.addWidget(self.output)
        self.addPage(runtimes)

        verify, layout = self.page(
            "04 / Component verification",
            "Run an end-to-end check using a temporary Studio test container.",
        )
        layout.addWidget(
            label(
                "The runtime healthcheck verifies its adapter port. For an end-to-end check, select a runtime, choose Start ROS demo, refresh the graph and select /studio/sine. A received message proves the publisher → ROS → WebSocket → GUI path.",
                "",
                True,
            )
        )
        self.kinematics_status = label(
            "SciPy bounded IK: "
            + ("installed" if importlib.util.find_spec("scipy") else "optional component missing"),
            "subtitle",
            True,
        )
        layout.addWidget(self.kinematics_status)
        self.kinematics_install = button(
            "Install Universal Kinematics Support · SciPy 1.15.3", self.install_kinematics
        )
        layout.addWidget(self.kinematics_install)
        layout.addWidget(button("Refresh installed component status", self.component_status))
        row = QHBoxLayout()
        self.verify_distro = QComboBox()
        for identifier, spec in catalog().items():
            self.verify_distro.addItem(spec.name, identifier)
        self.verify_button = button("Verify selected ROS runtime", self.verify_runtime)
        row.addWidget(self.verify_distro)
        row.addWidget(self.verify_button)
        layout.addLayout(row)
        self.components = QPlainTextEdit("No runtime test has been performed yet.")
        self.components.setReadOnly(True)
        layout.addWidget(self.components)
        layout.addWidget(
            label(
                "A successful image build alone is not an end-to-end ROS test. The main window provides a separate verification action.",
                "warning",
                True,
            )
        )
        self.addPage(verify)

        finish, layout = self.page(
            "05 / Ready to explore",
            "Your demo project opens immediately. Setup can be reopened from Help.",
        )
        self.summary = QPlainTextEdit()
        self.summary.setReadOnly(True)
        layout.addWidget(self.summary)
        self.demo = QCheckBox("Open the bundled offline demo when setup finishes")
        self.demo.setChecked(True)
        layout.addWidget(self.demo)
        self.addPage(finish)
        self.currentIdChanged.connect(self.page_changed)
        self.accepted.connect(self.completed)
        self.check_system()

    def page(self, title, subtitle):
        page = QWizardPage()
        page.setTitle(title)
        page.setSubTitle(subtitle)
        layout = QVBoxLayout(page)
        layout.setSpacing(16)
        return page, layout

    def skip_to_demo(self):
        if self.runner.busy:
            return
        self.report_lines.append(
            "All ROS runtime downloads explicitly deferred: offline demo selected."
        )
        self.accept()

    @property
    def installer(self):
        return Path(__file__).resolve().parents[3] / "scripts" / "install-docker-ubuntu.sh"

    def check_system(self):
        if self.runner.busy:
            return

        def done(report):
            self.available = report["docker_available"]
            self.system_text.setPlainText(
                "\n\n".join(f"{k}: {v}" for k, v in report.items() if k != "docker_available")
            )
            self.install_button.setEnabled(self.available)
            self.install_docker.setEnabled(
                not self.available and self.installer.exists() and bool(shutil.which("pkexec"))
            )
            self.report_lines.append("System check: " + report["Docker daemon / permissions"])

        self.runner.run(lambda _: system_report(), done)

    def review_installer(self):
        try:
            self.docker_output.setPlainText(self.installer.read_text())
        except OSError as exc:
            self.failed(str(exc))

    def authorize_docker(self):
        if self.process:
            return
        answer = QMessageBox.question(
            self,
            "Administrator authorization",
            "Install Docker Engine from the official Docker apt repository? This modifies system package sources and starts the Docker daemon. No existing containers are changed and no group membership is added. Review installer first if needed.",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.process = QProcess(self)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(
            lambda: self.docker_output.appendPlainText(
                bytes(self.process.readAllStandardOutput()).decode(errors="replace")
            )
        )
        self.process.finished.connect(self.docker_finished)
        self.process.errorOccurred.connect(self.docker_error)
        self.install_docker.setEnabled(False)
        self.busy_changed(True)
        self.process.start("pkexec", ["/bin/bash", str(self.installer)])

    def docker_error(self, error):
        self.docker_output.appendPlainText(self.process.errorString())
        if error == QProcess.ProcessError.FailedToStart:
            self.docker_finished(-1, None)

    def docker_finished(self, code, status):
        self.report_lines.append(f"Docker installer exit code: {code}")
        self.process.deleteLater()
        self.process = None
        self.check_system()

    def append(self, text):
        self.output.appendPlainText(text)

    def failed(self, text):
        self.report_lines.append("Failed: " + text)
        self.append("ERROR: " + text)
        QMessageBox.warning(self, "Setup operation failed", text)

    def inspect_images(self):
        def work(progress):
            results = {}
            for identifier, spec in catalog().items():
                try:
                    results[identifier] = inspect_registry(spec)
                    progress(f"{spec.name}: amd64 manifest available")
                except Exception as exc:
                    results[identifier] = str(exc)
            return results

        def done(results):
            for identifier, result in results.items():
                check = self.checked[identifier]
                if isinstance(result, dict):
                    check.setText(
                        f"ROS 2 {identifier.title()} · verified amd64 · base {result['base_bytes'] / 1e6:.1f} MB compressed + ROS packages"
                    )
                else:
                    check.setChecked(False)
                    check.setText(f"ROS 2 {identifier.title()} · unavailable: {result[:100]}")
                check.setEnabled(isinstance(result, dict) and catalog()[identifier].supported)

        self.runner.run(work, done)

    def install_runtimes(self):
        selected = [
            identifier
            for identifier, check in self.checked.items()
            if check.isChecked() and check.isEnabled()
        ]
        if not selected:
            return

        def work(progress):
            results = []
            for identifier in selected:
                try:
                    self.manager.install(identifier, progress)
                    results.append(
                        f"{identifier}: image built successfully; live verification pending"
                    )
                except Exception as exc:
                    results.append(f"{identifier}: FAILED: {exc}")
            return results

        def done(results):
            self.report_lines.extend(results)
            self.output.appendPlainText("\n".join(results))
            self.progress.setValue(1)
            self.component_status()

        self.runner.run(work, done)

    def busy_changed(self, busy):
        busy = busy or self.verifying or self.process is not None
        for which in (
            QWizard.WizardButton.NextButton,
            QWizard.WizardButton.BackButton,
            QWizard.WizardButton.FinishButton,
            QWizard.WizardButton.CancelButton,
        ):
            self.button(which).setEnabled(not busy)
        self.inspect_button.setEnabled(not busy)
        self.install_button.setEnabled(not busy and self.available)
        self.verify_button.setEnabled(not busy and self.available)
        self.offline_button.setEnabled(not busy)
        self.kinematics_install.setEnabled(not busy)
        self.progress.setRange(0, 0 if busy else 1)

    def component_status(self):
        def done(status):
            self.local_installed = status.installed
            self.components.setPlainText(
                status.message
                + "\n\nInstalled Studio images: "
                + (", ".join(status.installed) or "None")
                + "\n\nLive connection test: not run by this check. Use Verify ROS demo in the main window."
            )

        self.runner.run(lambda _: self.manager.status(), done)

    def install_kinematics(self):
        if self.runner.busy:
            return

        def work(progress):
            if sys.prefix == sys.base_prefix:
                raise ValueError(
                    "Restart with ./start.sh to use the local virtual environment before installing support"
                )
            progress("Installing pinned SciPy 1.15.3 into the current virtual environment…")
            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pip",
                    "install",
                    "--disable-pip-version-check",
                    "--no-deps",
                    "--only-binary=:all:",
                    "scipy==1.15.3",
                ],
                capture_output=True,
                text=True,
                timeout=300,
            )
            if result.returncode:
                raise ValueError(result.stderr[-4000:])
            importlib.invalidate_caches()
            return result.stdout[-4000:]

        def done(output):
            self.kinematics_status.setText(
                "SciPy 1.15.3 installed. Confirm the robot chain to run FK / IK checks."
            )
            self.components.appendPlainText(output)
            self.report_lines.append("Optional universal kinematics support installed")

        self.runner.run(work, done)

    def verify_runtime(self):
        if self.runner.busy or self.verifying:
            return
        distro = self.verify_distro.currentData()
        self.verifying = True
        self.verify_messages, self.verify_nodes = set(), []
        self.components.setPlainText(f"Verifying {distro}: starting temporary test container…")

        def work(_):
            if distro not in self.manager.status().installed:
                raise ValueError(f"{distro} Studio image is not installed")
            with socket.socket() as probe:
                probe.bind(("127.0.0.1", 0))
                port = probe.getsockname()[1]
            identifier = self.manager.start(
                distro, "setup-" + uuid.uuid4().hex, RuntimeConfig(port=port), integration=True
            )
            return identifier, port

        def connected(result):
            self.verify_container, port = result
            transport = RosbridgeTransport(f"ws://127.0.0.1:{port}", self)
            self.verify_transport = transport
            transport.subscribe("/studio/sine", "std_msgs/msg/Float64")
            transport.subscribe("/joint_states", "sensor_msgs/msg/JointState")
            transport.message.connect(self.verification_message)
            transport.graph.connect(self.verification_graph)
            transport.status.connect(self.components.appendPlainText)
            transport.error.connect(self.components.appendPlainText)
            transport.connect_transport()
            self.verify_deadline.start(45000)

        self.runner.run(work, connected, lambda error: self.finish_verification("FAILED: " + error))

    def verification_message(self, topic, message):
        self.verify_messages.add(topic)
        self.check_verification_result()

    def verification_graph(self, graph):
        if "nodes" in graph:
            self.verify_nodes = graph["nodes"]
        self.check_verification_result()

    def check_verification_result(self):
        if (
            self.verifying
            and {"/studio/sine", "/joint_states"} <= self.verify_messages
            and any("studio_demo_subscriber" in node for node in self.verify_nodes)
        ):
            self.finish_verification(
                f"PASS: {self.verify_distro.currentData()} publishers, subscriber node, adapter, graph discovery and both real ROS topics reached the GUI."
            )

    def finish_verification(self, result):
        self.verify_deadline.stop()
        if self.verify_transport:
            self.verify_transport.disconnect_transport()
            self.verify_transport.deleteLater()
            self.verify_transport = None
        self.report_lines.append(result)
        self.components.appendPlainText(result)
        self.verifying = False
        identifier, self.verify_container = self.verify_container, None
        if identifier:
            self.runner.run(
                lambda _: self.manager.stop(identifier),
                lambda _: self.components.appendPlainText(
                    "Temporary test container stopped and removed."
                ),
                lambda error: self.failed(
                    f"Test container {identifier[:12]} could not be cleaned up: {error}"
                ),
            )
        self.busy_changed(self.runner.busy)

    def page_changed(self, identifier):
        if identifier == 4:
            self.summary.setPlainText(
                "\n\n".join(self.report_lines)
                + "\n\nKnown installed Studio images: "
                + (", ".join(self.local_installed) or "None reported; refresh status in step 4")
                + "\n\nDeferred runtimes remain available through Project & Runtime. Hardware mode, calibration and controller execution are planned."
            )

    def completed(self):
        self.settings.set("setup_complete", True)
        self.settings.set("setup_report", self.report_lines)
        if self.demo.isChecked():
            self.verify_callback()

    def reject(self):
        if self.runner.busy or self.process or self.verifying:
            return
        super().reject()

    def closeEvent(self, event):
        if self.runner.busy or self.process or self.verifying:
            event.ignore()
        else:
            super().closeEvent(event)
