"""Desktop composition and application state transitions."""

from __future__ import annotations

import logging
import time
from datetime import datetime
from pathlib import Path

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QAction, QKeySequence
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QMainWindow,
    QMessageBox,
    QProgressDialog,
    QSpinBox,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
)

from ros_robot_studio.compatibility.checker import check
from ros_robot_studio.core.models import Project, Robot
from ros_robot_studio.core.settings import Settings, xdg_path
from ros_robot_studio.gui.motion_workspace import MotionWorkspace
from ros_robot_studio.gui.panels import MonitoringPanel, RobotPanel, RuntimePanel, SensorPanel
from ros_robot_studio.gui.sensor_dialog import SensorDialog
from ros_robot_studio.gui.theme import DARK, LIGHT
from ros_robot_studio.gui.widgets import label
from ros_robot_studio.gui.wizard import SetupWizard
from ros_robot_studio.gui.workers import TaskRunner
from ros_robot_studio.plugins.registry import PluginRegistry
from ros_robot_studio.projects.store import (
    EXAMPLE_URDF,
    asset_path,
    atomic_text,
    demo_project,
    load_project,
    portable_copy,
    save_project,
)
from ros_robot_studio.robots.geometry import load_robot
from ros_robot_studio.robots.model_manager import ModelUnavailableError, RobotModelManager
from ros_robot_studio.robots.urdf import parse_urdf
from ros_robot_studio.ros_bridge.transport import OfflineTransport
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport
from ros_robot_studio.runtime.manager import RuntimeManager, RuntimeStatus

log = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    def __init__(
        self, settings: Settings | None = None, manager=None, show_wizard=True, model_manager=None
    ):
        super().__init__()
        self.settings = settings or Settings()
        self.manager = manager or RuntimeManager()
        self.registry = PluginRegistry()
        self.model_manager = model_manager or RobotModelManager()
        self.model_progress = None
        self.initial_model_check_pending = True
        self.project = demo_project()
        self.project_path: Path | None = None
        self.model = parse_urdf(EXAMPLE_URDF)
        self.dirty = False
        self.loading = False
        self.container_id = None
        self.transport = None
        self.selected_topic = ""
        self.ros_status = "Offline demo"
        self.mode = "offline"
        self.docker_status = RuntimeStatus(False, "Checking Docker…")
        self.wizard = None
        self.runner = TaskRunner(self)
        self.runner.failed.connect(self.show_error)
        self.runner.progress.connect(self.activity)
        self.runner.busy_changed.connect(self.busy_changed)
        self.setWindowTitle("ROS Robot Studio")
        self.resize(1480, 1020)
        self.setMinimumSize(1050, 760)
        self.setUnifiedTitleAndToolBarOnMac(True)
        self.tabs = QTabWidget()
        self.robot_panel = RobotPanel(self.registry, models=self.model_manager)
        self.sensor_panel = SensorPanel()
        self.monitor = MonitoringPanel()
        self.runtime = RuntimePanel()
        for widget, name in [
            (self.robot_panel, "01  Robot Configuration"),
            (self.sensor_panel, "02  Sensor Configuration"),
            (self.monitor, "03  Live Monitoring"),
            (self.runtime, "04  Project & Runtime"),
        ]:
            self.tabs.addTab(widget, name)
        self.setCentralWidget(self.tabs)
        self.build_toolbar()
        self.connect_panels()
        self.status = label("")
        self.statusBar().addWidget(self.status, 1)
        self.mode_status = label("OFFLINE / SIMULATED", "badge")
        self.statusBar().addPermanentWidget(self.mode_status)
        self.apply_theme()
        self.motion = MotionWorkspace(self)
        self.apply_project()
        self.start_offline()
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.update_live)
        self.timer.start()
        self.health_timer = QTimer(self)
        self.health_timer.setInterval(10000)
        self.health_timer.timeout.connect(self.check_health)
        self.health_timer.start()
        QTimer.singleShot(0, self.refresh_runtime)
        if show_wizard and not self.settings.get("setup_complete", False):
            QTimer.singleShot(250, self.open_wizard)

    def build_toolbar(self):
        toolbar = QToolBar("Workspace")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        brand = label("◈  ROS ROBOT STUDIO   ", "section")
        toolbar.addWidget(brand)
        file_menu = self.menuBar().addMenu("Project")
        self.file_actions = []
        for name, callback, shortcut in [
            ("New project", self.new_project, QKeySequence.StandardKey.New),
            ("Open…", self.open_project_dialog, QKeySequence.StandardKey.Open),
            ("Save", self.save, QKeySequence.StandardKey.Save),
        ]:
            action = QAction(name, self)
            action.triggered.connect(callback)
            action.setShortcut(QKeySequence(shortcut))
            toolbar.addAction(action)
            file_menu.addAction(action)
            self.file_actions.append(action)
        save_as = QAction("Save as…", self)
        save_as.triggered.connect(lambda: self.save(save_as=True))
        file_menu.addAction(save_as)
        self.file_actions.append(save_as)
        toolbar.addSeparator()
        toolbar.addWidget(label("  ROS runtime  ", "subtitle"))
        self.distro = QComboBox()
        for identifier, spec in self.manager.specs.items():
            self.distro.addItem(spec.name, identifier)
        self.distro.setToolTip(
            "Session runtime. Switching does not overwrite the project’s saved default."
        )
        self.distro.currentIndexChanged.connect(self.switch_distro)
        toolbar.addWidget(self.distro)
        self.start_action = toolbar.addAction("▶  Start ROS demo", self.start_ros)
        self.stop_action = toolbar.addAction("■  Stop", self.stop_ros)
        self.offline_action = toolbar.addAction("Offline demo", self.request_offline)
        toolbar.addSeparator()
        toolbar.addAction("Settings", self.open_settings)
        help_menu = self.menuBar().addMenu("Help")
        help_menu.addAction("Setup assistant", self.open_wizard)
        help_menu.addAction("Open demo project", self.open_demo)
        help_menu.addAction("Open Pick-and-Place demo", self.open_pick_place_demo)
        file_menu.addAction("Open Pick-and-Place demo", self.open_pick_place_demo)
        help_menu.addAction("About", self.about)

    def connect_panels(self):
        self.robot_panel.changed.connect(self.mark_dirty)
        self.robot_panel.import_requested.connect(self.import_urdf)
        self.robot_panel.example_requested.connect(self.load_example)
        self.robot_panel.model_selected.connect(self.select_robot_model)
        self.robot_panel.install_requested.connect(self.install_robot_model)
        self.robot_panel.save_pose_requested.connect(self.save)
        self.sensor_panel.add_requested.connect(self.add_sensor)
        self.sensor_panel.edit_requested.connect(self.edit_sensor)
        self.sensor_panel.remove_requested.connect(self.remove_sensor)
        self.sensor_panel.monitor_requested.connect(self.monitor_sensor)
        self.monitor.changed.connect(self.mark_dirty)
        self.monitor.topic_selected.connect(self.subscribe_topic)
        self.monitor.refresh_requested.connect(
            lambda: self.transport.refresh_graph() if self.transport else None
        )
        for widget, callback in [
            (self.runtime.start, self.start_ros),
            (self.runtime.stop, self.stop_ros),
            (self.runtime.restart, self.restart_ros),
            (self.runtime.refresh, self.refresh_runtime),
            (self.runtime.logs, self.fetch_logs),
            (self.runtime.verify, self.verify_demo),
            (self.runtime.install, self.install_runtime),
            (self.runtime.remove, self.remove_runtime),
            (self.runtime.setup, self.open_wizard),
            (self.runtime.compatibility, self.show_compatibility),
            (self.runtime.export, self.export_project),
            (self.runtime.set_default, self.set_default),
        ]:
            widget.clicked.connect(callback)

    @property
    def selected_distro(self):
        return self.distro.currentData()

    def apply_theme(self):
        QApplication.instance().setStyleSheet(
            LIGHT if self.settings.get("theme") == "light" else DARK
        )

    def mark_dirty(self):
        if not self.loading:
            self.dirty = True
            self.update_title()

    def update_title(self):
        self.setWindowTitle(f"{self.project.name}{' *' if self.dirty else ''} — ROS Robot Studio")
        self.status.setText(
            f"{self.project.name}   /   {self.selected_distro.title()}   /   Docker: {'ready' if self.docker_status.available else 'unavailable'}   /   {self.ros_status}"
        )
        self.runtime.project_info.setText(
            f"{self.project.name}\n\n{self.project_path or 'Not saved yet'}\n\nProject default: {self.project.default_ros.title()}\nSession: {self.selected_distro.title()}\n{len(self.project.sensors)} sensors · format v{self.project.format_version}"
        )

    def collect(self):
        result = self.project.model_copy(deep=True)
        self.robot_panel.collect(result)
        self.motion.collect(result)
        result.monitoring.window_seconds = self.monitor.window.value()
        result.monitoring.topic = self.monitor.topic or result.monitoring.topic
        result.monitoring.fields = self.monitor.selected_fields() or self.monitor.restore_fields
        return Project.model_validate(result.model_dump())

    def apply_project(self):
        self.loading = True
        if self.mode == "offline":
            self.distro.blockSignals(True)
            self.distro.setCurrentIndex(self.distro.findData(self.project.default_ros))
            self.distro.blockSignals(False)
        self.motion.prepare_project_reload()
        self.robot_panel.set_project(self.project, self.model)
        self.sensor_panel.set_sensors(self.project.sensors)
        self.monitor.restore_monitoring(self.project.monitoring)
        if hasattr(self, "timer"):
            self.timer.setInterval(round(1000 / self.project.monitoring.update_hz))
        self.robot_panel.update_compatibility(self.selected_distro)
        self.loading = False
        self.dirty = False
        self.update_title()

    def confirm_discard(self):
        if self.runner.busy or self.motion.runner.busy:
            self.statusBar().showMessage("Wait for the current operation to finish.", 4000)
            return False
        if self.dirty:
            return (
                QMessageBox.question(
                    self,
                    "Unsaved changes",
                    "Discard unsaved changes?",
                    QMessageBox.StandardButton.Discard | QMessageBox.StandardButton.Cancel,
                )
                == QMessageBox.StandardButton.Discard
            )
        return True

    def new_project(self):
        if not self.confirm_discard():
            return
        self.project = Project()
        self.project_path = None
        self.model = parse_urdf(EXAMPLE_URDF)
        self.apply_project()
        self.tabs.setCurrentIndex(0)
        self.mark_dirty()

    def open_demo(self):
        if not self.confirm_discard():
            return
        self.project, self.project_path = demo_project(), None
        self.model = parse_urdf(EXAMPLE_URDF)
        self.apply_project()
        self.request_offline()

    def open_pick_place_demo(self, checked=False, prefer_so101=True):
        if not self.confirm_discard():
            return
        from ros_robot_studio.kinematics.solvers import KinematicsSolverManager
        from ros_robot_studio.motion.demos import pick_place_demo

        if not KinematicsSolverManager().catalog()[0].installed:
            self.motion.install_support()
            return
        if prefer_so101 and self.model_manager.status("so101_follower").state != "installed":
            answer = QMessageBox.question(
                self,
                "Pick-and-Place demo",
                "Install the original SO-101 model first? Choose No to open the working generic offline demo.",
            )
            if answer == QMessageBox.StandardButton.Yes:
                self.install_robot_model("so101_follower", after_demo=True)
                return

        def done(result):
            self.project, self.model, _ = result
            self.project_path = None
            self.apply_project()
            self.request_offline()
            self.robot_panel.detail_tabs.setCurrentWidget(self.motion.program_panel)
            self.motion.program_tabs.setCurrentIndex(1)

        self.runner.run(
            lambda progress: pick_place_demo(self.model_manager, prefer_so101, progress), done
        )

    def save(self, checked=False, save_as=False):
        if self.runner.busy:
            return
        try:
            project = self.collect()
        except ValueError as exc:
            self.show_error(str(exc))
            return
        path = self.project_path
        if path is None or save_as:
            filename, _ = QFileDialog.getSaveFileName(
                self,
                "Save robot project",
                str(path or Path.home() / "robot.studio.yaml"),
                "Studio project (*.yaml *.yml)",
            )
            if not filename:
                return
            path = Path(filename)
        old_path = self.project_path

        def work(_):
            result = portable_copy(project, old_path, path, models=self.model_manager)
            save_project(result, path)
            return result

        def done(result):
            self.project, self.project_path = result, path
            self.dirty = False
            known = list(self.settings.get("known_projects", []))
            if str(path) not in known:
                known.append(str(path))
            self.settings.set("known_projects", known)
            self.settings.set("last_project", str(path))
            self.update_title()
            self.activity(f"Project saved: {path}")

        self.runner.run(work, done)

    def open_project_dialog(self):
        if not self.confirm_discard():
            return
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Open robot project",
            self.settings.get("last_project", str(Path.home())),
            "Studio project (*.yaml *.yml)",
        )
        if filename:
            self.open_path(Path(filename), confirmed=True)

    def open_path(self, path, confirmed=False):
        if not confirmed and not self.confirm_discard():
            return

        def work(_):
            project = load_project(path)
            if project.robot.model_id:
                spec = self.model_manager.spec(
                    project.robot.model_id, project.robot.source_revision
                )
                if (
                    project.robot.model_version != spec.model_version
                    or project.robot.urdf != spec.reference
                ):
                    raise ValueError(
                        "Project model version/revision/reference does not match the reviewed catalog"
                    )
                try:
                    source = self.model_manager.resolve(spec.reference)
                except ModelUnavailableError as exc:
                    return project, None, str(exc)
                model = load_robot(source, require_meshes=True)
            else:
                model = load_robot(asset_path(project.robot.urdf, path))
            model.configuration(project.robot.joint_positions)
            if project.robot.end_effector and project.robot.end_effector not in model.links:
                raise ValueError("Saved end-effector frame is not present in the robot model")
            return project, model, None

        def done(result):
            project, model, missing = result
            if missing:
                answer = QMessageBox.question(
                    self,
                    "Install missing robot model",
                    missing
                    + "\n\nInstall/repair the exact pinned model and then open this project?",
                )
                if answer == QMessageBox.StandardButton.Yes:
                    self.install_robot_model(
                        project.robot.model_id,
                        project_to_open=project,
                        project_path=path,
                        confirmed=True,
                    )
                return
            self._open_loaded_project(project, model, path)

        self.runner.run(work, done)

    def _open_loaded_project(self, project, model, path):
        self.project, self.model, self.project_path = project, model, path
        known = list(self.settings.get("known_projects", []))
        if str(path) not in known:
            known.append(str(path))
        self.settings.set("known_projects", known)
        self.settings.set("last_project", str(path))
        self.apply_project()
        self.activity(f"Project opened: {path}; robot pose restored without hardware access")

    def export_project(self):
        if self.runner.busy:
            return
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Export portable project",
            str(Path.home() / "export.studio.yaml"),
            "Studio project (*.yaml)",
        )
        if not filename:
            return
        try:
            project = self.collect()
        except ValueError as exc:
            self.show_error(str(exc))
            return
        destination = Path(filename)
        source_path = self.project_path

        def work(_):
            result = portable_copy(project, source_path, destination, models=self.model_manager)
            save_project(result, destination)

        self.runner.run(work, lambda _: self.activity(f"Project and URDF exported: {destination}"))

    def import_urdf(self):
        if self.runner.busy:
            return
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "Import URDF or single-file Xacro",
            str(Path.home()),
            "Robot descriptions (*.urdf *.xacro *.xml)",
        )
        if not filename:
            return
        path = Path(filename)
        xacro = path.suffix == ".xacro"
        if xacro and not self.container_id:
            self.show_error(
                "Xacro processing requires a running Studio ROS container. Only single-file Xacro is supported; external includes must already exist in the container."
            )
            return
        container_id = self.container_id

        def work(_):
            source = path
            if xacro:
                text = self.manager.expand_xacro(container_id, path)
                model = parse_urdf(text)
                import hashlib

                source = (
                    xdg_path("cache")
                    / "expanded"
                    / (hashlib.sha256(text.encode()).hexdigest() + ".urdf")
                )
                atomic_text(source, text)
            else:
                model = load_robot(path)
            return model, source

        def done(result):
            self.model, source = result
            self.project.robot = Robot(
                profile="custom-urdf",
                urdf=str(source.resolve()),
                source_xacro=str(path.resolve()) if xacro else None,
            )
            self.robot_panel.activate_robot(self.project.robot, self.model)
            self.mark_dirty()
            self.activity(
                f"Imported {self.model.name}: {len(self.model.links)} links, {len(self.model.joints)} joints"
            )

        self.runner.run(work, done)

    def load_example(self):
        self.project.robot = Robot()
        self.model = parse_urdf(EXAMPLE_URDF)
        self.robot_panel.activate_robot(self.project.robot, self.model)
        self.mark_dirty()

    def refresh_model_catalog(self):
        if self.runner.busy:
            QTimer.singleShot(100, self.refresh_model_catalog)
            return
        self.runner.run(
            lambda _: {
                identifier: self.model_manager.status(identifier)
                for identifier in self.model_manager.catalog
            },
            self.robot_panel.set_catalog_status,
        )

    def select_robot_model(self, identifier):
        if self.runner.busy:
            return
        if identifier == "generic-arm":
            self.load_example()
            return
        if identifier == "custom-urdf":
            self.import_urdf()
            if self.robot_panel.active_model_selection != "custom-urdf" and not self.runner.busy:
                self.robot_panel.loading = True
                self.robot_panel.profile.setCurrentIndex(
                    self.robot_panel.profile.findData(self.robot_panel.active_model_selection)
                )
                self.robot_panel.loading = False
            return
        if identifier not in self.model_manager.catalog:
            self.show_error("Unknown robot model: " + str(identifier))
            return

        def work(_):
            status = self.model_manager.status(identifier)
            return status, load_robot(
                status.directory / self.model_manager.spec(identifier).urdf, require_meshes=True
            ) if status.state == "installed" else None

        def done(result):
            status, model = result
            self.robot_panel.set_catalog_status({identifier: status})
            if model:
                self._activate_catalog_model(identifier, model)

        self.runner.run(work, done)

    def _activate_catalog_model(self, identifier, model):
        spec = self.model_manager.spec(identifier)
        self.model = model
        self.project.robot = Robot(
            profile=spec.plugin_id or "custom-urdf",
            model_id=identifier,
            model_version=spec.model_version,
            source_revision=spec.source_revision,
            urdf=spec.reference,
            end_effector=spec.default_end_effector,
            joint_positions=model.default_configuration(),
        )
        self.robot_panel.activate_robot(self.project.robot, model)
        self.mark_dirty()
        self.activity(
            f"Loaded {spec.name} · revision {spec.source_revision} · {len(model.visuals)} visuals · offline only"
        )

    def install_robot_model(
        self, identifier, project_to_open=None, project_path=None, confirmed=False, after_demo=False
    ):
        if self.runner.busy or identifier not in self.model_manager.catalog:
            return
        spec = self.model_manager.spec(
            identifier, project_to_open.robot.source_revision if project_to_open else None
        )
        if not confirmed:
            answer = QMessageBox.question(
                self,
                "Install Robot Model",
                f"Install/verify {spec.name}?\n\n{spec.download_bytes / 1e6:.2f} MB · original URDF + {sum(entry.path.lower().endswith('.stl') for entry in spec.files)} STL files\nSource: {spec.source_repository}\nRevision: {spec.source_revision}\nLicense: {spec.license}\n\nFiles stay in the application model cache. No ROS, Docker, LeRobot or hardware installation is required.",
            )
            if answer != QMessageBox.StandardButton.Yes:
                return
        self.model_progress = QProgressDialog("Preparing original robot model…", "", 0, 0, self)
        self.model_progress.setWindowTitle("Robot Model Manager")
        self.model_progress.setWindowModality(Qt.WindowModality.ApplicationModal)
        self.model_progress.setCancelButton(None)
        self.model_progress.setMinimumDuration(0)
        self.model_progress.setAutoClose(False)
        self.model_progress.setAutoReset(False)
        self.model_progress.show()

        def progress(message):
            if self.model_progress:
                self.model_progress.setLabelText(message)
                percentage = message.split("%", 1)[0]
                if percentage.isdigit():
                    self.model_progress.setRange(0, 100)
                    self.model_progress.setValue(int(percentage))

        self.runner.progress.connect(progress)

        def close_progress():
            self.runner.progress.disconnect(progress)
            if self.model_progress:
                self.model_progress.close()
                self.model_progress.deleteLater()
                self.model_progress = None

        def work(update):
            source = self.model_manager.install(identifier, update, spec.source_revision)
            model = load_robot(source, require_meshes=True)
            if project_to_open:
                model.configuration(project_to_open.robot.joint_positions)
            return model, self.model_manager.status(identifier)

        def done(result):
            close_progress()
            model, status = result
            self.robot_panel.set_catalog_status({identifier: status})
            if after_demo:
                from ros_robot_studio.motion.demos import pick_place_demo

                def opened(result):
                    self.project, self.model, _ = result
                    self.project_path = None
                    self.apply_project()
                    self.robot_panel.detail_tabs.setCurrentWidget(self.motion.program_panel)
                    self.motion.program_tabs.setCurrentIndex(1)

                self.runner.run(
                    lambda progress: pick_place_demo(self.model_manager, True, progress), opened
                )
            elif project_to_open:
                self._open_loaded_project(project_to_open, model, project_path)
            else:
                self._activate_catalog_model(identifier, model)

        def failed(message):
            close_progress()
            self.show_error(message)

        self.runner.run(work, done, failed)

    def add_sensor(self):
        dialog = SensorDialog(frames=self.model.links, parent=self)
        if dialog.exec():
            self.project.sensors.append(dialog.result_sensor())
            self.sensor_panel.set_sensors(self.project.sensors)
            self.mark_dirty()

    def edit_sensor(self, row):
        if not 0 <= row < len(self.project.sensors):
            return
        dialog = SensorDialog(self.project.sensors[row], self.model.links, self)
        if dialog.exec():
            self.project.sensors[row] = dialog.result_sensor()
            self.sensor_panel.set_sensors(self.project.sensors)
            self.mark_dirty()

    def remove_sensor(self, row):
        if (
            0 <= row < len(self.project.sensors)
            and QMessageBox.question(
                self, "Remove sensor", f"Remove {self.project.sensors[row].name} from this project?"
            )
            == QMessageBox.StandardButton.Yes
        ):
            self.project.sensors.pop(row)
            self.sensor_panel.set_sensors(self.project.sensors)
            self.mark_dirty()

    def monitor_sensor(self, topic):
        self.tabs.setCurrentIndex(2)
        if not self.monitor.choose_topic(topic):
            QMessageBox.information(
                self,
                "Topic unavailable",
                f"{topic} is not present in the current transport graph. Start its publisher or refresh the graph.",
            )

    def detach_transport(self):
        if self.transport:
            old, self.transport = self.transport, None
            old.status.disconnect()
            old.graph.disconnect()
            old.message.disconnect()
            old.error.disconnect()
            old.disconnect_transport()
            old.deleteLater()
        self.selected_topic = ""
        self.monitor.clear_graph()

    def attach_transport(self, transport):
        was_loading = self.loading
        self.loading = True
        self.detach_transport()
        self.transport = transport
        transport.status.connect(self.connection_status)
        transport.graph.connect(self.monitor.set_graph)
        transport.message.connect(self.receive_message)
        transport.error.connect(self.transport_error)
        transport.subscribe("/joint_states", "sensor_msgs/msg/JointState")
        transport.connect_transport()
        self.monitor.choose_topic(self.project.monitoring.topic)
        self.loading = was_loading

    def start_offline(self):
        self.mode = "offline"
        self.monitor.mode.setText(
            "OFFLINE DEMO · All values below are simulated. No hardware is connected."
        )
        self.mode_status.setText("OFFLINE / SIMULATED")
        self.attach_transport(OfflineTransport(self))
        self.monitor.container.setText("Container: none")
        self.update_title()

    def request_offline(self):
        if self.runner.busy:
            return
        if self.container_id:
            self.stop_ros(after=self.start_offline)
        else:
            self.start_offline()

    def subscribe_topic(self, topic, message_type):
        if not self.transport:
            return
        if (
            self.selected_topic
            and self.selected_topic != "/joint_states"
            and self.selected_topic != topic
        ):
            self.transport.unsubscribe(self.selected_topic)
        self.selected_topic = topic
        self.transport.subscribe(topic, message_type)
        self.transport.topic_info(topic)

    def receive_message(self, topic, message):
        self.monitor.receive(topic, message)
        if topic == "/joint_states":
            names, positions = message.get("name", []), message.get("position", [])
            try:
                import math

                values = {
                    str(n): float(v)
                    for n, v in zip(names[:128], positions[:128])
                    if math.isfinite(float(v))
                }
                self.robot_panel.apply_live_joints(values)
            except (ValueError, TypeError):
                pass
        if getattr(self, "verification_started", None) and self.mode == "ros":
            self.verification_messages.add(topic)
            if {"/studio/sine", "/joint_states"} <= self.verification_messages:
                self.verification_started = None
                self.activity(
                    f"PASS: {self.selected_distro} end-to-end GUI received /studio/sine and /joint_states from the ROS container."
                )
                QMessageBox.information(
                    self,
                    "ROS demo verified",
                    "Both real ROS demo topics reached the GUI. Node discovery: "
                    + str(self.monitor.nodes.count())
                    + " nodes. Subscriber activity can be inspected in the container logs.",
                )

    def update_live(self):
        self.monitor.render()
        if (
            getattr(self, "verification_started", None)
            and time.monotonic() - self.verification_started > 20
        ):
            self.verification_started = None
            self.transport_error(
                "End-to-end verification timed out: both demo topics were not received within 20 seconds."
            )

    def connection_status(self, status):
        self.ros_status = status
        self.monitor.connection.setText(status)
        self.update_title()

    def transport_error(self, error):
        self.monitor.errors.setText(error[:220])
        self.activity("ROS: " + error)

    def activity(self, text):
        self.runtime.output.appendPlainText(f"{datetime.now().strftime('%H:%M:%S')}  {text}")
        self.runtime.progress.setText(text[-200:])
        log.info("%s", text)

    def show_error(self, message):
        self.activity("ERROR: " + message)
        QMessageBox.warning(self, "Operation failed", message[:4000])

    def busy_changed(self, busy):
        self.distro.setEnabled(not busy)
        for action in self.file_actions:
            action.setEnabled(not busy)
        self.monitor.setEnabled(not busy)
        self.robot_panel.setEnabled(not busy)
        self.sensor_panel.setEnabled(not busy)
        self.start_action.setEnabled(
            not busy and not self.container_id and self.docker_status.available
        )
        self.stop_action.setEnabled(not busy and bool(self.container_id))
        self.offline_action.setEnabled(not busy)
        for control in (
            self.runtime.start,
            self.runtime.stop,
            self.runtime.restart,
            self.runtime.refresh,
            self.runtime.logs,
            self.runtime.install,
            self.runtime.remove,
            self.runtime.export,
            self.runtime.set_default,
            self.runtime.setup,
            self.runtime.verify,
        ):
            control.setEnabled(not busy)
        self.runtime.start.setEnabled(
            not busy and not self.container_id and self.docker_status.available
        )
        self.runtime.stop.setEnabled(not busy and bool(self.container_id))
        self.runtime.restart.setEnabled(not busy and bool(self.container_id))
        self.runtime.logs.setEnabled(not busy and bool(self.container_id))
        self.runtime.verify.setEnabled(not busy and bool(self.container_id))
        self.runtime.install.setEnabled(not busy and self.docker_status.available)
        self.runtime.remove.setEnabled(not busy and self.docker_status.available)

    def refresh_runtime(self):
        if self.runner.busy:
            return

        def done(status):
            self.docker_status = status
            self.runtime.docker_status.setText(
                "● Docker ready"
                if status.available
                else "○ Docker unavailable · Offline demo is ready"
            )
            self.runtime.docker_status.setToolTip(status.message)
            self.runtime.table.setRowCount(len(self.manager.specs))
            from PySide6.QtWidgets import QTableWidgetItem

            for row, (identifier, spec) in enumerate(self.manager.specs.items()):
                values = [
                    spec.name,
                    spec.ubuntu,
                    spec.base_image,
                    f"Until {spec.eol}" if spec.supported else "EOL",
                    "Installed · Studio image"
                    if identifier in status.installed
                    else "Available · not installed"
                    if spec.supported
                    else "EOL · disabled",
                ]
                for column, value in enumerate(values):
                    self.runtime.table.setItem(row, column, QTableWidgetItem(value))
            if not status.available:
                self.activity(status.message)
            self.busy_changed(self.runner.busy)
            self.update_title()
            if self.initial_model_check_pending:
                self.initial_model_check_pending = False
                self.refresh_model_catalog()

        self.runner.run(lambda _: self.manager.status(), done)

    def install_runtime(self):
        if self.runner.busy:
            return
        distro = self.selected_distro
        self.tabs.setCurrentIndex(3)
        self.runner.run(
            lambda progress: self.manager.install(distro, progress),
            lambda _: QTimer.singleShot(0, self.refresh_runtime),
        )

    def start_ros(self, checked=False):
        if self.runner.busy or self.container_id:
            return
        distro = self.selected_distro
        if distro not in self.docker_status.installed:
            self.show_error(
                f"{distro.title()} Studio runtime is not installed. Open Project & Runtime and install it first."
            )
            return
        try:
            project = self.collect()
        except ValueError as exc:
            self.show_error(str(exc))
            return
        self.activity(check(project, distro, self.registry).text())

        def done(identifier):
            self.container_id = identifier
            self.mode = "ros"
            self.monitor.mode.setText(
                f"ROS CONTAINER DEMO · Real ROS {distro.title()} messages from simulated publishers. No hardware is connected."
            )
            self.mode_status.setText("ROS CONTAINER / DEMO")
            self.attach_transport(
                RosbridgeTransport(f"ws://127.0.0.1:{project.runtime.port}", self)
            )
            self.runtime.container_status.setText(
                f"{distro.title()} · {identifier[:12]} · starting"
            )
            self.monitor.container.setText(f"Container: {identifier[:12]}\nHealthcheck starting")
            self.activity(f"Started owned container {identifier[:12]}")

        self.runner.run(lambda _: self.manager.start(distro, project.id, project.runtime), done)

    def stop_ros(self, checked=False, after=None):
        if self.runner.busy:
            return
        if not self.container_id:
            if after:
                after()
            return
        identifier = self.container_id
        self.detach_transport()
        self.verification_started = None
        self.connection_status("Stopping ROS container…")

        def done(_):
            self.container_id = None
            self.runtime.container_status.setText("No container started")
            self.monitor.container.setText("Container: none")
            self.activity(f"Stopped and removed own container {identifier[:12]}")
            if after:
                QTimer.singleShot(0, after)
            else:
                self.mode = "stopped"
                self.mode_status.setText("STOPPED")
                self.connection_status("Disconnected")

        def failed(message):
            self.show_error("Container stop failed; container retained for retry: " + message)

        self.runner.run(lambda _: self.manager.stop(identifier), done, failed)

    def restart_ros(self):
        self.stop_ros(after=self.start_ros)

    def switch_distro(self, *_):
        if not hasattr(self, "monitor") or self.loading:
            return
        try:
            report = check(self.collect(), self.selected_distro, self.registry)
        except ValueError as exc:
            self.show_error(str(exc))
            return
        self.activity(report.text())
        self.robot_panel.update_compatibility(self.selected_distro)
        if self.container_id:
            self.stop_ros(after=self.start_ros)
        elif self.mode == "offline":
            self.start_offline()
        self.update_title()

    def set_default(self):
        self.project.default_ros = self.selected_distro
        self.mark_dirty()
        self.update_title()

    def show_compatibility(self):
        try:
            report = check(self.collect(), self.selected_distro, self.registry)
            QMessageBox.information(self, "ROS compatibility", report.text())
        except ValueError as exc:
            self.show_error(str(exc))

    def fetch_logs(self):
        if self.container_id and not self.runner.busy:
            identifier = self.container_id
            self.runner.run(lambda _: self.manager.logs(identifier), self.activity)

    def check_health(self):
        if self.container_id and not self.runner.busy:
            identifier = self.container_id

            def done(result):
                text = f"{identifier[:12]} · {result['status']} · health: {result['health']}"
                self.runtime.container_status.setText(text)
                self.monitor.container.setText(text)
                if result["status"] != "running":
                    self.transport_error(
                        "ROS container is not running. Fetch logs or stop it before restarting."
                    )

            self.runner.run(lambda _: self.manager.health(identifier), done, self.transport_error)

    def verify_demo(self):
        if not self.container_id or not self.transport:
            return
        self.verification_started = time.monotonic()
        self.verification_messages = set()
        self.transport.subscribe("/studio/sine", "std_msgs/msg/Float64")
        self.transport.subscribe("/joint_states", "sensor_msgs/msg/JointState")
        self.transport.refresh_graph()
        self.monitor.choose_topic("/studio/sine")
        self.activity("End-to-end verification running (20 second timeout)…")

    def remove_runtime(self):
        if self.runner.busy:
            return
        distro = self.selected_distro
        answer = QMessageBox.question(
            self,
            "Uninstall runtime",
            f"Remove only the Studio {distro.title()} image? Known project references are checked. Projects outside Studio’s known-project list cannot be discovered. Base images are retained.",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        known = list(self.settings.get("known_projects", []))
        current = self.project.default_ros

        def work(_):
            references = {current}
            for filename in known:
                path = Path(filename)
                if path.exists():
                    references.add(load_project(path).default_ros)
            self.manager.remove_image(distro, references)

        self.runner.run(work, lambda _: QTimer.singleShot(0, self.refresh_runtime))

    def open_wizard(self):
        if self.runner.busy:
            QTimer.singleShot(300, self.open_wizard)
            return
        if self.wizard:
            self.wizard.raise_()
            return
        self.wizard = SetupWizard(self.manager, self.settings, self.open_demo, self)
        self.wizard.setWindowModality(Qt.WindowModality.ApplicationModal)
        self.wizard.finished.connect(self.wizard_finished)
        self.wizard.show()

    def wizard_finished(self, result):
        self.wizard.deleteLater()
        self.wizard = None
        QTimer.singleShot(0, self.refresh_runtime)

    def open_settings(self):
        if self.runner.busy:
            return
        dialog = QDialog(self)
        dialog.setWindowTitle("Studio settings")
        layout = QVBoxLayout(dialog)
        form = QFormLayout()
        theme = QComboBox()
        theme.addItems(["dark", "light"])
        theme.setCurrentText(self.settings.get("theme", "dark"))
        port, memory, hz = QSpinBox(), QSpinBox(), QSpinBox()
        port.setRange(1024, 65535)
        port.setValue(self.project.runtime.port)
        memory.setRange(512, 32768)
        memory.setSingleStep(256)
        memory.setValue(self.project.runtime.memory_mb)
        hz.setRange(1, 30)
        hz.setValue(self.project.monitoring.update_hz)
        for name, widget in [
            ("Theme", theme),
            ("Local ROS port", port),
            ("Container memory · MB", memory),
            ("GUI plot updates · Hz", hz),
        ]:
            form.addRow(name, widget)
        layout.addLayout(form)
        layout.addWidget(
            label(
                "Runtime settings apply to the next container. Theme is stored in XDG user settings; runtime and plot options are saved with the project.",
                "subtitle",
                True,
            )
        )
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addWidget(buttons)
        if dialog.exec():
            self.settings.set("theme", theme.currentText())
            self.project.runtime.port = port.value()
            self.project.runtime.memory_mb = memory.value()
            self.project.monitoring.update_hz = hz.value()
            self.timer.setInterval(round(1000 / hz.value()))
            self.apply_theme()
            self.mark_dirty()

    def about(self):
        QMessageBox.information(
            self,
            "About ROS Robot Studio",
            "ROS Robot Studio 0.1.0\nPython / PySide6 / PyQtGraph / Docker SDK\nApache-2.0 · Source and documentation in the project directory.\n\nOffline demo: simulated data. Container demo: real ROS nodes with simulated measurements. STL mesh viewing and virtual joints are available. Hardware control, calibration and Isaac Sim are planned. Sensor thresholds are not certified safety functions.",
        )

    def closeEvent(self, event):
        if self.motion.runner.busy or (self.motion.wizard and self.motion.wizard.runner.busy):
            self.motion.cancel_work()
            if self.motion.wizard:
                self.motion.wizard.cancel_event.set()
            QTimer.singleShot(100, self.close)
            event.ignore()
            return
        if self.runner.busy:
            self.statusBar().showMessage("Wait for the background operation before closing.", 5000)
            event.ignore()
            return
        if not self.confirm_discard():
            event.ignore()
            return
        if self.container_id:
            event.ignore()
            # Stop asynchronously, then close after the worker has finished.
            self.dirty = False
            self.stop_ros(after=self.close)
            return
        self.motion.shutdown()
        self.timer.stop()
        self.health_timer.stop()
        self.detach_transport()
        if self.monitor.plot.plotItem is not None:
            self.monitor.plot.close()
        event.accept()
