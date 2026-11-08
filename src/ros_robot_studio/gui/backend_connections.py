"""Universal backend profiles and live connection inspection in the existing desktop."""

from pathlib import Path

import yaml
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.backends.domains import MAX_DOMAIN
from ros_robot_studio.backends.isaac import IsaacSimInstallationManager, IsaacSimSceneManager
from ros_robot_studio.backends.profiles import DEFAULTS, BackendProfile, new_profile
from ros_robot_studio.gui.widgets import button, label
from ros_robot_studio.projects.store import atomic_text


def combo(values, current):
    widget = QComboBox()
    widget.addItems(values)
    widget.setCurrentText(current)
    return widget


class BackendProfileDialog(QDialog):
    def __init__(self, main, profile, parent=None, is_new=False):
        super().__init__(parent)
        self.main, self.original = main, profile
        self.setWindowTitle("ROS Backend Profile")
        self.resize(780, 820)
        outer = QVBoxLayout(self)
        tabs = QTabWidget()
        outer.addWidget(tabs)
        page = QScrollArea()
        page.setWidgetResizable(True)
        content = QWidget()
        page.setWidget(content)
        form = QFormLayout(content)
        tabs.addTab(page, "Connection")
        self.name = QLineEdit(profile.name)
        self.kind = combo(list(DEFAULTS), profile.backend_type)
        self.mode = combo(["local_installation", "existing", "remote"], profile.connection_mode)
        self.distribution = combo(["humble", "jazzy", "lyrical"], profile.distribution)
        self.domain = QSpinBox()
        self.domain.setRange(0, MAX_DOMAIN)
        self.domain.setValue(profile.domain_id)
        self.network = combo(["bridge", "host"], profile.network)
        self.discovery = combo(["subnet", "localhost"], profile.discovery)
        self.port = QSpinBox()
        self.port.setRange(0, 65535)
        self.port.setSpecialValueText("Automatic")
        self.port.setValue(profile.port)
        self.enabled = QCheckBox("Enable this profile")
        self.enabled.setChecked(profile.enabled)
        self.read_only = QCheckBox("Read only · no motion commands")
        self.read_only.setChecked(profile.read_only)
        self.execution = QCheckBox("Request simulation execution")
        self.execution.setChecked(profile.execution_enabled)
        self.confirm = QCheckBox("I confirm this endpoint is a simulation with no robot hardware")
        self.confirm.setChecked(profile.simulation_confirmed)
        self.host = QCheckBox("Allow host DDS networking for this connection")
        self.host.setChecked(profile.network_confirmed)
        self.shared = QCheckBox("I explicitly allow sharing a domain with another profile")
        self.shared.setChecked(profile.shared_domain_confirmed)
        for title, w in [
            ("Name", self.name),
            ("Backend type", self.kind),
            ("Connection mode", self.mode),
            ("ROS distribution", self.distribution),
            ("ROS_DOMAIN_ID", self.domain),
            ("Middleware", QLabel("Fast DDS · rmw_fastrtps_cpp")),
            ("Network profile", self.network),
            ("Discovery", self.discovery),
            ("Local bridge port", self.port),
            ("", self.enabled),
            ("", self.read_only),
            ("", self.execution),
            ("", self.confirm),
            ("", self.host),
            ("", self.shared),
        ]:
            form.addRow(title, w)
        form.addRow(
            label(
                "Domain IDs are editable routing settings. They never grant hardware execution permission.",
                "warning",
                True,
            )
        )
        local = main.settings.get("backend_local", {}).get(profile.id, {})
        self.installation = QLineEdit(local.get("installation_path", ""))
        self.scene = QLineEdit(local.get("scene_path", ""))
        self.scene_directory = QLineEdit(local.get("scene_directory", ""))
        self.auto_play = QCheckBox("Auto Play owned simulation (explicit opt-in)")
        self.auto_play.setChecked(local.get("auto_play", False))
        self.headless = QCheckBox("Run owned Isaac process headless")
        self.headless.setChecked(local.get("headless", False))
        for title, entry, select_file in [
            ("Local Isaac installation", self.installation, False),
            ("Existing USD scene (optional)", self.scene, True),
            ("Generated scene directory (optional)", self.scene_directory, False),
        ]:
            row = QHBoxLayout()
            row.addWidget(entry)
            row.addWidget(
                button("Browse…", lambda checked=False, e=entry, f=select_file: self.browse(e, f))
            )
            form.addRow(title, row)
        form.addRow(button("Detect Isaac Sim 5.1", self.detect))
        form.addRow(self.headless)
        form.addRow(self.auto_play)
        form.addRow(
            label(
                "Local paths are saved only in this computer’s Settings. Existing/remote simulations do not require a local installation.",
                "subtitle",
                True,
            )
        )
        interfaces = QWidget()
        body = QVBoxLayout(interfaces)
        settings = QFormLayout()
        body.addLayout(settings)
        self.interfaces = {}
        for key, title in [
            ("state_topic", "JointState feedback"),
            ("command_topic", "JointState command"),
            ("action", "FollowJointTrajectory action"),
            ("clock_topic", "Simulation clock"),
            ("tf_topic", "TF"),
        ]:
            entry = QLineEdit(getattr(profile, key))
            self.interfaces[key] = entry
            settings.addRow(title, entry)
        self.controller = combo(
            ["follow_joint_trajectory", "joint_state_command"], profile.controller_interface
        )
        self.clock = QCheckBox("Use simulation time")
        self.clock.setChecked(profile.use_sim_time)
        settings.addRow("Controller interface", self.controller)
        settings.addRow(self.clock)
        self.bind = QCheckBox("Bind this profile and mapping to the currently loaded robot")
        self.bind.setChecked(is_new or not profile.model_signature)
        body.addWidget(self.bind)
        body.addWidget(
            label(
                "Saved model: "
                + (profile.robot_model_id or "custom/generic")
                + " · "
                + profile.model_signature[:16],
                "subtitle",
                True,
            )
        )
        names = [j.name for j in main.model.joints if j.movable]
        self.mapping = QTableWidget(len(names), 2)
        self.mapping.setHorizontalHeaderLabels(["URDF joint", "ROS joint"])
        self.mapping.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for row, name in enumerate(names):
            item = QTableWidgetItem(name)
            item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            self.mapping.setItem(row, 0, item)
            self.mapping.setItem(row, 1, QTableWidgetItem(profile.joint_mapping.get(name, name)))
        body.addWidget(self.mapping, 1)
        tabs.addTab(interfaces, "Robot / interfaces")
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.validate)
        buttons.rejected.connect(self.reject)
        outer.addWidget(buttons)
        self.kind.currentTextChanged.connect(self.type_changed)
        self.type_changed(profile.backend_type, reset=False)

    def browse(self, entry, file):
        path = (
            QFileDialog.getOpenFileName(
                self, "Select USD scene", entry.text(), "USD (*.usd *.usda *.usdc)"
            )[0]
            if file
            else QFileDialog.getExistingDirectory(self, "Select local directory", entry.text())
        )
        if path:
            entry.setText(path)

    def detect(self):
        installations = IsaacSimInstallationManager().detect([self.installation.text()])
        if installations:
            self.installation.setText(str(installations[0].path))
            QMessageBox.information(self, "Isaac installation", installations[0].version)
        else:
            QMessageBox.information(
                self,
                "Isaac installation",
                "No compatible local Isaac 5.1 installation detected. Browse to an installation or use an existing/remote ROS simulation.",
            )

    def type_changed(self, kind, reset=True):
        hardware = kind == "real_robot"
        prepared = kind in ("mujoco", "gazebo")
        self.read_only.setEnabled(not hardware and not prepared)
        self.execution.setEnabled(not hardware and not prepared)
        if reset:
            defaults = new_profile(kind)
            self.name.setText(defaults.name)
            self.domain.setValue(defaults.domain_id)
            self.network.setCurrentText("bridge" if kind == "ros_demo" else "host")
            self.read_only.setChecked(defaults.read_only)
            self.execution.setChecked(defaults.execution_enabled)
            self.confirm.setChecked(defaults.simulation_confirmed)
            self.controller.setCurrentText(defaults.controller_interface)
            self.clock.setChecked(defaults.use_sim_time)
        if hardware or prepared:
            self.read_only.setChecked(True)
            self.execution.setChecked(False)

    def values(self):
        data = self.original.model_dump()
        data.update(
            name=self.name.text().strip(),
            backend_type=self.kind.currentText(),
            connection_mode=self.mode.currentText(),
            distribution=self.distribution.currentText(),
            domain_id=self.domain.value(),
            network=self.network.currentText(),
            discovery=self.discovery.currentText(),
            port=self.port.value(),
            enabled=self.enabled.isChecked(),
            read_only=self.read_only.isChecked(),
            execution_enabled=self.execution.isChecked(),
            simulation_confirmed=self.confirm.isChecked(),
            network_confirmed=self.host.isChecked(),
            shared_domain_confirmed=self.shared.isChecked(),
            controller_interface=self.controller.currentText(),
            use_sim_time=self.clock.isChecked(),
            joint_mapping={
                self.mapping.item(i, 0).text(): self.mapping.item(i, 1).text().strip()
                for i in range(self.mapping.rowCount())
            },
        )
        data.update({key: entry.text().strip() for key, entry in self.interfaces.items()})
        profile = BackendProfile(**data)
        if self.bind.isChecked():
            profile = self.main.backend_manager.bind_profile(profile)
        return profile

    def local_values(self):
        return dict(
            installation_path=self.installation.text().strip(),
            scene_path=self.scene.text().strip(),
            scene_directory=self.scene_directory.text().strip(),
            headless=self.headless.isChecked(),
            auto_play=self.auto_play.isChecked(),
        )

    def validate(self):
        try:
            self.values()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid backend profile", str(exc))
            return
        self.accept()


class BackendConnectionsDialog(QDialog):
    def __init__(self, main):
        super().__init__(main)
        self.main = main
        self.manager = main.backend_manager
        self.displayed_selection = None
        self.setWindowTitle("ROS Backend Connection Manager")
        self.resize(1120, 820)
        body = QVBoxLayout(self)
        body.addWidget(label("ROS backend connections", "section"))
        body.addWidget(
            label(
                "Independent domains and endpoints · one active robot state · hardware remains read only",
                "subtitle",
                True,
            )
        )
        self.table = QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(
            ["Backend", "Type", "Domain", "ROS", "Status", "Bridge endpoint", "Active"]
        )
        self.table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QTableWidget.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self.inspect)
        body.addWidget(self.table, 2)
        row = QHBoxLayout()
        for title, callback in [
            ("Add", self.add),
            ("Edit", self.edit),
            ("Duplicate", self.duplicate),
            ("Delete", self.remove),
            ("Import", self.import_profile),
            ("Export", self.export_profile),
        ]:
            row.addWidget(button(title, callback))
        body.addLayout(row)
        row = QHBoxLayout()
        self.connect_button = button("Connect", self.connect_selected, True)
        row.addWidget(self.connect_button)
        self.disconnect_button = button("Disconnect", self.disconnect_selected)
        row.addWidget(self.disconnect_button)
        self.activate_button = button("Set Active", self.activate_selected, True)
        row.addWidget(self.activate_button)
        row.addWidget(button("Offline demo", main.request_offline))
        row.addWidget(button("Run Backend Diagnostics", self.diagnostics))
        body.addLayout(row)
        row = QHBoxLayout()
        self.create_scene_button = button("Create / validate Isaac scene", self.create_scene)
        self.start_isaac_button = button("Start owned Isaac process", self.start_isaac)
        self.stop_isaac_button = button("Stop owned Isaac process", self.stop_isaac)
        for w in (self.create_scene_button, self.start_isaac_button, self.stop_isaac_button):
            row.addWidget(w)
        row.addWidget(button("Install selected ROS runtime", self.install_runtime))
        body.addLayout(row)
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        self.details.setMaximumBlockCount(500)
        self.detail_tabs = QTabWidget()
        self.diagnostic_output = QPlainTextEdit()
        self.process_output = QPlainTextEdit()
        for widget, title in (
            (self.details, "Live state"),
            (self.diagnostic_output, "Diagnostics"),
            (self.process_output, "Isaac process log"),
        ):
            widget.setReadOnly(True)
            widget.setMaximumBlockCount(1000)
            self.detail_tabs.addTab(widget, title)
        body.addWidget(self.detail_tabs, 2)
        body.addWidget(button("Close", self.close))
        self.manager.changed.connect(self.refresh)
        main.isaac_processes.log.connect(self.process_log)
        main.isaac_processes.changed.connect(self.refresh)
        self.refresh()

    def selected_id(self):
        row = self.table.currentRow()
        return (
            self.table.item(row, 0).data(Qt.ItemDataRole.UserRole)
            if row >= 0 and self.table.item(row, 0)
            else None
        )

    def refresh(self, *_):
        selected = self.selected_id()
        if self.manager.selected_id != self.displayed_selection:
            selected = self.manager.selected_id
            self.displayed_selection = selected
        self.table.blockSignals(True)
        self.table.setRowCount(len(self.manager.profiles))
        for row, profile in enumerate(self.manager.profiles.values()):
            session = self.manager.sessions.get(profile.id)
            status = (
                session.status
                if session
                else "Error"
                if profile.id in self.manager.last_errors
                else "Not Implemented"
                if self.manager.registry.get(profile.backend_type).prepared
                else "Disconnected"
                if profile.enabled
                else "Not Configured"
            )
            values = [
                profile.name,
                profile.backend_type,
                str(profile.domain_id),
                profile.distribution,
                status,
                f"127.0.0.1:{session.port}"
                if session
                else "Automatic"
                if not profile.port
                else str(profile.port),
                "Active" if self.manager.active_id == profile.id else "",
            ]
            for col, text in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(text))
            self.table.item(row, 0).setData(Qt.ItemDataRole.UserRole, profile.id)
            if profile.id == selected:
                self.table.selectRow(row)
        if not selected and self.table.rowCount():
            self.table.selectRow(0)
        self.table.blockSignals(False)
        self.inspect()

    def inspect(self):
        identity = self.selected_id()
        profile = self.manager.profiles.get(identity)
        if not profile:
            return
        session = self.manager.sessions.get(identity)
        self.connect_button.setEnabled(
            profile.enabled
            and not session
            and not self.manager.registry.get(profile.backend_type).prepared
        )
        self.disconnect_button.setEnabled(bool(session) and not session.cancelled)
        ready = bool(
            session
            and not session.cancelled
            and "joint_state_feedback" in session.capabilities.available
            and not session.requires_reconnect
        )
        self.activate_button.setEnabled(ready)
        isaac = (
            profile.backend_type == "isaac_sim" and profile.connection_mode == "local_installation"
        )
        self.create_scene_button.setEnabled(
            isaac and not self.main.isaac_processes.processes.get(identity)
        )
        self.start_isaac_button.setEnabled(
            isaac and not self.main.isaac_processes.processes.get(identity)
        )
        self.stop_isaac_button.setEnabled(identity in self.main.isaac_processes.processes)
        lines = [
            f"{profile.name} · {profile.id}",
            f"Domain {profile.domain_id} · {profile.distribution} · {profile.middleware}",
            f"Installation process: {self.main.isaac_processes.status(identity)}",
        ]
        lines += [
            r.severity + ": " + r.message
            for r in self.manager.domains.inspect(profile, self.manager.profiles.values())
        ]
        if session:
            lines += [
                session.status,
                "Capabilities: " + ", ".join(sorted(session.capabilities.available)),
                session.capabilities.reason,
            ]
            if session.error:
                lines.append(session.error)
            lines += [
                "Nodes: " + ", ".join(session.graph.get("nodes", [])[:100]),
                "Topics: " + ", ".join(session.graph.get("topics", [])[:100]),
            ]
            state = self.manager.states.backend_snapshot(identity)
            if state:
                lines += [
                    f"Robot instance: {state.instance} · model {state.model_signature[:16]}",
                    f"Feedback valid: {state.valid} · source {state.source}",
                    str(dict(state.positions)),
                ]
        elif identity in self.manager.last_errors:
            lines.append(self.manager.last_errors[identity])
        self.details.setPlainText("\n".join(lines))

    def perform(self, callback):
        try:
            callback()
        except (ValueError, OSError, KeyError, yaml.YAMLError) as exc:
            QMessageBox.warning(self, "Backend operation", str(exc))

    def save_editor(self, editor, existing=False):
        if editor.exec() != QDialog.DialogCode.Accepted:
            return
        profile = editor.values()
        if existing:
            self.manager.update(profile)
        else:
            self.manager.add(profile, bind_current=False)
        local = dict(self.main.settings.get("backend_local", {}))
        local[profile.id] = dict(local.get(profile.id, {}), **editor.local_values())
        self.main.settings.set("backend_local", local)
        self.main.backend_workspace.local_changed.emit()
        self.refresh()

    def add(self):
        self.perform(
            lambda: self.save_editor(
                BackendProfileDialog(self.main, new_profile(), self, is_new=True)
            )
        )

    def edit(self):
        def work():
            identity = self.selected_id()
            if identity in self.manager.sessions or identity in self.main.isaac_processes.processes:
                raise ValueError(
                    "Disconnect and stop this owned simulator before editing its profile"
                )
            self.save_editor(
                BackendProfileDialog(self.main, self.manager.profiles[identity], self), True
            )

        self.perform(work)

    def duplicate(self):
        def work():
            original = self.manager.profiles[self.selected_id()]
            data = original.model_dump()
            data.update(
                id=new_profile(original.backend_type).id, name=original.name + " copy", port=0
            )
            self.manager.add(BackendProfile(**data), bind_current=False)

        self.perform(work)

    def remove(self):
        self.perform(lambda: self.manager.remove(self.selected_id()))

    def connect_selected(self):
        self.perform(lambda: self.main.start_profile_ros(self.selected_id(), activate=False))

    def disconnect_selected(self):
        self.perform(lambda: self.manager.disconnect_profile(self.selected_id()))

    def activate_selected(self):
        self.perform(lambda: self.main.activate_backend(self.selected_id()))

    def install_runtime(self):
        def work():
            profile = self.manager.profiles[self.selected_id()]
            if self.main.runner.busy:
                raise ValueError("Wait for the current runtime operation")
            self.main.runner.run(
                lambda progress: self.main.manager.install(profile.distribution, progress),
                lambda _: self.main.refresh_runtime(),
            )

        self.perform(work)

    def diagnostics(self):
        identity = self.selected_id()
        if not identity:
            return

        def done(status):
            if identity in self.manager.profiles:
                self.diagnostic_output.setPlainText(
                    "\n".join(
                        severity + ": " + message
                        for severity, message in self.manager.diagnostics(identity, status)
                    )
                )
                self.detail_tabs.setCurrentWidget(self.diagnostic_output)

        self.main.runner.run(lambda _: self.main.manager.status(), done)

    def export_profile(self):
        identity = self.selected_id()
        if not identity:
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Export portable backend profile",
            self.manager.profiles[identity].backend_type + ".backend.yaml",
            "YAML (*.yaml)",
        )
        if path:
            self.perform(
                lambda: atomic_text(
                    Path(path),
                    yaml.safe_dump(
                        dict(
                            schema_version=1,
                            profile=self.manager.profiles[identity].model_dump(mode="json"),
                        ),
                        sort_keys=False,
                    ),
                )
            )

    def import_profile(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Import backend profile", "", "YAML (*.yaml *.yml)"
        )
        if not path:
            return

        def work():
            source = Path(path)
            if source.stat().st_size > 100000:
                raise ValueError("Backend profile exceeds 100 KB")
            document = yaml.safe_load(source.read_text())
            if not isinstance(document, dict) or document.get("schema_version") != 1:
                raise ValueError("Unsupported backend profile document")
            self.manager.add(BackendProfile.model_validate(document["profile"]), bind_current=False)

        self.perform(work)

    def local(self, identity):
        return self.main.settings.get("backend_local", {}).get(identity, {})

    def create_scene(self):
        def work():
            identity = self.selected_id()
            profile = self.manager.profiles[identity]
            local = self.local(identity)
            self.manager._validate_binding(profile)
            directory = (
                Path(local["scene_directory"]).expanduser()
                if local.get("scene_directory")
                else self.main.model_manager.root.parent
                / "simulations"
                / identity
                / profile.model_signature[:16]
            )
            model = self.main.model
            positions = self.main.independent_positions()

            def done(request):
                settings = dict(self.main.settings.get("backend_local", {}))
                values = dict(settings.get(identity, {}))
                values["scene_request"] = str(request)
                settings[identity] = values
                self.main.settings.set("backend_local", settings)
                self.main.backend_workspace.local_changed.emit()
                self.details.setPlainText(
                    "Scene request prepared. Start Isaac to import and validate the articulation: "
                    + str(request)
                )

            self.main.runner.run(
                lambda _: IsaacSimSceneManager().prepare(
                    profile, model, directory, positions, local.get("scene_path")
                ),
                done,
            )

        self.perform(work)

    def start_isaac(self):
        self.perform(lambda: self.main.simulation_header.start_profile(self.selected_id()))

    def stop_isaac(self):
        self.main.isaac_processes.stop(self.selected_id())

    def process_log(self, identity, text):
        if identity == self.selected_id():
            self.process_output.appendPlainText(text[-10000:])
