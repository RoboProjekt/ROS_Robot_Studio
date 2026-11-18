"""Compact runtime controls backed by the existing backend/process managers."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from uuid import uuid4

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QFileDialog,
    QGridLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QToolBar,
    QWidget,
)

from ros_robot_studio.backends.isaac import IsaacSimInstallationManager, IsaacSimSceneManager
from ros_robot_studio.backends.profiles import DEFAULTS, new_profile
from ros_robot_studio.gui.backend_connections import BackendProfileDialog
from ros_robot_studio.gui.usb_robot import port_access_error, usb_ports

EXTENSIONS = {
    "isaac_sim": (".usd", ".usda", ".usdc"),
    "mujoco": (".xml", ".mjcf"),
    "gazebo": (".sdf", ".world"),
}


class SimulationHeader(QWidget):
    def __init__(self, main):
        super().__init__(main)
        self.main, self.workspace = main, main.backend_workspace
        self.manager, self.processes = main.backend_manager, main.isaac_processes
        self.operation_error = None
        self.pending = set()
        self.scene_ready_keys = {}
        self.port_profile = None
        body = QGridLayout(self)
        body.setContentsMargins(4, 3, 4, 3)
        body.addWidget(QLabel("ROS Runtime"), 0, 0)
        body.addWidget(main.distro, 0, 1)
        body.addWidget(QLabel("Simulation Runtime"), 0, 2)
        self.runtime = QComboBox()
        self.runtime.setMinimumWidth(195)
        body.addWidget(self.runtime, 0, 3)
        self.file_label = QLabel("Simulation File")
        body.addWidget(self.file_label, 0, 4)
        self.scene = QComboBox()
        self.scene.setMinimumWidth(180)
        self.scene.setMaximumWidth(430)
        body.addWidget(self.scene, 0, 5)
        self.browse = QPushButton("Browse…")
        body.addWidget(self.browse, 0, 6)
        self.history_menu = QPushButton("Recent…")
        from PySide6.QtWidgets import QMenu

        menu = QMenu(self.history_menu)
        menu.addAction("Remove selected entry", self.remove_recent)
        menu.addAction("Clear recent files for this simulator", lambda: self.remove_recent(True))
        self.history_menu.setMenu(menu)
        body.addWidget(self.history_menu, 0, 7)
        self.port_label = QLabel("Robot Connection")
        body.addWidget(self.port_label, 0, 4)
        self.robot_port = QComboBox()
        self.robot_port.setMinimumWidth(180)
        self.robot_port.setMaximumWidth(430)
        self.robot_port.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.robot_port.setToolTip(
            "Choose a USB serial adapter for the SO-101 Follower, or monitor ROS JointState."
        )
        body.addWidget(self.robot_port, 0, 5)
        self.refresh_ports_button = QPushButton("Refresh ports")
        body.addWidget(self.refresh_ports_button, 0, 6, 1, 2)
        self.start = QPushButton("Start Simulation")
        body.addWidget(self.start, 0, 8)
        self.connect_button = QPushButton("Connect ROS")
        body.addWidget(self.connect_button, 0, 9)
        self.auto_play = QCheckBox("Auto Play")
        self.auto_play.setToolTip(
            "Explicitly start playback in an owned Isaac simulation. Default: manual Play in Isaac."
        )
        body.addWidget(self.auto_play, 1, 0, 1, 2)
        self.status = QLabel()
        self.status.setWordWrap(True)
        body.addWidget(self.status, 1, 2, 1, 7)
        self.disconnect = QPushButton("Disconnect")
        body.addWidget(self.disconnect, 1, 9)
        self.runtime.activated.connect(self.select_runtime)
        self.scene.activated.connect(self.select_scene)
        self.robot_port.activated.connect(self.select_port)
        self.refresh_ports_button.clicked.connect(self.refresh_ports)
        self.refresh_ports_button.clicked.connect(self.refresh)
        self.browse.clicked.connect(self.browse_scene)
        self.auto_play.toggled.connect(self.save_auto_play)
        self.start.clicked.connect(
            lambda: self.perform(lambda: self.start_profile(self.manager.selected_id))
        )
        self.connect_button.clicked.connect(lambda: self.perform(self.connect_selected))
        self.disconnect.clicked.connect(lambda: self.perform(self.disconnect_selected))
        self.manager.changed.connect(self.refresh)
        self.manager.profiles_changed.connect(self.refresh)
        self.workspace.local_changed.connect(self.refresh)
        self.main.runner.busy_changed.connect(self.refresh)
        self.processes.changed.connect(self.refresh)
        self.processes.ready.connect(self.scene_ready)
        self.timer = QTimer(self)
        self.timer.setInterval(750)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start()
        self.refresh()

    def perform(self, callback):
        self.operation_error = None
        try:
            callback()
        except (ValueError, OSError, KeyError) as exc:
            self.operation_error = (self.manager.selected_id, str(exc))
            self.refresh_status()
            self.main.activity(str(exc))

    def profile(self):
        return self.manager.profiles.get(self.manager.selected_id)

    def can_jog(self, profile):
        return (
            profile.enabled
            and profile.usb_joint_jog
            and (self.main.project.robot.model_id == profile.robot_model_id == "so101_follower")
        )

    def local(self, identity=None):
        return dict(
            self.main.settings.get("backend_local", {}).get(
                identity or self.manager.selected_id, {}
            )
        )

    def save_local(self, identity, values):
        settings = dict(self.main.settings.get("backend_local", {}))
        settings[identity] = dict(settings.get(identity, {}), **values)
        self.main.settings.set("backend_local", settings)
        self.workspace.local_changed.emit()

    def refresh(self, *_):
        if self.main.closed:
            return
        busy = self.main.runner.busy
        self.runtime.setEnabled(not busy)
        self.runtime.blockSignals(True)
        self.runtime.clear()
        self.runtime.addItem("Offline Demo", None)
        kinds = ["ros_demo", "isaac_sim", "mujoco", "gazebo", "real_robot", "generic_ros"]
        for kind in kinds:
            profiles = [p for p in self.manager.profiles.values() if p.backend_type == kind]
            for p in profiles:
                self.runtime.addItem(p.name, p.id)
            if not profiles:
                name = DEFAULTS[kind]["name"]
                self.runtime.addItem(
                    name if kind == "real_robot" else name + " · configure…", "new:" + kind
                )
        self.runtime.setCurrentIndex(max(0, self.runtime.findData(self.manager.selected_id)))
        self.runtime.blockSignals(False)
        profile = self.profile()
        if profile:
            self.main.distro.blockSignals(True)
            self.main.distro.setCurrentIndex(self.main.distro.findData(profile.distribution))
            self.main.distro.blockSignals(False)
        kind = profile.backend_type if profile else "offline"
        hardware = kind == "real_robot"
        files = kind in EXTENSIONS
        for widget in (self.file_label, self.scene, self.browse, self.history_menu):
            widget.setVisible(not hardware)
        for widget in (self.port_label, self.robot_port, self.refresh_ports_button):
            widget.setVisible(hardware)
            widget.setEnabled(not busy)
        port_profile = profile.id if hardware else None
        if self.port_profile != port_profile:
            self.port_profile = port_profile
            if hardware:
                self.refresh_ports()
        self.scene.blockSignals(True)
        self.scene.clear()
        selected = self.local().get("scene_path", "")
        if files:
            default_scene = (
                "Generate from current robot"
                if kind == "mujoco"
                or (kind == "isaac_sim" and profile.connection_mode == "local_installation")
                else "Select a scene file…"
            )
            self.scene.addItem(default_scene, "")
            recent = self.main.settings.get("simulation_recent_files", {}).get(kind, [])
            for path in dict.fromkeys(([selected] if selected else []) + recent):
                exists = Path(path).is_file()
                self.scene.addItem(("" if exists else "Missing · ") + Path(path).name, path)
                self.scene.setItemData(self.scene.count() - 1, path, 3)  # Qt.ToolTipRole
            self.scene.setCurrentIndex(max(0, self.scene.findData(selected)))
        else:
            self.scene.addItem("Not required", "")
        self.scene.blockSignals(False)
        for widget in (self.scene, self.browse, self.history_menu):
            widget.setEnabled(files and not busy)
        self.auto_play.blockSignals(True)
        self.auto_play.setChecked(bool(self.local().get("auto_play", False)))
        self.auto_play.blockSignals(False)
        self.auto_play.setVisible(kind == "isaac_sim")
        self.auto_play.setEnabled(not busy)
        serial_selected = hardware and bool(self.robot_port.currentData())
        self.start.setText(
            ("USB Joint Control…" if self.can_jog(profile) else "Read USB Joints…")
            if serial_selected
            else "Connect Read Only"
            if hardware
            else "Start Simulation"
        )
        self.start.setEnabled(
            not busy
            and kind != "gazebo"
            and (
                not serial_selected
                or self.robot_port.model().item(self.robot_port.currentIndex()).isEnabled()
            )
        )
        self.connect_button.setEnabled(
            bool(profile) and not serial_selected and kind not in ("mujoco", "gazebo")
        )
        self.refresh_status()

    def refresh_ports(self):
        selected = self.local().get("usb_serial_port", "")
        self.robot_port.clear()
        self.robot_port.addItem("ROS JointState · read only", "")
        ports = usb_ports()
        for path, title in ports:
            self.robot_port.addItem(title, path)
            self.robot_port.setItemData(self.robot_port.count() - 1, path, 3)  # Qt.ToolTipRole
        if selected and self.robot_port.findData(selected) < 0:
            self.robot_port.addItem("Disconnected · " + Path(selected).name, selected)
            self.robot_port.model().item(self.robot_port.count() - 1).setEnabled(False)
        elif not ports:
            self.robot_port.addItem("No USB serial adapters detected")
            self.robot_port.model().item(self.robot_port.count() - 1).setEnabled(False)
        self.robot_port.setCurrentIndex(max(0, self.robot_port.findData(selected)))

    def select_port(self, index):
        self.operation_error = None
        if self.main.local_runtime.profile is self.profile():
            self.main.start_offline()
        self.save_local(
            self.manager.selected_id, {"usb_serial_port": self.robot_port.itemData(index)}
        )

    def set_status(self, text):
        if self.operation_error and self.operation_error[0] == self.manager.selected_id:
            text += "\nError: " + self.operation_error[1]
        self.status.setText(text)

    def refresh_status(self):
        if self.main.closed:
            return
        p = self.profile()
        if not p:
            self.set_status("Offline Demo · local robot state · Docker optional")
            return
        session = self.manager.sessions.get(p.id)
        active = self.manager.active_id == p.id
        if p.backend_type == "mujoco":
            running = self.main.local_runtime.profile is p
            self.set_status(
                f"MuJoCo · {self.main.ros_status if running else 'Start Simulation to load the scene or current URDF'}"
            )
            return
        if p.backend_type == "gazebo":
            self.set_status(f"{p.name} · adapter not implemented · files can be assigned locally")
            return
        port = self.robot_port.currentData() if p.backend_type == "real_robot" else ""
        if port:
            if self.main.local_runtime.profile is p:
                self.set_status(port_access_error(port) or f"SO-101 USB · {self.main.ros_status}")
                return
            self.set_status(
                port_access_error(port)
                or (
                    "SO-101 Follower · USB single-joint control"
                    if self.can_jog(p)
                    else "SO-101 Follower · USB joint readings · Read Only"
                )
            )
            return
        connection = session.status if session else "Disconnected"
        if p.backend_type == "isaac_sim":
            owned = p.id in self.processes.processes
            scene = (
                owned
                and p.id in self.processes.ready_reports
                and p.id not in self.processes.stopping
                and self.scene_ready_keys.get(p.id) == self.scene_key(p)
            )
            bridge = bool(session and session.transport and session.transport.connected)
            playing = bool(
                bridge
                and session.clock_seen
                and session.last_clock_received > 0
                and self.manager.clock() - session.last_clock_received < 1.5
            )
            state = bool(session and "joint_state_feedback" in session.capabilities.available)
            reconnect = bool(
                session
                and (
                    session.requires_reconnect
                    or (active and self.main.ros_execution.reconnect_required)
                )
            )
            ready = bool(
                active
                and session
                and session.capabilities.execution_permitted
                and playing
                and self.main.ros_execution.ready()
            )
            execution_status = (
                "Reconnect required" if reconnect else "Simulation Ready" if ready else connection
            )
            self.set_status(
                f"Process: {'Stopping (owned)' if p.id in self.processes.stopping else 'Running (owned)' if owned else 'Not owned / external unverified'} · Scene: {'Loaded & validated' if scene else 'Unverified'} · "
                f"Bridge: {'Connected' if bridge else 'Waiting'} · Play: {'Playing' if playing else 'Paused / waiting'} · "
                f"State: {'Available' if state else 'Waiting'} · {execution_status}"
            )
        else:
            self.set_status(
                f"{p.name} · domain {p.domain_id} · {connection} · "
                + (
                    "Execution: Read Only"
                    if p.read_only or p.backend_type == "real_robot"
                    else "Simulation execution"
                )
                + (" · Active" if active else " · Selected")
            )

    def confirm_change(self, title, message):
        return (
            QMessageBox.question(
                self,
                title,
                message,
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
                QMessageBox.StandardButton.Cancel,
            )
            == QMessageBox.StandardButton.Yes
        )

    def select_runtime(self, index):
        self.perform(lambda: self._select_runtime(self.runtime.itemData(index)))

    def _select_runtime(self, identity):
        if isinstance(identity, str) and identity.startswith("new:"):
            kind = identity.split(":", 1)[1]
            profile = new_profile(kind, distribution=self.main.selected_distro)
            if kind == "real_robot":
                profile = self.manager.add(profile, bind_current=False)
            else:
                editor = BackendProfileDialog(self.main, profile, self, is_new=True)
                if editor.exec() != QDialog.DialogCode.Accepted:
                    self.refresh()
                    return
                profile = self.manager.add(editor.values(), bind_current=False)
                self.save_local(profile.id, editor.local_values())
            identity = profile.id
        if identity == self.manager.selected_id:
            return
        if (
            self.main.motion.backend.status == "playing"
            or self.main.ros_execution.status == "executing"
        ):
            if not self.confirm_change(
                "Switch runtime", "Stop the current task and switch the active runtime?"
            ):
                self.refresh()
                return
        if self.main.mode == "local":
            self.main.start_offline()
        self.main.motion.stop()
        self.manager.deactivate()
        self.manager.select_profile(identity)
        if identity is None:
            self.main.request_offline()
        else:
            session = self.manager.sessions.get(identity)
            if session and "joint_state_feedback" in session.capabilities.available:
                self.workspace.activate(identity)
        self.refresh()

    def select_scene(self, index):
        path = self.scene.itemData(index)
        self.perform(lambda: self.assign_scene(path))

    def browse_scene(self):
        p = self.profile()
        if not p or p.backend_type not in EXTENSIONS:
            return
        patterns = " ".join("*" + e for e in EXTENSIONS[p.backend_type])
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Select simulation scene",
            self.local().get("scene_path", ""),
            f"Scene files ({patterns})",
        )
        if path:
            self.perform(lambda: self.assign_scene(path))

    def assign_scene(self, path):
        p = self.profile()
        if not p or p.backend_type not in EXTENSIONS:
            return
        old = self.local().get("scene_path", "")
        if old == path:
            return
        if p.backend_type == "mujoco" and self.main.local_runtime.profile is p:
            self.main.start_offline()
        if path:
            file = Path(path).expanduser().resolve()
            if file.suffix.lower() not in EXTENSIONS[p.backend_type] or not file.is_file():
                self.refresh()
                raise ValueError("Select an existing file of the simulator's supported type")
            path = str(file)
        busy = (
            p.id in self.processes.processes
            or p.id in self.manager.sessions
            or self.main.motion.backend.status == "playing"
            or self.main.ros_execution.status == "executing"
        )
        if busy:
            if not self.confirm_change(
                "Change simulation scene",
                f"Stop this backend's task and owned simulator before changing the scene?\n\n{old or '(none)'}\n→ {path or '(none)'}\nExternal simulator instances remain under their owner's control.",
            ):
                self.refresh()
                return
            self.main.motion.stop()
            self.pending.discard(p.id)
            self.manager.disconnect_profile(p.id)
            self.processes.stop(p.id)
        if path:
            recent = dict(self.main.settings.get("simulation_recent_files", {}))
            recent[p.backend_type] = [path] + [
                v for v in recent.get(p.backend_type, []) if v != path
            ][:11]
            self.main.settings.set("simulation_recent_files", recent)
        self.save_local(p.id, {"scene_path": path, "scene_request": ""})

    def remove_recent(self, clear=False):
        p = self.profile()
        if not p or p.backend_type not in EXTENSIONS:
            return
        path = self.scene.currentData()
        if path == self.local().get("scene_path") and path:
            self.assign_scene("")
            if self.local().get("scene_path"):
                return
        recent = dict(self.main.settings.get("simulation_recent_files", {}))
        recent[p.backend_type] = (
            [] if clear else [v for v in recent.get(p.backend_type, []) if v != path]
        )
        self.main.settings.set("simulation_recent_files", recent)
        self.refresh()

    def save_auto_play(self, enabled):
        p = self.profile()
        if p and p.backend_type == "isaac_sim":
            self.save_local(p.id, {"auto_play": enabled})

    def connect_selected(self):
        p = self.profile()
        if p:
            self.workspace.connect(p.id, activate=True)

    def disconnect_selected(self):
        p = self.profile()
        if p:
            if self.main.local_runtime.profile is p:
                self.main.start_offline()
            self.pending.discard(p.id)
            self.manager.disconnect_profile(p.id)

    def start_profile(self, identity):
        p = self.manager.profiles.get(identity)
        if not p:
            self.main.request_offline()
            return
        if p.backend_type == "real_robot" and self.robot_port.currentData():
            self.main.local_runtime.start_usb(p, self.robot_port.currentData(), self.can_jog(p))
            return
        if p.backend_type == "mujoco":
            self.main.local_runtime.start_mujoco(p, self.local(p.id).get("scene_path"))
            return
        if p.backend_type != "isaac_sim" or p.connection_mode != "local_installation":
            if p.backend_type == "gazebo":
                raise ValueError("This simulator adapter is not implemented")
            self.workspace.connect(identity, activate=True)
            return
        if not self.main.docker_status.available:
            self.main.open_docker_diagnostics()
            return
        if self.main.runner.busy:
            raise ValueError("Wait for the current runtime operation")
        self.manager.validate_connection(identity)
        local = self.local(identity)
        installation = IsaacSimInstallationManager().validate(local.get("installation_path", ""))
        if p.distribution not in installation.bridge_distributions:
            raise ValueError(
                "The selected Isaac installation does not provide this ROS distribution"
            )
        if not p.network_confirmed or not p.simulation_confirmed:
            raise ValueError(
                "Review host-network and simulation confirmation in Backends before starting Isaac"
            )
        selected_scene = local.get("scene_path", "")
        scene = Path(selected_scene).resolve() if selected_scene else None
        if scene is not None and (
            not scene.is_file() or scene.suffix.lower() not in EXTENSIONS["isaac_sim"]
        ):
            raise ValueError("Select an existing USD / USDA / USDC scene first")
        if identity in self.processes.processes:
            data = json.loads(Path(self.processes.requests[identity]).read_text())
            if (
                data.get("existing_scene") != (str(scene) if scene else None)
                or data.get("model_signature") != p.model_signature
            ):
                raise ValueError(
                    "The owned process uses another scene/model. Change the scene and confirm stopping it first"
                )
            if any(
                hashlib.sha256(Path(file).read_bytes()).hexdigest() != digest
                for file, digest in data.get("source_files", {}).items()
            ):
                raise ValueError(
                    "Scene sources changed. Stop the owned simulator and restart with a fresh scene request"
                )
            self.workspace.connect(identity, activate=True)
            return
        external = self.processes.external_instances(installation)
        other_owned = [key for key in self.processes.processes if key != identity]
        if external or other_owned:
            action = self.existing_instance_action(other_owned, external)
            if action == "connect":
                self.workspace.connect(identity, activate=True)
                return
            if action != "start":
                return
        if p.distribution not in self.main.docker_status.installed:
            raise ValueError(
                "Install the selected ROS runtime in Project & Runtime before starting the simulator"
            )
        directory = self.main.simulation_directory() / identity / uuid4().hex
        if scene is None and local.get("scene_directory"):
            directory = Path(local["scene_directory"]).expanduser() / directory.name
        model, positions = self.main.model, self.main.independent_positions()
        self.pending.add(identity)

        def prepared(request):
            if self.manager.profiles.get(identity) is not p or self.main.closed:
                self.pending.discard(identity)
                return
            self.save_local(identity, {"scene_request": str(request)})
            try:
                self.processes.start(
                    p,
                    installation,
                    request,
                    headless=local.get("headless", False),
                    auto_play=local.get("auto_play", False),
                )
                self.workspace.connect(identity, activate=self.manager.selected_id == identity)
            except (ValueError, OSError) as exc:
                self.pending.discard(identity)
                self.status.setText(str(exc))
                self.main.activity(str(exc))

        self.main.runner.run(
            lambda _: IsaacSimSceneManager().prepare(p, model, directory, positions, scene),
            prepared,
        )

    def existing_instance_action(self, owned, external):
        message = QMessageBox(self)
        message.setWindowTitle("Isaac Sim is already running")
        message.setText(
            f"Found {len(owned)} other owned and {len(external)} external Isaac process(es). "
            "Connect to existing ROS interfaces, or explicitly start another instance with the selected scene. "
            "Existing instances and their scenes will not be changed."
        )
        connect = message.addButton("Connect existing ROS", QMessageBox.ButtonRole.AcceptRole)
        another = message.addButton("Start another instance", QMessageBox.ButtonRole.ActionRole)
        message.addButton(QMessageBox.StandardButton.Cancel)
        message.setDefaultButton(connect)
        message.exec()
        return (
            "connect"
            if message.clickedButton() == connect
            else "start"
            if message.clickedButton() == another
            else "cancel"
        )

    def scene_key(self, profile):
        return (
            self.local(profile.id).get("scene_path", ""),
            profile.model_signature,
            profile.domain_id,
        )

    def scene_ready(self, identity, report):
        profile = self.manager.profiles.get(identity)
        if profile:
            try:
                data = json.loads(Path(self.processes.requests[identity]).read_text())
                self.scene_ready_keys[identity] = (
                    data.get("existing_scene", ""),
                    report.get("model_signature"),
                    report.get("domain_id"),
                )
                if "existing_scene" not in data:
                    self.main.activity(f"Generated Isaac USD: {report['scene']}")
            except (OSError, ValueError, KeyError):
                self.scene_ready_keys.pop(identity, None)
        self.pending.discard(identity)
        self.refresh_status()


def install_header(main):
    main.addToolBarBreak()
    toolbar = QToolBar("Simulation runtime")
    toolbar.setMovable(False)
    header = SimulationHeader(main)
    toolbar.addWidget(header)
    main.addToolBar(toolbar)
    return header
