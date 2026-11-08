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
        self.pending = set()
        self.scene_ready_keys = {}
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
        self.processes.changed.connect(self.refresh)
        self.processes.ready.connect(self.scene_ready)
        self.timer = QTimer(self)
        self.timer.setInterval(750)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start()
        self.refresh()

    def perform(self, callback):
        try:
            callback()
        except (ValueError, OSError, KeyError) as exc:
            self.status.setText(str(exc))
            self.main.activity(str(exc))

    def profile(self):
        return self.manager.profiles.get(self.manager.selected_id)

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
        self.runtime.blockSignals(True)
        self.runtime.clear()
        self.runtime.addItem("Offline Demo", None)
        kinds = ["ros_demo", "isaac_sim", "mujoco", "gazebo", "real_robot", "generic_ros"]
        for kind in kinds:
            profiles = [p for p in self.manager.profiles.values() if p.backend_type == kind]
            for p in profiles:
                self.runtime.addItem(p.name, p.id)
            if not profiles:
                self.runtime.addItem(DEFAULTS[kind]["name"] + " · configure…", "new:" + kind)
        self.runtime.setCurrentIndex(max(0, self.runtime.findData(self.manager.selected_id)))
        self.runtime.blockSignals(False)
        profile = self.profile()
        if profile:
            self.main.distro.blockSignals(True)
            self.main.distro.setCurrentIndex(self.main.distro.findData(profile.distribution))
            self.main.distro.blockSignals(False)
        kind = profile.backend_type if profile else "offline"
        files = kind in EXTENSIONS
        self.file_label.setText("Hardware Profile" if kind == "real_robot" else "Simulation File")
        self.scene.blockSignals(True)
        self.scene.clear()
        selected = self.local().get("scene_path", "")
        if files:
            self.scene.addItem("Select a scene file…", "")
            recent = self.main.settings.get("simulation_recent_files", {}).get(kind, [])
            for path in dict.fromkeys(([selected] if selected else []) + recent):
                exists = Path(path).is_file()
                self.scene.addItem(("" if exists else "Missing · ") + Path(path).name, path)
                self.scene.setItemData(self.scene.count() - 1, path, 3)  # Qt.ToolTipRole
            self.scene.setCurrentIndex(max(0, self.scene.findData(selected)))
        else:
            self.scene.addItem(profile.name if kind == "real_robot" else "Not required", "")
        self.scene.blockSignals(False)
        for widget in (self.scene, self.browse, self.history_menu):
            widget.setEnabled(files)
        self.auto_play.blockSignals(True)
        self.auto_play.setChecked(bool(self.local().get("auto_play", False)))
        self.auto_play.blockSignals(False)
        self.auto_play.setVisible(kind == "isaac_sim")
        self.start.setText("Connect Read Only" if kind == "real_robot" else "Start Simulation")
        self.start.setEnabled(not self.main.runner.busy and kind not in ("mujoco", "gazebo"))
        self.connect_button.setEnabled(bool(profile) and kind not in ("mujoco", "gazebo"))
        self.refresh_status()

    def refresh_status(self):
        if self.main.closed:
            return
        p = self.profile()
        if not p:
            self.status.setText("Offline Demo · local robot state · Docker optional")
            return
        session = self.manager.sessions.get(p.id)
        active = self.manager.active_id == p.id
        if p.backend_type in ("mujoco", "gazebo"):
            self.status.setText(
                f"{p.name} · adapter not implemented · files can be assigned locally"
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
            ready = bool(
                active and session and session.capabilities.execution_permitted and playing
            )
            self.status.setText(
                f"Process: {'Stopping (owned)' if p.id in self.processes.stopping else 'Running (owned)' if owned else 'Not owned / external unverified'} · Scene: {'Loaded & validated' if scene else 'Unverified'} · "
                f"Bridge: {'Connected' if bridge else 'Waiting'} · Play: {'Playing' if playing else 'Paused / waiting'} · "
                f"State: {'Available' if state else 'Waiting'} · {'Simulation Ready' if ready else connection}"
            )
        else:
            self.status.setText(
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
            profile = new_profile(identity.split(":", 1)[1], distribution=self.main.selected_distro)
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
            self.pending.discard(p.id)
            self.manager.disconnect_profile(p.id)

    def start_profile(self, identity):
        p = self.manager.profiles.get(identity)
        if not p:
            self.main.request_offline()
            return
        if p.backend_type != "isaac_sim" or p.connection_mode != "local_installation":
            if p.backend_type in ("mujoco", "gazebo"):
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
        scene = Path(local.get("scene_path", ""))
        if not scene.is_file() or scene.suffix.lower() not in EXTENSIONS["isaac_sim"]:
            raise ValueError("Select an existing USD / USDA / USDC scene first")
        if identity in self.processes.processes:
            data = json.loads(Path(self.processes.requests[identity]).read_text())
            if (
                data.get("existing_scene") != str(scene.resolve())
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
        directory = self.main.model_manager.root.parent / "simulations" / identity / uuid4().hex
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
