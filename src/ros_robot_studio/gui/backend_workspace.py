"""Integrate backend sessions with the existing single active viewer/execution UI."""

from PySide6.QtCore import QObject, QTimer, Signal
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QMessageBox

from ros_robot_studio.backends.isaac import IsaacSimProcessManager
from ros_robot_studio.backends.manager import ROSBackendManager
from ros_robot_studio.gui.backend_connections import BackendConnectionsDialog
from ros_robot_studio.gui.widgets import button, label


class BackendWorkspace(QObject):
    local_changed = Signal()

    def __init__(self, main):
        super().__init__(main)
        self.main = main
        self.dialog = None
        self.project_id = None
        self.closing = False
        self.selector_key = None
        self.manager = ROSBackendManager(main.manager, main.state_manager, self, runner=main.runner)
        self.processes = IsaacSimProcessManager(self)
        main.backend_manager = self.manager
        main.isaac_processes = self.processes
        self.manager.deactivating.connect(self.prepare_change)
        self.manager.active_changed.connect(self.activated)
        self.manager.changed.connect(self.refresh_selector)
        self.manager.profiles_changed.connect(self.profiles_changed)
        self.manager.failed.connect(main.show_error)
        self.manager.log.connect(main.activity)
        self.manager.execution_invalidated.connect(self.execution_invalidated)
        self.processes.error.connect(
            lambda identity, message: main.activity(identity + ": " + message)
        )
        row = QHBoxLayout()
        row.addWidget(label("ROS backend", "subtitle"))
        self.selector = QComboBox()
        row.addWidget(self.selector, 1)
        self.selector.activated.connect(self.select_active)
        main.runtime.backend_connections = button("Backend Connections…", self.open, True)
        row.addWidget(main.runtime.backend_connections)
        main.runtime.layout().insertLayout(0, row)
        self.load(main.project)

    def load(self, project):
        if self.manager.sessions:
            self.manager.disconnect_all(lambda: self.load(project))
            return
        self.project_id = project.id
        self.manager.load(project.backends, project.robot, project.active_backend)
        self.refresh_selector()

    def profiles_changed(self):
        if self.project_id == self.main.project.id:
            self.main.project.backends = [
                p.model_copy(deep=True) for p in self.manager.profiles.values()
            ]
            self.main.project.active_backend = self.manager.selected_id
        self.main.mark_dirty()

    def collect(self, project):
        if self.project_id == project.id:
            project.backends = [p.model_copy(deep=True) for p in self.manager.profiles.values()]
            project.active_backend = self.manager.selected_id

    def open(self):
        if self.dialog is None:
            self.dialog = BackendConnectionsDialog(self.main)
        self.dialog.show()
        self.dialog.raise_()
        self.dialog.activateWindow()

    def refresh_selector(self):
        if self.main.closed:
            return
        entries = []
        for profile in self.manager.profiles.values():
            session = self.manager.sessions.get(profile.id)
            ready = bool(
                session
                and not session.cancelled
                and not session.requires_reconnect
                and "joint_state_feedback" in session.capabilities.available
            )
            entries.append(
                (
                    profile.id,
                    f"{profile.name} · domain {profile.domain_id} · "
                    + (session.status if session else "Disconnected"),
                    ready,
                )
            )
        key = (tuple(entries), self.manager.active_id)
        if key != self.selector_key:
            self.selector_key = key
            self.selector.blockSignals(True)
            self.selector.clear()
            self.selector.addItem("Choose a connected backend…", None)
            for identity, title, ready in entries:
                self.selector.addItem(title, identity)
                self.selector.model().item(self.selector.count() - 1).setEnabled(ready)
            self.selector.setCurrentIndex(max(0, self.selector.findData(self.manager.active_id)))
            self.selector.blockSignals(False)
        session = self.manager.sessions.get(self.manager.active_id)
        if session:
            self.main.motion.play_button.setEnabled(
                session.capabilities.execution_permitted
                and not self.main.motion.runner.busy
                and getattr(self.main.motion, "kinematics_ready", False)
            )

    def select_active(self, index):
        identity = self.selector.itemData(index)
        if identity:
            try:
                self.activate(identity)
            except ValueError as exc:
                self.main.show_error(str(exc))

    def start_selected(self):
        identity = self.manager.selected_id
        if identity not in self.manager.profiles:
            identity = next(
                (
                    p.id
                    for p in self.manager.profiles.values()
                    if p.enabled and p.backend_type == "ros_demo"
                ),
                None,
            )
        if not identity:
            self.open()
            return
        self.connect(identity, activate=True)

    def connect(self, identity, activate=False):
        profile = self.manager.profiles[identity]
        if (
            activate
            and self.main.container_id
            and not self.manager.owns_container(self.main.container_id)
        ):
            self.main.stop_ros(after=lambda: self.connect(identity, True))
            return
        if not self.main.docker_status.available:
            self.main.open_docker_diagnostics()
            return
        if profile.distribution not in self.main.docker_status.installed:
            if (
                QMessageBox.question(
                    self.main,
                    "Install ROS runtime",
                    f"Build/install the current {profile.distribution} runtime before connecting?",
                )
                == QMessageBox.StandardButton.Yes
            ):

                def installed(_):
                    def status_ready(status):
                        self.main.docker_status = status
                        self.connect(identity, activate)

                    self.main.runner.run(lambda _: self.main.manager.status(), status_ready)

                self.main.runner.run(
                    lambda progress: self.main.manager.install(profile.distribution, progress),
                    installed,
                )
            return
        if activate:
            self.main.motion.stop()
        self.manager.connect_profile(identity, activate)

    def activate(self, identity):
        if self.main.container_id and not self.manager.owns_container(self.main.container_id):
            self.main.stop_ros(after=lambda: self.activate(identity))
            return
        self.manager.activate(identity)

    def prepare_change(self):
        if self.main.closed:
            return
        self.main.ros_execution.disconnect_backend()
        self.main.motion.stop()
        self.main.detach_transport()

    def activated(self, identity):
        main = self.main
        if main.closed:
            return
        if not identity:
            main.container_id = None
            if main.mode == "ros":
                main.start_offline()
            return
        session = self.manager.sessions[identity]
        profile = session.profile
        main.mode = "ros"
        main.container_id = session.container_id
        main.last_ros_clock = None
        main.project.execution = profile.execution_config()
        main.project.runtime = session.runtime_config.model_copy(deep=True)
        main.project.active_backend = identity
        main.ros_lease = main.state_manager.lease()
        main.distro.blockSignals(True)
        main.distro.setCurrentIndex(main.distro.findData(profile.distribution))
        main.distro.blockSignals(False)
        main.robot_panel.joint_controls.setEnabled(False)
        main.robot_panel.profile.setEnabled(False)
        main.monitor.source.setCurrentIndex(0)
        main.mode_status.setText(
            "ROS · "
            + profile.name
            + (" · READ ONLY" if session.payload["profile"] == "read_only" else " · SIMULATION")
        )
        main.monitor.mode.setText(
            f"Backend: {profile.name} · ROS domain {profile.domain_id} · {profile.distribution} · "
            + ("Read Only" if session.payload["profile"] == "read_only" else "Simulation")
        )
        main.attach_transport(session.transport)
        main.monitor.set_graph(session.graph)
        main.connection_status(f"{profile.name} · domain {profile.domain_id} · connected")
        if session.payload["profile"] != "read_only":
            main.ros_execution.connect_backend(
                session.transport,
                main.model,
                main.project.execution,
                session.payload,
                main.ros_lease,
            )
        main.motion.backend_choice.blockSignals(True)
        main.motion.backend_choice.setCurrentIndex(1)
        main.motion.backend_choice.blockSignals(False)
        main.runtime.container_status.setText(
            profile.name + " · " + (session.container_id or "")[:12]
        )
        main.monitor.container.setText(
            f"Backend {identity}\nDomain {profile.domain_id} · bridge 127.0.0.1:{session.port}"
        )
        main.mark_dirty()
        main.busy_changed(main.runner.busy)
        self.refresh_selector()

    def execution_invalidated(self, identity, reason):
        if identity == self.manager.active_id:
            self.main.ros_execution.fail(reason)

    def offline(self):
        self.manager.deactivate()

    def stop_active(self, after=None):
        identity = self.manager.active_id
        if identity is None:
            return False
        self.manager.disconnect_profile(identity, after)
        return True

    def change_distribution(self, distribution):
        identity = self.manager.selected_id
        if identity not in self.manager.profiles:
            return False
        old = self.manager.profiles[identity]
        profile = old.model_copy(update={"distribution": distribution})
        try:
            self.manager.registry.get(profile.backend_type).get_connection_requirements(profile)
        except ValueError:
            self.main.distro.blockSignals(True)
            self.main.distro.setCurrentIndex(self.main.distro.findData(old.distribution))
            self.main.distro.blockSignals(False)
            raise
        was_active = self.manager.active_id == identity

        def change():
            self.manager.update(profile)
            if was_active:
                self.connect(identity, True)

        self.manager.disconnect_profile(identity, change)
        return True

    def reset_model(self):
        self.manager.disconnect_all()

    def close_pending(self):
        if self.manager.sessions:
            if not self.closing:
                self.closing = True

                def stopped():
                    self.closing = False
                    self.main.close()

                self.manager.disconnect_all(stopped)
            return True
        if self.processes.processes:
            for identity in list(self.processes.processes):
                self.processes.stop(identity)
            QTimer.singleShot(100, self.main.close)
            return True
        return False

    def shutdown(self):
        self.manager.shutdown()
