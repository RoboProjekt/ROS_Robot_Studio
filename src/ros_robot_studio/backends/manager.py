"""One runtime/transport per backend; one active projection into the shared robot state."""

import hashlib
import json
import socket
import time
from dataclasses import dataclass, field, replace
from functools import partial

from PySide6.QtCore import QObject, QTimer, Signal

from ros_robot_studio.backends.domains import ROSDomainManager
from ros_robot_studio.backends.plugins import BackendCapabilities, BackendPluginRegistry
from ros_robot_studio.backends.profiles import BackendProfile
from ros_robot_studio.backends.transport import ReadOnlyRosbridgeTransport
from ros_robot_studio.gui.workers import TaskRunner
from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport


class PortPool:
    """Hold a local reservation until the Docker worker starts; report bind races normally."""

    def __init__(self):
        self.ports = {}
        self.sockets = {}

    def acquire(self, backend_id, requested=0):
        if backend_id in self.ports:
            raise ValueError("Backend already owns an endpoint")
        if requested and requested in self.ports.values():
            raise ValueError("Bridge port is already assigned to another backend")
        probe = socket.socket()
        try:
            probe.bind(("127.0.0.1", requested))
            probe.listen(1)
            port = probe.getsockname()[1]
        except OSError:
            probe.close()
            raise ValueError("Local bridge port unavailable; select another port") from None
        self.ports[backend_id], self.sockets[backend_id] = port, probe
        return port

    def release_socket(self, backend_id):
        probe = self.sockets.pop(backend_id, None)
        if probe:
            probe.close()

    def release(self, backend_id):
        self.release_socket(backend_id)
        self.ports.pop(backend_id, None)


@dataclass
class BackendSession:
    profile: BackendProfile
    payload: dict
    runtime_config: object
    source_lease: object
    port: int
    container_id: str | None = None
    transport: object = None
    status: str = "Connecting"
    graph: dict = field(default_factory=dict)
    controller_available: bool = False
    controller_time: float = 0.0
    remote_busy: bool = False
    clock_seen: bool = False
    last_clock_received: float = 0.0
    last_clock: float | None = None
    requires_reconnect: bool = False
    cancelled: bool = False
    activate_when_ready: bool = False
    error: str = ""
    started: float = 0.0
    executing: bool = False
    after_disconnect: list = field(default_factory=list)
    capabilities: BackendCapabilities = field(
        default_factory=lambda: BackendCapabilities(frozenset(), False, "Not connected")
    )


class ROSBackendManager(QObject):
    changed = Signal()
    profiles_changed = Signal()
    deactivating = Signal()
    active_changed = Signal(str)
    failed = Signal(str)
    log = Signal(str)
    execution_invalidated = Signal(str, str)

    def __init__(
        self,
        runtime,
        states,
        parent=None,
        runner=None,
        transport_factory=None,
        readonly_transport_factory=None,
        ports=None,
        clock=time.monotonic,
    ):
        super().__init__(parent)
        self.runtime, self.states = runtime, states
        self.clock = clock
        self.runner = runner or TaskRunner(self)
        self.ports = ports or PortPool()
        self.registry = BackendPluginRegistry()
        self.domains = ROSDomainManager()
        self.transport_factory = transport_factory or RosbridgeTransport
        self.readonly_transport_factory = readonly_transport_factory or ReadOnlyRosbridgeTransport
        self.profiles = {}
        self.sessions = {}
        self.last_errors = {}
        self.active_id = None
        self.selected_id = None
        self.robot = None
        self.closed = False
        self.timer = QTimer(self)
        self.timer.setInterval(250)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def load(self, profiles, robot, selected_id=None):
        if self.sessions:
            raise ValueError("Disconnect existing backends before replacing project profiles")
        validated = [BackendProfile.model_validate(p.model_dump()) for p in profiles]
        if len({p.id for p in validated}) != len(validated):
            raise ValueError("Duplicate backend IDs")
        self.profiles = {p.id: p for p in validated}
        self.robot = robot
        self.selected_id = selected_id if selected_id in self.profiles else None
        self.last_errors.clear()
        self.changed.emit()

    def bind_profile(self, profile):
        if self.states.model is None:
            raise ValueError("Select a robot first")
        data = profile.model_dump()
        data.update(
            model_signature=UniversalKinematicsEngine(self.states.model).signature,
            robot_model_id=self.robot.model_id if self.robot else None,
            model_revision=(self.robot.source_revision or "") if self.robot else "",
            joint_mapping=self.registry.get(profile.backend_type).configure_joint_mapping(
                profile, self.states.model
            ),
        )
        return BackendProfile(**data)

    def add(self, profile, bind_current=True):
        profile = BackendProfile.model_validate(profile.model_dump())
        if profile.id in self.profiles:
            raise ValueError("Backend ID already exists")
        if len(self.profiles) >= 32:
            raise ValueError("At most 32 profiles per project")
        if bind_current:
            profile = self.bind_profile(profile)
        self.profiles[profile.id] = profile
        self.profiles_changed.emit()
        self.changed.emit()
        return profile

    def update(self, profile):
        if profile.id in self.sessions:
            raise ValueError("Disconnect before editing domain, mapping or controller settings")
        if profile.id not in self.profiles:
            raise ValueError("Unknown backend")
        self.profiles[profile.id] = BackendProfile.model_validate(profile.model_dump())
        self.profiles_changed.emit()
        self.changed.emit()

    def remove(self, backend_id):
        if backend_id in self.sessions:
            raise ValueError("Disconnect before deleting this backend")
        self.profiles.pop(backend_id)
        self.states.remove_backend(backend_id)
        if self.selected_id == backend_id:
            self.selected_id = None
        self.last_errors.pop(backend_id, None)
        self.profiles_changed.emit()
        self.changed.emit()

    def owns_transport(self, transport):
        return transport is not None and any(
            s.transport is transport for s in self.sessions.values()
        )

    def owns_container(self, identifier):
        return identifier is not None and any(
            s.container_id == identifier for s in self.sessions.values()
        )

    def _validate_binding(self, profile):
        model = self.states.model
        if model is None:
            raise ValueError("Load a robot before connecting")
        signature = UniversalKinematicsEngine(model).signature
        if profile.model_signature != signature:
            raise ValueError(
                "Backend model differs; explicitly bind the current model and confirm its mapping"
            )
        if self.robot and (
            profile.robot_model_id != self.robot.model_id
            or profile.model_revision != (self.robot.source_revision or "")
        ):
            raise ValueError("Backend model ID/revision differs from the selected robot")
        self.registry.get(profile.backend_type).configure_joint_mapping(profile, model)

    def validate_connection(self, backend_id):
        """Side-effect-free checks shared by simulator and ROS startup."""
        profile = BackendProfile.model_validate(self.profiles[backend_id].model_dump())
        if not profile.enabled:
            raise ValueError("Backend profile is disabled")
        plugin = self.registry.get(profile.backend_type)
        plugin.get_connection_requirements(profile)
        self._validate_binding(profile)
        failures = [
            r.message
            for r in self.domains.inspect(profile, self.profiles.values())
            if r.severity == "FAIL"
        ]
        if failures:
            raise ValueError("; ".join(failures))
        return profile

    def connect_profile(self, backend_id, activate=False):
        if self.closed:
            raise ValueError("Backend manager is shut down")
        if backend_id in self.sessions:
            session = self.sessions[backend_id]
            if session.cancelled:
                raise ValueError("Wait for disconnect to finish")
            if activate:
                session.activate_when_ready = True
                self.refresh()
            return session
        profile = self.validate_connection(backend_id)
        plugin = self.registry.get(profile.backend_type)
        port = self.ports.acquire(backend_id, profile.port)
        try:
            config = profile.runtime_config(port)
            token = hashlib.sha256(backend_id.encode()).hexdigest()[:20]
            config.namespace = "/studio_backend_" + token
            positions = {
                n: self.states.snapshot().positions[n]
                for n in self.states.model.default_configuration()
            }
            payload = plugin.prepare_runtime(profile, self.states.model, positions, config)
            payload["control_namespace"] = "/studio/backend_" + token + "/execution"
            source = (
                "ros_read_only"
                if payload["profile"] == "read_only"
                else "ros_simulation"
                if profile.backend_type == "ros_demo"
                else "external_simulation"
            )
            lease = self.states.register_backend(
                backend_id, profile.domain_id, profile.distribution, source
            )
            session = BackendSession(
                profile,
                payload,
                config,
                lease,
                port,
                activate_when_ready=activate,
                started=self.clock(),
            )
            self.sessions[backend_id] = session
            self.last_errors.pop(backend_id, None)
            self.changed.emit()

            def start(_):
                self.ports.release_socket(backend_id)
                return self.runtime.start(
                    profile.distribution, "backend-" + backend_id, config, robot_config=payload
                )

            self.runner.run(
                start, partial(self._started, session), partial(self._start_failed, session)
            )
            return session
        except Exception:
            self.ports.release(backend_id)
            raise

    def _started(self, session, identifier):
        session.container_id = identifier
        if session.cancelled or self.closed:
            self._stop_container(session)
            return
        cls = (
            self.readonly_transport_factory
            if session.payload["profile"] == "read_only"
            else self.transport_factory
        )
        transport = cls(f"ws://127.0.0.1:{session.port}", self)
        session.transport = transport
        transport.message.connect(partial(self._message, session))
        transport.graph.connect(partial(self._graph, session))
        transport.connection_changed.connect(partial(self._connection, session))
        transport.error.connect(partial(self._transport_error, session))
        transport.subscribe(session.profile.state_topic, "sensor_msgs/msg/JointState")
        transport.subscribe(session.profile.clock_topic, "rosgraph_msgs/msg/Clock")
        transport.subscribe(session.profile.tf_topic, "tf2_msgs/msg/TFMessage")
        if session.payload["profile"] != "read_only":
            transport.subscribe(
                session.payload["control_namespace"] + "/events", "std_msgs/msg/String"
            )
        transport.connect_transport()
        self.log.emit(
            f"Backend {session.profile.name}: domain {session.profile.domain_id}, bridge 127.0.0.1:{session.port}"
        )
        self.changed.emit()

    def _start_failed(self, session, message):
        self.last_errors[session.profile.id] = message
        self._finish_disconnect(session)
        self.failed.emit(session.profile.name + ": " + message)

    def _live(self, session):
        return (
            self.sessions.get(session.profile.id) is session
            and not session.cancelled
            and not self.closed
        )

    def _graph(self, session, graph):
        if not self._live(session):
            return
        if "topics" in graph:
            session.graph.update(graph)
        elif "nodes" in graph:
            session.graph["nodes"] = graph["nodes"]
        self._status(session)
        self.changed.emit()

    def _connection(self, session, connected):
        if not self._live(session):
            return
        if not connected:
            session.controller_available = False
            self.states.invalidate_backend(session.profile.id, "Backend connection lost")
        else:
            session.error = ""
        self._status(session)
        self.changed.emit()

    def _transport_error(self, session, message):
        if self._live(session):
            session.error = str(message)[:2000]
            self.changed.emit()

    def _message(self, session, topic, message):
        if not self._live(session):
            return
        profile = session.profile
        try:
            if topic == profile.state_topic:
                self.states.joint_state(session.source_lease, message, profile.joint_mapping)
            elif topic == profile.clock_topic:
                stamp = message.get("clock", {})
                value = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
                if (
                    session.last_clock is not None
                    and value < session.last_clock
                    and not session.requires_reconnect
                ):
                    session.requires_reconnect = True
                    self.states.invalidate_backend(
                        profile.id, "Simulation clock reset; reconnect and replan"
                    )
                    self.execution_invalidated.emit(
                        profile.id, "Simulation clock reset; reconnect and replan"
                    )
                if session.last_clock is not None and value > session.last_clock:
                    session.last_clock_received = self.clock()
                session.last_clock = value
                session.clock_seen = True
            elif topic == session.payload["control_namespace"] + "/events":
                data = json.loads(message.get("data", ""))
                if (
                    data.get("session") != session.payload["session"]
                    or data.get("model_signature") != profile.model_signature
                ):
                    return
                if data.get("simulation") != session.payload["profile"]:
                    return
                if data.get("kind") == "controller":
                    session.controller_available = bool(data.get("available"))
                    session.remote_busy = bool(data.get("busy"))
                    session.controller_time = self.clock()
            self._status(session)
            self.changed.emit()
        except (ValueError, TypeError, KeyError) as exc:
            session.error = str(exc)
            self.states.invalidate_backend(profile.id, "Invalid feedback: " + str(exc))
            self.execution_invalidated.emit(profile.id, "Invalid backend feedback: " + str(exc))
            self.changed.emit()

    def _status(self, session):
        if session.cancelled:
            return
        profile = session.profile
        state = self.states.backend_snapshot(profile.id)
        if state and self.states.backend_stale(profile.id, profile.stale_timeout):
            state = replace(state, valid=False)
        controller = session.controller_available and self.clock() - session.controller_time < 1.5
        session.capabilities = self.registry.get(profile.backend_type).get_execution_capabilities(
            profile, session.graph, state, controller, session.clock_seen
        )
        if session.requires_reconnect:
            session.status = "Error"
            session.capabilities = BackendCapabilities(
                session.capabilities.available, False, "Reconnect required"
            )
        elif not session.transport or not session.transport.connected:
            session.status = "Error" if self.clock() - session.started > 30 else "Connecting"
        elif state and state.valid:
            session.status = (
                "Executing"
                if session.executing
                else "Execution Ready"
                if session.capabilities.execution_permitted
                else "Controller Available"
                if controller
                else "Robot State Available"
            )
        else:
            session.status = "Connected"

    def refresh(self):
        if self.closed:
            return
        changed = False
        for session in list(self.sessions.values()):
            old = session.status
            self._status(session)
            changed |= old != session.status
            if (
                session.activate_when_ready
                and not session.cancelled
                and not session.requires_reconnect
                and "joint_state_feedback" in session.capabilities.available
            ):
                session.activate_when_ready = False
                try:
                    self.activate(session.profile.id)
                except ValueError as exc:
                    self.failed.emit(str(exc))
        if changed:
            self.changed.emit()

    def select_profile(self, backend_id):
        if backend_id is not None and backend_id not in self.profiles:
            raise ValueError("Unknown backend profile")
        if self.selected_id != backend_id:
            self.selected_id = backend_id
            self.profiles_changed.emit()
            self.changed.emit()

    def activate(self, backend_id):
        session = self.sessions.get(backend_id)
        if (
            not session
            or session.cancelled
            or not session.transport
            or not session.transport.connected
        ):
            raise ValueError("Connect the backend before activating it")
        self._validate_binding(session.profile)
        self._status(session)
        if (
            session.requires_reconnect
            or self.states.backend_stale(backend_id, session.profile.stale_timeout)
            or "joint_state_feedback" not in session.capabilities.available
        ):
            raise ValueError("Activation requires fresh typed, model-matched JointState feedback")
        if self.active_id == backend_id:
            return
        self.deactivating.emit()
        self.active_id = self.selected_id = backend_id
        self.states.activate_backend(backend_id)
        self.active_changed.emit(backend_id)
        self.changed.emit()

    def deactivate(self):
        if self.active_id is None:
            return
        self.deactivating.emit()
        self.active_id = None
        self.states.activate_source("offline", "local simulation")
        self.active_changed.emit("")
        self.changed.emit()

    def mark_execution(self, backend_id, status):
        session = self.sessions.get(backend_id)
        if session:
            session.executing = status == "executing"
            self._status(session)
            self.changed.emit()

    def disconnect_profile(self, backend_id, after=None):
        session = self.sessions.get(backend_id)
        if not session:
            if after:
                after()
            return
        if after:
            session.after_disconnect.append(after)
        if session.cancelled:
            return
        if self.active_id == backend_id:
            self.deactivate()
        session.cancelled = True
        session.status = "Disconnecting"
        self.states.remove_backend(backend_id)
        if session.transport:
            session.transport.disconnect_transport()
            session.transport.deleteLater()
            session.transport = None
        self.changed.emit()
        if session.container_id:
            self._stop_container(session)
        # Otherwise _started/_start_failed will complete cleanup of the pending start.

    def _stop_container(self, session):
        self.runner.run(
            lambda _: self.runtime.stop(session.container_id),
            lambda _: self._finish_disconnect(session),
            partial(self._stop_failed, session),
        )

    def _stop_failed(self, session, message):
        session.cancelled = False
        session.status = "Error"
        session.requires_reconnect = True
        session.error = "Stop failed; retained for retry: " + message
        self.failed.emit(session.error)
        self.changed.emit()

    def _finish_disconnect(self, session):
        backend_id = session.profile.id
        if self.sessions.get(backend_id) is session:
            self.sessions.pop(backend_id, None)
        self.states.remove_backend(backend_id)
        self.ports.release(backend_id)
        self.changed.emit()
        callbacks, session.after_disconnect = session.after_disconnect, []
        for callback in callbacks:
            callback()

    def disconnect_all(self, after=None):
        remaining = set(self.sessions)
        if not remaining:
            if after:
                after()
            return

        def finished(backend_id):
            remaining.discard(backend_id)
            if not remaining and after:
                after()

        for backend_id in list(remaining):
            self.disconnect_profile(backend_id, partial(finished, backend_id))

    def shutdown(self, after=None):
        self.closed = True
        self.timer.stop()
        self.disconnect_all(after)

    def diagnostics(self, backend_id, runtime_status=None):
        profile = self.profiles[backend_id]
        rows = [
            ("PASS", "Backend profile: " + profile.name),
            ("PASS", "Middleware: " + profile.middleware),
        ]
        rows += [
            (r.severity, r.message) for r in self.domains.inspect(profile, self.profiles.values())
        ]
        if runtime_status:
            rows.append(("PASS" if runtime_status.available else "FAIL", runtime_status.message))
            rows.append(
                (
                    "PASS" if profile.distribution in runtime_status.installed else "FAIL",
                    "Current ROS runtime image: " + profile.distribution,
                )
            )
        else:
            rows.append(("NOT TESTED", "Docker/runtime image status"))
        try:
            self._validate_binding(profile)
            rows.append(("PASS", "Robot model/revision and complete unique joint mapping"))
        except ValueError as exc:
            rows.append(("FAIL", str(exc)))
        session = self.sessions.get(backend_id)
        if not session:
            return rows + [
                ("SKIPPED", "Backend transport, ROS graph, feedback and controller: connect first")
            ]
        self._status(session)
        rows.append(
            (
                "PASS" if session.transport and session.transport.connected else "FAIL",
                "ROS adapter / WebSocket connection",
            )
        )
        rows.append(
            ("PASS" if session.graph.get("nodes") else "WARNING", "ROS graph / node discovery")
        )
        rows.append(
            (
                "PASS" if "joint_state_feedback" in session.capabilities.available else "FAIL",
                "Typed fresh JointState feedback",
            )
        )
        rows.append(
            (
                "PASS" if session.controller_available else "WARNING",
                "Trajectory controller discovery (not a motion test)",
            )
        )
        rows.append(
            (
                "PASS"
                if profile.read_only or profile.backend_type == "real_robot"
                else "PASS"
                if session.capabilities.execution_permitted
                else "WARNING",
                session.capabilities.reason,
            )
        )
        rows.append(
            (
                "PASS" if self.active_id == backend_id else "SKIPPED",
                "Active viewer/monitoring projection",
            )
        )
        consistent = self.active_id is None or self.states.snapshot().backend_id == self.active_id
        rows.append(("PASS" if consistent else "FAIL", "Active backend/state identity consistency"))
        if session.error:
            rows.append(("WARNING", session.error))
        return rows
