import threading
from unittest.mock import MagicMock

import pytest

from ros_robot_studio.backends.manager import ROSBackendManager
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.robot_state import RobotStateManager
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.ros_backend import ROS2ExecutionBackend
from ros_robot_studio.ros_bridge.transport import Transport


class FakePorts:
    def __init__(self):
        self.ports = {}
        self.sequence = 19000

    def acquire(self, identity, port=0):
        if port and port in self.ports.values():
            raise ValueError("Port conflict")
        self.sequence += 1
        self.ports[identity] = port or self.sequence
        return self.ports[identity]

    def release_socket(self, identity):
        pass

    def release(self, identity):
        self.ports.pop(identity, None)


class FakeTransport(Transport):
    def __init__(self, url, parent=None):
        super().__init__(parent)
        self.url = url
        self.connected = False
        self.subscriptions = set()
        self.published = []

    def connect_transport(self):
        self.connected = True
        self.connection_changed.emit(True)
        self.graph.emit(
            {
                "topics": ["/joint_states"],
                "types": ["sensor_msgs/msg/JointState"],
                "nodes": ["/fixture"],
            }
        )

    def disconnect_transport(self):
        self.connected = False
        self.subscriptions.clear()
        self.connection_changed.emit(False)

    def subscribe(self, topic, message_type=""):
        self.subscriptions.add(topic)

    def publish(self, topic, message_type, message):
        self.published.append((topic, message))


class FakeReadOnlyTransport(FakeTransport):
    def publish(self, *_):
        raise PermissionError("Read only")


@pytest.fixture
def manager(qtbot):
    p, m, _ = pick_place_demo(prefer_so101=False)
    states = RobotStateManager()
    states.bind(m, p.robot, m.default_configuration())
    runtime = MagicMock()
    runtime.start.side_effect = lambda *args, **kwargs: "owned-" + str(runtime.start.call_count)
    instance = ROSBackendManager(
        runtime,
        states,
        transport_factory=FakeTransport,
        readonly_transport_factory=FakeReadOnlyTransport,
        ports=FakePorts(),
    )
    instance.load([], p.robot)
    errors = []
    instance.failed.connect(errors.append)
    yield instance, runtime, states, m, errors
    instance.shutdown()
    qtbot.waitUntil(lambda: not instance.sessions and not instance.runner.busy)
    instance.deleteLater()


def connected(manager, qtbot, kind="ros_demo", domain=10, **values):
    instance, _, states, m, _ = manager
    p = instance.add(new_profile(kind, domain_id=domain, **values))
    session = instance.connect_profile(p.id)
    qtbot.waitUntil(lambda: session.transport is not None)
    return p, session


def feedback(session, model, position, stamp=1):
    q = dict(model.default_configuration(), axis_x=position)
    session.transport.message.emit(
        "/joint_states",
        {
            "name": list(q),
            "position": list(q.values()),
            "header": {"stamp": {"sec": stamp, "nanosec": 0}},
        },
    )


def test_two_sessions_have_distinct_endpoints_states_and_routes(manager, qtbot):
    instance, runtime, states, m, errors = manager
    a, sa = connected(manager, qtbot, domain=1)
    b, sb = connected(manager, qtbot, domain=4)
    feedback(sa, m, 0.03)
    feedback(sb, m, 0.06)
    assert sa.port != sb.port and sa.transport is not sb.transport
    assert sa.payload["control_namespace"] != sb.payload["control_namespace"]
    assert [c.args[2].domain_id for c in runtime.start.call_args_list] == [1, 4]
    instance.activate(a.id)
    assert states.snapshot().positions["axis_x"] == 0.03
    backend = ROS2ExecutionBackend(states)
    backend.connect_backend(sa.transport, m, a.execution_config(), sa.payload, states.lease())
    backend.send("goal", id="fixture", trajectory={})
    assert len(sa.transport.published) == 1 and not sb.transport.published
    instance.activate(b.id)
    assert sa.transport.connected and states.snapshot().ros_domain == 4
    with pytest.raises(ConnectionError, match="no longer active"):
        backend.send("goal", id="late", trajectory={})
    feedback(sa, m, 0.04, 2)
    assert states.snapshot().positions["axis_x"] == 0.06
    backend.disconnect_backend()
    backend.deleteLater()
    assert not errors


def test_activation_needs_typed_fresh_state_and_model_binding(manager, qtbot):
    instance, _, states, m, _ = manager
    a, session = connected(manager, qtbot)
    with pytest.raises(ValueError, match="fresh"):
        instance.activate(a.id)
    feedback(session, m, 0.03)
    session.graph["types"] = ["std_msgs/msg/String"]
    with pytest.raises(ValueError, match="typed"):
        instance.activate(a.id)
    session.graph["types"] = ["sensor_msgs/msg/JointState"]
    session.profile.model_signature = "incorrect"
    with pytest.raises(ValueError, match="model differs"):
        instance.activate(a.id)
    session.profile.model_signature = a.model_signature
    instance.activate(a.id)
    session.transport.connection_changed.emit(False)
    assert not states.snapshot().valid


def test_hardware_monitoring_never_creates_command_route(manager, qtbot):
    instance, _, states, m, _ = manager
    profile, session = connected(
        manager, qtbot, "real_robot", 0, network="host", network_confirmed=True
    )
    assert session.payload["profile"] == "read_only"
    assert not any("/execution/" in topic for topic in session.transport.subscriptions)
    feedback(session, m, 0.03)
    instance.activate(profile.id)
    assert states.snapshot().backend == "ros_read_only"
    assert not session.capabilities.execution_permitted
    backend = ROS2ExecutionBackend(states)
    backend.connect_backend(
        session.transport, m, profile.execution_config(), session.payload, states.lease()
    )
    with pytest.raises(ConnectionError, match="read only"):
        backend.send("goal")
    assert not session.transport.published
    backend.disconnect_backend()
    backend.deleteLater()


def test_start_cancel_race_cleans_owned_container_and_lease(manager, qtbot):
    instance, runtime, states, _, _ = manager
    started = threading.Event()
    release = threading.Event()

    def start(*args, **kwargs):
        started.set()
        assert release.wait(5)
        return "owned-pending"

    runtime.start.side_effect = start
    p = instance.add(new_profile())
    session = instance.connect_profile(p.id)
    qtbot.waitUntil(started.is_set)
    done = []
    instance.disconnect_profile(p.id, lambda: done.append(True))
    release.set()
    qtbot.waitUntil(lambda: bool(done))
    runtime.stop.assert_called_once_with("owned-pending")
    assert (
        not instance.sessions and not instance.ports.ports and states.backend_snapshot(p.id) is None
    )
    assert not states.update(session.source_lease, states.model.default_configuration())


def test_failed_start_releases_endpoint_and_reports_error(manager, qtbot):
    instance, runtime, states, _, errors = manager
    runtime.start.side_effect = RuntimeError("fixture start failed")
    p = instance.add(new_profile())
    instance.connect_profile(p.id)
    qtbot.waitUntil(lambda: bool(errors))
    assert (
        not instance.sessions and not instance.ports.ports and states.backend_snapshot(p.id) is None
    )
    assert "fixture start failed" in errors[0]


def test_connected_profile_cannot_change_domain_or_be_deleted(manager, qtbot):
    instance, _, _, _, _ = manager
    p, _ = connected(manager, qtbot)
    with pytest.raises(ValueError, match="Disconnect"):
        instance.update(p.model_copy(update={"domain_id": 11}))
    with pytest.raises(ValueError, match="Disconnect"):
        instance.remove(p.id)


def test_clock_reset_invalidates_execution_without_cross_backend_effect(manager, qtbot):
    instance, _, states, m, _ = manager
    a, sa = connected(manager, qtbot, domain=1)
    b, sb = connected(manager, qtbot, domain=4)
    feedback(sa, m, 0.03)
    feedback(sb, m, 0.06)
    instance.activate(b.id)
    sa.transport.message.emit("/clock", {"clock": {"sec": 3, "nanosec": 0}})
    sa.transport.message.emit("/clock", {"clock": {"sec": 2, "nanosec": 0}})
    assert sa.requires_reconnect and not sb.requires_reconnect
    assert states.snapshot().backend_id == b.id and states.snapshot().valid
    with pytest.raises(ValueError, match="fresh"):
        instance.activate(a.id)
