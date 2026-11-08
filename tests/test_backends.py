from dataclasses import replace

import pytest
from pydantic import ValidationError

from ros_robot_studio.backends.domains import ROSDomainManager, validate_domain
from ros_robot_studio.backends.plugins import BackendPluginRegistry
from ros_robot_studio.backends.profiles import DEFAULTS, new_profile
from ros_robot_studio.backends.transport import ReadOnlyRosbridgeTransport
from ros_robot_studio.core.models import Project
from ros_robot_studio.core.robot_state import RobotStateManager
from ros_robot_studio.motion.demos import pick_place_demo


@pytest.mark.parametrize("domain", [0, 1, 2, 3, 4, 10, 232])
def test_custom_backend_domains_are_not_identity(domain):
    a = new_profile("isaac_sim", domain_id=domain)
    b = new_profile("isaac_sim", domain_id=domain)
    assert a.id != b.id and a.domain_id == b.domain_id == domain
    assert (
        Project.model_validate(Project(backends=[a, b], active_backend=a.id).model_dump()).backends[
            1
        ]
        == b
    )


@pytest.mark.parametrize("domain", [-1, 233, True, 1.5, "1"])
def test_invalid_domain_types_rejected(domain):
    with pytest.raises((ValidationError, ValueError)):
        new_profile("ros_demo", domain_id=domain)
    with pytest.raises(ValueError):
        validate_domain(domain)


def test_backend_defaults_and_hardware_gate_survive_project_load():
    for kind, values in DEFAULTS.items():
        assert new_profile(kind).domain_id == values["domain_id"]
    hardware = new_profile("real_robot")
    data = Project(backends=[hardware]).model_dump()
    data["backends"][0]["execution_enabled"] = True
    with pytest.raises(ValidationError, match="read only"):
        Project.model_validate(data)
    with pytest.raises(ValidationError):
        new_profile("isaac_sim", installation_ref="/home/someone/isaacsim")
    with pytest.raises(ValidationError):
        new_profile("isaac_sim", scene_ref="../escape.usd")


def test_shared_domain_and_ephemeral_ports_are_diagnosed():
    a = new_profile("isaac_sim", domain_id=110)
    b = new_profile("isaac_sim", domain_id=110)
    d = ROSDomainManager(ephemeral_range=(32768, 60999))
    report = d.inspect(a, [a, b])
    assert any(r.severity == "WARNING" and "ephemeral" in r.message for r in report)
    assert any(r.severity == "FAIL" for r in report)
    a.shared_domain_confirmed = True
    assert not any(r.severity == "FAIL" for r in d.inspect(a, [a, b]))


def test_plugin_prepared_and_readonly_capabilities():
    registry = BackendPluginRegistry()
    for kind in ("mujoco", "gazebo"):
        with pytest.raises(ValueError, match="Not Implemented"):
            registry.get(kind).get_connection_requirements(new_profile(kind))
    with pytest.raises(ValueError, match="does not support"):
        registry.get("isaac_sim").get_connection_requirements(
            new_profile("isaac_sim", distribution="lyrical")
        )
    p, m, _ = pick_place_demo(prefer_so101=False)
    states = RobotStateManager()
    states.bind(m, p.robot, m.default_configuration())
    cap = registry.get("real_robot").get_execution_capabilities(
        new_profile("real_robot"),
        {"topics": ["/joint_states"], "types": ["sensor_msgs/msg/JointState"]},
        states.snapshot(),
        True,
        True,
    )
    assert "joint_state_feedback" in cap.available and not cap.execution_permitted
    assert "joint_position_control" not in cap.available


def test_readonly_transport_blocks_raw_command_routes(qtbot):
    t = ReadOnlyRosbridgeTransport("ws://127.0.0.1:1")
    for data in (
        {"op": "publish", "topic": "/joint_command"},
        {"op": "advertise", "topic": "/joint_command"},
        {"op": "call_service", "service": "/controller/switch"},
    ):
        with pytest.raises(PermissionError):
            t.send(data)
    with pytest.raises(PermissionError):
        t.publish("/joint_command", "sensor_msgs/msg/JointState", {})
    t.send({"op": "call_service", "service": "/rosapi/topics"})
    t.deleteLater()


def test_background_sources_and_reconnection_never_override_active_state():
    p, m, _ = pick_place_demo(prefer_so101=False)
    s = RobotStateManager()
    s.bind(m, p.robot, m.default_configuration())
    a = s.register_backend("a", 1, "humble")
    b = s.register_backend("b", 4, "jazzy")
    qa = dict(m.default_configuration(), axis_x=0.1)
    qb = dict(qa, axis_x=0.2)
    assert s.update(a, qa, source_timestamp=10)
    s.activate_backend("a")
    assert s.snapshot().ros_domain == 1
    assert s.update(b, qb, source_timestamp=10) and s.snapshot().positions["axis_x"] == 0.1
    s.activate_backend("b")
    assert s.snapshot().positions["axis_x"] == 0.2
    assert s.update(a, qa, source_timestamp=11) and s.snapshot().backend_id == "b"
    assert not s.update(b, qa, source_timestamp=9)
    a2 = s.register_backend("a", 1, "humble")
    assert not s.update(a, qa, source_timestamp=12)
    assert s.update(a2, qa, source_timestamp=12)
    s.bind(m, p.robot, m.default_configuration())
    assert not s.update(a2, qa, source_timestamp=13)


def test_active_backend_publishes_global_monotone_sequences():
    p, m, _ = pick_place_demo(prefer_so101=False)
    s = RobotStateManager()
    s.bind(m, p.robot, m.default_configuration())
    observed = []
    s.subscribe(lambda state: observed.append(state.sequence))
    a = s.register_backend("a", 1, "humble")
    b = s.register_backend("b", 2, "humble")
    for lease in (a, b):
        s.update(lease, m.default_configuration(), source_timestamp=1)
    for i in range(3):
        s.activate_backend("a")
        s.update(a, m.default_configuration(), source_timestamp=i + 2)
        s.activate_backend("b")
        s.update(b, m.default_configuration(), source_timestamp=i + 2)
    assert observed == sorted(set(observed))
    assert replace(s.snapshot(), backend_id="a").instance == s.instance
