import copy
import json

import numpy as np
import pytest
from PySide6.QtCore import QObject, Signal

from ros_robot_studio.core.robot_state import RobotStateManager
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.ros_backend import ROS2ExecutionBackend
from ros_robot_studio.motion.ros_converter import TrajectoryToROSConverter
from ros_robot_studio.runtime.controller_core import SimulatedTrajectory
from ros_robot_studio.runtime.robot_config import simulation_config


def test_converter_exactly_preserves_cubic_and_limits():
    project, model, plan = pick_place_demo(prefer_so101=False)
    for step in plan.steps:
        if step.action == "wait":
            continue
        trajectory = step.trajectory
        message = TrajectoryToROSConverter().convert(trajectory, model)
        controller = SimulatedTrajectory(message, model, trajectory.start)
        for t in np.linspace(0, trajectory.duration, 17):
            q, v, a = controller.sample(t)
            for name in q:
                assert q[name] == pytest.approx(trajectory.sample(t)[name], abs=1e-8)
                assert v[name] == pytest.approx(trajectory.velocity(t)[name], abs=1e-7)
                assert a[name] == pytest.approx(trajectory.acceleration(t)[name], abs=1e-6)
        bad = copy.deepcopy(message)
        bad["points"][-1]["time_from_start"] = {"sec": 0, "nanosec": 0}
        with pytest.raises(ValueError):
            SimulatedTrajectory(bad, model, trajectory.start)
    message["joint_names"][0] = "unknown"
    with pytest.raises(ValueError):
        SimulatedTrajectory(message, model, plan.initial)


class Transport(QObject):
    message = Signal(str, dict)
    connection_changed = Signal(bool)

    def __init__(self):
        super().__init__()
        self.connected = True
        self.sent = []

    def subscribe(self, *_):
        pass

    def publish(self, topic, kind, message):
        self.sent.append(
            (topic, json.loads(message["data"]) if topic.endswith("command") else message["data"])
        )


@pytest.fixture
def backend(qtbot):
    project, model, plan = pick_place_demo(prefer_so101=False)
    states = RobotStateManager()
    states.bind(model, project.robot, plan.initial, "base_link", "tool0")
    lease = states.activate_source("ros_simulation", "/joint_states")
    states.update(lease, plan.initial)
    payload = simulation_config(model, plan.initial, project.execution)
    transport = Transport()
    backend = ROS2ExecutionBackend(states)
    backend.connect_backend(transport, model, project.execution, payload, lease)

    def event(kind, **data):
        backend.receive(
            "/studio/execution/events",
            {
                "data": json.dumps(
                    dict(
                        kind=kind,
                        session=payload["session"],
                        model_signature=payload["model_signature"],
                        simulation=payload["profile"],
                        **data,
                    )
                )
            },
        )

    event("controller", available=True)
    yield backend, states, plan, transport, event
    backend.disconnect_backend()


def test_execution_requires_feedback_and_accepted_result(backend):
    b, states, plan, transport, event = backend
    b.execute(plan)
    assert b.status == "executing"
    goal = b.goal_id
    event("accepted", id=goal)
    assert b.status == "executing"
    event("succeeded", id=goal, error_code=0)
    b.tick()
    assert b.index == 1 and b.status == "executing"
    event("rejected", id=b.goal_id, message="no")
    assert b.status == "failed"
    assert transport.sent[-1][1]["operation"] == "cancel"


def test_stale_start_disconnect_and_cancel(backend):
    b, states, plan, transport, event = backend
    states.invalidate("missing")
    with pytest.raises(ValueError):
        b.execute(plan)
    states.update(states.lease(), plan.initial)
    b.execute(plan)
    b.abort()
    assert b.status == "cancelled"
    b.execute(plan)
    transport.connected = False
    b.connection_changed(False)
    assert b.status == "failed" and b.reconnect_required
    transport.connected = True
    b.connection_changed(True)
    assert b.status == "failed"


def test_goal_result_does_not_override_observed_feedback(backend):
    b, states, plan, transport, event = backend
    b.execute(plan)
    # Skip home, then open-gripper endpoint is different from its start.
    event("accepted", id=b.goal_id)
    event("succeeded", id=b.goal_id, error_code=0)
    b.tick()
    goal = b.goal_id
    event("accepted", id=goal)
    event("succeeded", id=goal, error_code=0)
    b.await_feedback_until = b.clock() - 0.1
    b.tick()
    assert b.status == "failed" and not states.snapshot().valid


def test_velocity_violation_and_start_mismatch_rejected():
    project, model, plan = pick_place_demo(prefer_so101=False)
    trajectory = next(s.trajectory for s in plan.steps if s.action == "move_cartesian")
    message = TrajectoryToROSConverter().convert(trajectory, model)
    wrong = dict(trajectory.start, axis_x=0.19)
    with pytest.raises(ValueError, match="Start position"):
        SimulatedTrajectory(message, model, wrong)
    bad = copy.deepcopy(message)
    bad["points"][1]["velocities"][0] = 1000
    with pytest.raises(ValueError):
        SimulatedTrajectory(bad, model, trajectory.start)


def test_timeout_never_counts_publish_as_success(backend):
    b, states, plan, transport, event = backend
    b.execute(plan)
    assert b.status == "executing"
    b.deadline = b.clock() - 1
    b.tick()
    assert b.status == "timeout" and not states.snapshot().valid
    assert transport.sent[-1][1]["operation"] == "cancel"


def test_result_without_acceptance_fails(backend):
    b, states, plan, transport, event = backend
    b.execute(plan)
    event("succeeded", id=b.goal_id, error_code=0)
    assert b.status == "failed"


def test_conversion_failure_cannot_leave_executing(backend, monkeypatch):
    b, states, plan, transport, event = backend

    def fail(*args, **kwargs):
        raise ValueError("invalid mapping")

    monkeypatch.setattr(TrajectoryToROSConverter, "convert", fail)
    b.execute(plan)
    assert b.status == "failed"
    assert not any(
        data.get("operation") == "goal" for topic, data in transport.sent if isinstance(data, dict)
    )
