import threading
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import pytest

from ros_robot_studio.core.models import Robot
from ros_robot_studio.core.robot_state import RobotStateManager
from ros_robot_studio.motion.demos import pick_place_demo


@pytest.fixture
def state():
    project, model, plan = pick_place_demo(prefer_so101=False)
    manager = RobotStateManager()
    manager.bind(model, project.robot, model.default_configuration(), "base_link", "tool0")
    return manager, model, plan


def test_manual_state_and_fk_missing_velocity(state):
    manager, model, plan = state
    q = model.default_configuration()
    q["axis_x"] = 0.1
    assert manager.update(manager.lease(), q, source="manual")
    value = manager.snapshot()
    assert value.positions["axis_x"] == 0.1 and value.velocities is None and value.efforts is None
    np.testing.assert_allclose(value.pose[:3], plan.chain.forward_kinematics(q)[:3, 3])
    with pytest.raises(TypeError):
        value.positions["axis_x"] = 3


def test_source_and_model_leases_reject_late_updates(state):
    manager, model, _ = state
    old = manager.lease()
    lease = manager.activate_source("ros_simulation", "/joint_states")
    assert not manager.update(old, model.default_configuration())
    assert manager.update(lease, model.default_configuration(), source_timestamp=10)
    assert not manager.update(
        lease, dict(model.default_configuration(), axis_x=0.1), source_timestamp=9
    )
    assert manager.snapshot().positions["axis_x"] == 0
    manager.bind(model, Robot(), model.default_configuration())
    assert not manager.update(lease, model.default_configuration(), source_timestamp=20)


def test_jointstate_mapping_malformed_and_missing(state):
    manager, model, _ = state
    lease = manager.activate_source("ros_simulation", "/joint_states")
    names = list(reversed(list(model.default_configuration())))
    mapping = {n: "robot/" + n for n in names}
    message = {
        "name": [mapping[n] for n in names],
        "position": [0.01] * len(names),
        "velocity": [0.02] * len(names),
        "effort": [],
        "header": {"stamp": {"sec": 5, "nanosec": 4}},
    }
    assert manager.joint_state(lease, message, mapping)
    assert (
        manager.snapshot().positions["axis_z"] == 0.01
        and manager.snapshot().velocities["axis_x"] == 0.02
    )
    assert manager.snapshot().efforts is None
    for change in (
        {"name": ["x"] * len(names)},
        {"position": [float("nan")] * len(names)},
        {"velocity": [1]},
        {"header": {"stamp": {"sec": -1, "nanosec": 0}}},
    ):
        with pytest.raises(ValueError):
            manager.joint_state(lease, dict(message, **change), mapping)
    assert not manager.joint_state(
        lease, {"name": [mapping["axis_x"]], "position": [0.03]}, mapping
    )
    assert not manager.snapshot().valid and "Missing" in manager.snapshot().diagnostic


def test_stale_and_concurrent_order(state):
    manager, model, _ = state
    seen = []
    manager.subscribe(lambda state: seen.append(state.sequence))
    lease = manager.lease()
    with ThreadPoolExecutor(max_workers=4) as pool:
        assert all(
            pool.map(lambda _: manager.update(lease, model.default_configuration()), range(40))
        )
    assert seen == sorted(seen) and len(set(seen)) == 40
    assert not manager.stale()
    manager.activate_source("ros_simulation", "/joint_states")
    assert manager.stale()
    manager.invalidate("lost connection")
    assert not manager.snapshot().valid


def test_analytical_trajectory_derivatives(state):
    manager, model, plan = state
    segment = next(
        s.trajectory
        for s in plan.steps
        if any(abs(s.trajectory.end[n] - s.trajectory.start[n]) > 0.01 for n in s.trajectory.start)
    )
    t = segment.duration * 0.4
    h = 1e-5
    before, after = segment.sample(t - h), segment.sample(t + h)
    v = segment.velocity(t)
    a = segment.acceleration(t)
    for n in v:
        assert v[n] == pytest.approx((after[n] - before[n]) / (2 * h), abs=1e-7)
        assert a[n] == pytest.approx(
            (segment.velocity(t + h)[n] - segment.velocity(t - h)[n]) / (2 * h), abs=1e-7
        )
    assert manager.update(
        manager.lease(),
        segment.sample(t),
        velocities=v,
        accelerations=a,
        source="offline trajectory",
    )
    assert manager.snapshot().velocities == v


def test_worker_notifications_are_queued_to_qt_thread(qtbot, state):
    from PySide6.QtCore import QObject, QThread, Slot
    from PySide6.QtWidgets import QApplication

    from ros_robot_studio.gui.robot_monitor import StateSignals

    manager, model, _ = state
    bridge = StateSignals()
    manager.subscribe(bridge.changed.emit)
    seen = []

    class Receiver(QObject):
        @Slot(object)
        def receive(self, value):
            seen.append((QThread.currentThread(), value.sequence))

    receiver = Receiver()
    bridge.changed.connect(receiver.receive)
    thread = threading.Thread(
        target=lambda: manager.update(manager.lease(), model.default_configuration())
    )
    thread.start()
    thread.join()
    qtbot.waitUntil(lambda: bool(seen))
    assert seen[0][0] == QApplication.instance().thread()


def test_monitor_never_mixes_receive_and_ros_epoch_timestamps(qtbot, state):
    from ros_robot_studio.gui.robot_monitor import RobotStatePanel

    manager, model, _ = state
    panel = RobotStatePanel()
    qtbot.addWidget(panel)
    manager.subscribe(lambda value: panel.receive(value, model))
    lease = manager.activate_source("ros_simulation", "/joint_states")
    assert not panel.buffer.samples
    manager.update(lease, model.default_configuration(), source_timestamp=1794000000.0)
    manager.update(lease, model.default_configuration(), source_timestamp=1794000000.05)
    assert len(panel.buffer.samples) == 2
    assert panel.buffer.samples[-1][0] - panel.buffer.samples[0][0] == pytest.approx(0.05, abs=1e-6)
    panel.plot.close()
