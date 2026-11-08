"""Simulation-only sequencer. All motion feedback comes back through real ROS JointStates."""

from __future__ import annotations

import json
import time
from uuid import uuid4

from PySide6.QtCore import QObject, QTimer, Signal

from ros_robot_studio.motion.ros_converter import TrajectoryToROSConverter


class ROS2ExecutionBackend(QObject):
    changed = Signal(str)
    error = Signal(str)

    def __init__(self, state_manager, parent=None, clock=time.monotonic):
        super().__init__(parent)
        self.states = state_manager
        self.clock = clock
        self.transport = None
        self.status = "not_connected"
        self.controller = False
        self.remote_busy = False
        self.controller_time = 0.0
        self.plan = None
        self.index = 0
        self.goal_id = None
        self.elapsed = 0.0
        self.wait_until = None
        self.await_feedback_until = None
        self.session = ""
        self.model = None
        self.lease = None
        self.config = None
        self.last_feedback = 0.0
        self.deadline = 0.0
        self.reconnect_required = False
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)

    def connect_backend(self, transport, model, config, payload, lease):
        self.disconnect_backend()
        self.transport = transport
        self.model = model
        self.config = config
        self.session = payload["session"]
        self.signature = payload["model_signature"]
        self.lease = lease
        self.status = "connected"
        self.reconnect_required = False
        transport.subscribe("/studio/execution/events", "std_msgs/msg/String")
        transport.message.connect(self.receive)
        transport.connection_changed.connect(self.connection_changed)
        self.timer.start()
        self.changed.emit(self.status)

    def connection_changed(self, connected):
        if not connected:
            self.controller = False
            if self.status == "executing":
                self.fail("ROS connection lost; execution aborted; no automatic continuation")
            else:
                self.status = "not_connected"
                self.changed.emit(self.status)
        elif self.reconnect_required:
            self.status = "failed"
            self.changed.emit("Reconnect requires explicit runtime reconnection")
        else:
            self.status = "connected"
            self.changed.emit(self.status)

    def disconnect_backend(self):
        if self.transport:
            self.abort()
            try:
                self.transport.message.disconnect(self.receive)
                self.transport.connection_changed.disconnect(self.connection_changed)
            except RuntimeError:
                pass
        self.transport = None
        self.timer.stop()
        self.controller = False
        self.status = "not_connected"
        self.goal_id = None
        self.plan = None

    def ready(self):
        return bool(
            self.transport
            and self.transport.connected
            and self.controller
            and self.clock() - self.controller_time < 1.5
            and not self.states.stale(self.config.stale_timeout)
            and not self.reconnect_required
        )

    def send(self, operation, **data):
        if not self.transport or not self.transport.connected:
            raise ConnectionError("ROS transport unavailable")
        payload = dict(
            operation=operation, session=self.session, model_signature=self.signature, **data
        )
        self.transport.publish(
            "/studio/execution/command", "std_msgs/msg/String", {"data": json.dumps(payload)}
        )

    def execute(self, plan):
        if self.config.profile not in ("managed_simulation", "external_simulation"):
            raise ValueError("Hardware execution is disabled")
        if (
            self.config.profile == "external_simulation"
            and not self.config.external_simulation_confirmed
        ):
            raise ValueError("External simulation must be explicitly confirmed")
        if self.status == "executing" or self.remote_busy:
            raise ValueError("Controller is busy; wait for cancellation/result before restarting")
        if not self.ready():
            raise ValueError(
                "Fresh robot feedback and an available simulation controller are required"
            )
        if not plan.valid or plan.chain.model is not self.model or plan.hardware_ready:
            raise ValueError("Only validated model-matched simulation plans are allowed")
        q = self.read_joint_positions()
        if any(abs(q[n] - v) > self.config.start_tolerance for n, v in plan.initial.items()):
            raise ValueError("ROS start differs from plan; replan using fresh feedback")
        self.plan = plan
        self.index = 0
        self.elapsed = 0.0
        self.status = "executing"
        self.changed.emit(self.status)
        self.next_step()

    def next_step(self):
        if self.status != "executing":
            return
        if self.index >= len(self.plan.steps):
            self.status = "succeeded"
            self.changed.emit(self.status)
            return
        if not self.ready():
            self.fail("Controller or state unavailable before next step")
            return
        step = self.plan.steps[self.index]
        if any(
            abs(self.read_joint_positions()[n] - v) > self.config.start_tolerance
            for n, v in step.trajectory.start.items()
        ):
            self.fail("Step start deviates from feedback; replan required")
            return
        self.await_feedback_until = None
        self.wait_until = None
        self.goal_id = None
        if step.action == "wait":
            self.wait_until = self.clock() + step.trajectory.duration
            return
        mapping = self.config.joint_mapping or {n: n for n in step.trajectory.start}
        mapping = {n: mapping.get(n, n) for n in step.trajectory.start}
        message = TrajectoryToROSConverter().convert(step.trajectory, self.model, mapping)
        self.goal_id = uuid4().hex
        self.deadline = self.clock() + step.trajectory.duration + 10
        self.last_feedback = self.clock()
        self.accepted = False
        try:
            self.send("goal", id=self.goal_id, trajectory=message)
        except (ValueError, ConnectionError) as exc:
            self.fail(str(exc))

    def receive(self, topic, message):
        if topic != "/studio/execution/events" or not self.transport:
            return
        try:
            data = json.loads(message.get("data", ""))
        except (ValueError, TypeError):
            return
        if data.get("session") != self.session or data.get("model_signature") != self.signature:
            return
        if data.get("simulation") != self.config.profile:
            return
        kind = data.get("kind")
        if kind == "controller":
            self.controller = bool(data.get("available"))
            self.remote_busy = bool(data.get("busy"))
            self.controller_time = self.clock()
            if self.status in ("connected", "ready"):
                self.status = "ready" if self.ready() else "connected"
                self.changed.emit(self.status)
            return
        if self.status != "executing" or data.get("id") != self.goal_id:
            return
        if kind == "accepted":
            self.accepted = True
        elif kind == "feedback":
            self.last_feedback = self.clock()
            self.elapsed = self.plan.steps[self.index].start_time + max(
                0.0, float(data.get("elapsed", 0))
            )
            self.changed.emit(self.status)
        elif kind == "succeeded":
            if not self.accepted:
                self.fail("Result received without an accepted action goal")
                return
            if data.get("error_code") != 0:
                self.fail("Action result reports failure")
                return
            self.await_feedback_until = self.clock() + self.config.stale_timeout
        elif kind in ("rejected", "failed", "cancelled"):
            self.fail(f"{kind}: " + data.get("message", "Action did not succeed"))

    def tick(self):
        if not self.transport:
            return
        if self.transport.connected and not self.reconnect_required:
            self.transport.publish(
                "/studio/execution/heartbeat", "std_msgs/msg/String", {"data": self.session}
            )
        if self.status == "connected" and self.ready():
            self.status = "ready"
            self.changed.emit(self.status)
        if self.status != "executing":
            return
        if not self.ready():
            self.fail("Stale robot state or controller feedback")
            return
        now = self.clock()
        if self.wait_until is not None:
            self.elapsed = self.plan.steps[self.index].end_time - max(0.0, self.wait_until - now)
            self.changed.emit(self.status)
            if now >= self.wait_until:
                self.complete_step()
        elif self.await_feedback_until is not None:
            end = self.plan.steps[self.index].trajectory.end
            actual = self.read_joint_positions()
            if all(abs(actual[n] - v) <= self.config.goal_tolerance for n, v in end.items()):
                self.complete_step()
            elif now > self.await_feedback_until:
                self.fail("Action succeeded but observed end state is outside tolerance")
        elif self.goal_id and (now > self.deadline or now - self.last_feedback > 3):
            self.fail("Execution timeout / missing action feedback", timeout=True)

    def complete_step(self):
        self.elapsed = self.plan.steps[self.index].end_time
        self.index += 1
        self.changed.emit(self.status)
        self.next_step()

    def abort(self):
        if self.status == "executing":
            try:
                self.send("cancel")
            except ConnectionError:
                pass
            self.status = "cancelled"
            self.goal_id = None
            self.wait_until = None
            self.changed.emit(self.status)

    def fail(self, message, timeout=False):
        try:
            self.send("cancel")
        except ConnectionError:
            pass
        self.status = "timeout" if timeout else "failed"
        self.goal_id = None
        self.reconnect_required = True
        self.states.invalidate(message, self.status)
        self.error.emit(message)
        self.changed.emit(self.status)

    def read_joint_positions(self):
        state = self.states.snapshot()
        return {
            n: state.positions[n]
            for n in self.model.default_configuration()
            if n in state.positions
        }

    def read_state(self):
        return self.states.snapshot()

    def set_gripper(self, values):
        raise ValueError(
            "Use a validated gripper trajectory step; direct hardware commands are disabled"
        )
