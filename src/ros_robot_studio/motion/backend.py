"""Deterministic offline execution interface. No hardware or ROS imports."""

import math
from dataclasses import dataclass
from typing import Protocol


class ExecutionBackend(Protocol):
    def connect(self, model, joints, gripper_joints=()): ...
    def disconnect(self): ...
    def read_joint_positions(self) -> dict[str, float]: ...
    def execute(self, trajectory): ...
    def set_gripper(self, values): ...
    def abort(self): ...
    def read_state(self): ...


@dataclass
class PreviewState:
    status: str
    joints: dict[str, float]
    elapsed: float
    step: int
    object_transform: object = None
    attached: bool = False


class OfflinePreviewBackend:
    def __init__(self):
        self.status = "disconnected"
        self.model, self.plan = None, None
        self.joints, self.grippers = {}, set()
        self.elapsed, self.step = 0.0, 0

    def connect(self, model, joints, gripper_joints=()):
        model.configuration(joints)
        self.plan = None
        self.elapsed, self.step = 0.0, 0
        self.model, self.joints, self.grippers = model, dict(joints), set(gripper_joints)
        self.status = "idle"

    def disconnect(self):
        self.abort()
        self.plan = None
        self.status = "disconnected"

    def read_joint_positions(self):
        return dict(self.joints)

    def execute(self, trajectory):
        if self.model is None or self.status == "disconnected":
            raise ValueError("Offline backend is not connected")
        if not trajectory.valid or trajectory.chain.model is not self.model:
            raise ValueError("Only a valid plan for the connected model can be previewed")
        self.plan, self.elapsed = trajectory, 0.0
        self.joints, self.step = trajectory.sample(0)
        self.status = "playing"

    def advance(self, seconds):
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError("Preview time must be finite and cannot run backwards")
        if self.status == "playing":
            self.elapsed = min(self.elapsed + seconds, self.plan.duration)
            self.joints, self.step = self.plan.sample(self.elapsed)
            if self.elapsed >= self.plan.duration:
                self.status = "finished"
        return self.read_state()

    def pause(self):
        if self.status == "playing":
            self.status = "paused"

    def resume(self):
        if self.status == "paused":
            self.status = "playing"

    def abort(self):
        if self.status != "disconnected":
            self.status = "stopped"

    def reset(self):
        if self.plan:
            self.elapsed = 0.0
            self.joints, self.step = self.plan.sample(0)
            self.status = "idle"

    def set_gripper(self, values):
        if set(values) - self.grippers:
            raise ValueError("Joint is not mapped to the offline gripper")
        updated = dict(self.joints, **values)
        self.model.configuration(updated)
        self.joints = updated

    def read_state(self):
        transform, attached = (
            self.plan.object_pose(self.elapsed, self.joints) if self.plan else (None, False)
        )
        return PreviewState(
            self.status, dict(self.joints), self.elapsed, self.step, transform, attached
        )


class UnconfiguredExecutionBackend:
    def __init__(self, name):
        self.name = name

    def __getattr__(self, name):
        raise NotImplementedError(
            f"{self.name} execution backend is planned, not connected or authorized"
        )
