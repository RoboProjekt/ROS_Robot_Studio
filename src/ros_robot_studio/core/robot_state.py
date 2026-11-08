"""One thread-safe, model-bound robot state. No Qt, ROS or execution dependencies."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, replace
from types import MappingProxyType
from uuid import uuid4

from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.robots.urdf import pose_rpy


def frozen(values):
    return MappingProxyType(dict(values)) if values is not None else None


@dataclass(frozen=True)
class StateLease:
    instance: str
    epoch: int
    backend: str


@dataclass(frozen=True)
class RobotState:
    instance: str
    model_signature: str
    robot_id: str
    revision: str
    name: str
    urdf: str
    joint_names: tuple[str, ...]
    positions: object
    velocities: object
    accelerations: object
    efforts: object
    pose: tuple[float, ...] | None
    reference_frame: str
    end_effector: str
    timestamp: float
    source_timestamp: float | None
    backend: str
    source: str
    execution_status: str
    valid: bool
    diagnostic: str
    sequence: int
    progress: float = 0.0

    def fields(self):
        fields = {f"position.{n}": v for n, v in self.positions.items()}
        for kind, values in [
            ("velocity", self.velocities),
            ("acceleration", self.accelerations),
            ("effort", self.efforts),
        ]:
            if values is not None:
                fields.update({f"{kind}.{n}": v for n, v in values.items()})
        if self.pose:
            fields.update(
                {f"tool.{n}": v for n, v in zip(("x", "y", "z", "roll", "pitch", "yaw"), self.pose)}
            )
        fields["execution.progress"] = self.progress
        return fields


class RobotStateManager:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.lock = threading.RLock()
        self.listeners = []
        self.model = None
        self.state = None
        self.epoch = 0
        self.instance = ""
        self.last_source_time = None

    def subscribe(self, callback):
        with self.lock:
            self.listeners.append(callback)

    def _publish(self, state):
        self.state = state
        # Called under the RLock: sequence ordering also applies to emitted notifications.
        for listener in tuple(self.listeners):
            listener(state)
        return state

    def bind(self, model, robot, positions, base=None, end=None):
        with self.lock:
            self.model = model
            self.instance = uuid4().hex
            self.epoch += 1
            self.last_source_time = None
            resolved = model.configuration(positions)
            base = base if base in model.links else model.root
            end = (
                end
                if end in model.links
                else (robot.end_effector if robot.end_effector in model.links else model.links[-1])
            )
            transforms = model.transforms(positions)
            import numpy as np

            pose = pose_rpy(np.linalg.inv(transforms[base]) @ transforms[end])
            ordered = []
            links = [model.root]
            for parent in links:
                for joint in model.joints:
                    if joint.parent == parent:
                        links.append(joint.child)
                        if joint.movable:
                            ordered.append(joint.name)
            return self._publish(
                RobotState(
                    self.instance,
                    UniversalKinematicsEngine(model).signature,
                    robot.model_id or model.name,
                    robot.source_revision or "",
                    model.name,
                    robot.urdf or str(model.source_path or ""),
                    tuple(ordered),
                    frozen(resolved),
                    None,
                    None,
                    None,
                    pose,
                    base,
                    end,
                    self.clock(),
                    None,
                    "offline",
                    "manual",
                    "idle",
                    True,
                    "",
                    1,
                )
            )

    def lease(self):
        with self.lock:
            return StateLease(self.instance, self.epoch, self.state.backend)

    def activate_source(self, backend, source):
        if backend not in ("offline", "ros_simulation", "external_simulation"):
            raise ValueError("Hardware state sources are disabled")
        with self.lock:
            self.epoch += 1
            self.last_source_time = None
            state = self.state
            self._publish(
                replace(
                    state,
                    backend=backend,
                    source=source,
                    velocities=None,
                    accelerations=None,
                    efforts=None,
                    timestamp=self.clock(),
                    source_timestamp=None,
                    valid=backend == "offline",
                    diagnostic=""
                    if backend == "offline"
                    else "Waiting for fresh model-matched feedback",
                    execution_status="idle",
                    sequence=state.sequence + 1,
                )
            )
            return self.lease()

    def update(
        self,
        lease,
        positions,
        *,
        velocities=None,
        accelerations=None,
        efforts=None,
        source=None,
        source_timestamp=None,
        status=None,
        progress=None,
    ):
        with self.lock:
            if lease != self.lease():
                return False
            state = self.state
            now = self.clock()
            if not math.isfinite(now):
                raise ValueError("Invalid receive timestamp")
            if source_timestamp is not None:
                if not math.isfinite(source_timestamp) or source_timestamp < 0:
                    raise ValueError("Invalid source timestamp")
                if self.last_source_time is not None and source_timestamp <= self.last_source_time:
                    return False
            independent = set(self.model.default_configuration())
            if set(positions) - independent:
                raise ValueError("Unknown or coupled position joints")
            for values in (positions, velocities, accelerations, efforts):
                if values is not None:
                    if set(values) - set(state.joint_names) or not all(
                        math.isfinite(float(v)) for v in values.values()
                    ):
                        raise ValueError("Invalid joint values")
            missing = independent - set(positions)
            valid = not missing
            resolved = self.model.configuration(positions) if valid else dict(positions)
            if valid:
                for joint in self.model.joints:
                    if not joint.movable or not joint.mimic:
                        continue
                    driver, factor, _ = self.model.driver(joint.name)
                    if velocities is not None and driver in velocities:
                        velocities = dict(velocities, **{joint.name: factor * velocities[driver]})
                    if accelerations is not None and driver in accelerations:
                        accelerations = dict(
                            accelerations, **{joint.name: factor * accelerations[driver]}
                        )
            pose = None
            if valid:
                import numpy as np

                transforms = self.model.transforms(positions)
                pose = pose_rpy(
                    np.linalg.inv(transforms[state.reference_frame])
                    @ transforms[state.end_effector]
                )
            if source_timestamp is not None:
                self.last_source_time = source_timestamp
            self._publish(
                replace(
                    state,
                    positions=frozen(resolved),
                    velocities=frozen(velocities),
                    accelerations=frozen(accelerations),
                    efforts=frozen(efforts),
                    pose=pose,
                    timestamp=now,
                    source_timestamp=source_timestamp,
                    source=source or state.source,
                    execution_status=status or state.execution_status,
                    progress=state.progress
                    if progress is None
                    else min(1.0, max(0.0, float(progress))),
                    valid=valid,
                    diagnostic="Missing joints: " + ", ".join(sorted(missing)) if missing else "",
                    sequence=state.sequence + 1,
                )
            )
            return valid

    def joint_state(self, lease, message, mapping=None):
        """Map ROS names, never array order. Reject malformed, unknown and inconsistent mimic data."""
        with self.lock:
            if lease != self.lease():
                return False
            names = message.get("name", [])
            positions = message.get("position", [])
            if (
                not isinstance(names, list)
                or len(names) > 256
                or not all(isinstance(n, str) for n in names)
                or len(names) != len(set(names))
                or len(names) != len(positions)
            ):
                raise ValueError("JointState names/positions must be unique and aligned")
            mapping = mapping or {n: n for n in self.state.joint_names}
            if len(set(mapping.values())) != len(mapping):
                raise ValueError("Joint mapping must be one-to-one")
            inverse = {remote: local for local, remote in mapping.items()}
            if set(names) - inverse.keys():
                raise ValueError("Unknown JointState names; configure mapping")
            q = {inverse[n]: float(v) for n, v in zip(names, positions)}
            independent = set(self.model.default_configuration())
            supplied = {n: v for n, v in q.items() if n in independent}
            if independent <= supplied.keys():
                resolved = self.model.configuration(supplied)
                if any(abs(resolved[n] - v) > 1e-5 for n, v in q.items()):
                    raise ValueError("Mimic feedback disagrees with URDF")
            dynamic = {}
            for field in ("velocity", "effort"):
                values = message.get(field, [])
                if not isinstance(values, list) or (values and len(values) != len(names)):
                    raise ValueError("Malformed " + field + " array")
                dynamic[field] = (
                    {inverse[n]: float(v) for n, v in zip(names, values)} if values else None
                )
            stamp = message.get("header", {}).get("stamp", {})
            sec, nsec = stamp.get("sec", 0), stamp.get("nanosec", 0)
            if (
                not isinstance(sec, int)
                or not isinstance(nsec, int)
                or sec < 0
                or not 0 <= nsec < 1_000_000_000
            ):
                raise ValueError("Invalid ROS timestamp")
            stamp = sec + nsec * 1e-9 if sec or nsec else None
            return self.update(
                lease,
                supplied,
                velocities=dynamic["velocity"],
                efforts=dynamic["effort"],
                source_timestamp=stamp,
                source="ROS JointState",
            )

    def set_frames(self, base, end):
        with self.lock:
            if base not in self.model.links or end not in self.model.links:
                raise ValueError("Unknown display frame")
            state = self.state
            pose = None
            if state.valid:
                import numpy as np

                transforms = self.model.transforms(
                    {n: state.positions[n] for n in self.model.default_configuration()}
                )
                pose = pose_rpy(np.linalg.inv(transforms[base]) @ transforms[end])
            self._publish(
                replace(
                    state,
                    reference_frame=base,
                    end_effector=end,
                    pose=pose,
                    sequence=state.sequence + 1,
                )
            )

    def execution(self, status, progress=0.0):
        with self.lock:
            if self.state:
                self._publish(
                    replace(
                        self.state,
                        execution_status=status,
                        progress=progress,
                        sequence=self.state.sequence + 1,
                    )
                )

    def invalidate(self, reason, status="error"):
        with self.lock:
            if self.state:
                self._publish(
                    replace(
                        self.state,
                        valid=False,
                        diagnostic=reason,
                        execution_status=status,
                        sequence=self.state.sequence + 1,
                    )
                )

    def stale(self, maximum_age=1.0):
        with self.lock:
            return (
                not self.state
                or not self.state.valid
                or (
                    self.state.backend != "offline"
                    and self.clock() - self.state.timestamp > maximum_age
                )
            )

    def snapshot(self):
        with self.lock:
            return self.state
