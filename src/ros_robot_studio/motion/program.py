"""Sequential task compilation. IK, trajectories, collision and execution stay separate."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np

from ros_robot_studio.kinematics.solvers import KinematicsSolverManager
from ros_robot_studio.motion.positions import PositionManager
from ros_robot_studio.motion.schema import MotionTask, SavedPosition
from ros_robot_studio.motion.trajectory import JointTrajectory, TrajectoryGenerator
from ros_robot_studio.robots.urdf import origin_transform

COLLISION_NOTICE = "Collision checking not available – preview only"


class CollisionValidator(Protocol):
    def validate(self, trajectory) -> tuple[bool, str]: ...


class UnavailableCollisionValidator:
    def validate(self, trajectory):
        return False, COLLISION_NOTICE


@dataclass
class CompiledStep:
    step_id: str
    name: str
    trajectory: JointTrajectory
    start_time: float
    action: str
    attach: bool = False
    release: bool = False
    tool_at_end: np.ndarray | None = None

    @property
    def end_time(self):
        return self.start_time + self.trajectory.duration


@dataclass
class CompiledTask:
    source: MotionTask
    chain: object
    initial: dict[str, float]
    steps: list[CompiledStep] = field(default_factory=list)
    errors: dict[int, str] = field(default_factory=dict)
    notes: list[str] = field(
        default_factory=lambda: [
            COLLISION_NOTICE,
            "Joint-space interpolation; no straight Cartesian path or contact physics",
        ]
    )
    object_initial: np.ndarray | None = None

    @property
    def valid(self):
        return not self.errors and len(self.steps) == len(self.source.steps) and bool(self.steps)

    @property
    def hardware_ready(self):
        return False

    @property
    def duration(self):
        return self.steps[-1].end_time if self.steps else 0.0

    def sample(self, seconds):
        if not self.valid:
            raise ValueError("Cannot sample an invalid task")
        for index, step in enumerate(self.steps):
            if seconds < step.end_time or index == len(self.steps) - 1:
                return step.trajectory.sample(seconds - step.start_time), index
        return dict(self.initial), 0

    def object_pose(self, seconds, joints):
        if self.object_initial is None:
            return None, False
        transform = self.object_initial.copy()
        attachment = None
        for step in self.steps:
            if step.end_time > seconds:
                break
            if not step.attach and not step.release:
                continue
            tool = step.tool_at_end
            if attachment is not None:
                transform = tool @ attachment
            if step.release:
                attachment = None
            if step.attach:
                attachment = np.linalg.inv(tool) @ transform
        if attachment is not None:
            tool = self.chain.model.transforms(joints)[self.chain.end_effector]
            transform = tool @ attachment
        return transform, attachment is not None


class TaskCompiler:
    def __init__(
        self, chain, positions: PositionManager, options=None, solver="auto", collision=None
    ):
        self.chain, self.positions = chain, positions
        self.options, self.solver = options, solver
        self.generator = TrajectoryGenerator(chain.engine)
        self.collision = collision or UnavailableCollisionValidator()

    def _ik(self, pose, current, mode, axis="Z", cancel=None):
        result = KinematicsSolverManager().solve(
            self.chain,
            pose,
            current,
            solver=self.solver,
            mode=mode,
            axis=axis,
            options=self.options,
            cancel=cancel,
        )
        if not result.success:
            raise ValueError(result.message)
        return result.joint_positions

    def resolve_position(self, position: SavedPosition, current, cancel=None):
        if position.kind == "joint":
            return self.positions.resolve_joint(position, self.chain, current)
        return self._ik(position.pose, current, position.pose.mode, position.pose.axis, cancel)

    def compile(self, task: MotionTask, initial_joints, cancel=None, progress=lambda message: None):
        task = MotionTask.model_validate(task.model_dump())
        self.chain.model.configuration(initial_joints)
        current = self.chain.engine.seed(initial_joints)
        compiled = CompiledTask(task.model_copy(deep=True), self.chain, dict(current))
        elapsed = 0.0
        if task.virtual_object:
            if task.virtual_object.frame not in self.chain.model.links:
                compiled.errors[0] = "No known transform for the virtual object reference frame"
                return compiled
            reference = self.chain.model.transforms(current)[task.virtual_object.frame]
            compiled.object_initial = reference @ origin_transform(
                task.virtual_object.position, task.virtual_object.rpy
            )
        for index, step in enumerate(task.steps):
            if cancel is not None and cancel.is_set():
                compiled.errors[index] = "Task compilation cancelled"
                break
            try:
                progress(f"{index + 1}/{len(task.steps)} · {step.name or step.action}")
                target = dict(current)
                if step.action in ("move_joint", "move_cartesian"):
                    position = self.positions.get(step.position_id)
                    if position.kind != ("joint" if step.action == "move_joint" else "cartesian"):
                        raise ValueError("Action and saved position type disagree")
                    target = self.resolve_position(position, current, cancel)
                elif step.action == "home":
                    reference = step.position_id or task.home_position_id
                    if reference:
                        position = self.positions.get(reference)
                        if position.kind != "joint":
                            raise ValueError("Home must be an explicitly taught joint position")
                        target = self.positions.resolve_joint(position, self.chain, current)
                    else:
                        zero = self.chain.engine.seed()
                        target.update({name: zero[name] for name in self.chain.active_joints})
                elif step.action in ("open_gripper", "close_gripper"):
                    if not step.gripper_values or set(step.gripper_values) - set(
                        self.chain.gripper_joints
                    ):
                        raise ValueError(
                            "Gripper action requires confirmed gripper joints and explicit URDF SI values"
                        )
                    target.update(step.gripper_values)
                    self.chain.model.configuration(target)
                elif step.action == "move_relative":
                    reference = self.chain.frame_transform(
                        step.reference_frame or self.chain.base_link, current
                    )
                    actual = self.chain.forward_kinematics(current)
                    desired = actual.copy()
                    desired[:3, 3] += reference[:3, :3] @ np.array(step.relative_xyz)
                    relative_rotation = origin_transform((0, 0, 0), step.relative_rpy)[:3, :3]
                    desired[:3, :3] = (
                        reference[:3, :3] @ relative_rotation @ reference[:3, :3].T @ actual[:3, :3]
                    )
                    mode = "position_only" if not any(step.relative_rpy) else "full_pose"
                    target = self._ik(desired, current, mode, cancel=cancel)
                elif step.action != "wait":
                    raise ValueError("Unsupported action")
                if step.attach_object and step.action != "close_gripper":
                    raise ValueError(
                        "Object attachment is only available for a close-gripper action"
                    )
                if step.release_object and step.action != "open_gripper":
                    raise ValueError("Object release is only available for an open-gripper action")
                if step.action == "wait":
                    trajectory = JointTrajectory(dict(current), dict(current), step.duration)
                else:
                    trajectory = self.generator.generate(
                        current, target, task.settings, step.duration
                    )
                name = step.name or step.action.replace("_", " ").title()
                compiled.steps.append(
                    CompiledStep(
                        step.id,
                        name,
                        trajectory,
                        elapsed,
                        step.action,
                        step.attach_object,
                        step.release_object,
                        self.chain.model.transforms(trajectory.end)[self.chain.end_effector],
                    )
                )
                elapsed += trajectory.duration
                current = dict(trajectory.end)
                if trajectory.demo_limits:
                    compiled.notes.append(
                        "Missing URDF velocity limits: demo-only fallback for "
                        + ", ".join(trajectory.demo_limits)
                    )
                if trajectory.unchecked_accelerations:
                    compiled.notes.append(
                        "Acceleration limits not supplied for "
                        + ", ".join(trajectory.unchecked_accelerations)
                    )
            except (ValueError, RuntimeError, NotImplementedError) as exc:
                compiled.errors[index] = str(exc)
                break
        compiled.notes = list(dict.fromkeys(compiled.notes))
        return compiled
