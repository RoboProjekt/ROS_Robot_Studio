"""Reusable taught positions and future framed pose-provider contracts."""

from typing import Protocol
from uuid import uuid4

from ros_robot_studio.motion.schema import CartesianPose, SavedPosition
from ros_robot_studio.robots.urdf import pose_rpy


class PositionManager:
    def __init__(self, positions=None):
        self.positions = [p.model_copy(deep=True) for p in (positions or [])]

    def get(self, identifier):
        for position in self.positions:
            if position.id == identifier:
                return position
        raise ValueError(f"Saved position not found: {identifier}")

    def add(self, position):
        position = SavedPosition.model_validate(position.model_dump())
        if any(
            p.name.casefold() == position.name.casefold() and p.id != position.id
            for p in self.positions
        ):
            raise ValueError("A saved position with this name already exists")
        for index, existing in enumerate(self.positions):
            if existing.id == position.id:
                self.positions[index] = position.model_copy(deep=True)
                return
        self.positions.append(position.model_copy(deep=True))

    def remove(self, identifier, tasks=()):
        if any(
            task.home_position_id == identifier
            or any(step.position_id == identifier for step in task.steps)
            for task in tasks
        ):
            raise ValueError("Position is referenced by a task; remove those steps first")
        self.get(identifier)
        self.positions = [p for p in self.positions if p.id != identifier]

    def duplicate(self, identifier):
        original = self.get(identifier)
        names = {p.name.casefold() for p in self.positions}
        name = original.name[:90] + " copy"
        number = 2
        while name.casefold() in names:
            name = original.name[:90] + f" copy {number}"
            number += 1
        copy = original.model_copy(deep=True, update={"id": str(uuid4()), "name": name})
        self.add(copy)
        return copy

    def capture_joint(self, name, chain, joints):
        chain.model.configuration(joints)
        return SavedPosition(
            name=name, kind="joint", model_signature=chain.engine.signature, joints=dict(joints)
        )

    def capture_cartesian(self, name, chain, joints, mode="position_only"):
        pose = pose_rpy(chain.forward_kinematics(joints))
        return SavedPosition(
            name=name,
            kind="cartesian",
            pose=CartesianPose(
                frame=chain.base_link,
                end_effector=chain.end_effector,
                position=pose[:3],
                rpy=pose[3:],
                mode=mode,
            ),
        )

    def resolve_joint(self, position, chain, current):
        if position.kind != "joint" or position.model_signature != chain.engine.signature:
            raise ValueError(
                "Joint position belongs to another robot model; re-teach it explicitly"
            )
        unknown = set(position.joints) - set(chain.engine.bounds())
        if unknown:
            raise ValueError(
                "Saved position contains unknown or coupled joints: " + ", ".join(unknown)
            )
        q = dict(current, **position.joints)
        chain.model.configuration(q)
        return q


class PoseProvider(Protocol):
    def get_pose(self, target_id: str) -> CartesianPose: ...


class ManualPoseProvider:
    def __init__(self, poses):
        self.poses = poses

    def get_pose(self, target_id):
        if target_id not in self.poses:
            raise ValueError("Manual target not found")
        return self.poses[target_id].model_copy(deep=True)


class SavedPoseProvider:
    def __init__(self, positions: PositionManager):
        self.positions = positions

    def get_pose(self, target_id):
        position = self.positions.get(target_id)
        if position.kind != "cartesian":
            raise ValueError("Target is not a framed Cartesian pose")
        return position.pose.model_copy(deep=True)


class TFFramePoseProvider:
    def get_pose(self, target_id):
        raise NotImplementedError("External TF pose provider is not configured")


class VisionPoseProvider:
    def __init__(self, estimates=None):
        self.estimates = estimates

    def get_position(self, target_id):
        if self.estimates is None or target_id not in self.estimates:
            raise ValueError("No valid current vision estimate for this target")
        return self.estimates[target_id]

    def get_pose(self, target_id):
        if self.estimates is not None:
            raise ValueError("A planar estimate needs explicit tool orientation and a grasp offset")
        raise NotImplementedError(
            "Vision localization needs calibration and a 3D pose provider; 2D detection alone is insufficient"
        )

    def draft(self, target_id, end_effector, rpy, offset):
        estimate = self.get_position(target_id)
        return CartesianPose(
            frame=estimate.frame_id,
            end_effector=end_effector,
            rpy=rpy,
            position=tuple(position + delta for position, delta in zip(estimate.position, offset)),
        )
