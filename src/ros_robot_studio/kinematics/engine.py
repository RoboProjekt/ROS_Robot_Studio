"""Robot-neutral chain analysis using the existing URDF/FK implementation."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from ros_robot_studio.robots.urdf import (
    RobotModel,
    UnsupportedKinematicsError,
    origin_transform,
    parse_urdf,
)


@dataclass(frozen=True)
class ChainAnalysis:
    base_link: str
    end_effector: str
    joints: tuple[str, ...]
    active_joints: tuple[str, ...]
    max_position_rank: int
    max_pose_rank: int
    max_axis_rank: int
    candidates: tuple[str, ...]
    notes: tuple[str, ...]

    @property
    def full_pose_available(self) -> bool:
        return self.max_pose_rank == 6


class UniversalKinematicsEngine:
    def __init__(self, model: RobotModel):
        self.model = model
        self.joints = {joint.name: joint for joint in model.joints}
        self.parents = {joint.child: joint for joint in model.joints}
        self.signature = hashlib.sha256(
            json.dumps(
                dict(name=model.name, links=model.links, joints=[asdict(j) for j in model.joints]),
                sort_keys=True,
            ).encode()
        ).hexdigest()
        parents = {j.parent for j in model.joints}
        self.end_effector_candidates = tuple(link for link in model.links if link not in parents)

    @classmethod
    def load_robot(cls, path: Path | str):
        return cls(parse_urdf(Path(path)))

    def driver(self, name: str) -> tuple[str, float, float]:
        return self.model.driver(name)

    def bounds(self) -> dict[str, tuple[float, float]]:
        return dict(self.model.effective_joint_bounds)

    def seed(self, supplied=None) -> dict[str, float]:
        supplied = supplied or {}
        unknown = set(supplied) - self.joints.keys()
        if unknown:
            raise ValueError("Unknown joints: " + ", ".join(sorted(unknown)))
        bounds = self.bounds()
        q = {}
        for name, (lo, hi) in bounds.items():
            value = float(supplied.get(name, 0.0))
            if not math.isfinite(value):
                raise ValueError("Joint values must be finite")
            q[name] = float(np.clip(value, lo, hi))
        self.model.configuration(q)
        return q

    def create_chain(
        self, base_link: str, end_effector: str, active_joints=None, gripper_joints=()
    ):
        if base_link not in self.model.links or end_effector not in self.model.links:
            raise ValueError("Unknown base or end-effector frame")
        path, link = [], end_effector
        while link != base_link:
            joint = self.parents.get(link)
            if joint is None:
                raise ValueError("Base frame must be an ancestor of the end effector")
            if joint.kind in ("floating", "planar"):
                raise UnsupportedKinematicsError(
                    f"{joint.kind} joints are not supported by the serial-chain solver"
                )
            path.append(joint)
            link = joint.parent
        path.reverse()
        eligible = []
        for joint in path:
            if joint.movable:
                name = self.driver(joint.name)[0]
                if (
                    name not in eligible
                    and name not in gripper_joints
                    and self.bounds()[name][0] < self.bounds()[name][1]
                ):
                    eligible.append(name)
        active = eligible if active_joints is None else list(active_joints)
        if len(active) != len(set(active)) or set(active) - set(eligible):
            raise ValueError(
                "Active arm joints must be independent chain drivers; gripper joints are excluded"
            )
        if any(
            name not in self.joints or not self.joints[name].movable or self.joints[name].mimic
            for name in gripper_joints
        ):
            raise ValueError("Gripper mapping requires independent movable URDF joints")
        return KinematicChain(
            self, base_link, end_effector, tuple(path), tuple(active), tuple(gripper_joints)
        )


class KinematicChain:
    def __init__(self, engine, base, end, path, active, grippers):
        self.engine, self.model = engine, engine.model
        self.base_link, self.end_effector = base, end
        self.path, self.active_joints, self.gripper_joints = path, active, grippers
        self.lower = np.array([engine.bounds()[name][0] for name in active])
        self.upper = np.array([engine.bounds()[name][1] for name in active])
        self._analysis = None

    def forward_kinematics(self, joint_positions=None):
        transforms = self.model.transforms(joint_positions)
        return np.linalg.inv(transforms[self.base_link]) @ transforms[self.end_effector]

    def frame_transform(self, reference: str, joint_positions):
        if reference not in self.model.links:
            raise ValueError(
                f"No known transform for frame {reference!r}; camera/world TF is not configured"
            )
        transforms = self.model.transforms(joint_positions)
        return np.linalg.inv(transforms[self.base_link]) @ transforms[reference]

    def target_matrix(self, pose, joint_positions):
        if pose.end_effector != self.end_effector:
            raise ValueError("Cartesian target end effector differs from the selected chain")
        return self.frame_transform(pose.frame, joint_positions) @ origin_transform(
            pose.position, pose.rpy
        )

    def geometric_jacobian(self, positions, task="full_pose", axis="Z"):
        q = self.engine.seed(positions)
        current = self.forward_kinematics(q)
        columns = []
        for index, name in enumerate(self.active_joints):
            step = min(1e-6, (self.upper[index] - q[name]) / 2)
            if step < 1e-9:
                step = -min(1e-6, (q[name] - self.lower[index]) / 2)
            if not math.isfinite(step) or abs(step) < 1e-12:
                step = 1e-6
            changed = self.forward_kinematics(dict(q, **{name: q[name] + step}))
            dp = (changed[:3, 3] - current[:3, 3]) / step
            dr = (changed[:3, :3] @ current[:3, :3].T - np.eye(3)) / step
            angular = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]]) / 2
            if task == "position_only":
                columns.append(dp)
            elif task == "axis":
                axis_index = "XYZ".index(axis)
                columns.append(
                    np.r_[dp, (changed[:3, axis_index] - current[:3, axis_index]) / step]
                )
            else:
                columns.append(np.r_[dp, angular])
        dimension = 3 if task == "position_only" else 6
        return np.column_stack(columns) if columns else np.zeros((dimension, 0))

    def conditioning(self, positions, mode="position_only", axis="Z"):
        matrix = self.geometric_jacobian(positions, mode, axis)
        values = np.linalg.svd(matrix, compute_uv=False)
        rank = int(np.count_nonzero(values > 1e-6))
        expected = min(
            len(self.active_joints), 3 if mode == "position_only" else 5 if mode == "axis" else 6
        )
        smallest = float(values[rank - 1]) if rank else 0.0
        condition = float(values[0] / smallest) if rank and rank >= expected else float("inf")
        return rank, condition, smallest

    def analyze(self):
        if self._analysis:
            return self._analysis
        ranks = [0, 0, 0]
        rng = np.random.default_rng(729)
        for trial in range(13):
            q = self.engine.seed()
            for index, name in enumerate(self.active_joints):
                lo = self.lower[index] if np.isfinite(self.lower[index]) else -math.pi
                hi = self.upper[index] if np.isfinite(self.upper[index]) else math.pi
                q[name] = float(np.clip(0 if trial == 0 else rng.uniform(lo, hi), lo, hi))
            for i, mode in enumerate(("position_only", "full_pose", "axis")):
                rank, _, _ = self.conditioning(q, mode)
                ranks[i] = max(ranks[i], rank)
        notes = []
        if ranks[1] < 6:
            notes.append(
                f"Maximum sampled pose rank {ranks[1]}/6: independent full-pose control is unavailable"
            )
        if len(self.engine.end_effector_candidates) > 1:
            notes.append("Several terminal frames exist; confirm arm/tool/gripper assignment")
        self._analysis = ChainAnalysis(
            self.base_link,
            self.end_effector,
            tuple(j.name for j in self.path),
            self.active_joints,
            ranks[0],
            ranks[1],
            ranks[2],
            self.engine.end_effector_candidates,
            tuple(notes),
        )
        return self._analysis

    def inverse_kinematics(self, target_pose, initial_guess=None, mode="position_only", **kwargs):
        from ros_robot_studio.kinematics.solvers import KinematicsSolverManager

        return KinematicsSolverManager().solve(
            self, target_pose, initial_guess, mode=mode, **kwargs
        )
