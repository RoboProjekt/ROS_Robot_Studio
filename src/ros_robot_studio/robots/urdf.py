"""Strict tree validation and forward kinematics for URDF link/joint structure."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from defusedxml import ElementTree as ET


@dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    xyz: tuple[float, ...]
    rpy: tuple[float, ...]
    axis: tuple[float, ...]
    lower: float | None
    upper: float | None


@dataclass
class RobotModel:
    name: str
    links: list[str]
    joints: list[Joint]
    root: str

    def positions(self, values: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        """Zero-pose or joint-pose link origins, in metres in the root frame."""
        values = values or {}
        transforms = {self.root: np.eye(4)}
        children: dict[str, list[Joint]] = {}
        for joint in self.joints:
            children.setdefault(joint.parent, []).append(joint)
        queue = [self.root]
        for parent in queue:
            for joint in children.get(parent, []):
                roll, pitch, yaw = joint.rpy
                local = np.eye(4)
                local[:3, :3] = (
                    rotation((0, 0, 1), yaw)
                    @ rotation((0, 1, 0), pitch)
                    @ rotation((1, 0, 0), roll)
                )
                local[:3, 3] = joint.xyz
                motion = np.eye(4)
                value = values.get(joint.name, 0.0)
                if joint.kind in ("revolute", "continuous"):
                    motion[:3, :3] = rotation(joint.axis, value)
                elif joint.kind == "prismatic":
                    motion[:3, 3] = np.array(joint.axis) * value
                transforms[joint.child] = transforms[parent] @ local @ motion
                queue.append(joint.child)
        return {name: matrix[:3, 3] for name, matrix in transforms.items()}


def rotation(axis, angle: float) -> np.ndarray:
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def vector(text: str | None, default: str) -> tuple[float, ...]:
    values = tuple(float(v) for v in (text or default).split())
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("URDF vector must contain exactly three finite numbers")
    return values


def parse_urdf(source: Path | str) -> RobotModel:
    text = source.read_text(encoding="utf-8") if isinstance(source, Path) else source
    if len(text) > 5_000_000:
        raise ValueError("URDF exceeds 5 MB limit")
    root = ET.fromstring(text)
    if root.tag != "robot":
        raise ValueError("URDF root must be <robot>")
    links = [link.get("name", "") for link in root.findall("link")]
    if not links or "" in links or len(links) != len(set(links)):
        raise ValueError("URDF links must have unique, non-empty names")
    joints = []
    names, children = set(), set()
    for element in root.findall("joint"):
        name, kind = element.get("name", ""), element.get("type", "")
        if not name or name in names:
            raise ValueError("URDF joint names must be unique and non-empty")
        if kind not in ("fixed", "revolute", "continuous", "prismatic", "floating", "planar"):
            raise ValueError(f"{name}: unknown joint type {kind}")
        names.add(name)
        parent_el, child_el = element.find("parent"), element.find("child")
        if parent_el is None or child_el is None:
            raise ValueError(f"{name}: missing parent or child")
        parent, child = parent_el.get("link"), child_el.get("link")
        if parent not in links or child not in links or parent == child or child in children:
            raise ValueError(f"{name}: invalid link relation or multiple parents")
        children.add(child)
        origin, axis_el, limit = element.find("origin"), element.find("axis"), element.find("limit")
        xyz = vector(origin.get("xyz") if origin is not None else None, "0 0 0")
        rpy = vector(origin.get("rpy") if origin is not None else None, "0 0 0")
        axis = vector(axis_el.get("xyz") if axis_el is not None else None, "1 0 0")
        if np.linalg.norm(axis) == 0:
            raise ValueError(f"{name}: zero joint axis")
        lower = upper = None
        if kind in ("revolute", "prismatic"):
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"{name}: joint limits missing")
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError(f"{name}: invalid limits")
        joints.append(Joint(name, kind, parent, child, xyz, rpy, axis, lower, upper))
    roots = set(links) - children
    if len(roots) != 1:
        raise ValueError("URDF must contain exactly one root link")
    root_name = roots.pop()
    visited, queue = set(), [root_name]
    adjacency = {}
    for joint in joints:
        adjacency.setdefault(joint.parent, []).append(joint.child)
    for current in queue:
        if current in visited:
            raise ValueError("URDF contains a cycle")
        visited.add(current)
        queue.extend(adjacency.get(current, []))
    if visited != set(links):
        raise ValueError("URDF contains disconnected links or a cycle")
    return RobotModel(root.get("name", "Robot"), links, joints, root_name)
