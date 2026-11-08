"""Safe URDF parsing and one shared homogeneous-transform kinematics implementation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path

import numpy as np
from defusedxml import ElementTree as ET


class UnsupportedKinematicsError(ValueError):
    """The URDF describes a mechanism that the open-chain implementation cannot handle."""


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
    mimic: str | None = None
    multiplier: float = 1.0
    offset: float = 0.0
    velocity: float | None = None
    effort: float | None = None
    acceleration: float | None = None

    @property
    def movable(self) -> bool:
        return self.kind in ("revolute", "continuous", "prismatic")


@dataclass(frozen=True)
class Visual:
    link: str
    xyz: tuple[float, ...]
    rpy: tuple[float, ...]
    mesh: str | None = None
    scale: tuple[float, ...] = (1, 1, 1)
    color: tuple[float, ...] = (0.55, 0.75, 0.78, 1.0)
    primitive: str | None = None
    dimensions: tuple[float, ...] = ()

    def local_transform(self) -> np.ndarray:
        transform = origin_transform(self.xyz, self.rpy)
        transform[:3, :3] = transform[:3, :3] @ np.diag(self.scale)
        return transform


@dataclass(frozen=True)
class MeshGeometry:
    vertices: np.ndarray
    faces: np.ndarray


@dataclass
class RobotModel:
    name: str
    links: list[str]
    joints: list[Joint]
    root: str
    visuals: list[Visual] = field(default_factory=list)
    mesh_references: list[str] = field(default_factory=list)
    meshes: dict[str, MeshGeometry] = field(default_factory=dict)
    warnings: list[str] = field(default_factory=list)
    source_path: Path | None = None

    @cached_property
    def joint_by_name(self):
        return {joint.name: joint for joint in self.joints}

    def driver(self, name: str) -> tuple[str, float, float]:
        joint = self.joint_by_name[name]
        if not joint.mimic:
            return name, 1.0, 0.0
        source, factor, offset = self.driver(joint.mimic)
        return source, factor * joint.multiplier, offset * joint.multiplier + joint.offset

    @cached_property
    def effective_joint_bounds(self) -> dict[str, tuple[float, float]]:
        bounds = {
            j.name: (
                j.lower if j.lower is not None else -np.inf,
                j.upper if j.upper is not None else np.inf,
            )
            for j in self.joints
            if j.movable and not j.mimic
        }
        for joint in self.joints:
            if not joint.movable or not joint.mimic:
                continue
            source, factor, offset = self.driver(joint.name)
            lower, upper = joint.lower, joint.upper
            if factor == 0:
                if (lower is not None and offset < lower) or (upper is not None and offset > upper):
                    raise UnsupportedKinematicsError("Inconsistent mimic joint limits")
                continue
            lo = (
                ((lower - offset) / factor)
                if lower is not None
                else (-np.inf if factor > 0 else np.inf)
            )
            hi = (
                ((upper - offset) / factor)
                if upper is not None
                else (np.inf if factor > 0 else -np.inf)
            )
            lo, hi = min(lo, hi), max(lo, hi)
            old = bounds[source]
            bounds[source] = (max(old[0], lo), min(old[1], hi))
        if any(lo > hi for lo, hi in bounds.values()):
            raise UnsupportedKinematicsError("Coupled joint limits have no feasible interval")
        return bounds

    def default_configuration(self) -> dict[str, float]:
        """Feasible pose nearest zero, respecting independent and coupled joint limits."""
        return {
            name: max(lo, min(hi, 0.0)) for name, (lo, hi) in self.effective_joint_bounds.items()
        }

    def configuration(self, values: dict[str, float] | None = None) -> dict[str, float]:
        """Validate SI values and evaluate mimic relationships, including chained mimics."""
        joints = {joint.name: joint for joint in self.joints}
        supplied = values or {}
        if set(supplied) - joints.keys():
            raise ValueError("Unknown joints: " + ", ".join(sorted(set(supplied) - joints.keys())))
        result = self.default_configuration()
        for name, value in supplied.items():
            joint = joints[name]
            if not joint.movable or joint.mimic:
                raise ValueError(
                    f"{name}: fixed, unsupported or coupled joints cannot be set directly"
                )
            result[name] = float(value)
        resolving = set()

        def evaluate(name):
            joint = joints[name]
            if name in resolving:
                raise ValueError("Cyclic mimic relationship")
            if joint.mimic:
                resolving.add(name)
                result[name] = joint.multiplier * evaluate(joint.mimic) + joint.offset
                resolving.remove(name)
            value = result.get(name, 0.0)
            if not math.isfinite(value):
                raise ValueError(f"{name}: joint value must be finite")
            if joint.lower is not None and value < joint.lower - 1e-8:
                raise ValueError(f"{name}: below URDF limit {joint.lower:g}")
            if joint.upper is not None and value > joint.upper + 1e-8:
                raise ValueError(f"{name}: above URDF limit {joint.upper:g}")
            return value

        for joint in self.joints:
            if joint.movable:
                evaluate(joint.name)
        return result

    @cached_property
    def joint_origins(self) -> dict[str, np.ndarray]:
        return {joint.name: origin_transform(joint.xyz, joint.rpy) for joint in self.joints}

    def transforms(self, values: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        """Root-frame link poses: T_parent @ T_joint_origin @ T_joint_motion."""
        resolved = self.configuration(values)
        transforms = {self.root: np.eye(4)}
        children: dict[str, list[Joint]] = {}
        for joint in self.joints:
            children.setdefault(joint.parent, []).append(joint)
        queue = [self.root]
        for parent in queue:
            for joint in children.get(parent, []):
                motion = np.eye(4)
                value = resolved.get(joint.name, 0.0)
                if joint.kind in ("revolute", "continuous"):
                    motion[:3, :3] = rotation(joint.axis, value)
                elif joint.kind == "prismatic":
                    motion[:3, 3] = np.array(joint.axis) * value
                transforms[joint.child] = (
                    transforms[parent] @ self.joint_origins[joint.name] @ motion
                )
                queue.append(joint.child)
        return transforms

    def positions(self, values: dict[str, float] | None = None) -> dict[str, np.ndarray]:
        return {name: matrix[:3, 3] for name, matrix in self.transforms(values).items()}

    def visual_transform(self, visual: Visual, transforms: dict[str, np.ndarray]) -> np.ndarray:
        """Mesh coordinates use their own origin and scale after the link pose."""
        return transforms[visual.link] @ visual.local_transform()


def rotation(axis, angle: float) -> np.ndarray:
    axis = np.array(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin_transform(xyz, rpy) -> np.ndarray:
    roll, pitch, yaw = rpy
    transform = np.eye(4)
    transform[:3, :3] = (
        rotation((0, 0, 1), yaw) @ rotation((0, 1, 0), pitch) @ rotation((1, 0, 0), roll)
    )
    transform[:3, 3] = xyz
    return transform


def pose_rpy(transform: np.ndarray) -> tuple[float, ...]:
    """XYZ metres and intrinsic roll/pitch/yaw radians (Rz @ Ry @ Rx), gimbal-lock safe."""
    matrix = transform[:3, :3]
    pitch = math.atan2(-matrix[2, 0], math.hypot(matrix[0, 0], matrix[1, 0]))
    if abs(math.cos(pitch)) > 1e-9:
        roll, yaw = math.atan2(matrix[2, 1], matrix[2, 2]), math.atan2(matrix[1, 0], matrix[0, 0])
    else:
        roll, yaw = math.atan2(-matrix[1, 2], matrix[1, 1]), 0.0
    return (*map(float, transform[:3, 3]), roll, pitch, yaw)


def vector(text: str | None, default: str) -> tuple[float, ...]:
    values = tuple(float(v) for v in (text or default).split())
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("URDF vector must contain exactly three finite numbers")
    return values


def origin(element):
    node = element.find("origin")
    return (
        vector(node.get("xyz") if node is not None else None, "0 0 0"),
        vector(node.get("rpy") if node is not None else None, "0 0 0"),
    )


def material_color(element, materials):
    material = element if element.tag == "material" else element.find("material")
    if material is None:
        return (0.55, 0.75, 0.78, 1.0)
    color = material.find("color")
    if color is None:
        return materials.get(material.get("name"), (0.55, 0.75, 0.78, 1.0))
    rgba = tuple(float(v) for v in color.get("rgba", "").split())
    if len(rgba) != 4 or not all(math.isfinite(v) and 0 <= v <= 1 for v in rgba):
        raise ValueError("Invalid URDF material color")
    return rgba


def parse_urdf(source: Path | str) -> RobotModel:
    if isinstance(source, Path) and source.stat().st_size > 5_000_000:
        raise ValueError("URDF exceeds 5 MB limit")
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
            raise UnsupportedKinematicsError(
                f"{name}: invalid link relation, closed chain or multiple parents; only open trees are supported"
            )
        children.add(child)
        axis_el, limit = element.find("axis"), element.find("limit")
        xyz, rpy = origin(element)
        axis = vector(axis_el.get("xyz") if axis_el is not None else None, "1 0 0")
        norm = float(np.linalg.norm(axis))
        if kind in ("revolute", "continuous", "prismatic"):
            if norm == 0:
                raise ValueError(f"{name}: zero joint axis")
            axis = tuple(v / norm for v in axis)
        lower = upper = None
        if kind in ("revolute", "prismatic"):
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"{name}: joint limits missing")
            lower, upper = float(limit.get("lower")), float(limit.get("upper"))
            if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
                raise ValueError(f"{name}: invalid limits")
        dynamic_limits = []
        for attribute in ("velocity", "effort", "acceleration"):
            value = (
                float(limit.get(attribute))
                if limit is not None and limit.get(attribute) is not None
                else None
            )
            if value is not None and (not math.isfinite(value) or value < 0):
                raise ValueError(f"{name}: invalid {attribute} limit")
            dynamic_limits.append(value)
        mimic = element.find("mimic")
        if mimic is not None and (
            not mimic.get("joint") or kind not in ("revolute", "continuous", "prismatic")
        ):
            raise ValueError(f"{name}: invalid mimic definition")
        multiplier = float(mimic.get("multiplier", "1")) if mimic is not None else 1.0
        offset = float(mimic.get("offset", "0")) if mimic is not None else 0.0
        if not math.isfinite(multiplier) or not math.isfinite(offset):
            raise ValueError(f"{name}: invalid mimic coefficients")
        joints.append(
            Joint(
                name,
                kind,
                parent,
                child,
                xyz,
                rpy,
                axis,
                lower,
                upper,
                mimic.get("joint") if mimic is not None else None,
                multiplier,
                offset,
                *dynamic_limits,
            )
        )
    roots = set(links) - children
    if len(roots) != 1:
        raise UnsupportedKinematicsError(
            "URDF must contain one root; closed/disconnected mechanisms are not supported"
        )
    root_name = roots.pop()
    visited, queue, adjacency = set(), [root_name], {}
    for joint in joints:
        adjacency.setdefault(joint.parent, []).append(joint.child)
    for current in queue:
        if current in visited:
            raise UnsupportedKinematicsError("Closed-chain URDF is not supported")
        visited.add(current)
        queue.extend(adjacency.get(current, []))
    if visited != set(links):
        raise ValueError("URDF contains disconnected links or a cycle")
    by_name = {joint.name: joint for joint in joints}
    for joint in joints:
        chain, current = set(), joint
        while current.mimic:
            if current.name in chain or current.mimic not in by_name:
                raise ValueError(f"{joint.name}: invalid or cyclic mimic joint")
            chain.add(current.name)
            current = by_name[current.mimic]
            if not current.movable:
                raise ValueError(f"{joint.name}: mimic source is not movable")
    materials = {}
    for material in root.findall("material"):
        materials[material.get("name")] = material_color(material, {})
    visuals, references = [], set()
    for link in root.findall("link"):
        for mesh in link.findall(".//mesh"):
            filename = mesh.get("filename", "")
            if not filename:
                raise ValueError("Mesh filename is empty")
            references.add(filename)
        for element in link.findall("visual"):
            xyz, rpy = origin(element)
            color = material_color(element, materials)
            geometry = element.find("geometry")
            if geometry is None:
                raise ValueError(f"{link.get('name')}: visual geometry missing")
            mesh = geometry.find("mesh")
            if mesh is not None:
                scale = vector(mesh.get("scale"), "1 1 1")
                if any(v <= 0 for v in scale):
                    raise ValueError("Mesh scale must be positive")
                visuals.append(
                    Visual(link.get("name"), xyz, rpy, mesh.get("filename"), scale, color)
                )
            else:
                for kind, attributes in [
                    ("box", ("size",)),
                    ("sphere", ("radius",)),
                    ("cylinder", ("radius", "length")),
                ]:
                    primitive = geometry.find(kind)
                    if primitive is not None:
                        dims = tuple(
                            float(v) for attr in attributes for v in primitive.get(attr, "").split()
                        )
                        expected = 3 if kind == "box" else len(attributes)
                        if len(dims) != expected or not all(
                            math.isfinite(v) and v > 0 for v in dims
                        ):
                            raise ValueError(f"Invalid {kind} dimensions")
                        visuals.append(
                            Visual(
                                link.get("name"),
                                xyz,
                                rpy,
                                color=color,
                                primitive=kind,
                                dimensions=dims,
                            )
                        )
                        break
                else:
                    raise ValueError("Unsupported visual geometry")
    model = RobotModel(
        root.get("name", "Robot"),
        links,
        joints,
        root_name,
        visuals,
        sorted(references),
        source_path=source.resolve() if isinstance(source, Path) else None,
    )
    # Validate coupled feasibility without assuming zero satisfies every mimic limit.
    model.effective_joint_bounds
    return model
