"""Offline visual mesh loading. Original geometry stays unchanged; no ROS imports."""

from pathlib import Path, PurePosixPath

import numpy as np
import trimesh

from ros_robot_studio.robots.urdf import MeshGeometry, RobotModel, Visual, parse_urdf


def safe_relative_path(value: str) -> PurePosixPath:
    path = PurePosixPath(value)
    if (
        not value
        or "\\" in value
        or "%" in value
        or any(character in value for character in ":?#")
        or path.is_absolute()
        or any(part in (".", "..", "") for part in value.split("/"))
    ):
        raise ValueError(f"Unsafe relative asset path: {value}")
    return path


def resolve_mesh_uri(
    uri: str, directory: Path, package_roots: dict[str, Path] | None = None
) -> Path:
    """Resolve only files inside the selected model or an explicit package root."""
    root = directory.resolve()
    reference = uri
    if uri.startswith("package://"):
        package, separator, reference = uri[len("package://") :].partition("/")
        if not separator or package not in (package_roots or {}):
            raise ValueError(f"Package mesh needs an explicit package root: {uri}")
        root = package_roots[package].resolve()
    relative = safe_relative_path(reference)
    result = (root / str(relative)).resolve()
    if not result.is_relative_to(root):
        raise ValueError(f"Mesh escapes model directory: {uri}")
    if not result.is_file():
        raise FileNotFoundError(f"Missing mesh: {uri}")
    return result


def load_stl(path: Path) -> MeshGeometry:
    if path.stat().st_size > 80_000_000:
        raise ValueError(f"Mesh exceeds 80 MB limit: {path.name}")
    # Explicit loader, no scene resources, preprocessing, repair, unit conversion or network fetches.
    with path.open("rb") as stream:
        mesh = trimesh.load_mesh(stream, file_type="stl", process=False)
    vertices = np.asarray(mesh.vertices, dtype=np.float32)
    faces = np.asarray(mesh.faces, dtype=np.uint32)
    if not len(vertices) or not len(faces) or len(faces) > 1_000_000:
        raise ValueError(f"Empty or oversized STL: {path.name}")
    if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
        raise ValueError(f"Invalid STL vertices: {path.name}")
    if faces.ndim != 2 or faces.shape[1] != 3 or int(faces.max()) >= len(vertices):
        raise ValueError(f"Invalid STL faces: {path.name}")
    return MeshGeometry(vertices, faces)


def visual_key(visual: Visual, index: int) -> str:
    return visual.mesh or f"primitive:{index}"


def load_robot(path: Path, require_meshes: bool = False, package_roots=None) -> RobotModel:
    """Blocking load intended for a worker. Invalid required assets fail before UI activation."""
    model = parse_urdf(path)
    for reference in model.mesh_references:
        try:
            asset = resolve_mesh_uri(reference, path.parent, package_roots)
            if asset.suffix.lower() != ".stl":
                raise ValueError(f"Only STL meshes are currently supported: {reference}")
            model.meshes[reference] = load_stl(asset)
        except (OSError, ValueError) as exc:
            if require_meshes:
                raise ValueError(str(exc)) from exc
            model.warnings.append(str(exc))
    for index, visual in enumerate(model.visuals):
        if visual.primitive:
            if visual.primitive == "box":
                geometry = trimesh.creation.box(extents=visual.dimensions)
            elif visual.primitive == "sphere":
                geometry = trimesh.creation.icosphere(subdivisions=2, radius=visual.dimensions[0])
            else:
                geometry = trimesh.creation.cylinder(
                    radius=visual.dimensions[0], height=visual.dimensions[1], sections=32
                )
            model.meshes[visual_key(visual, index)] = MeshGeometry(
                np.asarray(geometry.vertices, dtype=np.float32),
                np.asarray(geometry.faces, dtype=np.uint32),
            )
    return model
