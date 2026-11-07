"""Atomic YAML persistence. Relative assets resolve against the project file."""

from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml

from ros_robot_studio.core.models import Project, Sensor

DATA = Path(__file__).resolve().parents[1] / "data"
EXAMPLE_URDF = DATA / "examples" / "generic_arm.urdf"


def load_project(path: Path) -> Project:
    if path.stat().st_size > 2_000_000:
        raise ValueError("Project exceeds 2 MB limit")
    return Project.model_validate(yaml.safe_load(path.read_text(encoding="utf-8")))


def atomic_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix="." + path.name, dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def save_project(project: Project, path: Path) -> None:
    # Revalidate even if a caller mutated a nested list in-place.
    validated = Project.model_validate(project.model_dump())
    atomic_text(
        path, yaml.safe_dump(validated.model_dump(mode="json"), sort_keys=False, allow_unicode=True)
    )


def asset_path(reference: str | None, project_path: Path | None) -> Path:
    if not reference:
        return EXAMPLE_URDF
    path = Path(reference)
    return (
        path if path.is_absolute() else (project_path.parent if project_path else Path.cwd()) / path
    )


def portable_copy(project: Project, old_path: Path | None, destination: Path) -> Project:
    """Copy the parsed URDF beside the project, so Save As and export stay usable."""
    result = project.model_copy(deep=True)
    source = asset_path(project.robot.urdf, old_path)
    target = destination.parent / (destination.stem + ".assets") / "robot.urdf"
    if source.resolve() != target.resolve():
        atomic_text(target, source.read_text(encoding="utf-8"))
    result.robot.urdf = str(target.relative_to(destination.parent))
    # Xacro is provenance only. Exported runtime uses the expanded URDF.
    if result.robot.source_xacro:
        result.robot.source_xacro = str(asset_path(result.robot.source_xacro, old_path).resolve())
    return result


def demo_project() -> Project:
    return Project(
        name="Atlas · Research Arm",
        description="A three-axis research arm. Offline signals are simulated; no hardware is connected.",
        sensors=[
            Sensor(
                name="Joint encoders",
                kind="Joint encoder",
                topic="/joint_states",
                unit="rad",
                quantity="position",
            ),
            Sensor(
                name="Reference signal",
                topic="/studio/sine",
                quantity="amplitude",
                unit="V",
                lower_limit=-1.2,
                upper_limit=1.2,
                warning=0.9,
            ),
        ],
    )
