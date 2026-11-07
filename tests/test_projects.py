from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from ros_robot_studio.core.models import Project, Sensor, Transform
from ros_robot_studio.projects.store import (
    asset_path,
    demo_project,
    load_project,
    portable_copy,
    save_project,
)


def test_project_roundtrip_and_sensor_offsets(tmp_path):
    project = demo_project()
    project.sensors.append(
        Sensor(
            name="Wrist force",
            kind="Force / torque",
            topic="/wrist/force",
            unit="N",
            transform=Transform(
                kind="dynamic", translation=(0.1, -0.2, 0.3), rotation=(0.01, 0.02, 0.03)
            ),
        )
    )
    path = tmp_path / "research.yaml"
    save_project(project, path)
    loaded = load_project(path)
    assert loaded == project
    assert loaded.sensors[-1].transform.translation == (0.1, -0.2, 0.3)
    assert loaded.sensors[-1].transform.kind == "dynamic"
    assert not list(tmp_path.glob(".research*"))


@pytest.mark.parametrize(
    "data", [dict(format_version=99), dict(name=""), dict(default_ros="foxy"), dict(unknown=1)]
)
def test_invalid_project(data, tmp_path):
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(data))
    with pytest.raises(ValidationError):
        load_project(path)


@pytest.mark.parametrize(
    "values",
    [
        dict(lower_limit=2, upper_limit=1),
        dict(topic="relative"),
        dict(sample_rate=0),
        dict(warning=float("nan")),
    ],
)
def test_invalid_sensor(values):
    with pytest.raises(ValueError):
        Sensor(name="Test", **values)


def test_portable_save_as_copies_asset_and_preserves_source(tmp_path):
    original = demo_project()
    path = tmp_path / "a" / "project.yaml"
    copy = portable_copy(original, None, path)
    save_project(copy, path)
    destination = tmp_path / "b" / "export.yaml"
    exported = portable_copy(copy, path, destination)
    save_project(exported, destination)
    assert asset_path(exported.robot.urdf, destination).exists()
    assert not Path(exported.robot.urdf).is_absolute()
    assert original.robot.urdf is None
    assert asset_path(copy.robot.urdf, path).exists()


def test_project_distro_override_does_not_damage_source(tmp_path):
    project = demo_project()
    project.runtime.overrides["jazzy"] = {"controller_package": "test"}
    path = tmp_path / "project.yaml"
    save_project(project, path)
    loaded = load_project(path)
    assert loaded.default_ros == "humble"
    assert loaded.runtime.overrides["jazzy"] == {"controller_package": "test"}


def test_duplicate_sensor_ids():
    sensor = Sensor(name="A")
    with pytest.raises(ValueError):
        Project(sensors=[sensor, sensor])
