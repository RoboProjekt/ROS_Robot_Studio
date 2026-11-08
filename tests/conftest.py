import hashlib
import io
import json
import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("PYTEST_QT_API", "pyside6")

import pytest
import trimesh

from ros_robot_studio.robots.model_manager import RobotModelManager


def pytest_addoption(parser):
    parser.addoption("--run-docker", action="store_true", default=False)
    parser.addoption("--run-model-download", action="store_true", default=False)
    parser.addoption("--run-isaac", action="store_true", default=False)


def pytest_collection_modifyitems(config, items):
    if not config.getoption("--run-isaac"):
        for item in items:
            if "isaac" in item.keywords:
                item.add_marker(
                    pytest.mark.skip(
                        reason="Explicit --run-isaac required; starts local GPU simulator"
                    )
                )
    if not config.getoption("--run-docker"):
        for item in items:
            if "integration" in item.keywords:
                item.add_marker(
                    pytest.mark.skip(
                        reason="Explicit --run-docker required; creates only Studio test containers"
                    )
                )

    if not config.getoption("--run-model-download"):
        for item in items:
            if "model_download" in item.keywords:
                item.add_marker(
                    pytest.mark.skip(
                        reason="Explicit --run-model-download required for original SO-101 asset download"
                    )
                )


@pytest.fixture
def tiny_catalog(tmp_path):
    urdf = b"""<robot name="Tiny test arm">
      <material name="gold"><color rgba="1 0.8 0.1 1"/></material>
      <link name="base"/>
      <link name="hand"><visual><origin xyz="0.1 0.2 0.3" rpy="0 0 1.5707963267948966"/>
       <geometry><mesh filename="assets/part.stl" scale="2 3 4"/></geometry><material name="gold"/></visual></link>
      <joint name="hinge" type="revolute"><parent link="base"/><child link="hand"/>
       <origin xyz="1 0 0"/><axis xyz="0 0 1"/>
       <limit lower="-1.5" upper="1.5" effort="1" velocity="1"/></joint></robot>"""
    data = {
        "arm.urdf": urdf,
        "assets/part.stl": trimesh.creation.box(extents=(0.01, 0.02, 0.03)).export(file_type="stl"),
        "LICENSE": b"Test fixture owned by the application; Apache-2.0",
    }
    manifest = dict(
        schema_version=1,
        id="test_arm",
        name="Test Arm",
        manufacturer="Studio test",
        model="Test",
        variant="test",
        model_version="1.0.0",
        description="Small original test fixture",
        urdf="arm.urdf",
        mesh_directory="assets",
        source_repository="https://example.invalid/model",
        source_base_url="https://example.invalid/" + "a" * 40,
        source_revision="a" * 40,
        license="Apache-2.0",
        license_file="LICENSE",
        default_end_effector="hand",
        calibration="test",
        backends={"offline_viewer": "supported", "hardware": "planned"},
        files=[
            dict(
                path=name,
                source_path=name,
                size=len(value),
                sha256=hashlib.sha256(value).hexdigest(),
            )
            for name, value in data.items()
        ],
    )
    catalog = tmp_path / "catalog"
    catalog.mkdir()
    (catalog / "test.json").write_text(json.dumps(manifest))
    calls = []

    def opener(request, timeout):
        relative = request.full_url.split("a" * 40 + "/", 1)[1]
        calls.append(relative)
        return io.BytesIO(data[relative])

    manager = RobotModelManager(tmp_path / "cache", catalog, opener)
    return manager, data, calls, manifest


@pytest.fixture(autouse=True)
def isolated_model_data(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_DATA_HOME", str(tmp_path / "xdg-data"))
