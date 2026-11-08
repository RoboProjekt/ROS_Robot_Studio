"""Small local fixtures exercise the same installer used for the original robot."""

import io
import json
import urllib.error
from pathlib import Path

import pytest

from ros_robot_studio.core.models import Project, Robot
from ros_robot_studio.projects.store import load_project, portable_copy, save_project
from ros_robot_studio.robots.geometry import load_robot, resolve_mesh_uri, safe_relative_path
from ros_robot_studio.robots.model_manager import (
    ModelUnavailableError,
    RobotModelManager,
    RobotModelManifest,
)


def test_so101_reviewed_manifest():
    manager = RobotModelManager()
    spec = manager.spec("so101_follower")
    assert spec.source_revision == "a758567c3978dfeefe282ede0500085a48fe8f78"
    assert spec.urdf == "so101_new_calib.urdf"
    assert spec.variant == "follower" and spec.calibration == "new_calib"
    assert len([f for f in spec.files if f.path.endswith(".stl")]) == 13
    assert spec.download_bytes == 16159288
    assert spec.license == "Apache-2.0"
    assert spec.backends["offline_viewer"] == "supported"
    assert spec.backends["hardware"] == "planned" and not spec.drivers
    assert not spec.verified_ros_distributions


def test_install_verify_and_offline_reuse(tiny_catalog):
    manager, data, calls, _ = tiny_catalog
    assert manager.status("test_arm").state == "not_installed"
    path = manager.install("test_arm")
    assert manager.status("test_arm").state == "installed"
    assert set(calls) == set(data)
    assert manager.resolve(manager.spec("test_arm").reference) == path
    model = load_robot(path, require_meshes=True)
    assert len(model.meshes) == 1
    assert model.visuals[0].color == (1, 0.8, 0.1, 1)
    assert model.visuals[0].scale == (2, 3, 4)
    receipt = json.loads((path.parent / "installation.json").read_text())
    assert receipt["files"] and receipt["source_revision"] == "a" * 40
    calls.clear()
    manager.install("test_arm")
    assert calls == []


def test_damaged_and_missing_assets_repaired_selectively(tiny_catalog):
    manager, _, calls, _ = tiny_catalog
    path = manager.install("test_arm")
    mesh = path.parent / "assets/part.stl"
    mesh.write_bytes(b"broken")
    assert manager.status("test_arm").state == "damaged"
    calls.clear()
    manager.install("test_arm")
    assert calls == ["assets/part.stl"]
    mesh.unlink()
    assert manager.status("test_arm").state == "incomplete"
    with pytest.raises(ModelUnavailableError):
        manager.resolve(manager.spec("test_arm").reference)
    manager.install("test_arm")
    assert manager.status("test_arm").state == "installed"


def test_missing_receipt_reuses_all_original_files(tiny_catalog):
    manager, _, calls, _ = tiny_catalog
    path = manager.install("test_arm")
    (path.parent / "installation.json").unlink()
    calls.clear()
    assert manager.status("test_arm").state == "incomplete"
    manager.install("test_arm")
    assert calls == []


def test_failed_download_never_activates_partial_model(tiny_catalog):
    manager, _, _, _ = tiny_catalog

    def failure(request, timeout):
        raise urllib.error.URLError("offline")

    manager.opener = failure
    with pytest.raises(ValueError, match="internet"):
        manager.install("test_arm")
    assert manager.status("test_arm").state == "not_installed"
    assert not list(manager.root.glob("**/.install-*"))


def test_checksum_failure_preserves_existing_tree(tiny_catalog):
    manager, _, _, _ = tiny_catalog
    path = manager.install("test_arm")
    original_urdf = path.read_bytes()
    (path.parent / "assets/part.stl").write_bytes(b"bad")
    manager.opener = lambda *args, **kwargs: io.BytesIO(b"bad")
    with pytest.raises(ValueError, match="Checksum"):
        manager.install("test_arm")
    assert path.read_bytes() == original_urdf
    assert manager.status("test_arm").state == "damaged"


@pytest.mark.parametrize(
    "path",
    [
        "../escape.stl",
        "/etc/passwd",
        "assets/../../escape",
        r"..\escape",
        "file:///etc/passwd",
        "%2e%2e/escape",
        "part.stl?x=1",
        "part.stl#x",
    ],
)
def test_asset_traversal_rejected(path):
    with pytest.raises(ValueError):
        safe_relative_path(path)


def test_manifest_traversal_and_mutable_source_rejected(tiny_catalog):
    _, _, _, manifest = tiny_catalog
    corrupt = json.loads(json.dumps(manifest))
    corrupt["files"][0]["path"] = "../escape"
    with pytest.raises(ValueError):
        RobotModelManifest.model_validate(corrupt)
    manifest["source_base_url"] = "https://example.invalid/main"
    with pytest.raises(ValueError, match="immutable"):
        RobotModelManifest.model_validate(manifest)


def test_mesh_symlink_escape_and_missing_detected(tmp_path):
    model = tmp_path / "model"
    model.mkdir()
    elsewhere = tmp_path / "outside.stl"
    elsewhere.write_bytes(b"outside")
    (model / "escape.stl").symlink_to(elsewhere)
    with pytest.raises(ValueError, match="escapes"):
        resolve_mesh_uri("escape.stl", model)
    with pytest.raises(FileNotFoundError):
        resolve_mesh_uri("missing.stl", model)


def test_missing_mesh_rejects_required_scene(tiny_catalog):
    manager, _, _, _ = tiny_catalog
    path = manager.install("test_arm")
    (path.parent / "assets/part.stl").unlink()
    with pytest.raises(ValueError, match="Missing mesh"):
        load_robot(path, require_meshes=True)
    model = load_robot(path)
    assert model.warnings and not model.meshes


def test_unknown_revision_is_not_silently_upgraded(tiny_catalog):
    manager, _, _, _ = tiny_catalog
    with pytest.raises(ValueError, match="substituted"):
        manager.install("test_arm", revision="b" * 40)


def test_model_project_roundtrip_without_absolute_paths(tmp_path):
    manager = RobotModelManager()
    spec = manager.spec("so101_follower")
    robot = Robot(
        profile="so101",
        model_id=spec.id,
        model_version=spec.model_version,
        source_revision=spec.source_revision,
        urdf=spec.reference,
        joint_positions={"shoulder_pan": 0.3, "gripper": 0.6},
        end_effector=spec.default_end_effector,
    )
    project = Project(name="SO-101 test", robot=robot)
    destination = tmp_path / "robot.yaml"
    exported = portable_copy(project, None, destination)
    save_project(exported, destination)
    restored = load_project(destination)
    assert restored == exported
    assert "model://so101_follower/" in destination.read_text()
    assert str(Path.home()) not in destination.read_text()
    assert restored.robot.joint_positions["gripper"] == 0.6
    assert restored.robot.simulation.status == "planned"
    assert restored.robot.simulation.asset.path is None
    assert restored.robot.joint_mapping_status == "not_calibrated"


def test_v1_project_upgrades_without_changing_source(tmp_path):
    path = tmp_path / "legacy.yaml"
    original = "format_version: 1\nname: Legacy arm\n"
    path.write_text(original)
    project = load_project(path)
    assert project.format_version == 2
    assert project.robot.joint_positions == {}
    assert path.read_text() == original
