import json

import pytest

from ros_robot_studio.backends.isaac import (
    IsaacSimInstallationManager,
    IsaacSimProcessManager,
    IsaacSimSceneManager,
)
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.robots.geometry import load_robot


def installation(tmp_path, script="#!/bin/sh\nexit 0\n"):
    root = tmp_path / "fake-isaac"
    root.mkdir()
    (root / "VERSION").write_text("5.1.0-test")
    for name in ("isaac-sim.sh", "python.sh"):
        path = root / name
        path.write_text(script)
        path.chmod(0o755)
    (root / "exts/isaacsim.ros2.bridge/humble/lib").mkdir(parents=True)
    (root / "exts/isaacsim.asset.importer.urdf").mkdir()
    return IsaacSimInstallationManager().validate(root)


def request(tmp_path, profile):
    path = tmp_path / "scene-request.json"
    data = dict(
        backend_id=profile.id,
        domain_id=profile.domain_id,
        model_signature="model",
        request_id="request",
        output_directory=str(tmp_path),
        expected_joints={"hinge": {}},
    )
    path.write_text(json.dumps(data))
    return path, data


def test_installation_detection_and_version_rejection(tmp_path, monkeypatch):
    found = installation(tmp_path)
    monkeypatch.setenv("ISAAC_PATH", str(found.path))
    detector = IsaacSimInstallationManager()
    assert detector.detect([str(found.path)])[0] == found
    (found.path / "VERSION").write_text("6.0.0")
    with pytest.raises(ValueError, match="5.1"):
        detector.validate(found.path)


def test_missing_installation_is_optional(tmp_path):
    with pytest.raises(ValueError, match="Select an Isaac"):
        IsaacSimInstallationManager().validate(tmp_path / "absent")


def test_scene_request_uses_existing_fk_hashes_and_rejects_stale_result(tiny_catalog, tmp_path):
    models, *_ = tiny_catalog
    source = models.install("test_arm")
    model = load_robot(source, require_meshes=True)
    manager = IsaacSimSceneManager()
    path = manager.prepare(new_profile("isaac_sim"), model, tmp_path / "scene")
    data = json.loads(path.read_text())
    assert data["expected_link_positions"]["hand"] == [1.0, 0.0, 0.0]
    assert data["expected_joints"]["hinge"]["axis_parent"] == [0, 0, 1]
    assert len(data["source_files"]) == 2
    (path.parent / "scene-validation.json").write_text(
        json.dumps(
            dict(
                request_id="old-request",
                model_signature=data["model_signature"],
                domain_id=data["domain_id"],
            )
        )
    )
    with pytest.raises(ValueError, match="provenance"):
        manager.validate_result(path)
    with pytest.raises(ValueError, match="existing USD"):
        manager.prepare(
            new_profile("isaac_sim"), model, tmp_path / "scene", existing_scene="absent.usd"
        )


def test_process_rejects_mismatched_domain_before_start(qtbot, tmp_path):
    profile = new_profile("isaac_sim")
    path, data = request(tmp_path, profile)
    data["domain_id"] += 1
    path.write_text(json.dumps(data))
    manager = IsaacSimProcessManager()
    with pytest.raises(ValueError, match="backend/domain/model"):
        manager.start(profile, installation(tmp_path), path)
    assert not manager.processes


def test_process_exit_without_validated_scene_reports_error(qtbot, tmp_path):
    profile = new_profile("isaac_sim")
    path, _ = request(tmp_path, profile)
    manager = IsaacSimProcessManager()
    errors = []
    manager.error.connect(lambda identity, text: errors.append(text))
    manager.start(profile, installation(tmp_path), path)
    qtbot.waitUntil(lambda: bool(errors) and not manager.processes)
    assert "exited with code 0" in errors[0]
    manager.stop("foreign-process")
    manager.deleteLater()


def test_owned_process_ready_environment_and_shutdown(qtbot, tmp_path):
    profile = new_profile("isaac_sim", discovery="localhost")
    path, data = request(tmp_path, profile)
    (tmp_path / "scene-validation.json").write_text(
        json.dumps(dict(data, valid=True, joint_names=["hinge"]))
    )
    fake = installation(
        tmp_path,
        '#!/bin/sh\necho "DOMAIN=$ROS_DOMAIN_ID LOCAL=$ROS_LOCALHOST_ONLY DDS=$FASTRTPS_DEFAULT_PROFILES_FILE"\necho "STUDIO_SCENE_READY {}"\nexec sleep 30\n',
    )
    manager = IsaacSimProcessManager()
    errors = []
    manager.error.connect(lambda identity, text: errors.append(text))
    process = manager.start(profile, fake, path, headless=True)
    qtbot.waitUntil(lambda: profile.id in manager.ready_reports)
    assert "DOMAIN=1 LOCAL=0" in manager.tails[profile.id]
    assert "fastdds_localhost.xml" in manager.tails[profile.id]
    assert "--headless" in process.arguments()
    manager.stop(profile.id)
    qtbot.waitUntil(lambda: not manager.processes)
    assert not errors
    manager.deleteLater()
