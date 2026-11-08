"""Explicit original-source integration. No Docker, ROS or physical device access."""

import numpy as np
import pytest

from ros_robot_studio.robots.geometry import load_robot
from ros_robot_studio.robots.model_manager import RobotModelManager
from ros_robot_studio.robots.urdf import pose_rpy


@pytest.mark.model_download
def test_original_so101_install_meshes_limits_fk_and_offline_reuse(tmp_path):
    manager = RobotModelManager(root=tmp_path / "robots")
    path = manager.install("so101_follower")
    assert manager.status("so101_follower").state == "installed"
    model = load_robot(path, require_meshes=True)
    assert model.name == "so101_new_calib"
    assert len(model.links) == 8 and len(model.joints) == 7
    assert len(model.visuals) == 17 and len(model.meshes) == 13
    joints = {joint.name: joint for joint in model.joints}
    movable = {joint.name for joint in model.joints if joint.movable}
    assert movable == {
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    }
    assert joints["gripper_frame_joint"].kind == "fixed"
    assert joints["gripper_frame_joint"].axis == (0, 0, 0)
    assert joints["gripper"].lower == -0.174533
    assert joints["gripper"].upper == 1.74533
    assert joints["shoulder_pan"].lower == -1.91986
    assert joints["shoulder_pan"].upper == 1.91986
    assert not model.warnings
    zero = model.transforms()
    changed = model.transforms({"shoulder_lift": 0.4, "gripper": 0.3})
    assert not np.allclose(zero["gripper_frame_link"], changed["gripper_frame_link"])
    assert np.isfinite(pose_rpy(changed["gripper_frame_link"])).all()
    for matrix in changed.values():
        np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-10)
        assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1)
    # Offline re-opening must never invoke the downloader.
    original_opener = manager.opener

    def no_network(*args, **kwargs):
        raise AssertionError("Already installed model attempted a download")

    manager.opener = no_network
    assert manager.install("so101_follower") == path
    # A real corrupted mesh is repaired selectively from the pinned HTTPS source.
    mesh = path.parent / "assets/waveshare_mounting_plate_so101_v2.stl"
    mesh.write_bytes(b"corrupt integration-test file")
    assert manager.status("so101_follower").state == "damaged"
    requests = []

    def monitored(request, **kwargs):
        requests.append(request.full_url)
        return original_opener(request, **kwargs)

    manager.opener = monitored
    manager.install("so101_follower")
    assert len(requests) == 1 and requests[0].endswith(mesh.name)
    assert manager.status("so101_follower").state == "installed"
