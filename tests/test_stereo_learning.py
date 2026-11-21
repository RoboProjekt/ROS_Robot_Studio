"""Metric stereo regressions, learned input contract and scene split isolation."""

import numpy as np
import pytest

from ros_robot_studio.vision.stereo_learning.geometry import (
    calibration,
    depth_to_disparity,
    project,
    reconstruct,
    resize_calibration,
    surface_reference,
    visibility,
)
from ros_robot_studio.vision.stereo_learning.inference import sgbm


def test_principal_point_offset_and_metric_reprojection():
    cal = calibration(160, 96, 120, 0.08, np.eye(4), cx_offset=3)
    z = np.full((96, 160), 0.45)
    d = depth_to_disparity(z, cal)
    assert d[45, 80] == pytest.approx(120 * 0.08 / 0.45 - 3)
    points = reconstruct(d, cal)
    np.testing.assert_allclose(points[..., 2], z, atol=1e-6)
    uv, cam = project([[0, 0, 0.45], [0.05, 0.02, 0.5]], np.eye(4), cal["K_left"])
    assert uv[0] == pytest.approx([79.5, 47.5])
    np.testing.assert_allclose(cam[1], [0.05, 0.02, 0.5])


def test_resize_disparity_and_intrinsics_together():
    cal = calibration(160, 96, 120, 0.08, np.eye(4), cx_offset=3)
    z = np.full((96, 160), 0.5)
    d = depth_to_disparity(z, cal)
    small = resize_calibration(cal, 80, 32)
    disparity = np.full((32, 80), d[0, 0] / 2)
    xyz = reconstruct(disparity, small)
    np.testing.assert_allclose(xyz[..., 2], 0.5, atol=1e-6)
    assert small["P_right"][0][2] == pytest.approx((cal["P_right"][0][2] + 0.5) / 2 - 0.5)


def test_visibility_rejects_left_border_and_occlusion():
    cal = calibration(160, 96, 120, 0.08, np.eye(4))
    z = np.full((96, 160), 0.48, np.float32)
    right = z.copy()
    right[30:50, 30:50] = 0.3
    d, mask = visibility(z, right, cal)
    assert d[0, 0] == pytest.approx(20)
    assert not mask[:, :20].any()
    assert not mask[30:50, 50:70].any()
    assert mask[0, 40]
    with pytest.raises(ValueError, match="resolution"):
        reconstruct(np.zeros((24, 40)), cal)


def test_sgbm_recovers_known_textured_plane():
    rng = np.random.default_rng(123)
    left = rng.integers(0, 255, (96, 160, 3), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-20] = left[:, 20:]
    disparity, valid = sgbm(left, right, 64)
    assert valid.mean() > 0.5
    assert np.median(np.abs(disparity[valid.astype(bool)] - 20)) < 0.2


def test_surface_reference_is_the_same_point_in_both_frames():
    pose = np.array([[1, 0, 0, 0.1], [0, 0.6, -0.8, 0.2], [0, 0.8, 0.6, 0.3], [0, 0, 0, 1]])
    cloud = np.array([[[0, 0.1, 0.4], [0.1, 0.2, 0.45], [0.2, -0.1, 0.6]]])
    camera, world = surface_reference(cloud, np.ones((1, 3), bool), pose)
    np.testing.assert_allclose(world, (pose @ np.r_[camera, 1])[:3])
    # A separate median in world coordinates generally selects a different point.
    independent = np.median(cloud.reshape(-1, 3) @ pose[:3, :3].T + pose[:3, 3], axis=0)
    assert np.linalg.norm(independent - world) > 0.01


def test_resumed_training_matches_uninterrupted_weights(tmp_path):
    import json

    torch = pytest.importorskip("torch")
    from ros_robot_studio.vision.stereo_learning.training import train

    root = tmp_path / "data"
    root.mkdir()
    rng = np.random.default_rng(101)
    records = []
    for i in range(4):
        left = rng.integers(0, 255, (32, 64, 3), dtype=np.uint8)
        right = np.roll(left, -8, axis=1)
        filename = f"{i}.npz"
        valid = np.indices((32, 64))[1] >= 8
        np.savez(
            root / filename,
            left=left,
            right=right,
            disparity=np.where(valid, 8, np.nan).astype(np.float32),
            valid=valid,
            segmentation=np.ones((32, 64), np.uint8),
        )
        records.append(dict(file=filename, scene_id=str(i), split="train"))
    (root / "manifest.json").write_text(json.dumps(dict(samples=records)))
    config = dict(
        seed=101,
        model=dict(max_disparity=16, channels=4),
        train=dict(device="cpu", threads=1, batch_size=2, learning_rate=0.001, epochs=2),
    )
    whole, resumed = tmp_path / "whole", tmp_path / "resumed"
    train(config, root, whole, epochs=2, overfit=4)
    train(config, root, resumed, epochs=1, overfit=4)
    train(config, root, resumed, epochs=2, overfit=4, resume=resumed / "last.pt")
    a = torch.load(whole / "last.pt", weights_only=False)
    b = torch.load(resumed / "last.pt", weights_only=False)
    assert all(torch.isfinite(tensor).all() for tensor in a["model"].values())
    assert all(torch.equal(a["model"][name], b["model"][name]) for name in a["model"])
    assert a["history"][-1]["validation_epe_px"] == b["history"][-1]["validation_epe_px"]


def test_model_only_uses_rectified_rgb_and_has_pixel_output():
    torch = pytest.importorskip("torch")
    from ros_robot_studio.vision.stereo_learning.model import CompactStereo

    torch.set_num_threads(1)
    model = CompactStereo(max_disparity=64, channels=8).eval()
    with torch.inference_mode():
        disparity, confidence = model(torch.rand(1, 3, 64, 128), torch.rand(1, 3, 64, 128))
    assert disparity.shape == confidence.shape == (1, 1, 64, 128)
    assert torch.isfinite(disparity).all()
    assert (disparity >= 0).all() and (disparity < 64).all()
    assert (confidence >= 0).all() and (confidence <= 1).all()


def test_adapter_rejects_exposure_skew_before_running_model():
    from ros_robot_studio.vision.stereo_learning.adapter import LearnedStereo
    from ros_robot_studio.vision.stream import RGBFrame

    estimator = LearnedStereo.__new__(LearnedStereo)
    left = RGBFrame(np.zeros((32, 64, 3), np.uint8), dict(stamp_ns=1))
    right = RGBFrame(left.rgb, dict(stamp_ns=33_000_001))
    with pytest.raises(ValueError, match="25 ms"):
        estimator.estimate(left, right, "world")
