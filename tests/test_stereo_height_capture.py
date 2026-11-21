"""Height/angle convention, parallel baseline and strong calibrated fisheye geometry."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from ros_robot_studio.vision.stereo_learning.geometry import calibration, project
from ros_robot_studio.vision.stereo_learning.height_capture import (
    StrongFisheye,
    height_mount,
    resample_reference,
    source_maps,
)


@pytest.mark.parametrize("reference", ["surface", "normal"])
def test_height_is_perpendicular_and_rotations_remain_identical(reference):
    rng = np.random.default_rng(3101)
    camera = dict(height_range_m=[0.10, 0.20], view_angle_deg=[0, 45], angle_reference=reference)
    angles = []
    for i in range(24):
        eye, usd, meta = height_mount(camera, rng, i)
        optical = usd @ np.diag([1, -1, -1])
        pose = np.eye(4)
        pose[:3, :3] = optical
        pose[:3, 3] = eye - optical[:, 0] * 0.03
        cal = calibration(320, 192, 112, 0.06, pose)
        right = np.asarray(cal["world_from_right"])
        np.testing.assert_allclose(right[:3, :3], pose[:3, :3])
        assert np.linalg.norm(right[:3, 3] - pose[:3, 3]) == pytest.approx(0.06)
        assert 0.10 - 1e-9 <= eye[2] + 0.005 <= 0.20 + 1e-9
        actual = np.degrees(np.arcsin(-optical[2, 2]))
        assert actual == pytest.approx(meta["view_angle_surface_deg"])
        angles.append(meta["requested_angle_deg"])
        uv, _ = project([meta["work_center_world"]], pose, cal["K_left"])
        assert 0 <= uv[0, 1] < 192
    assert min(angles) == 0 and max(angles) == 45


def test_resampling_inverse_depth_retains_an_analytic_plane():
    source = calibration(640, 384, 170, 0.06, np.eye(4))
    target = calibration(320, 192, 112, 0.06, np.eye(4))
    coords = source_maps(source, target)
    y, x = np.indices((384, 640))
    inverse = 2 + x * 0.0001 + y * 0.0003
    z = (1 / inverse).astype(np.float32)
    rgb = np.zeros((384, 640, 3), np.uint8)
    labels = np.zeros((384, 640), np.uint8)
    _, depth, _ = resample_reference((rgb, z, labels), coords)
    expected = 1 / (2 + coords[..., 0] * 0.0001 + coords[..., 1] * 0.0003)
    np.testing.assert_allclose(depth, expected, atol=1e-7)


def test_extreme_fisheye_is_calibrated_and_has_valid_output_support():
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root / "configs/stereo/high-height-fisheye.yaml").read_text())
    source = calibration(1280, 768, 112.85, 0.06, np.eye(4))
    target = calibration(320, 192, 112.03, 0.06, np.eye(4))
    lens = StrongFisheye(source, target, config["optics"], np.random.default_rng(3101))
    for side in lens.sides.values():
        assert side["max_lens_displacement_px"] > 25
        assert side["support"].mean() > 0.75
    frame = np.zeros((768, 1280, 3), np.uint8)
    rectified, raw = lens.capture(frame, "left", np.random.default_rng(1), False)
    assert rectified.shape == raw.shape == (192, 320, 3)


def test_near_range_is_not_rejected_by_legacy_twenty_cm_limit():
    from ros_robot_studio.vision.stereo import OpenCVStereo
    from ros_robot_studio.vision.stream import RGBFrame

    class FixedStereo(OpenCVStereo):
        def compute_disparity(self, left, right):
            return np.full(left.shape[:2], 72, np.float32)

    frames = []
    for side, x in (("left", 0), ("right", 0.06)):
        frame_id = side + "_optical"
        frames.append(
            RGBFrame(
                np.zeros((96, 160, 3), np.uint8),
                dict(
                    frame_id=frame_id,
                    stamp_ns=1,
                    transforms={"world": ([x, 0, 0], [0, 0, 0, 1])},
                    camera_info=dict(
                        width=160,
                        height=96,
                        frame_id=frame_id,
                        k=[120, 0, 79.5, 0, 120, 47.5, 0, 0, 1],
                        d=[],
                        model="plumb_bob",
                    ),
                ),
            )
        )
    default = FixedStereo().estimate(*frames, "world")[3]
    near = FixedStereo(depth_range=(0.02, 1.5)).estimate(*frames, "world")[3]
    assert not np.isfinite(default).any()
    np.testing.assert_allclose(near[..., 2], 0.10, atol=1e-6)


def test_sgbm_reverse_check_survives_search_range_equal_to_image_width():
    from ros_robot_studio.vision.stereo_learning.inference import sgbm

    rng = np.random.default_rng(3102)
    left = rng.integers(0, 255, (96, 160, 3), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-80] = left[:, 80:]
    disparity, valid = sgbm(left, right, 160)
    mask = valid.astype(bool)
    assert mask.mean() > 0.35
    assert np.median(np.abs(disparity[mask] - 80)) < 0.2
