"""Optical-map direction, metric stereo preservation and matched defect evaluation."""

import numpy as np
import pytest

from ros_robot_studio.vision.stereo_learning.camera_effects import (
    CameraEffects,
    RobustnessSuite,
    pixel_grid,
    project_raw,
    undistort_raw,
)
from ros_robot_studio.vision.stereo_learning.geometry import calibration
from ros_robot_studio.vision.stereo_learning.inference import sgbm


def plane():
    cal = calibration(160, 96, 120, 0.08, np.eye(4))
    rng = np.random.default_rng(101)
    left = rng.integers(0, 255, (96, 160, 3), dtype=np.uint8)
    right = np.zeros_like(left)
    right[:, :-20] = left[:, 20:]
    sample = dict(
        left=left,
        right=right,
        depth=np.full((96, 160), 0.48, np.float32),
        disparity=np.full((96, 160), 20, np.float32),
        valid=np.indices((96, 160))[1] >= 20,
    )
    return sample, dict(seed=101, calibration=cal)


@pytest.mark.parametrize(
    "model,coefficients", [("fisheye", [0.31, 0.085, 0, 0]), ("plumb_bob", [-0.02, 0.005, 0, 0, 0])]
)
def test_projection_and_inverse_preserve_pixel_centres(model, coefficients):
    _, meta = plane()
    k = np.array(meta["calibration"]["K_left"])
    points = pixel_grid(160, 96)
    raw = project_raw(points, k, coefficients, model)
    restored = undistort_raw(raw, k, coefficients, model)
    np.testing.assert_allclose(restored, points, atol=1e-5)
    shift = np.linalg.norm(raw - points, axis=-1)
    assert 0.5 < shift.max() < 5
    if model == "fisheye":
        # Zero fisheye coefficients are equidistant projection, not pinhole identity.
        zero = project_raw(points, k, [0, 0, 0, 0], model)
        assert np.linalg.norm(zero - points, axis=-1).max() > shift.max() * 3
    else:
        np.testing.assert_allclose(
            project_raw(points, k, [0, 0, 0, 0, 0], model), points, atol=1e-7
        )


def test_true_fisheye_correction_retains_known_stereo_disparity():
    sample, meta = plane()
    config = dict(
        seed=101,
        profiles=[
            dict(
                id="lens",
                lens_model="fisheye",
                distortion_left=[0.31, 0.085, 0, 0],
                distortion_right=[0.30, 0.08, 0, 0],
            )
        ],
    )
    suite = RobustnessSuite(config)
    corrected, _, _ = suite.transform("lens", sample, meta)
    disparity, valid = sgbm(corrected["left"], corrected["right"], 64)
    mask = corrected["valid"] & valid.astype(bool)
    assert mask.mean() > 0.4
    assert np.median(np.abs(disparity[mask] - 20)) < 0.3


def test_conditions_share_reference_support_and_noise_is_reproducible():
    sample, meta = plane()
    profiles = [
        dict(id="control"),
        dict(id="noise", blur_sigma=[0.4, 0.65], noise_sigma_dn=[1, 1], gain=[1, 0.97]),
    ]
    suite = RobustnessSuite(dict(seed=7, profiles=profiles))
    control, _, _ = suite.transform("control", sample, meta)
    degraded, _, _ = suite.transform("noise", sample, meta)
    repeated, _, _ = suite.transform("noise", sample, meta)
    np.testing.assert_array_equal(degraded["left"], repeated["left"])
    np.testing.assert_array_equal(control["valid"], degraded["valid"])
    np.testing.assert_array_equal(degraded["depth"], sample["depth"])
    np.testing.assert_array_equal(degraded["disparity"], sample["disparity"])
    np.testing.assert_array_equal(control["left"], sample["left"])
    assert not np.array_equal(degraded["left"], sample["left"])
    assert not control["valid"][:3].any()


def test_right_calibration_error_is_measured_without_moving_left_gt():
    sample, meta = plane()
    profile = dict(
        id="error",
        lens_model="fisheye",
        distortion_left=[0.31, 0.085, 0, 0],
        distortion_right=[0.30, 0.08, 0, 0],
        assumed_distortion_right=[0.285, 0.08, 0, 0],
    )
    effect = CameraEffects(profile)
    assert effect.maps(meta["calibration"], "left")["residual_px"].max() < 1e-5
    assert 0.1 < effect.maps(meta["calibration"], "right")["residual_px"].max() < 2
    with pytest.raises(ValueError, match="left-camera"):
        CameraEffects({**profile, "assumed_distortion_left": [0.3, 0.08, 0, 0]})
