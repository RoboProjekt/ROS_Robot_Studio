"""Mild raw-camera defects, calibrated recovery and common stereo reference support.

The left camera is always geometrically corrected with its true model. Calibration
stress affects only the right camera, so the original left optical-Z/Q reference
grid remains meaningful. Ground-truth depth/labels are never image-warped blindly.
"""

import time

import cv2
import numpy as np


def pixel_grid(width, height):
    yy, xx = np.indices((height, width), dtype=np.float32)
    return np.stack((xx, yy), axis=-1)


def project_raw(ideal_pixels, k, distortion, model):
    k = np.asarray(k, float)
    points = np.asarray(ideal_pixels, float).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(points, k, None)
    if model == "fisheye":
        result = cv2.fisheye.distortPoints(normalized, k, np.asarray(distortion, float))
    elif model == "plumb_bob":
        xyz = np.c_[normalized[:, 0], np.ones(len(points))]
        result, _ = cv2.projectPoints(
            xyz, np.zeros(3), np.zeros(3), k, np.asarray(distortion, float)
        )
    else:
        raise ValueError("Lens model must be fisheye or plumb_bob")
    return result.reshape(np.asarray(ideal_pixels).shape)


def undistort_raw(raw_pixels, k, distortion, model):
    points = np.asarray(raw_pixels, float).reshape(-1, 1, 2)
    k, distortion = np.asarray(k, float), np.asarray(distortion, float)
    if model == "fisheye":
        result = cv2.fisheye.undistortPoints(points, k, distortion, P=k)
    elif model == "plumb_bob":
        result = cv2.undistortPoints(points, k, distortion, P=k)
    else:
        raise ValueError("Lens model must be fisheye or plumb_bob")
    return result.reshape(np.asarray(raw_pixels).shape)


def remap(image, coordinates, interpolation=cv2.INTER_LINEAR):
    return cv2.remap(
        image,
        coordinates[..., 0].astype(np.float32),
        coordinates[..., 1].astype(np.float32),
        interpolation,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )


class CameraEffects:
    """One defect profile with cached raw/recovery maps for each camera calibration."""

    def __init__(self, profile):
        self.profile = profile
        self.cache = {}
        for key, default in (
            ("blur_sigma", 0),
            ("motion_pixels", 0),
            ("noise_sigma_dn", 0),
            ("gain", 1),
        ):
            values = profile.get(key, [default, default])
            if len(values) != 2 or not np.isfinite(values).all() or min(values) < 0:
                raise ValueError(f"{key} must contain two finite nonnegative values")
        if "assumed_distortion_left" in profile:
            raise ValueError("This benchmark requires correctly rectified left-camera geometry")

    def maps(self, cal, side):
        model = self.profile.get("lens_model")
        k = np.asarray(cal["K_" + side], float)
        key = (cal["width"], cal["height"], side, tuple(k.ravel()))
        if key in self.cache:
            return self.cache[key]
        grid = pixel_grid(cal["width"], cal["height"])
        raw_map, recovery_map, raw_points, residual = grid, grid, grid, np.zeros(grid.shape[:2])
        if model:
            distortion = self.profile["distortion_" + side]
            if (
                len(distortion) != (4 if model == "fisheye" else 5)
                or not np.isfinite(distortion).all()
            ):
                raise ValueError("Invalid distortion coefficients for the selected lens model")
            assumed = self.profile.get("assumed_distortion_" + side, distortion)
            if len(assumed) != len(distortion) or not np.isfinite(assumed).all():
                raise ValueError("Invalid assumed distortion coefficients")
            # Raw destination -> ideal source for synthesis, then ideal destination -> raw source.
            raw_map = undistort_raw(grid, k, distortion, model)
            recovery_map = project_raw(grid, k, assumed, model)
            raw_points = project_raw(grid, k, distortion, model)
            residual = np.linalg.norm(undistort_raw(raw_points, k, assumed, model) - grid, axis=-1)
        raw_support = (
            remap(np.ones(grid.shape[:2], np.float32), raw_map) > 0.99999
            if model
            else np.ones(grid.shape[:2], bool)
        )
        index = 0 if side == "left" else 1
        sigma = self.profile.get("blur_sigma", [0, 0])[index]
        motion = self.profile.get("motion_pixels", [0, 0])[index]
        if motion and (int(motion) != motion or int(motion) % 2 != 1):
            raise ValueError("Motion kernel must be zero or a positive odd integer")
        # Conservative source footprint for Gaussian + horizontal motion interpolation.
        radius = int(np.ceil(4 * sigma)) + int(motion) // 2
        if radius:
            raw_support = cv2.erode(
                raw_support.astype(np.uint8),
                np.ones((2 * radius + 1, 2 * radius + 1), np.uint8),
                borderType=cv2.BORDER_CONSTANT,
                borderValue=0,
            ).astype(bool)
        support = (
            remap(raw_support.astype(np.float32), recovery_map) > 0.99999 if model else raw_support
        )
        # Pull true raw coordinates through the assumed correction when defining shared visibility.
        true_output = (
            undistort_raw(
                raw_points,
                k,
                self.profile.get(
                    "assumed_distortion_" + side, self.profile.get("distortion_" + side)
                ),
                model,
            )
            if model
            else grid
        )
        physical_support = remap(support.astype(np.float32), true_output) > 0.99999
        result = dict(
            raw_map=raw_map,
            recovery_map=recovery_map,
            support=physical_support,
            shift_px=np.linalg.norm(raw_points - grid, axis=-1),
            residual_px=residual,
            roundtrip_px=np.linalg.norm(
                undistort_raw(raw_points, k, self.profile["distortion_" + side], model) - grid,
                axis=-1,
            )
            if model
            else np.zeros(grid.shape[:2]),
        )
        self.cache[key] = result
        return result

    def capture(self, image, cal, side, rng):
        maps = self.maps(cal, side)
        index = 0 if side == "left" else 1
        raw = remap(image, maps["raw_map"]) if self.profile.get("lens_model") else image.copy()
        sigma = self.profile.get("blur_sigma", [0, 0])[index]
        motion = int(self.profile.get("motion_pixels", [0, 0])[index])
        if sigma:
            radius = int(np.ceil(4 * sigma))
            raw = cv2.GaussianBlur(raw, (2 * radius + 1, 2 * radius + 1), sigma)
        if motion:
            raw = cv2.filter2D(raw, -1, np.full((1, motion), 1 / motion, np.float32))
        gain = self.profile.get("gain", [1, 1])[index]
        noise = self.profile.get("noise_sigma_dn", [0, 0])[index]
        if gain != 1 or noise:
            raw = np.clip(
                raw.astype(np.float32) * gain + rng.normal(0, noise, raw.shape), 0, 255
            ).astype(np.uint8)
        begin = time.perf_counter()
        corrected = remap(raw, maps["recovery_map"]) if self.profile.get("lens_model") else raw
        elapsed = (time.perf_counter() - begin) * 1000
        return corrected, raw, elapsed


class RobustnessSuite:
    """Same left GT grid and same physically retained reference pixels in every condition."""

    def __init__(self, config):
        self.config = config
        profiles = config["profiles"]
        names = [profile["id"] for profile in profiles]
        if len(names) != len(set(names)) or any(
            not name.replace("_", "").isalnum() for name in names
        ):
            raise ValueError("Profile IDs must be unique alphanumeric names with underscores")
        self.effects = {p["id"]: CameraEffects(p) for p in profiles}

    def photographic_support(self, sample, cal):
        supports = {
            side: np.logical_and.reduce(
                [e.maps(cal, side)["support"] for e in self.effects.values()]
            )
            for side in ("left", "right")
        }
        yy, xx = np.indices(sample["valid"].shape, dtype=np.float32)
        right = cv2.remap(
            supports["right"].astype(np.float32),
            xx - sample["disparity"],
            yy,
            cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
        )
        return supports["left"] & (right > 0.99999)

    def common_support(self, sample, cal):
        return sample["valid"] & self.photographic_support(sample, cal)

    def transform(self, name, sample, meta):
        cal = meta["calibration"]
        effect = self.effects[name]
        rng = np.random.default_rng(np.random.SeedSequence([self.config["seed"], meta["seed"]]))
        result, raw, elapsed = dict(sample), {}, 0.0
        for side in ("left", "right"):
            result[side], raw[side], milliseconds = effect.capture(sample[side], cal, side, rng)
            elapsed += milliseconds
        result["photographic_support"] = self.photographic_support(sample, cal)
        result["valid"] = sample["valid"] & result["photographic_support"]
        return (
            result,
            dict(
                rectification_ms=elapsed,
                original_reference_pixels=int(sample["valid"].sum()),
                retained_reference_pixels=int(result["valid"].sum()),
            ),
            raw,
        )
