"""Height-constrained parallel rigs and wide-render -> raw fisheye -> rectified data."""

import math

import cv2
import numpy as np

from ros_robot_studio.vision.geometry import look_at_usd

from .camera_effects import pixel_grid, project_raw, remap, undistort_raw


def height_mount(camera, rng, index):
    low, high = camera["height_range_m"]
    amin, amax = camera["view_angle_deg"]
    if not 0 < low <= high or not 0 <= amin <= amax <= 90:
        raise ValueError("Invalid height or view-angle range")
    height = low if index % 12 == 0 else high if index % 12 == 1 else rng.uniform(low, high)
    angle = amin if index % 10 == 0 else amax if index % 10 == 1 else rng.uniform(amin, amax)
    reference = camera["angle_reference"]
    if reference not in ("surface", "normal"):
        raise ValueError("angle_reference must be surface or normal")
    surface_angle = angle if reference == "surface" else 90 - angle
    surface_z = camera.get("surface_z_m", -0.005)
    center = np.array([0.2, 0.0]) + rng.uniform(-0.015, 0.015, 2)
    # A horizontal axis can still see the table below image centre. Keep its ROI within the FOV.
    aim = max(surface_angle, camera.get("minimum_target_depression_deg", 20))
    offset = height / math.tan(math.radians(aim)) if aim < 90 else 0
    eye = np.r_[center - [0, offset], surface_z + height]
    direction = np.array(
        [0, math.cos(math.radians(surface_angle)), -math.sin(math.radians(surface_angle))]
    )
    rotation = look_at_usd(eye, eye + direction)
    return (
        eye,
        rotation,
        dict(
            camera_height_m=float(height),
            view_angle_surface_deg=float(surface_angle),
            view_angle_normal_deg=float(90 - surface_angle),
            angle_reference=reference,
            requested_angle_deg=float(angle),
            work_center_world=[*center.tolist(), surface_z],
        ),
    )


def place_objects(eye, optical, cal, rng):
    """Sample visible object centres on their physical height planes; retain table bounds."""
    k = np.asarray(cal["K_left"])
    placed, result = [], []
    for z, radius, fallback in (
        (0.0225, 0.040, [0.12, -0.12]),
        (-0.005, 0.058, [0.27, 0.10]),
        (0.020, 0.037, [0.30, -0.10]),
    ):
        point = None
        for _ in range(120):
            u, v = rng.uniform(
                [cal["width"] * 0.18, cal["height"] * 0.22],
                [cal["width"] * 0.82, cal["height"] * 0.92],
            )
            ray = optical @ np.array([(u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], 1])
            if ray[2] >= -1e-6:
                continue
            depth = (z - eye[2]) / ray[2]
            candidate = eye + depth * ray
            if depth < 0.035 or not (
                -0.36 + radius < candidate[0] < 0.38 - radius
                and -0.32 + radius < candidate[1] < 0.32 - radius
            ):
                continue
            if any(
                np.linalg.norm(candidate[:2] - other[:2]) < radius + other_radius + 0.006
                for other, other_radius in placed
            ):
                continue
            point = candidate
            break
        if point is None:
            point = np.r_[fallback, z]
        placed.append((point, radius))
        result.append(point)
    return result


def source_maps(render_cal, output_cal):
    grid = pixel_grid(output_cal["width"], output_cal["height"])
    a, b = np.asarray(output_cal["K_left"]), np.asarray(render_cal["K_left"])
    coords = grid.astype(float)
    coords[..., 0] = (grid[..., 0] - a[0, 2]) / a[0, 0] * b[0, 0] + b[0, 2]
    coords[..., 1] = (grid[..., 1] - a[1, 2]) / a[1, 1] * b[1, 1] + b[1, 2]
    return coords


def resample_reference(source, coordinates):
    """Interpolate inverse Z, exact on planes, and reject mixed semantic/depth footprints."""
    rgb, depth, labels = source
    x, y = np.floor(coordinates[..., 0]).astype(int), np.floor(coordinates[..., 1]).astype(int)
    bounds = (x >= 0) & (x + 1 < depth.shape[1]) & (y >= 0) & (y + 1 < depth.shape[0])
    x, y = np.clip(x, 0, depth.shape[1] - 2), np.clip(y, 0, depth.shape[0] - 2)
    z = np.stack([depth[y + dy, x + dx] for dy, dx in ((0, 0), (0, 1), (1, 0), (1, 1))])
    semantics = np.stack([labels[y + dy, x + dx] for dy, dx in ((0, 0), (0, 1), (1, 0), (1, 1))])
    valid = bounds & np.isfinite(z).all(0) & (z > 0).all(0)
    valid &= (semantics == semantics[0]).all(0)
    valid &= np.max(z, axis=0) - np.min(z, axis=0) < 0.003 + 0.05 * np.min(z, axis=0)
    dx, dy = coordinates[..., 0] - x, coordinates[..., 1] - y
    weights = np.stack(((1 - dx) * (1 - dy), dx * (1 - dy), (1 - dx) * dy, dx * dy))
    safe_z = np.where(np.isfinite(z) & (z > 0), z, np.inf)
    sampled = np.sum(weights / safe_z, axis=0)
    output_z = np.full(sampled.shape, np.nan, np.float32)
    valid &= sampled > 0
    output_z[valid] = 1 / sampled[valid]
    return remap(rgb, coordinates), output_z, remap(labels, coordinates, cv2.INTER_NEAREST)


class StrongFisheye:
    """Spherical theta lens, including its circular support and raw-domain sensor degradation."""

    def __init__(self, render_cal, output_cal, config, rng, unseen=False):
        self.config = config
        self.width, self.height = output_cal["width"], output_cal["height"]
        self.sides = {}
        grid = pixel_grid(self.width, self.height)
        for side in ("left", "right"):
            k1_range = config["unseen_k1_range"] if unseen else config["k1_range"]
            d = np.array([rng.uniform(*k1_range), rng.uniform(*config["k2_range"]), 0, 0])
            fov = float(rng.uniform(*config["raw_horizontal_fov_deg"]))
            theta = math.radians(fov / 2)
            theta_distorted = theta * (1 + d[0] * theta**2 + d[1] * theta**4)
            focal = (self.width - 1) / 2 / theta_distorted
            raw_k = np.array(
                [[focal, 0, (self.width - 1) / 2], [0, focal, (self.height - 1) / 2], [0, 0, 1.0]]
            )
            source_k = np.asarray(render_cal["K_" + side])
            target_k = np.asarray(output_cal["K_" + side])
            raw_map = undistort_raw(grid, raw_k, d, "fisheye", destination_k=source_k)
            recovery = project_raw(grid, target_k, d, "fisheye", destination_k=raw_k)
            sigma = float(rng.uniform(*config["blur_sigma_range"]))
            noise = float(rng.uniform(*config["noise_sigma_dn_range"]))
            gain = float(rng.uniform(*config["gain_range"]))
            jpeg = int(rng.choice(config["jpeg_qualities"]))
            raw_support = (
                remap(np.ones((render_cal["height"], render_cal["width"]), np.float32), raw_map)
                > 0.99999
            )
            radius = int(np.ceil(4 * config["blur_sigma_range"][1]))
            raw_support = cv2.erode(
                raw_support.astype(np.uint8),
                np.ones((2 * radius + 1, 2 * radius + 1), np.uint8),
                borderType=cv2.BORDER_CONSTANT,
                borderValue=0,
            )
            support = remap(raw_support.astype(np.float32), recovery) > 0.99999
            # Check the full mapping with distinct raw/render/output intrinsics.
            restored = undistort_raw(recovery, raw_k, d, "fisheye", destination_k=target_k)
            if np.max(np.linalg.norm(restored - grid, axis=-1)) > 1e-4:
                raise ValueError("Strong fisheye mapping failed its optical round-trip")
            shift = np.linalg.norm(project_raw(grid, raw_k, d, "fisheye") - grid, axis=-1)
            self.sides[side] = dict(
                raw_map=raw_map,
                recovery=recovery,
                support=support,
                raw_k=raw_k.tolist(),
                distortion=d.tolist(),
                raw_horizontal_fov_deg=fov,
                blur_sigma=sigma,
                noise_sigma_dn=noise,
                gain=gain,
                jpeg_quality=jpeg,
                max_lens_displacement_px=float(shift.max()),
            )

    def capture(self, render_rgb, side, rng, degraded):
        values = self.sides[side]
        raw = remap(render_rgb, values["raw_map"])
        if degraded:
            sigma = values["blur_sigma"]
            if sigma:
                radius = int(np.ceil(4 * sigma))
                raw = cv2.GaussianBlur(raw, (2 * radius + 1, 2 * radius + 1), sigma)
            raw = np.clip(
                raw.astype(float) * values["gain"]
                + rng.normal(0, values["noise_sigma_dn"], raw.shape),
                0,
                255,
            ).astype(np.uint8)
            ok, encoded = cv2.imencode(
                ".jpg",
                cv2.cvtColor(raw, cv2.COLOR_RGB2BGR),
                [cv2.IMWRITE_JPEG_QUALITY, values["jpeg_quality"]],
            )
            if not ok:
                raise OSError("Raw-camera JPEG simulation failed")
            raw = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
        return remap(raw, values["recovery"]), raw

    def metadata(self):
        return {
            side: {
                key: value
                for key, value in values.items()
                if key not in ("raw_map", "recovery", "support")
            }
            for side, values in self.sides.items()
        }
