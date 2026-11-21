"""Rectified stereo with explicit principal-point offset and optical Z in metres."""

import cv2
import numpy as np


def calibration(width, height, focal, baseline, world_from_left, cx_offset=0.0):
    if width < 16 or height < 16 or focal <= 0 or not 0.02 <= baseline <= 0.3:
        raise ValueError("Invalid stereo dimensions, focal length or baseline")
    cx, cy = (width - 1) / 2, (height - 1) / 2
    k1 = np.array([[focal, 0, cx], [0, focal, cy], [0, 0, 1]], float)
    k2 = k1.copy()
    k2[0, 2] += cx_offset
    p1 = np.c_[k1, np.zeros(3)]
    p2 = np.c_[k2, [-focal * baseline, 0, 0]]
    q = np.array(
        [
            [1, 0, 0, -cx],
            [0, 1, 0, -cy],
            [0, 0, 0, focal],
            [0, 0, 1 / baseline, (k2[0, 2] - cx) / baseline],
        ]
    )
    right = np.asarray(world_from_left, float).copy()
    right[:3, 3] += right[:3, 0] * baseline
    return dict(
        width=width,
        height=height,
        K_left=k1.tolist(),
        K_right=k2.tolist(),
        P_left=p1.tolist(),
        P_right=p2.tolist(),
        Q=q.tolist(),
        baseline_m=baseline,
        world_from_left=np.asarray(world_from_left).tolist(),
        world_from_right=right.tolist(),
        distortion_left=[],
        distortion_right=[],
        rectified=True,
        depth_type="optical_z_m",
        disparity_type="u_left_minus_u_right_px",
    )


def depth_to_disparity(z, cal):
    p1, p2 = np.asarray(cal["P_left"]), np.asarray(cal["P_right"])
    z = np.asarray(z, float)
    with np.errstate(divide="ignore", invalid="ignore"):
        disparity = -p2[0, 3] / z + p1[0, 2] - p2[0, 2]
    return disparity.astype(np.float32)


def reconstruct(disparity, cal, valid=None):
    disparity = np.asarray(disparity, np.float32)
    if disparity.shape != (cal["height"], cal["width"]):
        raise ValueError("Disparity and calibration resolution differ")
    xyz = cv2.reprojectImageTo3D(disparity, np.asarray(cal["Q"], float))
    mask = np.isfinite(xyz).all(axis=2) & (xyz[..., 2] > 0)
    if valid is not None:
        mask &= valid
    xyz[~mask] = np.nan
    return xyz


def project(world_points, world_from_camera, k):
    points = np.asarray(world_points, float).reshape(-1, 3)
    pose = np.asarray(world_from_camera, float)
    optical = (points - pose[:3, 3]) @ pose[:3, :3]
    uvw = optical @ np.asarray(k, float).T
    return uvw[:, :2] / uvw[:, 2:], optical


def surface_reference(optical_points, mask, world_from_camera):
    """One physical reference, selected in optical coordinates and then transformed."""
    usable = mask & np.isfinite(optical_points).all(axis=-1) & (optical_points[..., 2] > 0)
    if not usable.any():
        return None, None
    camera = np.median(optical_points[usable], axis=0)
    world = (np.asarray(world_from_camera) @ np.r_[camera, 1])[:3]
    return camera, world


def visibility(left_z, right_z, cal, tolerance_m=0.003, max_depth_m=2.0):
    disparity = depth_to_disparity(left_z, cal)
    yy, xx = np.indices(left_z.shape)
    right_x = xx - disparity
    sampled = cv2.remap(
        right_z.astype(np.float32),
        right_x.astype(np.float32),
        yy.astype(np.float32),
        cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    valid = np.isfinite(disparity) & (left_z > 0) & (left_z < max_depth_m)
    valid &= (right_x >= 0) & (right_x < left_z.shape[1] - 1)
    valid &= np.isfinite(sampled) & (np.abs(sampled - left_z) < tolerance_m)
    return disparity, valid


def resize_calibration(cal, width, height):
    import copy

    result = copy.deepcopy(cal)
    sx, sy = width / cal["width"], height / cal["height"]
    for key in ("K_left", "K_right", "P_left", "P_right"):
        matrix = np.asarray(cal[key], float)
        matrix[0] *= sx
        matrix[1] *= sy
        matrix[0, 2] += (sx - 1) / 2
        matrix[1, 2] += (sy - 1) / 2
        result[key] = matrix.tolist()
    # Q is recomputed from the rectified projection matrices, including cx difference.
    a, b = np.asarray(result["P_left"]), np.asarray(result["P_right"])
    baseline = -b[0, 3] / b[0, 0]
    result["Q"] = [
        [1, 0, 0, -a[0, 2]],
        [0, a[0, 0] / a[1, 1], 0, -a[1, 2] * a[0, 0] / a[1, 1]],
        [0, 0, 0, a[0, 0]],
        [0, 0, 1 / baseline, (b[0, 2] - a[0, 2]) / baseline],
    ]
    result.update(width=width, height=height)
    return result
