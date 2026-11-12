"""Optical coordinates, validated CameraInfo and intersection with a known plane."""

from dataclasses import dataclass

import numpy as np

USD_FROM_OPTICAL = np.diag([1.0, -1.0, -1.0])


def look_at_usd(position, target):
    forward = np.asarray(target, dtype=float) - position
    length = np.linalg.norm(forward)
    if not np.isfinite(length) or length < 1e-8:
        raise ValueError("Camera and target must be distinct finite points")
    forward /= length
    up = (0, 1, 0) if abs(forward[2]) > 0.999 else (0, 0, 1)
    right = np.cross(forward, up)
    right /= np.linalg.norm(right)
    return np.column_stack((right, np.cross(right, forward), -forward))


def usd_rpy(position, target):
    from scipy.spatial.transform import Rotation

    return tuple(Rotation.from_matrix(look_at_usd(position, target)).as_euler("xyz", degrees=True))


@dataclass(frozen=True)
class Intrinsics:
    matrix: np.ndarray
    distortion: np.ndarray
    model: str
    width: int
    height: int

    @classmethod
    def from_info(cls, info, frame):
        if (info["width"], info["height"], info["frame_id"]) != (
            frame.width,
            frame.height,
            frame.frame_id,
        ):
            raise ValueError("CameraInfo resolution or optical frame does not match the image")
        matrix = np.asarray(info["k"], dtype=float).reshape(3, 3)
        distortion = np.asarray(info["d"], dtype=float)
        model = info["model"]
        if (
            not np.isfinite(matrix).all()
            or not np.isfinite(distortion).all()
            or min(matrix[0, 0], matrix[1, 1]) <= 0
            or not np.allclose(matrix[2], (0, 0, 1))
            or not 0 <= matrix[0, 2] <= frame.width
            or not 0 <= matrix[1, 2] <= frame.height
            or model not in ("", "plumb_bob", "rational_polynomial", "equidistant")
        ):
            raise ValueError("Invalid or unsupported CameraInfo intrinsics")
        if info.get("binning_x", 0) > 1 or info.get("binning_y", 0) > 1 or info.get("roi", False):
            raise ValueError("Binned and cropped CameraInfo requires explicit rectification")
        lengths = {"plumb_bob": (5,), "rational_polynomial": (8,), "equidistant": (4,)}
        if distortion.ndim != 1 or (
            distortion.size and distortion.size not in lengths.get(model, ())
        ):
            raise ValueError("CameraInfo distortion coefficients do not match its model")
        return cls(matrix, distortion, model, frame.width, frame.height)

    def ray(self, pixel):
        point = np.asarray(pixel, dtype=float)
        if (
            point.shape != (2,)
            or not np.isfinite(point).all()
            or not (0 <= point[0] < self.width and 0 <= point[1] < self.height)
        ):
            raise ValueError("Pixel outside image")
        if np.any(self.distortion):
            import cv2

            correction = (
                cv2.fisheye.undistortPoints if self.model == "equidistant" else cv2.undistortPoints
            )
            try:
                x, y = correction(point.reshape(1, 1, 2), self.matrix, self.distortion)[0, 0]
            except cv2.error as exc:
                raise ValueError("CameraInfo distortion correction failed") from exc
            return np.array([x, y, 1.0])
        return np.linalg.solve(self.matrix, (*point, 1.0))


def transform_matrix(translation, quaternion):
    q = np.asarray(quaternion, dtype=float)
    offset = np.asarray(translation, dtype=float)
    if (
        q.shape != (4,)
        or offset.shape != (3,)
        or not np.isfinite(offset).all()
        or not np.isclose(np.linalg.norm(q), 1, atol=1e-5)
    ):
        raise ValueError("Invalid TF translation or rotation")
    x, y, z, w = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), offset[0]],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), offset[1]],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), offset[2]],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )


def intersect_plane(ray, camera_to_plane, plane):
    origin = camera_to_plane[:3, 3]
    direction = camera_to_plane[:3, :3] @ ray
    normal = np.asarray(plane.normal)
    normal = normal / np.linalg.norm(normal)
    direction = direction / np.linalg.norm(direction)
    denominator = np.dot(normal, direction)
    if not np.isfinite(denominator) or abs(denominator) < 1e-8:
        raise ValueError("Camera ray is parallel to the work plane")
    distance = np.dot(normal, plane.point - origin) / denominator
    if not np.isfinite(distance) or distance <= 0:
        raise ValueError("Work plane intersection is behind the camera")
    point = origin + distance * direction
    if plane.bounds and (
        np.any(point < np.asarray(plane.bounds[0]) - 1e-8)
        or np.any(point > np.asarray(plane.bounds[1]) + 1e-8)
    ):
        raise ValueError("Estimated position is outside the work area")
    return point
