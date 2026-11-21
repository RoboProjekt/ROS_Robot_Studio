"""Timestamp-paired stereo perception using calibrated ROS images and TF2."""

from collections import deque
from dataclasses import dataclass

import numpy as np

from ros_robot_studio.vision.geometry import Intrinsics, transform_matrix

PAIR_SLOP_NS = 25_000_000
MAX_AGE_NS = 750_000_000
WORK_FRAME = "work_surface (+Y of robot_base is left)"


class StereoPairer:
    def __init__(self):
        self.clear()

    def clear(self):
        self.queues = (deque(maxlen=4), deque(maxlen=4))
        self.last = [None, None]

    def push(self, side, frame, now_ns):
        stamp = frame.metadata["stamp_ns"]
        if not stamp or now_ns - frame.metadata["received_wall_ns"] > MAX_AGE_NS:
            return
        previous = self.last[side]
        if previous is not None and stamp < previous:
            self.clear()
        elif previous == stamp:
            return
        self.last[side] = stamp
        self.queues[side].append(frame)

    def pair(self, now_ns):
        candidates = []
        for queue in self.queues:
            while queue and now_ns - queue[0].metadata["received_wall_ns"] > MAX_AGE_NS:
                queue.popleft()
        for left in self.queues[0]:
            for right in self.queues[1]:
                stamps = (left.metadata["stamp_ns"], right.metadata["stamp_ns"])
                skew = abs(stamps[0] - stamps[1])
                if skew <= PAIR_SLOP_NS:
                    candidates.append((skew, -min(stamps), left, right))
        if not candidates:
            return None
        _, _, left, right = min(candidates, key=lambda item: item[:2])
        for queue, frame in zip(self.queues, (left, right), strict=True):
            while queue.popleft() is not frame:
                pass
        return left, right


@dataclass(frozen=True)
class StereoResult:
    left: np.ndarray
    right: np.ndarray
    disparity: np.ndarray
    objects: tuple
    stamp_ns: int
    depth_error: str = ""


class OpenCVStereo:
    """Depth estimator interface: estimate(left, right, base) -> rectified images and points."""

    def __init__(self, depth_range=(0.2, 1.2)):
        import cv2

        if (
            len(depth_range) != 2
            or not np.isfinite(depth_range).all()
            or not 0 < depth_range[0] < depth_range[1]
        ):
            raise ValueError("Stereo depth range must contain two increasing positive distances")
        self.depth_range = tuple(depth_range)
        self.cv = cv2
        self.rectification = None
        self.key = None
        self.matcher = None

    def estimate(self, left, right, base):
        cv = self.cv
        if left.rgb.shape != right.rgb.shape or left.frame_id == right.frame_id:
            raise ValueError("Stereo needs distinct optical frames with matching resolutions.")
        if not left.metadata.get("camera_info") or not right.metadata.get("camera_info"):
            raise ValueError("Waiting for the two CameraInfo messages.")
        if any(base not in frame.metadata["transforms"] for frame in (left, right)):
            raise ValueError(f"Waiting for image-time TF2 to {base}.")
        left_tf, right_tf = (
            transform_matrix(*frame.metadata["transforms"][base]) for frame in (left, right)
        )
        relative = np.linalg.inv(right_tf) @ left_tf
        rotation, translation = relative[:3, :3], relative[:3, 3]
        baseline = float(np.linalg.norm(translation))
        if not 0.02 <= baseline <= 0.3 or translation[0] >= -0.75 * baseline:
            raise ValueError("Check the left/right optical frames and stereo baseline.")
        a, b = (Intrinsics.from_info(f.metadata["camera_info"], f) for f in (left, right))
        if any(info.model not in ("plumb_bob", "rational_polynomial", "") for info in (a, b)):
            raise ValueError("Stereo currently requires pinhole CameraInfo.")
        size = (left.width, left.height)
        key = (
            size,
            a.matrix.tobytes(),
            b.matrix.tobytes(),
            a.distortion.tobytes(),
            b.distortion.tobytes(),
            np.round(relative, 7).tobytes(),
        )
        if self.key != key:
            r1, r2, p1, p2, q, _, _ = cv.stereoRectify(
                a.matrix,
                a.distortion,
                b.matrix,
                b.distortion,
                size,
                rotation,
                translation,
                flags=cv.CALIB_ZERO_DISPARITY,
                alpha=0,
            )
            maps = tuple(
                cv.initUndistortRectifyMap(info.matrix, info.distortion, r, p, size, cv.CV_16SC2)
                for info, r, p in ((a, r1, p1), (b, r2, p2))
            )
            desired = int(np.ceil(p1[0, 0] * baseline / 0.25 / 16)) * 16
            disparities = min(max(16, (size[0] // 2 // 16) * 16), max(16, desired))
            self.matcher = cv.StereoSGBM_create(
                minDisparity=0,
                numDisparities=disparities,
                blockSize=5,
                P1=8 * 25,
                P2=32 * 25,
                disp12MaxDiff=1,
                uniquenessRatio=8,
                speckleWindowSize=60,
                speckleRange=2,
            )
            self.rectification = maps, q, r1
            self.key = key
        maps, q, r1 = self.rectification
        lrgb, rrgb = (
            cv.remap(f.rgb, *mapping, cv.INTER_LINEAR)
            for f, mapping in zip((left, right), maps, strict=True)
        )
        disparity = self.compute_disparity(lrgb, rrgb)
        points = cv.reprojectImageTo3D(disparity, q)
        valid = (disparity > 0) & np.isfinite(points).all(axis=2)
        valid &= (points[..., 2] >= self.depth_range[0]) & (points[..., 2] <= self.depth_range[1])
        points[~valid] = np.nan
        transform = left_tf.copy()
        transform[:3, :3] = left_tf[:3, :3] @ r1.T
        return lrgb, rrgb, disparity, points, transform

    def compute_disparity(self, left, right):
        return (
            self.matcher.compute(
                self.cv.cvtColor(left, self.cv.COLOR_RGB2GRAY),
                self.cv.cvtColor(right, self.cv.COLOR_RGB2GRAY),
            ).astype(np.float32)
            / 16
        )


def identify(rgb, points, to_base):
    import cv2

    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    masks = (
        (
            "red_cube",
            "red cube",
            cv2.inRange(hsv, (0, 95, 65), (10, 255, 255))
            | cv2.inRange(hsv, (170, 95, 65), (179, 255, 255)),
        ),
        ("blue_bowl", "blue bowl", cv2.inRange(hsv, (92, 90, 55), (133, 255, 255))),
    )
    objects = []
    for identity, label, mask in masks:
        count, components, stats, centers = cv2.connectedComponentsWithStats(mask)
        strong = [i for i in range(1, count) if stats[i, cv2.CC_STAT_AREA] >= 100]
        if not strong:
            continue
        strong.sort(key=lambda i: (-stats[i, cv2.CC_STAT_AREA], centers[i, 0]))
        for index, component in enumerate(strong[:5]):
            object_id = identity if len(strong) == 1 else f"{identity}_{index + 1}"
            x, y, w, h, _ = (int(value) for value in stats[component])
            surface = None
            if points is not None:
                middle = cv2.erode(
                    (components == component).astype(np.uint8), np.ones((5, 5), np.uint8)
                )
                cloud = points[middle.astype(bool)]
                cloud = cloud[np.isfinite(cloud).all(axis=1)]
                if len(cloud) >= 16 and len(cloud) >= 0.10 * np.count_nonzero(middle):
                    median = np.median(cloud, axis=0)
                    if np.median(np.abs(cloud[:, 2] - median[2])) < 0.035:
                        surface = (to_base @ np.r_[median, 1])[:3].tolist()
            objects.append(
                {
                    "id": object_id,
                    "class": label,
                    "xyz": surface,
                    "position_valid": surface is not None,
                    "position_kind": "surface",
                    "bbox": [x, y, w, h],
                }
            )
    return tuple(objects)


def perceive(estimator, left, right, base):
    try:
        lrgb, rrgb, disparity, points, pose = estimator.estimate(left, right, base)
        error = "" if np.isfinite(points).any() else "No valid stereo depth."
    except ValueError as exc:
        lrgb, rrgb = left.rgb, right.rgb
        disparity = np.full(left.rgb.shape[:2], -1, dtype=np.float32)
        points = pose = None
        error = str(exc)
    return StereoResult(
        lrgb, rrgb, disparity, identify(lrgb, points, pose), left.metadata["stamp_ns"], error
    )


def stereo_profiles(backend_id, base, baseline_m=0.08, distance_m=0.45):
    from ros_robot_studio.vision.geometry import look_at_usd, usd_rpy
    from ros_robot_studio.vision.profiles import CameraMount, CameraProfile

    if not 0.04 <= baseline_m <= 0.12 or not 0.30 <= distance_m <= 0.60:
        raise ValueError("Choose a baseline of 40–120 mm and a work distance of 30–60 cm.")
    target = np.array([0.20, 0.0, 0.025])
    eye = target + distance_m * np.array([0.0, -0.8, 0.6])
    right = look_at_usd(eye, target)[:, 0]
    rpy = usd_rpy(eye, target)
    return tuple(
        CameraProfile(
            id=f"so101_stereo_{side}",
            name=f"SO-101 stereo {side}",
            backend_id=backend_id,
            rgb_topic=f"/so101/stereo/{side}/image_raw",
            info_topic=f"/so101/stereo/{side}/camera_info",
            camera_frame=f"so101_stereo_{side}_link",
            optical_frame=f"so101_stereo_{side}_optical_frame",
            base_frame=base,
            fps=15,
            mount=CameraMount(
                position=tuple(eye + sign * baseline_m / 2 * right),
                usd_rpy_deg=rpy,
                horizontal_fov=65,
            ),
        )
        for side, sign in (("left", -1), ("right", 1))
    )
