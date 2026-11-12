"""Frame-matched detections and metric estimates on an explicitly known plane."""

from dataclasses import dataclass

import numpy as np

from ros_robot_studio.vision.geometry import Intrinsics, intersect_plane, transform_matrix


@dataclass(frozen=True)
class PlanarEstimate:
    position: tuple[float, float, float]
    frame_id: str
    stamp_ns: int
    camera_id: str
    backend_id: str
    pixel: tuple[float, float]
    method: str = "Ray–plane intersection"


class VisionResultManager:
    def __init__(self):
        self.clear()

    def clear(self):
        self.frame = None
        self.detections = ()
        self.seconds = 0.0

    def update(self, result):
        self.frame, self.detections, self.seconds = result

    def estimate(self, camera, finding, output_frame, reference="center", manual=None):
        if self.frame is None or not camera.plane:
            raise ValueError("No work plane configured")
        frame = self.frame
        x1, _, x2, y2 = finding.box
        pixel = (
            manual
            if manual is not None
            else (
                ((x1 + x2) / 2, min(y2, frame.height - 1))
                if reference == "bottom"
                else finding.center
            )
        )
        meta = frame.metadata
        if not meta.get("camera_info") or not meta["stamp_ns"]:
            raise ValueError("No matching time-stamped CameraInfo")
        intrinsics = Intrinsics.from_info(meta["camera_info"], frame)
        poses = meta.get("transforms", {})

        def pose(name):
            if name == camera.optical_frame:
                return np.eye(4)
            if name not in poses:
                raise ValueError(
                    meta.get("tf_errors", {}).get(name, f"Missing time-stamped TF for {name}")
                )
            return transform_matrix(*poses[name])

        plane_from_optical = pose(camera.plane.frame)
        point = intersect_plane(intrinsics.ray(pixel), plane_from_optical, camera.plane)
        transformed = pose(output_frame) @ np.linalg.solve(plane_from_optical, [*point, 1])
        return PlanarEstimate(
            tuple(transformed[:3]),
            output_frame,
            meta["stamp_ns"],
            camera.id,
            camera.backend_id,
            tuple(pixel),
        )
