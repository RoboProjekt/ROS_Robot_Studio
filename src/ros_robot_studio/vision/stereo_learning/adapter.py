"""Drop-in depth estimator for StereoWorkspace(estimator_factory=...)."""

import numpy as np

from ros_robot_studio.vision.stereo import PAIR_SLOP_NS, OpenCVStereo, perceive

from .inference import TorchEstimator


class LearnedStereo(OpenCVStereo):
    """Retain Studio's CameraInfo, image-time TF2, rectification and object contract."""

    def __init__(self, checkpoint, device="cpu", confidence=0.08):
        self.estimator = TorchEstimator(checkpoint, device, confidence)
        depth_range = (
            self.estimator.checkpoint["config"].get("camera", {}).get("depth_range_m", (0.2, 1.2))
        )
        super().__init__(depth_range=depth_range)

    def estimate(self, left, right, base):
        if abs(left.metadata["stamp_ns"] - right.metadata["stamp_ns"]) > PAIR_SLOP_NS:
            raise ValueError("Stereo exposure skew exceeds Studio's 25 ms pairing limit")
        return super().estimate(left, right, base)

    def compute_disparity(self, left, right):
        disparity, _, valid = self.estimator.predict(left, right)
        return np.where(valid, disparity, -1).astype(np.float32)


def estimate_objects(checkpoint, left_frame, right_frame, reference_frame, device="cpu"):
    estimator = LearnedStereo(checkpoint, device)
    result = perceive(estimator, left_frame, right_frame, reference_frame)
    return tuple(
        dict(
            object_id=obj["id"],
            xyz=obj["xyz"],
            reference_frame=reference_frame,
            timestamp_ns=result.stamp_ns,
            valid=obj["position_valid"],
            reference_kind=obj["position_kind"],
        )
        for obj in result.objects
    )
