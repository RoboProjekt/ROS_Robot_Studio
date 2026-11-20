"""Reusable estimators sharing rectification and geometry with Studio's stereo contract."""

import time

import cv2
import numpy as np


def sgbm(left, right, max_disparity=128):
    cv2.setNumThreads(1)
    disparities = int(np.ceil(max_disparity / 16)) * 16
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=disparities,
        blockSize=5,
        P1=8 * 25,
        P2=32 * 25,
        disp12MaxDiff=1,
        uniquenessRatio=8,
        speckleWindowSize=40,
        speckleRange=2,
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    a, b = (cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) for image in (left, right))
    # Padding removes SGBM's fixed search-range margin. Actual right-image bounds are checked below.
    a, b = (
        cv2.copyMakeBorder(image, 0, 0, disparities, 0, cv2.BORDER_CONSTANT) for image in (a, b)
    )
    disparity = matcher.compute(a, b).astype(np.float32)[:, disparities:] / 16
    reverse = (
        cv2.ximgproc.createRightMatcher(matcher).compute(b, a).astype(np.float32)[:, disparities:]
        / 16
    )
    yy, xx = np.indices(disparity.shape)
    xright = xx - disparity
    back = cv2.remap(
        reverse,
        xright.astype(np.float32),
        yy.astype(np.float32),
        cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=999,
    )
    valid = (
        (disparity > 0)
        & (xright >= 0)
        & (xright < left.shape[1] - 1)
        & (np.abs(back + disparity) < 1.5)
    )
    return disparity, valid.astype(np.float32)


class TorchEstimator:
    def __init__(self, checkpoint, device="cpu", confidence=0.08):
        import torch

        from .training import restore_model

        self.torch = torch
        self.device = device
        self.threshold = confidence
        self.model, self.checkpoint = restore_model(checkpoint, device)
        self.size = (
            self.checkpoint["config"]["image"]["width"],
            self.checkpoint["config"]["image"]["height"],
        )

    def predict(self, left, right):
        torch = self.torch
        if left.shape != right.shape or left.ndim != 3 or left.shape[2] != 3:
            raise ValueError("Two same-size RGB arrays are required")
        if left.dtype != np.uint8 or right.dtype != np.uint8:
            raise ValueError("RGB inputs must be uint8 in the range 0..255")
        width, height = self.size
        tensors = [
            torch.from_numpy(cv2.resize(image, (width, height)).transpose(2, 0, 1).copy())
            .float()
            .unsqueeze(0)
            .to(self.device)
            / 255
            for image in (left, right)
        ]
        with torch.inference_mode():
            disparity, quality = self.model(*tensors)
        result = cv2.resize(disparity[0, 0].cpu().numpy(), (left.shape[1], left.shape[0]))
        result *= left.shape[1] / width
        quality = cv2.resize(quality[0, 0].cpu().numpy(), (left.shape[1], left.shape[0]))
        yy, xx = np.indices(result.shape)
        valid = (quality >= self.threshold) & (xx - result >= 0) & (result > 0)
        return result, quality, valid


def timed_model(estimator, left, right):
    if estimator.device.startswith("cuda"):
        estimator.torch.cuda.synchronize()
    begin = time.perf_counter()
    result = estimator.predict(left, right)
    if estimator.device.startswith("cuda"):
        estimator.torch.cuda.synchronize()
    return result, (time.perf_counter() - begin) * 1000
