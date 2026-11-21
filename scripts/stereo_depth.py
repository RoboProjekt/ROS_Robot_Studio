#!/usr/bin/env python3
"""Standalone train, resume, benchmark, inference and ONNX commands for SO-101 stereo."""

import argparse
import hashlib
import json
import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("CUBLAS_WORKSPACE_CONFIG", ":4096:8")
os.environ.setdefault("OMP_NUM_THREADS", "2")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/robot-studio-stereo-matplotlib")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import yaml

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, default=Path("configs/stereo/so101.yaml"))
commands = parser.add_subparsers(dest="command", required=True)
training = commands.add_parser("train")
training.add_argument("--dataset", type=Path, required=True)
training.add_argument("--output", type=Path, required=True)
training.add_argument("--resume", type=Path)
training.add_argument("--epochs", type=int)
training.add_argument(
    "--overfit", type=int, help="Use this many training samples for an overfit check"
)
benchmark = commands.add_parser("benchmark")
benchmark.add_argument("--dataset", type=Path, required=True)
benchmark.add_argument("--checkpoint", type=Path, required=True)
benchmark.add_argument("--output", type=Path, required=True)
benchmark.add_argument(
    "--split", choices=["validation", "test", "unseen", "asynchronous"], default="test"
)
benchmark.add_argument("--device")
robustness = commands.add_parser("robustness", help="Matched mild blur/lens/calibration tests")
robustness.add_argument("--dataset", type=Path, required=True)
robustness.add_argument("--checkpoint", type=Path, required=True)
robustness.add_argument("--output", type=Path, required=True)
robustness.add_argument(
    "--effects-config", type=Path, default=Path("configs/stereo/robustness-mild.yaml")
)
robustness.add_argument("--device")
infer = commands.add_parser("infer")
infer.add_argument("--checkpoint", type=Path, required=True)
infer.add_argument("--left", type=Path, required=True)
infer.add_argument("--right", type=Path, required=True)
infer.add_argument(
    "--calibration", type=Path, required=True, help="Sample metadata JSON or calibration JSON"
)
infer.add_argument("--output", type=Path, required=True)
infer.add_argument("--device", default="cpu")
infer.add_argument(
    "--raw-fisheye",
    action="store_true",
    help="Rectify raw fisheye RGB using per-pair optics metadata",
)
export = commands.add_parser("export")
export.add_argument("--checkpoint", type=Path, required=True)
export.add_argument("--output", type=Path, required=True)
export.add_argument("--sample", type=Path, help="Use a real saved NPZ pair for ONNX parity")
unpack = commands.add_parser("unpack", help="Save a generated NPZ pair as RGB PNGs and metadata")
unpack.add_argument("--sample", type=Path, required=True)
unpack.add_argument("--output", type=Path, required=True)
unpack.add_argument(
    "--raw", action="store_true", help="Unpack raw fisheye exposures instead of rectified RGB"
)


def main():
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if args.command == "train":
        from ros_robot_studio.vision.stereo_learning.training import train

        report = train(
            config,
            args.dataset,
            args.output,
            resume=args.resume,
            epochs=args.epochs,
            overfit=args.overfit,
        )
    elif args.command == "benchmark":
        from ros_robot_studio.vision.stereo_learning.evaluation import benchmark

        report = benchmark(
            config, args.dataset, args.checkpoint, args.output, split=args.split, device=args.device
        )
    elif args.command == "robustness":
        from ros_robot_studio.vision.stereo_learning.robustness import run

        report = run(
            config, args.dataset, args.checkpoint, args.output, args.effects_config, args.device
        )
    elif args.command == "infer":
        import cv2
        import numpy as np

        from ros_robot_studio.vision.stereo_learning.geometry import reconstruct
        from ros_robot_studio.vision.stereo_learning.inference import TorchEstimator

        left, right = (cv2.imread(str(path)) for path in (args.left, args.right))
        if left is None or right is None:
            raise ValueError("Both RGB image files must be readable")
        left, right = (cv2.cvtColor(image, cv2.COLOR_BGR2RGB) for image in (left, right))
        value = json.loads(args.calibration.read_text())
        cal = value.get("calibration", value)
        photographic_support = None
        if args.raw_fisheye:
            from ros_robot_studio.vision.stereo_learning.camera_effects import (
                pixel_grid,
                project_raw,
                remap,
            )

            if value.get("optics_model") != "fisheye" or "optics" not in value:
                raise ValueError(
                    "Raw fisheye inference requires calibrated per-camera optics metadata"
                )
            grid = pixel_grid(cal["width"], cal["height"])
            corrected = []
            photographic_support = []
            for side, image in zip(("left", "right"), (left, right)):
                optical = value["optics"][side]
                mapping = project_raw(
                    grid,
                    cal["K_" + side],
                    optical["distortion"],
                    "fisheye",
                    destination_k=optical["raw_k"],
                )
                corrected.append(remap(image, mapping))
                photographic_support.append(
                    remap(np.ones(image.shape[:2], np.float32), mapping) > 0.99999
                )
            left, right = corrected
        if not cal.get("rectified"):
            raise ValueError("Provide rectified RGB images and their rectified calibration")
        estimator = TorchEstimator(args.checkpoint, args.device)
        disparity, quality, valid = estimator.predict(left, right)
        if photographic_support is not None:
            yy, xx = np.indices(disparity.shape, dtype=np.float32)
            right_support = cv2.remap(
                photographic_support[1].astype(np.float32),
                xx - disparity,
                yy,
                cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0,
            )
            valid &= photographic_support[0] & (right_support > 0.99999)
        points = reconstruct(disparity, cal, valid)
        valid &= np.isfinite(points).all(axis=2)
        limits = (
            estimator.checkpoint["config"].get("camera", {}).get("depth_range_m", (0, float("inf")))
        )
        valid &= (points[..., 2] >= limits[0]) & (points[..., 2] <= limits[1])
        points[~valid] = np.nan
        args.output.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            args.output,
            disparity=disparity,
            confidence=quality,
            valid=valid,
            optical_xyz_m=points,
            depth_m=points[..., 2],
        )
        report = dict(output=str(args.output.resolve()), valid_fraction=float(valid.mean()))
    elif args.command == "unpack":
        import cv2
        import numpy as np

        args.output.mkdir(parents=True, exist_ok=True)
        with np.load(args.sample) as sample:
            for side in ("left", "right"):
                if not cv2.imwrite(
                    str(args.output / f"{side}.png"),
                    cv2.cvtColor(sample[("raw_" if args.raw else "") + side], cv2.COLOR_RGB2BGR),
                ):
                    raise OSError("Could not save the RGB image")
        shutil.copyfile(args.sample.with_suffix(".json"), args.output / "metadata.json")
        report = dict(output=str(args.output.resolve()))
    else:
        import numpy as np
        import onnxruntime as ort
        import torch

        from ros_robot_studio.vision.stereo_learning.training import restore_model

        model, value = restore_model(args.checkpoint)
        width, height = (value["config"]["image"][k] for k in ("width", "height"))
        torch.manual_seed(value["config"]["seed"])
        examples = (torch.rand(1, 3, height, width), torch.rand(1, 3, height, width))
        if args.sample:
            with np.load(args.sample) as sample:
                examples = tuple(
                    torch.from_numpy(sample[key].transpose(2, 0, 1).copy()).float().unsqueeze(0)
                    / 255
                    for key in ("left", "right")
                )
        args.output.parent.mkdir(parents=True, exist_ok=True)
        torch.onnx.export(
            model,
            examples,
            str(args.output),
            input_names=["left_rgb", "right_rgb"],
            output_names=["left_disparity_px", "confidence"],
            opset_version=17,
            dynamo=False,
        )
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        session = ort.InferenceSession(
            str(args.output), options, providers=["CPUExecutionProvider"]
        )
        with torch.inference_mode():
            expected = model(*examples)
        actual = session.run(
            None, dict(zip(("left_rgb", "right_rgb"), (x.numpy() for x in examples)))
        )
        differences = [float(np.max(np.abs(a - b.numpy()))) for a, b in zip(actual, expected)]
        assert differences[0] < 0.002 and differences[1] < 0.0002, differences
        effective_geometry = dict(value["config"].get("camera", {}))
        if value["config"].get("generation_mode") == "height_angle":
            effective_geometry.pop("distances_m", None)
            effective_geometry.pop("mount_jitter_deg", None)
            effective_geometry["relative_rotation_deg"] = [0, 0, 0]
        manifest = dict(
            schema_version=1,
            architecture="CompactStereo",
            license="Apache-2.0",
            pytorch_dependency_license="BSD-3-Clause",
            checkpoint=str(args.checkpoint.resolve()),
            checkpoint_sha256=hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
            onnx=args.output.name,
            onnx_sha256=hashlib.sha256(args.output.read_bytes()).hexdigest(),
            dataset_fingerprint=value["dataset_hash"],
            seed=value["config"]["seed"],
            selected_epoch_1based=value["epoch"] + 1,
            selected_validation_epe_px=value["best"],
            parameters=sum(parameter.numel() for parameter in model.parameters()),
            opset=17,
            input_shape=[1, 3, height, width],
            input_dtype="float32",
            channels="RGB",
            normalization="divide uint8 by 255; range 0..1",
            preprocessing="rectify using pair-specific calibration; resize to training shape",
            padding="internal convolution zero padding; no external padding",
            output="left rectified disparity in pixels at the input resolution",
            output_to_original="bilinear resize and multiply disparity by original_width / model_width",
            confidence="maximum coarse disparity softmax probability, not calibrated accuracy",
            minimum_confidence=0.08,
            max_disparity_px=value["model_config"]["max_disparity"],
            geometry="use original-resolution calibrated Q including principal-point shift",
            camera_geometry=effective_geometry,
            optics_training=value["config"].get("optics", {}),
            runtime_differences=differences,
        )
        args.output.with_suffix(".json").write_text(json.dumps(manifest, indent=2))
        report = manifest
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
