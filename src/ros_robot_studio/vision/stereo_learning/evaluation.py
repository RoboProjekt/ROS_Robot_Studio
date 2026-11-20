"""Matched-image benchmarks, including coverage, subgroup errors and rendered failure cases."""

import json
import os
import subprocess
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

from .geometry import reconstruct
from .inference import TorchEstimator, sgbm, timed_model


def process_gpu_memory_mb():
    """Sample this process's driver allocation, including context overhead when supported."""
    try:
        output = subprocess.check_output(
            [
                "nvidia-smi",
                "--query-compute-apps=pid,used_gpu_memory",
                "--format=csv,noheader,nounits",
            ],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=5,
        )
        for line in output.splitlines():
            pid, memory = line.split(",")
            if int(pid.strip()) == os.getpid():
                return float(memory.strip())
    except (OSError, ValueError, subprocess.SubprocessError):
        pass
    return None


def position(points, region, cal):
    region = cv2.erode(region.astype(np.uint8), np.ones((3, 3), np.uint8)) != 0
    total = int(region.sum())
    cloud = points[region]
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    if total < 16:
        return None, "object region smaller than 16 pixels"
    if len(cloud) < 16 or len(cloud) < 0.10 * total:
        return None, "insufficient valid stereo support"
    median = np.median(cloud, axis=0)
    if np.median(np.abs(cloud[:, 2] - median[2])) > 0.035:
        return None, "depth spread exceeds 35 mm"
    world = (np.asarray(cal["world_from_left"]) @ np.r_[median, 1])[:3]
    return world, None


def summarize(rows):
    result = {}
    for name in ("sgbm", "learned"):
        selected = [r for r in rows if r["method"] == name]
        if not selected:
            continue
        errors = np.concatenate([r["depth_errors_mm"] for r in selected])
        shared = np.concatenate([r["common_depth_errors_mm"] for r in selected])
        fg = np.concatenate([r["foreground_errors_mm"] for r in selected])
        obj = [o for r in selected for o in r["objects"]]
        valid = [o["error_mm"] for o in obj if o["error_mm"] is not None]
        reasons = defaultdict(int)
        for o in obj:
            if o["error_mm"] is None:
                reasons[o["reason"]] += 1
        timing = {
            key: float(np.median([r[key] for r in selected]))
            for key in ("rectification_ms", "estimator_ms", "postprocess_ms")
        }
        result[name] = dict(
            samples=len(selected),
            valid_pixels=sum(r["valid_pixels"] for r in selected),
            reference_pixels=sum(r["reference_pixels"] for r in selected),
            valid_pixel_fraction=sum(r["valid_pixels"] for r in selected)
            / max(1, sum(r["reference_pixels"] for r in selected)),
            depth_median_mm=float(np.median(errors)) if errors.size else None,
            depth_p95_mm=float(np.percentile(errors, 95)) if errors.size else None,
            common_valid_pixels=int(shared.size),
            common_depth_median_mm=float(np.median(shared)) if shared.size else None,
            common_depth_p95_mm=float(np.percentile(shared, 95)) if shared.size else None,
            foreground_median_mm=float(np.median(fg)) if fg.size else None,
            foreground_p95_mm=float(np.percentile(fg, 95)) if fg.size else None,
            valid_objects=len(valid),
            object_opportunities=len(obj),
            object_valid_fraction=len(valid) / max(1, len(obj)),
            object_median_mm=float(np.median(valid)) if valid else None,
            object_p95_mm=float(np.percentile(valid, 95)) if valid else None,
            rejected=dict(reasons),
            timing_median_ms=timing,
        )
    return result


def marker_triangulation(sample, metadata):
    cal = metadata["calibration"]
    records = []
    for label in range(10, 14):
        a = np.argwhere(sample["segmentation"] == label)
        b = np.argwhere(sample["segmentation_right"] == label)
        if not len(a) or not len(b):
            continue
        # Semantic IDs associate the known rendered fiducials. Coordinates use actual pixel centroids.
        left, right = a[:, ::-1].mean(0), b[:, ::-1].mean(0)
        homogeneous = cv2.triangulatePoints(
            np.asarray(cal["P_left"]),
            np.asarray(cal["P_right"]),
            left.reshape(2, 1),
            right.reshape(2, 1),
        )
        xyz = homogeneous[:3, 0] / homogeneous[3, 0]
        world = (np.asarray(cal["world_from_left"]) @ np.r_[xyz, 1])[:3]
        truth = np.array([(0.08, -0.12, 0), (0.32, -0.12, 0), (0.08, 0.13, 0), (0.32, 0.13, 0)])[
            label - 10
        ]
        if xyz[2] > 0 and abs(left[1] - right[1]) < 1.5:
            records.append(float(np.linalg.norm(world - truth) * 1000))
    return records


def object_classes(rows):
    result = {}
    for identity in ("red_cube", "blue_bowl", "reference"):
        result[identity] = {}
        for method in ("sgbm", "learned"):
            objects = [
                o
                for row in rows
                if row["method"] == method
                for o in row["objects"]
                if o["id"] == identity
            ]
            errors = [obj["error_mm"] for obj in objects if obj["error_mm"] is not None]
            result[identity][method] = dict(
                valid=len(errors),
                opportunities=len(objects),
                median_mm=float(np.median(errors)) if errors else None,
                p95_mm=float(np.percentile(errors, 95)) if errors else None,
            )
    return result


def benchmark(config, dataset, checkpoint, output, *, split="test", device=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    root = Path(dataset)
    manifest = json.loads((root / "manifest.json").read_text())
    records = [r for r in manifest["samples"] if r["split"] == split]
    if not records:
        raise ValueError("Empty benchmark split")
    estimator = TorchEstimator(checkpoint, device or config["train"]["device"])
    torch = estimator.torch
    torch.set_num_threads(config["train"]["threads"])
    sample = np.load(root / records[0]["file"])
    for _ in range(3):
        estimator.predict(sample["left"], sample["right"])
    if estimator.device.startswith("cuda"):
        torch.cuda.reset_peak_memory_stats()
    rows, markers, rectification_maps = [], [], {}
    examples = []
    for index, record in enumerate(records):
        filename = root / record["file"]
        meta = json.loads(filename.with_suffix(".json").read_text())
        cal = meta["calibration"]
        with np.load(filename) as archive:
            sample = {key: archive[key].copy() for key in archive.files}
        # Apply actual OpenCV remaps even though the synthetic pinhole calibration is already rectified.
        # Map setup is cached by intrinsics, as it is in Studio. GT and Q use this same pixel grid.
        start = time.perf_counter()
        a, b = sample["left"], sample["right"]
        assert cal["rectified"] and a.shape[:2] == (cal["height"], cal["width"])
        key = (
            cal["width"],
            cal["height"],
            tuple(np.ravel(cal["K_left"])),
            tuple(np.ravel(cal["K_right"])),
        )
        if key not in rectification_maps:
            rectification_maps[key] = tuple(
                cv2.initUndistortRectifyMap(
                    np.asarray(cal["K_" + side]),
                    None,
                    np.eye(3),
                    np.asarray(cal["P_" + side]),
                    (cal["width"], cal["height"]),
                    cv2.CV_16SC2,
                )
                for side in ("left", "right")
            )
        a, b = (
            cv2.remap(image, *mapping, cv2.INTER_LINEAR)
            for image, mapping in zip((a, b), rectification_maps[key])
        )
        rect_ms = (time.perf_counter() - start) * 1000
        start = time.perf_counter()
        classical, classical_quality = sgbm(a, b, config["model"]["max_disparity"])
        classic_ms = (time.perf_counter() - start) * 1000
        (learned, quality, valid), learned_ms = timed_model(estimator, a, b)
        predictions = {
            "sgbm": (classical, classical_quality.astype(bool), classic_ms),
            "learned": (learned, valid, learned_ms),
        }
        markers.extend(marker_triangulation(sample, meta))
        stored, clouds = {}, {}
        for method, (disparity, pred_valid, elapsed) in predictions.items():
            start = time.perf_counter()
            points = reconstruct(disparity, cal, pred_valid)
            low, high = config["camera"]["depth_range_m"]
            accepted = (
                np.isfinite(points).all(2) & (points[..., 2] >= low) & (points[..., 2] <= high)
            )
            points[~accepted] = np.nan
            clouds[method] = points
            target = sample["valid"] & (sample["depth"] > low) & (sample["depth"] < high)
            common = target & accepted
            depth_errors = np.abs(points[..., 2][common] - sample["depth"][common]) * 1000
            foreground = (sample["segmentation"] > 0) & (sample["segmentation"] < 4)
            fg_common = common & foreground
            foreground_errors = (
                np.abs(points[..., 2][fg_common] - sample["depth"][fg_common]) * 1000
            )
            objects = []
            for obj in meta["objects"]:
                world, reason = position(points, sample["segmentation"] == obj["mask_value"], cal)
                if obj["reference_world"] is None:
                    world, reason = None, "reference object not visible in left image"
                error = (
                    float(np.linalg.norm(world - np.asarray(obj["reference_world"])) * 1000)
                    if world is not None
                    else None
                )
                objects.append(
                    dict(
                        id=obj["id"],
                        error_mm=error,
                        xyz=world.tolist() if world is not None else None,
                        reason=reason,
                    )
                )
            post_ms = (time.perf_counter() - start) * 1000
            rows.append(
                dict(
                    id=record["id"],
                    method=method,
                    split=split,
                    distance_m=meta["distance_m"],
                    baseline_m=meta["baseline_m"],
                    blur_sigma=meta["blur_sigma"],
                    occluded=meta["occluded"],
                    textured_cube=meta["textured_cube"],
                    time_offset_ms=meta["right_time_offset_ms"],
                    depth_errors_mm=depth_errors,
                    foreground_errors_mm=foreground_errors,
                    reference_pixels=int(target.sum()),
                    valid_pixels=int(common.sum()),
                    objects=objects,
                    rectification_ms=rect_ms,
                    estimator_ms=elapsed,
                    postprocess_ms=post_ms,
                )
            )
            stored[method] = np.where(accepted, disparity, np.nan)
        shared_mask = (
            target & np.isfinite(clouds["sgbm"]).all(2) & np.isfinite(clouds["learned"]).all(2)
        )
        for row in rows[-2:]:
            row["common_depth_errors_mm"] = (
                np.abs(clouds[row["method"]][..., 2][shared_mask] - sample["depth"][shared_mask])
                * 1000
            )
        examples.append((record["id"], sample, stored, rows[-1]))
        if index % 30 == 0:
            print(f"BENCHMARK {split} {index + 1}/{len(records)}", flush=True)
    overall = summarize(rows)
    groups = {}
    for field in (
        "distance_m",
        "baseline_m",
        "blur_sigma",
        "occluded",
        "textured_cube",
        "time_offset_ms",
    ):
        groups[field] = {
            str(value): summarize([r for r in rows if r[field] == value])
            for value in sorted(set(r[field] for r in rows))
        }
    report = dict(
        split=split,
        overall=overall,
        groups=groups,
        object_classes=object_classes(rows),
        marker_triangulation=dict(
            valid=len(markers),
            median_mm=float(np.median(markers)) if markers else None,
            p95_mm=float(np.percentile(markers, 95)) if markers else None,
        ),
        model_checkpoint=str(Path(checkpoint).resolve()),
        model_peak_allocated_mb=torch.cuda.max_memory_allocated() / 2**20
        if estimator.device.startswith("cuda")
        else 0,
        model_peak_reserved_mb=torch.cuda.max_memory_reserved() / 2**20
        if estimator.device.startswith("cuda")
        else 0,
        model_process_gpu_mb_at_end=process_gpu_memory_mb()
        if estimator.device.startswith("cuda")
        else 0,
    )
    (output / f"{split}.json").write_text(json.dumps(report, indent=2))
    serialized = [
        {
            k: v
            for k, v in row.items()
            if k not in ("depth_errors_mm", "foreground_errors_mm", "common_depth_errors_mm")
        }
        for row in rows
    ]
    (output / f"{split}-per-sample.json").write_text(json.dumps(serialized, indent=2))
    make_figures(examples, output, split)
    return report


def make_figures(examples, output, split):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ranked = sorted(
        examples,
        key=lambda r: np.mean(
            [o["error_mm"] if o["error_mm"] is not None else 200 for o in r[3]["objects"]]
        ),
    )
    selected = [("successful", ranked[0]), ("difficult", ranked[-1])]
    for label, (identity, sample, stored, row) in selected:
        fig, axes = plt.subplots(1, 5, figsize=(17, 3.6), layout="constrained")
        cmap = plt.get_cmap("turbo").copy()
        cmap.set_bad("#25282c")
        images = [
            sample["left"],
            sample["right"],
            np.where(sample["valid"], sample["disparity"], np.nan),
            stored["sgbm"],
            stored["learned"],
        ]
        titles = [
            "Left RGB",
            "Right RGB",
            "Simulator disparity",
            "StereoSGBM disparity",
            "Trained disparity",
        ]
        for ax, image, title in zip(axes, images, titles):
            rendered = ax.imshow(
                image,
                cmap=cmap,
                vmin=0 if image.ndim == 2 else None,
                vmax=128 if image.ndim == 2 else None,
            )
            ax.set_title(title)
            ax.axis("off")
        fig.suptitle(f"Isaac Sim · {identity} · {label} · disparity in pixels")
        fig.colorbar(
            rendered, ax=list(axes[2:]), fraction=0.025, pad=0.01, label="pixels (dark = invalid)"
        )
        fig.savefig(output / f"{split}-{label}.png", dpi=140)
        plt.close(fig)
