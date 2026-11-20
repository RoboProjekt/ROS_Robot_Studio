#!/usr/bin/env python3
"""Verify generated stereo geometry, scene splits, reference frames and artifact hashes."""

import argparse
import hashlib
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from ros_robot_studio.vision.stereo_learning.geometry import (  # noqa: E402
    depth_to_disparity,
    project,
    reconstruct,
    surface_reference,
)


def audit(root):
    manifest_path = root / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    groups, offsets, integrity, controls = {}, defaultdict(set), [], {}
    identities = [record["id"] for record in manifest["samples"]]
    assert len(identities) == len(set(identities)), "Duplicate sample identity"
    errors, preview = defaultdict(list), defaultdict(list)
    counts = Counter()
    landmarks = [[0.08, -0.12, 0], [0.32, -0.12, 0], [0.08, 0.13, 0], [0.32, 0.13, 0]]
    for record in manifest["samples"]:
        path = root / record["file"]
        metadata = path.with_suffix(".json")
        meta = json.loads(metadata.read_text())
        cal = meta["calibration"]
        scene, split = record["scene_id"], record["split"]
        assert groups.setdefault(scene, split) == split, "Scene leakage"
        counts[split] += 1
        control = (meta["seed"], cal, meta["objects"])
        if scene in controls:
            original = controls[scene]
            assert control[:2] == original[:2], "Variants must share seed and calibration"
            for a, b in zip(control[2], original[2]):
                if a["reference_camera"] is None:
                    assert b["reference_camera"] is None
                else:
                    np.testing.assert_allclose(
                        a["reference_camera"], b["reference_camera"], atol=1e-6
                    )
        else:
            controls[scene] = control
        if split == "asynchronous":
            offsets[scene].add(meta["right_time_offset_ms"])
        pose = np.asarray(cal["world_from_left"])
        right = np.asarray(cal["world_from_right"])
        np.testing.assert_allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-7)
        np.testing.assert_allclose(
            right[:3, 3] - pose[:3, 3], pose[:3, 0] * cal["baseline_m"], atol=1e-7
        )
        with np.load(path) as sample:
            shape = (cal["height"], cal["width"])
            assert sample["left"].shape == sample["right"].shape == (*shape, 3)
            assert sample["left"].dtype == sample["right"].dtype == np.uint8
            assert sample["valid"].dtype == bool
            mask = sample["valid"]
            expected = depth_to_disparity(sample["depth"], cal)
            np.testing.assert_allclose(sample["disparity"][mask], expected[mask], atol=1e-5)
            points = reconstruct(sample["disparity"], cal, mask)
            np.testing.assert_allclose(points[..., 2][mask], sample["depth"][mask], atol=1e-6)
            yy, xx = np.indices(shape)
            k = np.asarray(cal["K_left"])
            xyz = (
                np.stack(((xx - k[0, 2]) / k[0, 0], (yy - k[1, 2]) / k[1, 1], np.ones(shape)), -1)
                * sample["depth"][..., None]
            )
            for obj in meta["objects"]:
                region = (
                    cv2.erode(
                        (sample["segmentation"] == obj["mask_value"]).astype(np.uint8),
                        np.ones((3, 3), np.uint8),
                    )
                    != 0
                )
                camera_ref, world_ref = surface_reference(xyz, region, pose)
                if camera_ref is None:
                    assert obj["reference_camera"] is None
                else:
                    np.testing.assert_allclose(camera_ref, obj["reference_camera"], atol=1e-6)
                    np.testing.assert_allclose(world_ref, obj["reference_world"], atol=1e-6)
                    work_ref = (np.asarray(cal["work_from_world"]) @ np.r_[world_ref, 1])[:3]
                    np.testing.assert_allclose(work_ref, obj["reference_work"], atol=1e-6)
            if not record["id"].endswith("clean"):
                continue
            for side, key in (("left", "segmentation"), ("right", "segmentation_right")):
                uv, _ = project(landmarks, cal["world_from_" + side], cal["K_" + side])
                for i, point in enumerate(uv):
                    pixels = np.argwhere(sample[key] == 10 + i)
                    if len(pixels):
                        error = float(np.linalg.norm(pixels[:, ::-1].mean(0) - point))
                        errors[side].append(error)
                        if split == "train" and meta["frame_id"] < 12:
                            preview[side].append(error)
        # Hash every variant, including degraded pairs.
    for record in manifest["samples"]:
        path = root / record["file"]
        integrity.append(
            dict(
                file=record["file"],
                sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                metadata_sha256=hashlib.sha256(path.with_suffix(".json").read_bytes()).hexdigest(),
            )
        )
    assert all(
        value == set(manifest["config"]["dataset"]["time_offsets_ms"]) for value in offsets.values()
    )
    for side in ("left", "right"):
        assert max(preview[side]) < 2.5
    checks = json.loads((root / "geometry_checks.json").read_text())
    plane_max_mm = (
        max(item["table_z_error_m"] for item in checks if item["table_z_error_m"] is not None)
        * 1000
    )
    assert plane_max_mm < 1.5
    report = dict(
        pairs=sum(counts.values()),
        scenes=len(groups),
        splits=dict(counts),
        manifest_sha256=hashlib.sha256(manifest_path.read_bytes()).hexdigest(),
        max_table_plane_error_mm=plane_max_mm,
        matched_asynchronous_scenes=len(offsets),
        projection={
            side: dict(
                count=len(values),
                median_px=float(np.median(values)),
                p95_px=float(np.percentile(values, 95)),
                max_px=max(values),
                preview_max_px=max(preview[side]),
            )
            for side, values in errors.items()
        },
    )
    (root / "data-integrity.json").write_text(json.dumps(integrity, indent=2))
    (root / "audit.json").write_text(json.dumps(report, indent=2))
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(audit(args.dataset), indent=2))
