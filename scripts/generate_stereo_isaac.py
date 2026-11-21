#!/usr/bin/env python3
"""Capture calibrated, synchronous SO-101 stereo references using Isaac Sim's SDK."""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--config", type=Path, default=Path("configs/stereo/so101.yaml"))
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--preview", action="store_true", help="Only twelve scenes for geometry review")
parser.add_argument(
    "--baseline-m", type=float, help="Override the baseline profiles with one baseline"
)
parser.add_argument("--distance-m", type=float, help="Override the working-distance profiles")
parser.add_argument(
    "--only-split",
    choices=["train", "validation", "test", "unseen", "asynchronous"],
    help="Capture one split, preserving its original scene seeds",
)
args = parser.parse_args()
repo = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(repo / "src"))
output = args.output.resolve()
if output.exists() and any(output.iterdir()):
    raise SystemExit("Choose an empty output directory to preserve existing samples")
output.mkdir(parents=True, exist_ok=True)

from isaacsim import SimulationApp  # noqa: E402 -- SDK bootstrap follows argument validation.

app = SimulationApp(
    {
        "headless": True,
        "renderer": "RaytracedLighting",
        "width": 320,
        "height": 192,
        "extra_args": [
            "--/rtx/directLighting/sampledLighting/enabled=false",
            "--/rtx/directLighting/sampledLighting/autoEnableLightCountThreshold=100000",
        ],
    }
)


def generate():
    import cv2
    import numpy as np
    import omni.replicator.core as rep
    import omni.usd
    import yaml
    from isaacsim.core.utils.semantics import add_update_semantics
    from pxr import Gf, UsdGeom, UsdLux

    from ros_robot_studio.vision.geometry import look_at_usd
    from ros_robot_studio.vision.stereo_learning.geometry import (
        calibration,
        project,
        surface_reference,
        visibility,
    )
    from ros_robot_studio.vision.stereo_learning.height_capture import (
        StrongFisheye,
        height_mount,
        place_objects,
        resample_reference,
        source_maps,
    )

    config = yaml.safe_load(args.config.read_text())
    if args.baseline_m is not None:
        config["camera"]["baseline_m"] = args.baseline_m
        config["camera"]["baselines_m"] = [args.baseline_m]
    if args.distance_m is not None:
        config["camera"]["distances_m"] = [args.distance_m]
    seed = config["seed"]
    width, height = (config["image"][key] for key in ("width", "height"))
    height_mode = config.get("generation_mode") == "height_angle"
    render_config = config.get("render", config["image"])
    render_width, render_height = (render_config[key] for key in ("width", "height"))
    render_fov = render_config["horizontal_fov_deg"]
    source = repo / "Projects/SO101_Stereo_Workcell.usda"
    assert omni.usd.get_context().open_stage(str(source))
    stage = omni.usd.get_context().get_stage()
    # The tracked fixture layer supplies the same red cube, bowl and stereo camera names as Studio.
    root = "/World/StudioStereoFixtures"
    cube = UsdGeom.Cube.Get(stage, root + "/RedCube")
    bowl = UsdGeom.Mesh.Get(stage, root + "/BlueBowl")
    assert cube and bowl
    reference = UsdGeom.Cube.Define(stage, root + "/Reference")
    reference.CreateSizeAttr(0.05)
    reference.CreateDisplayColorAttr([(0.7, 0.6, 0.2)])
    for label, geom in (("red_cube", cube), ("blue_bowl", bowl), ("reference", reference)):
        add_update_semantics(geom.GetPrim(), label)
    camera_paths = [f"/World/StudioCameras/so101_stereo_{side}" for side in ("left", "right")]
    lights = UsdLux.DomeLight.Define(stage, "/World/TrainingLight")
    lights.CreateIntensityAttr(650)
    lights.CreateColorAttr((1, 1, 1))

    def pose(prim, rotation, position):
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = position
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTransformOp().Set(Gf.Matrix4d(matrix.T.tolist()))

    # A planar procedural colour mosaic provides texture without external assets.
    table = UsdGeom.Mesh.Define(stage, "/World/TrainingTexture")
    n = 25
    vertices = [
        (x, y, -0.0045)
        for y in np.linspace(-0.35, 0.35, n + 1)
        for x in np.linspace(-0.4, 0.4, n + 1)
    ]
    faces = [
        (j * (n + 1) + i, j * (n + 1) + i + 1, (j + 1) * (n + 1) + i + 1, (j + 1) * (n + 1) + i)
        for j in range(n)
        for i in range(n)
    ]
    table.CreatePointsAttr(vertices)
    table.CreateFaceVertexCountsAttr([4] * len(faces))
    table.CreateFaceVertexIndicesAttr([k for face in faces for k in face])
    table.CreateSubdivisionSchemeAttr("none")
    if height_mode:
        add_update_semantics(table.GetPrim(), "training_table")
    table_color = table.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
    ref_tiles = UsdGeom.Mesh.Define(stage, root + "/Reference/Texture")
    vertices = [
        (x, y, 0.0251) for y in np.linspace(-0.025, 0.025, 9) for x in np.linspace(-0.025, 0.025, 9)
    ]
    faces = [
        (j * 9 + i, j * 9 + i + 1, (j + 1) * 9 + i + 1, (j + 1) * 9 + i)
        for j in range(8)
        for i in range(8)
    ]
    ref_tiles.CreatePointsAttr(vertices)
    ref_tiles.CreateFaceVertexCountsAttr([4] * 64)
    ref_tiles.CreateFaceVertexIndicesAttr([k for face in faces for k in face])
    ref_tiles.CreateSubdivisionSchemeAttr("none")
    ref_color = ref_tiles.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
    occluder = UsdGeom.Cube.Define(stage, root + "/Occluder")
    occluder.CreateSizeAttr(0.035)
    occluder.CreateDisplayColorAttr([(0.1, 0.5, 0.15)])
    if height_mode:
        add_update_semantics(occluder.GetPrim(), "occluder")
    landmarks = []
    for i, point in enumerate(
        [(0.08, -0.12, 0), (0.32, -0.12, 0), (0.08, 0.13, 0), (0.32, 0.13, 0)]
    ):
        sphere = UsdGeom.Sphere.Define(stage, f"/World/Landmark_{i}")
        sphere.CreateRadiusAttr(0.003)
        sphere.CreateDisplayColorAttr([(1, 1, 0.01)])
        pose(sphere, np.eye(3), point)
        add_update_semantics(sphere.GetPrim(), f"landmark_{i}")
        landmarks.append(list(point))

    render_products, annotators = [], []
    for path in camera_paths:
        product = rep.create.render_product(path, (render_width, render_height))
        render_products.append(product)
        group = {}
        for name in ("rgb", "distance_to_image_plane", "semantic_segmentation"):
            annotator = rep.AnnotatorRegistry.get_annotator(name)
            annotator.attach(product)
            group[name] = annotator
        annotators.append(group)
    rep.orchestrator.set_capture_on_play(False)

    def render():
        rep.orchestrator.step(rt_subframes=3, pause_timeline=True)
        result = []
        for group in annotators:
            rgb = group["rgb"].get_data()[..., :3].copy()
            depth = (
                group["distance_to_image_plane"]
                .get_data()
                .reshape(render_height, render_width)
                .copy()
            )
            semantic = group["semantic_segmentation"].get_data()
            segmentation = np.zeros((render_height, render_width), np.uint8)
            labels = {
                "red_cube": 1,
                "blue_bowl": 2,
                "reference": 3,
                "training_table": 4,
                "occluder": 6,
                **{f"landmark_{i}": 10 + i for i in range(4)},
            }
            for identity, definition in semantic["info"]["idToLabels"].items():
                if definition.get("class") in labels:
                    segmentation[
                        semantic["data"].reshape(render_height, render_width) == int(identity)
                    ] = labels[definition["class"]]
            result.append((rgb, depth, segmentation))
        return result

    schedule = []
    scene_sequence = 0
    for split, key in (
        ("train", "train_scenes"),
        ("validation", "validation_scenes"),
        ("test", "test_scenes"),
        ("unseen", "unseen_scenes"),
        ("asynchronous", "asynchronous_scenes"),
    ):
        count = config["dataset"][key] if not args.preview else (12 if split == "train" else 0)
        for index in range(count):
            offsets = config["dataset"]["time_offsets_ms"] if split == "asynchronous" else [0]
            for offset in offsets:
                schedule.append((split, index, scene_sequence, offset))
            scene_sequence += 1
    meta_manifest, checks = [], []
    source_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    for capture_index, (split, index, sequence, requested_offset) in enumerate(schedule):
        if args.only_split and split != args.only_split:
            continue
        scene_seed = seed + sequence * 1009
        rng = np.random.default_rng(scene_seed)
        baselines = config["camera"].get("baselines_m", [config["camera"]["baseline_m"]])
        distances = config["camera"]["distances_m"]
        baseline = float(baselines[index % len(baselines)])
        distance = float(distances[(index // len(baselines)) % len(distances)])
        fov = config["image"]["horizontal_fov_deg"]
        if split == "unseen" and not height_mode:
            baseline, fov = 0.12, 72.0
            distance = (0.33, 0.57)[index % 2]
        mount = {}
        if height_mode:
            eye, rotation, mount = height_mount(config["camera"], rng, index)
            distance = float(np.linalg.norm(eye - np.asarray(mount["work_center_world"])))
        else:
            target = np.array([0.2, 0, 0.025])
            eye = target + distance * np.array([0, -0.8, 0.6])
            jitter = 0.004 * config["camera"]["mount_jitter_deg"]
            eye += rng.uniform(-jitter, jitter, 3)
            look = target + rng.uniform(-jitter, jitter, 3)
            rotation = look_at_usd(eye, look)
        focal = width / (2 * math.tan(math.radians(fov) / 2))
        optical = rotation @ np.diag([1, -1, -1])
        left_pos = eye - optical[:, 0] * baseline / 2
        left_pose = np.eye(4)
        left_pose[:3, :3], left_pose[:3, 3] = optical, left_pos
        cal = calibration(width, height, focal, baseline, left_pose)
        render_focal = render_width / (2 * math.tan(math.radians(render_fov) / 2))
        render_cal = calibration(render_width, render_height, render_focal, baseline, left_pose)
        cal.update(
            left_frame=config["camera"]["optical_frames"][0],
            right_frame=config["camera"]["optical_frames"][1],
            world_frame="world",
            work_frame=config["camera"]["reference_frame"],
            work_from_world=[[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.005], [0, 0, 0, 1]],
        )
        for path, sign in zip(camera_paths, (-1, 1)):
            camera = UsdGeom.Camera.Get(stage, path)
            pose(camera, rotation, eye + sign * optical[:, 0] * baseline / 2)
            camera.GetHorizontalApertureAttr().Set(20.0)
            camera.GetVerticalApertureAttr().Set(20.0 * render_height / render_width)
            camera.GetFocalLengthAttr().Set(20.0 / (2 * math.tan(math.radians(render_fov) / 2)))
        red_pos = np.array([rng.uniform(0.06, 0.32), rng.uniform(-0.14, 0.12), 0.0225])
        if index % 10 == 0:
            red_pos[0] = -0.005 if index % 20 == 0 else 0.40
        blue_pos = np.array([rng.uniform(0.1, 0.32), rng.uniform(0.025, 0.14), -0.005])
        ref_pos = np.array([rng.uniform(0.09, 0.32), rng.uniform(-0.09, 0.08), 0.020])
        if height_mode:
            red_pos, blue_pos, ref_pos = place_objects(eye, optical, cal, rng)
            center = np.asarray(mount["work_center_world"])
            span = min(0.10, 0.45 * mount["camera_height_m"])
            landmark_radius = min(0.03, max(0.003, distance * 1.7 / focal))
            landmarks = []
            for landmark_index, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
                point = [
                    float(center[0] + sx * span),
                    float(center[1] + sy * span),
                    -0.0045 + landmark_radius,
                ]
                sphere = UsdGeom.Sphere.Get(stage, f"/World/Landmark_{landmark_index}")
                sphere.GetRadiusAttr().Set(landmark_radius)
                pose(sphere, np.eye(3), point)
                landmarks.append(point)
        pose(cube, np.eye(3), red_pos)
        pose(bowl, np.eye(3), blue_pos)
        pose(reference, np.eye(3), ref_pos)
        textured = index % 4 != 0
        for prim in cube.GetPrim().GetChildren():
            UsdGeom.Imageable(prim).GetVisibilityAttr().Set(
                "inherited" if textured else "invisible"
            )
        smooth_bowl = index % 4 == 1
        bowl.GetDisplayColorPrimvar().Set(
            [(0.01, 0.025, 0.65 if smooth_bowl else float(rng.uniform(0.25, 0.9)))] * (7 * 36)
        )
        table_color.Set([tuple(map(float, c)) for c in rng.uniform(0.1, 0.55, (25 * 25, 3))])
        ref_color.Set([tuple(map(float, c)) for c in rng.uniform(0.05, 0.95, (64, 3))])
        # Cycle independently of the 3 baselines and 3 working distances.
        occluded = (index // 9) % 2 == 0
        occ_pos = red_pos + np.array([0.005, -0.025, 0.04]) if occluded else [1, 1, -1]
        pose(occluder, np.eye(3), occ_pos)
        lights.GetIntensityAttr().Set(float(rng.uniform(450, 950)))
        source_pair = render()
        mapping = source_maps(render_cal, cal)
        pair = (
            [resample_reference(part, mapping) for part in source_pair]
            if height_mode
            else source_pair
        )
        original_right = pair[1]
        offset_ms = 0
        if split == "asynchronous":
            offset_ms = requested_offset
            # A real second rendered state moves objects at 0.3 m/s between the exposures.
            delta = np.array([0.3 * offset_ms / 1000, 0, 0])
            pose(cube, np.eye(3), red_pos + delta)
            pose(reference, np.eye(3), ref_pos + delta)
            delayed = render()
            source_pair[1] = delayed[1]
            pair[1] = resample_reference(delayed[1], mapping) if height_mode else delayed[1]
        lens = (
            StrongFisheye(render_cal, cal, config["optics"], rng, split == "unseen")
            if height_mode
            else None
        )
        if lens:
            for side, part in zip(("left", "right"), pair):
                part[1][~lens.sides[side]["support"]] = np.nan
        disparity, valid = visibility(
            pair[0][1],
            original_right[1],
            cal,
            max_depth_m=config["camera"].get("depth_range_m", [0, 2])[1] if height_mode else 2.0,
        )
        yy, xx = np.indices((height, width))
        rays = np.stack(
            ((xx - (width - 1) / 2) / focal, (yy - (height - 1) / 2) / focal, np.ones_like(xx)),
            axis=-1,
        )
        camera_xyz = rays * pair[0][1][..., None]
        world_xyz = camera_xyz @ optical.T + left_pos
        objects = []
        for label, identity in (("red_cube", 1), ("blue_bowl", 2), ("reference", 3)):
            region = (
                cv2.erode((pair[0][2] == identity).astype(np.uint8), np.ones((3, 3), np.uint8)) != 0
            )
            usable = region & np.isfinite(pair[0][1]) & (pair[0][1] > 0)
            camera_ref, world_ref = surface_reference(camera_xyz, usable, left_pose)
            ref = world_ref.tolist() if world_ref is not None else None
            objects.append(
                dict(
                    id=label,
                    mask_value=identity,
                    reference_world=ref,
                    reference_camera=camera_ref.tolist() if camera_ref is not None else None,
                    reference_work=(np.asarray(ref) + [0, 0, 0.005]).tolist() if ref else None,
                    reference_rule="coordinatewise median of optical XYZ in eroded left semantic mask; rigidly transformed to world/work",
                    support=int(usable.sum()),
                )
            )
        # Calibration validation uses actual landmark centroids and analytic table ray intersections.
        projected, _ = project(landmarks, left_pose, cal["K_left"])
        observed = []
        for i, uv in enumerate(projected):
            locations = np.argwhere(pair[0][2] == 10 + i)
            if len(locations):
                actual = locations[:, ::-1].mean(axis=0)
                observed.append(float(np.linalg.norm(actual - uv)))
        projected_right, _ = project(landmarks, cal["world_from_right"], cal["K_right"])
        observed_right = []
        for i, uv in enumerate(projected_right):
            locations = np.argwhere(pair[1][2] == 10 + i)
            if len(locations):
                observed_right.append(float(np.linalg.norm(locations[:, ::-1].mean(axis=0) - uv)))
        table_mask = (pair[0][2] == (4 if height_mode else 0)) & valid
        plane_error = np.abs(world_xyz[..., 2] + 0.0045)
        # Only pixels on the texture surface, within its finite world extent.
        plane_mask = (
            table_mask & (np.abs(world_xyz[..., 0]) < 0.39) & (np.abs(world_xyz[..., 1]) < 0.34)
        )
        plane_error_m = float(np.median(plane_error[plane_mask])) if plane_mask.any() else None
        checks.append(
            dict(
                scene=sequence,
                landmark_error_px=observed,
                right_landmark_error_px=observed_right,
                table_z_error_m=plane_error_m,
            )
        )
        if sequence < 12:
            assert (
                observed and max(observed) < 3.5
                if height_mode
                else observed and max(observed) < 2.5
            ), checks[-1]
            assert (
                observed_right and max(observed_right) < 3.5
                if height_mode
                else observed_right and max(observed_right) < 2.5
            ), checks[-1]
            assert plane_error_m is not None and plane_error_m < 0.0015, checks[-1]
            assert valid.sum() > width * height * (0.001 if height_mode else 0.2)
        group_id = f"{split}_{index:04d}"
        quality_index = int(rng.integers(1, len(config["dataset"]["blur_sigmas"])))
        jpeg_index = int(rng.integers(1, len(config["dataset"]["jpeg_qualities"])))
        for quality in ("clean", "degraded"):
            sigma = 0.0 if quality == "clean" else config["dataset"]["blur_sigmas"][quality_index]
            jpeg = 100 if quality == "clean" else config["dataset"]["jpeg_qualities"][jpeg_index]
            rgbs = []
            raws = []
            for side_index, (rgb, _, _) in enumerate(pair):
                if lens:
                    side = ("left", "right")[side_index]
                    image, raw = lens.capture(
                        source_pair[side_index][0], side, rng, quality == "degraded"
                    )
                    rgbs.append(image)
                    raws.append(raw)
                    continue
                image = rgb.copy()
                if sigma:
                    image = cv2.GaussianBlur(image, (0, 0), sigma)
                    image = np.clip(
                        (image.astype(float) - 127.5) * (0.75 + 0.25 * rng.random()) + 127.5, 0, 255
                    ).astype(np.uint8)
                    ok, encoded = cv2.imencode(
                        ".jpg",
                        cv2.cvtColor(image, cv2.COLOR_RGB2BGR),
                        [cv2.IMWRITE_JPEG_QUALITY, jpeg],
                    )
                    assert ok
                    image = cv2.cvtColor(cv2.imdecode(encoded, cv2.IMREAD_COLOR), cv2.COLOR_BGR2RGB)
                rgbs.append(image)
            identity = (
                group_id
                + (f"_offset{offset_ms:03d}" if split == "asynchronous" else "")
                + "_"
                + quality
            )
            folder = output / split
            folder.mkdir(exist_ok=True)
            raw_values = dict(raw_left=raws[0], raw_right=raws[1]) if lens else {}
            np.savez_compressed(
                folder / (identity + ".npz"),
                left=rgbs[0],
                right=rgbs[1],
                depth=pair[0][1].astype(np.float32),
                disparity=disparity,
                valid=valid,
                segmentation=pair[0][2],
                segmentation_right=pair[1][2],
                **raw_values,
            )
            metadata = dict(
                schema_version=1,
                id=identity,
                scene_id=group_id,
                frame_id=sequence,
                seed=scene_seed,
                split=split,
                simulator="Isaac Sim 5.1",
                calibration=cal,
                objects=objects,
                distance_m=distance,
                baseline_m=baseline,
                blur_sigma=sigma,
                jpeg_quality=jpeg,
                occluded=occluded,
                textured_cube=textured,
                right_time_offset_ms=offset_ms,
                left_timestamp_ns=sequence * 1_000_000_000 + 1,
                right_timestamp_ns=sequence * 1_000_000_000 + 1 + offset_ms * 1_000_000,
                source_sha256=source_hash,
                **mount,
            )
            if height_mode:
                metadata.update(
                    landmarks_world=landmarks,
                    render_calibration=render_cal,
                    optics=lens.metadata(),
                    optics_model="fisheye",
                    quality=quality,
                )
                metadata["blur_sigma"] = (
                    max(v["blur_sigma"] for v in lens.sides.values())
                    if quality == "degraded"
                    else 0.0
                )
            (folder / (identity + ".json")).write_text(json.dumps(metadata, indent=2))
            meta_manifest.append(
                dict(id=identity, scene_id=group_id, split=split, file=f"{split}/{identity}.npz")
            )
        if capture_index % 20 == 0:
            print(f"STEREO_CAPTURES {capture_index + 1}/{len(schedule)}", flush=True)
    (output / "manifest.json").write_text(
        json.dumps(
            dict(schema_version=1, config=config, source_sha256=source_hash, samples=meta_manifest),
            indent=2,
        )
    )
    (output / "geometry_checks.json").write_text(json.dumps(checks, indent=2))
    stage.GetRootLayer().Export(str(output / "last-scene.usda"))
    print(f"STEREO_DONE {len(meta_manifest)} samples", flush=True)


try:
    generate()
finally:
    app.close()
