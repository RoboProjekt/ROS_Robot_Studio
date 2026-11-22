import base64
import json
import time

import numpy as np
import pytest

from ros_robot_studio.gui.stereo_workspace import table_height
from ros_robot_studio.vision.moveit import ARM, make_request
from ros_robot_studio.vision.orchestrator import (
    Clarification,
    Proposal,
    chat,
    ground_objects,
    mock_reply,
    target_geometry,
    validate_reply,
)
from ros_robot_studio.vision.stereo import (
    OpenCVStereo,
    StereoPairer,
    perceive,
    perceive_usb_pair,
    stereo_profiles,
)
from ros_robot_studio.vision.stream import RGBFrame


def synthetic_pair(distance, *, disparity=True, wall=None):
    cv = pytest.importorskip("cv2")
    rng = np.random.default_rng(101)
    w, h, focal = 640, 480, 360
    image = rng.integers(35, 130, (h, w), dtype=np.uint8).repeat(3).reshape(h, w, 3)
    texture = rng.integers(120, 245, (55, 55), dtype=np.uint8)
    image[210:265, 305:360] = np.stack(
        (texture, np.full_like(texture, 12), np.full_like(texture, 12)), 2
    )
    mask = np.zeros((h, w), np.uint8)
    cv.circle(mask, (453, 237), 26, 1, 9)
    image[mask != 0, :2] = (16, 32)
    image[mask != 0, 2] = rng.integers(145, 245, np.count_nonzero(mask))
    shift = round(focal * 0.08 / distance)
    right = np.zeros_like(image)
    if disparity:
        right[:, :-shift] = image[:, shift:]
    else:
        right.fill(100)
    stamp = 2_000_000_000
    wall = time.time_ns() if wall is None else wall
    frames = []
    for side, rgb, x in (("left", image, 0), ("right", right, 0.08)):
        frame = side + "_optical"
        frames.append(
            RGBFrame(
                rgb,
                dict(
                    stamp_ns=stamp,
                    received_wall_ns=wall,
                    frame_id=frame,
                    transforms={"robot_base": ([x, 0, 0], [0, 0, 0, 1])},
                    camera_info=dict(
                        width=w,
                        height=h,
                        frame_id=frame,
                        k=[focal, 0, w / 2, 0, focal, h / 2, 0, 0, 1],
                        d=[],
                        model="",
                    ),
                ),
            )
        )
    return tuple(frames)


@pytest.mark.parametrize("distance", [0.3, 0.45, 0.6])
def test_stereo_recovers_three_depths(distance):
    result = perceive(OpenCVStereo(), *synthetic_pair(distance), "robot_base")
    cube = next(candidate for candidate in result.objects if candidate["id"] == "red_cube")
    assert cube["position_valid"]
    assert cube["xyz"][2] == pytest.approx(distance, abs=0.015)


def test_missing_depth_retains_semantics_and_blocks_planning():
    result = perceive(OpenCVStereo(), *synthetic_pair(0.45, disparity=False), "robot_base")
    snapshot = {"scene_id": "missing-depth", "objects": result.objects, "table_z": 0.0}
    reply = validate_reply(mock_reply("Success", snapshot), snapshot, "Pick the red cube")
    assert isinstance(reply, Proposal)
    with pytest.raises(ValueError, match="Valid stereo depth"):
        target_geometry(reply, snapshot)
    assert isinstance(
        validate_reply(mock_reply("Ambiguous", snapshot), snapshot, ""), Clarification
    )
    with pytest.raises(ValueError, match="unknown"):
        validate_reply(mock_reply("Unknown ID", snapshot), snapshot, "")
    with pytest.raises(ValueError, match="JSON"):
        validate_reply(mock_reply("Invalid JSON", snapshot), snapshot, "")


def test_pairer_timestamp_reset_and_old_frames():
    left, right = synthetic_pair(0.45, wall=1_000_000_000)
    pairer = StereoPairer()
    pairer.push(0, left, 1_000_000_000)
    pairer.push(1, right, 1_000_000_000)
    assert pairer.pair(1_000_000_000) == (left, right)
    frame = RGBFrame(right.rgb, dict(right.metadata, stamp_ns=1))
    pairer.push(0, RGBFrame(left.rgb, dict(left.metadata, stamp_ns=3_000_000_000)), 1_000_000_000)
    pairer.push(1, frame, 1_000_000_000)
    assert pairer.pair(1_000_000_000) is None
    pairer.push(0, left, 3_000_000_000)
    assert pairer.pair(3_000_000_000) is None


def test_stereo_profiles_are_parallel_with_unique_ros_names():
    left, right = stereo_profiles("simulation", "base_link")
    assert np.linalg.norm(np.subtract(left.mount.position, right.mount.position)) == pytest.approx(
        0.08
    )
    assert left.mount.usd_rpy_deg == right.mount.usd_rpy_deg
    assert left.optical_frame != right.optical_frame
    assert left.rgb_topic != right.rgb_topic


def test_real_stereo_requires_matching_policy_depth_and_leaves_xyz_unset():
    left, right = synthetic_pair(0.45)
    wall = left.metadata["received_wall_ns"]
    pair = RGBFrame(np.concatenate((left.rgb, right.rgb), axis=1), left.metadata)
    levels = np.full_like(left.rgb, 110)
    depth = RGBFrame(levels, dict(left.metadata, encoding="mono8"))
    older = RGBFrame(levels, dict(depth.metadata, stamp_ns=left.metadata["stamp_ns"] - 1))
    pairer = StereoPairer(slop_ns=0)
    pairer.push(0, pair, wall)
    pairer.push(1, older, wall)
    assert pairer.pair(wall) is None
    pairer.push(1, depth, wall)
    result = perceive_usb_pair(*pairer.pair(wall))
    assert np.array_equal(result.right, right.rgb)
    assert np.array_equal(result.relative_depth, levels[:, :, 0])
    assert result.disparity is None
    assert {obj["id"] for obj in result.objects} == {"red_cube", "blue_bowl"}
    assert all(not obj["position_valid"] and obj["xyz"] is None for obj in result.objects)
    with pytest.raises(ValueError, match="matching USB"):
        perceive_usb_pair(pair, older)


def test_qwen_request_sends_both_usb_pair_and_policy_depth(monkeypatch):
    captured = {}

    def fake_request(_, path, payload):
        assert path == "/api/chat"
        captured.update(payload)
        return {"message": {"content": "fixture"}}

    monkeypatch.setattr("ros_robot_studio.vision.orchestrator.request_json", fake_request)
    snapshot = {"scene_id": "usb-scene", "depth_kind": "relative"}
    assert (
        chat(
            "http://localhost:11434",
            "qwen",
            "Use the cube",
            b"pair",
            snapshot,
            relative_depth_image=b"depth",
        )
        == "fixture"
    )
    assert [base64.b64decode(image) for image in captured["messages"][1]["images"]] == [
        b"pair",
        b"depth",
    ]
    assert "no measured robot-base positions" in captured["messages"][0]["content"]


def test_grounding_assigns_distinct_ids_and_scales_boxes(monkeypatch):
    detections = {
        "scene_id": "usb-scene",
        "objects": [
            {"label": "Green cube", "bbox_2d": [600, 200, 800, 400]},
            {"label": "green cube", "bbox_2d": [100, 300, 200, 400]},
            {"label": " Green cube ", "bbox_2d": [100, 300, 200, 400]},
        ],
    }
    monkeypatch.setattr(
        "ros_robot_studio.vision.orchestrator.request_json",
        lambda *args: {"message": {"content": json.dumps(detections)}},
    )
    objects = ground_objects(
        "http://localhost", "qwen", "green cubes", b"image", "usb-scene", (640, 480)
    )
    assert [obj["id"] for obj in objects] == ["green_cube_1", "green_cube_2"]
    assert [obj["bbox"] for obj in objects] == [[64, 144, 64, 48], [384, 96, 128, 96]]
    assert all(obj["xyz"] is None and not obj["position_valid"] for obj in objects)
    with pytest.raises(ValueError, match="different scene"):
        ground_objects("http://localhost", "qwen", "green cube", b"image", "old-scene", (640, 480))
    detections["objects"][0]["bbox_2d"] = [600, 200, 500, 1001]
    with pytest.raises(ValueError, match="bbox_2d"):
        ground_objects("http://localhost", "qwen", "green cube", b"image", "usb-scene", (640, 480))


def test_proposal_resolves_scene_bound_geometry_in_the_app():
    snapshot = {
        "scene_id": "workcell",
        "frame": "robot_base",
        "table_z": -0.005,
        "objects": [
            {
                "id": "red_cube",
                "class": "red cube",
                "xyz": [0.21, -0.075, 0.05],
                "position_valid": True,
            },
            {
                "id": "blue_bowl",
                "class": "blue bowl",
                "xyz": [0.18, 0.055, 0.026],
                "position_valid": True,
            },
        ],
    }
    raw = mock_reply("Success", snapshot)
    with pytest.raises(ValueError, match="different scene"):
        validate_reply(raw, dict(snapshot, scene_id="replacement"), "")
    reply = validate_reply(raw, snapshot, "Place the cube to the left of the bowl.")
    geometry = dict(target_geometry(reply, snapshot), reference_id=reply.reference_id)
    assert geometry["surface"] == [0.21, -0.075, 0.05]
    assert geometry["grasp_center"] == [0.21, -0.075, 0.0225]
    assert geometry["place_center"] == pytest.approx([0.18, 0.15, 0.0225])
    request = make_request(geometry, snapshot, dict.fromkeys((*ARM, "gripper"), 0.0), 1)
    assert request["base"] == "base_link"
    assert len(request["phases"]) == 5
    assert request["phases"][-1]["xyz"][2] == pytest.approx(0.0345)


def test_work_surface_requires_identity_robot_base_alias():
    left, _ = synthetic_pair(0.45)
    camera = stereo_profiles("simulation", "base_link")[0]
    identity = ([0, 0, 0], [0, 0, 0, 1])
    transforms = {
        "robot_base": identity,
        "base_link": identity,
        "so101_work_surface": ([0, 0, 0.005], [0, 0, 0, 1]),
    }
    frame = RGBFrame(left.rgb, dict(left.metadata, transforms=transforms))
    assert table_height(frame, camera) == pytest.approx(-0.005)
    transforms["base_link"] = ([0, 0, 0.1], [0, 0, 0, 1])
    assert table_height(frame, camera) is None
