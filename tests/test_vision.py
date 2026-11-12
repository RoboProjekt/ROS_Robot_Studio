import json
import struct
from threading import Event
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.models import Project
from ros_robot_studio.motion.positions import VisionPoseProvider
from ros_robot_studio.projects.store import load_project, save_project
from ros_robot_studio.vision.detection import (
    DETECTORS,
    ArucoDetector,
    DetectorSettings,
    HSVDetector,
    ObjectDetectionManager,
)
from ros_robot_studio.vision.geometry import (
    Intrinsics,
    intersect_plane,
    look_at_usd,
    transform_matrix,
)
from ros_robot_studio.vision.profiles import CameraManager, CameraProfile, WorkPlane
from ros_robot_studio.vision.results import VisionResultManager
from ros_robot_studio.vision.stream import (
    MAX_IMAGE,
    CameraStream,
    PacketReader,
    RGBFrame,
    decode_frame,
)


def camera(**updates):
    return CameraProfile.model_validate(dict(name="RGB", backend_id="runtime") | updates)


def test_portable_cameras_round_trip_and_separate_identical_topics(tmp_path):
    first = new_profile("ros_demo", distribution="humble")
    second = new_profile("ros_demo", distribution="jazzy")
    overhead = camera(backend_id=first.id, plane=WorkPlane(point=(0, 0, 0)))
    other = camera(backend_id=second.id)
    project = Project(backends=[first, second], cameras=[overhead, other])
    filename = tmp_path / "vision.studio.yaml"
    save_project(project, filename)
    assert load_project(filename).cameras == [overhead, other]
    assert Project.model_validate({"name": "Old project"}).cameras == []

    manager = CameraManager()
    manager.load(project.cameras)
    assert manager.selected_id == overhead.id
    with pytest.raises(ValueError, match="unique"):
        manager.save(camera(backend_id=first.id))
    manager.select(other.id)
    manager.remove(other.id)
    assert manager.selected_id == overhead.id
    assert manager.discover(
        {
            "topics": ["/rgb", "/rgb/compressed", "/joints"],
            "types": [
                "sensor_msgs/msg/Image",
                "sensor_msgs/CompressedImage",
                "sensor_msgs/msg/JointState",
            ],
        }
    ) == {"/rgb": "raw", "/rgb/compressed": "compressed"}


@pytest.mark.parametrize(
    ("encoding", "payload"),
    [
        ("rgb8", b"\xff\x00\x11\x01\x03\x05\x90\x90"),
        ("bgr8", b"\x11\x00\xff\x05\x03\x01\x90\x90"),
        ("mono8", b"\xff\x01\x90\x90\x90\x90\x90\x90"),
    ],
)
def test_raw_formats_respect_stride_and_publish_immutable_rgb(encoding, payload):
    meta = dict(encoding=encoding, frame_id="optical", stamp_ns=12, width=2, height=1, step=8)
    for big_endian in (True, False):
        frame = decode_frame(dict(meta, big_endian=big_endian), payload)
        expected = [[[255] * 3, [1] * 3]] if encoding == "mono8" else [[[255, 0, 17], [1, 3, 5]]]
        np.testing.assert_array_equal(frame.rgb, expected)
        assert not frame.rgb.flags.writeable
    with pytest.raises(ValueError, match="stride or payload"):
        decode_frame(meta, payload[:-1])
    with pytest.raises(ValueError, match="Unsupported"):
        decode_frame(dict(meta, encoding="16UC1"), payload)


@pytest.mark.parametrize("image_format", ["PNG", "JPEG"])
def test_compressed_rgb_uses_qt_codecs(image_format, qapp):
    from PySide6.QtCore import QBuffer, QIODevice
    from PySide6.QtGui import QColor, QImage

    image = QImage(13, 11, QImage.Format.Format_RGB888)
    image.fill(QColor(218, 15, 34))
    buffer = QBuffer()
    buffer.open(QIODevice.OpenModeFlag.WriteOnly)
    assert image.save(buffer, image_format)
    meta = dict(encoding="compressed", frame_id="optical", stamp_ns=22)
    frame = decode_frame(meta, bytes(buffer.data()))
    assert (frame.width, frame.height) == (13, 11)
    np.testing.assert_allclose(frame.rgb[5, 6], [218, 15, 34], atol=4)
    with pytest.raises(ValueError, match="JPEG or PNG"):
        decode_frame(meta, b"invalid image")


def test_binary_packets_fragmentation_and_limits():
    meta = dict(kind="image", epoch=0)
    header = json.dumps(meta).encode()
    packet = struct.pack("!II", len(header), 3) + header + b"rgb"
    decoder = PacketReader()
    decoded = []
    for index in range(0, len(packet * 2), 5):
        decoded.extend(decoder.feed((packet * 2)[index : index + 5]))
    assert decoded == [(meta, b"rgb"), (meta, b"rgb")]
    assert not decoder.buffer
    with pytest.raises(ValueError, match="length"):
        list(decoder.feed(struct.pack("!II", 1, MAX_IMAGE + 1)))


def test_receiver_uses_existing_container_and_bound_backend():
    profile = camera()
    session = SimpleNamespace(
        profile=SimpleNamespace(id=profile.backend_id, use_sim_time=True, clock_topic="/clock"),
        container_id="owned-camera-container",
    )
    stream = CameraStream(profile, session)
    command = stream.command()
    assert command[:4] == ["docker", "exec", "-i", session.container_id]
    config = json.loads(command[-1])
    assert config["clock_topic"] == "/clock"
    assert set(config["targets"]) == {profile.world_frame, profile.base_frame}
    session.profile.id = "another-backend"
    with pytest.raises(ValueError, match="own connected backend"):
        stream.command()


def test_projection_world_base_and_optical_and_draft():
    from ros_robot_studio.vision.detection import Finding

    plane = WorkPlane(point=(0, 0, 0), bounds=((-2, -2, 0), (2, 2, 0)))
    profile = camera(plane=plane)
    info = dict(
        width=100,
        height=100,
        frame_id=profile.optical_frame,
        k=[80, 0, 50, 0, 80, 50, 0, 0, 1],
        d=[],
        model="plumb_bob",
    )
    transforms = {
        "world": ([1, 0, 2], [1, 0, 0, 0]),
        "base_link": ([0.5, 0, 2], [1, 0, 0, 0]),
    }
    metadata = dict(
        camera_info=info, stamp_ns=25, frame_id=profile.optical_frame, transforms=transforms
    )
    result = VisionResultManager()
    result.update((RGBFrame(np.zeros((100, 100, 3), np.uint8), metadata), (), 0.01))
    finding = Finding(0, "Red", 1.0, (50, 40, 66, 60), "mask fill")
    world = result.estimate(profile, finding, "world")
    base = result.estimate(profile, finding, "base_link")
    optical = result.estimate(profile, finding, profile.optical_frame)
    np.testing.assert_allclose(world.position, [1.2, 0, 0])
    np.testing.assert_allclose(base.position, [0.7, 0, 0])
    np.testing.assert_allclose(optical.position, [0.2, 0, 2])
    np.testing.assert_allclose(
        result.estimate(profile, finding, "world", "bottom").position, [1.2, -0.25, 0]
    )
    provider = VisionPoseProvider({"detection": base})
    assert provider.get_position("detection") == base
    with pytest.raises(ValueError, match="explicit tool orientation"):
        provider.get_pose("detection")
    pose = provider.draft("detection", "tool", (0, 0, 0), (0, 0, 0.05))
    np.testing.assert_allclose(pose.position, [0.7, 0, 0.05])
    del transforms["base_link"]
    with pytest.raises(ValueError, match="Missing time-stamped TF"):
        result.estimate(profile, finding, "base_link")
    metadata["camera_info"]["width"] = 10
    with pytest.raises(ValueError, match="resolution"):
        result.estimate(profile, finding, "world")


def test_projection_guards_and_usd_look_at():
    direction = np.array([1, 0, -2])
    rotation = look_at_usd((0, 0, 2), (1, 0, 0))
    np.testing.assert_allclose(-rotation[:, 2], direction / np.linalg.norm(direction))
    np.testing.assert_allclose(np.linalg.det(rotation), 1)
    with pytest.raises(ValueError, match="distinct"):
        look_at_usd((0, 0, 0), (0, 0, 0))
    with pytest.raises(ValueError, match="rotation"):
        transform_matrix((0, 0, 0), (0, 0, 0, 0))
    plane = WorkPlane(point=(0, 0, 0), bounds=((-1, -1, 0), (1, 1, 0)))
    pose = transform_matrix((0, 0, 2), (0, 0, 0, 1))
    for ray, problem in [([1, 0, 0], "parallel"), ([0, 0, 1], "behind"), ([2, 0, -1], "outside")]:
        with pytest.raises(ValueError, match=problem):
            intersect_plane(ray, pose, plane)
    with pytest.raises(ValueError, match="outside image"):
        Intrinsics(np.eye(3), np.zeros(0), "", 30, 30).ray((30, 3))


def test_hsv_multi_object_and_aruco():
    cv2 = pytest.importorskip("cv2")
    image = np.zeros((200, 280, 3), np.uint8)
    image[20:80, 20:80] = (255, 0, 0)
    image[100:140, 25:65] = (255, 0, 0)
    image[100:150, 100:160] = (0, 0, 255)
    detector = HSVDetector()
    detector.initialize(DetectorSettings())
    findings = detector.detect(image)
    assert [item.box for item in findings] == [(20, 20, 80, 80), (25, 100, 65, 140)]
    assert all(item.score_kind == "mask fill" and item.score == 1 for item in findings)

    if not hasattr(cv2, "aruco"):
        pytest.skip("OpenCV contrib not installed")
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 17, 100
    )
    image.fill(255)
    image[40:140, 100:200] = marker[:, :, None]
    aruco = ArucoDetector()
    aruco.initialize(DetectorSettings())
    (found,) = aruco.detect(image)
    assert found.class_id == 17 and found.score is None and len(found.corners) == 4


def test_slow_inference_replaces_pending_frame_and_stops(qtbot):
    started, release, shutdown = Event(), Event(), Event()
    frames = []

    class Slow:
        def initialize(self, configuration, model):
            pass

        def get_classes(self):
            return ()

        def detect(self, image):
            frames.append(int(image[0, 0, 0]))
            started.set()
            release.wait(3)
            return []

        def shutdown(self):
            shutdown.set()

    manager = ObjectDetectionManager()
    with patch.dict(DETECTORS, {"slow": ("Slow", Slow, ())}):
        manager.start("slow", DetectorSettings(fps=30), None, camera())
        try:
            for sequence in range(30):
                manager.submit(
                    RGBFrame(
                        np.full((1, 1, 3), sequence, np.uint8),
                        dict(sequence=sequence, stamp_ns=sequence),
                    )
                )
                if sequence == 0:
                    assert started.wait(2)
            assert manager.drops == 28
            release.set()
            qtbot.waitUntil(lambda: manager.processed == 2)
            result, error, processed, _ = manager.poll()
            assert frames == [0, 29] and processed == 2 and not error
            assert result[0].metadata["stamp_ns"] == 29
        finally:
            manager.stop()
            release.set()
            manager.thread.join(2)
        assert shutdown.is_set() and not manager.thread.is_alive()
