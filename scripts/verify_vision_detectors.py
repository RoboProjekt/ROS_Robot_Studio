"""Run native detector selection against two real, isolated ROS CompressedImage publishers."""

import argparse
import json
import subprocess
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QPushButton
from verify_gui_usability import choose_scene_file
from verify_vision_isaac import click

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.vision.profiles import CameraProfile

TOPIC = "/studio_vision_verification/image/compressed"
PUBLISH = """import sys
from pathlib import Path
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

rclpy.init()
node = rclpy.create_node('studio_vision_fixture')
publisher = node.create_publisher(CompressedImage, sys.argv[1], qos_profile_sensor_data)
payload = Path('/tmp/studio-vision-fixture.jpg').read_bytes()

def publish():
    message = CompressedImage()
    message.header.stamp = node.get_clock().now().to_msg()
    message.header.frame_id = 'fixture_optical'
    message.format = 'jpeg'
    message.data = payload
    publisher.publish(message)

node.create_timer(0.13, publish)
rclpy.spin(node)
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--image", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    assert args.image.is_file() and args.model.is_file()
    marker_file = output / "generated-marker17.jpg"
    marker = cv2.aruco.generateImageMarker(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 17, 160
    )
    pixels = np.full((360, 480), 255, np.uint8)
    pixels[100:260, 160:320] = marker
    assert cv2.imwrite(str(marker_file), pixels)
    app = QApplication([])
    assert app.platformName() == "xcb"
    errors = []
    window = MainWindow(Settings(output / "settings.json"), show_wizard=False)
    window.show_error = errors.append
    window.backend_manager.failed.disconnect()
    window.backend_manager.failed.connect(errors.append)
    window.resize(1400, 1000)
    window.show()
    vision = window.vision
    processes = []

    def wait(predicate, timeout=45, cleanup=False):
        deadline = time.monotonic() + timeout
        while not predicate():
            QTest.qWait(20)
            assert cleanup or not errors, errors
            assert time.monotonic() < deadline, (
                errors,
                vision.status.text(),
                vision.detector_status.text(),
                {key: session.status for key, session in window.backend_manager.sessions.items()},
            )

    def button(label):
        return next(item for item in vision.panel.findChildren(QPushButton) if item.text() == label)

    def find_labels():
        return {item.finding.label for item in vision.results.detections}

    def choose(combo, index):
        assert index >= 0
        combo.setFocus()
        combo.showPopup()
        view = combo.view()
        QTest.keyClick(view, Qt.Key.Key_Home)
        for _ in range(index):
            QTest.keyClick(view, Qt.Key.Key_Down)
        QTest.keyClick(view, Qt.Key.Key_Return)
        assert combo.currentIndex() == index

    def capture(name):
        click(button("Pause / resume display"))
        assert window.grab().save(str(output / name))
        click(button("Pause / resume display"))

    profiles = []
    try:
        wait(
            lambda: not window.initial_model_check_pending
            and not window.runner.busy
            and not window.motion.runner.busy
        )
        assert window.docker_status.available, window.docker_status.diagnostics
        for domain, filename, name in (
            (41, args.image.resolve(), "YOLO reference image"),
            (42, marker_file, "ArUco reference marker"),
        ):
            profile = window.backend_manager.add(
                new_profile(
                    "generic_ros",
                    name=name,
                    distribution="humble",
                    domain_id=domain,
                    discovery="localhost",
                    network="host",
                    network_confirmed=True,
                )
            )
            profiles.append(profile)
            camera = CameraProfile(
                name=name,
                backend_id=profile.id,
                rgb_topic=TOPIC,
                encoding="compressed",
                optical_frame="fixture_optical",
            )
            vision.save_camera(camera)
            vision.connect()
            wait(
                lambda: profile.id in window.backend_manager.sessions
                and window.backend_manager.sessions[profile.id].transport is not None
                and window.backend_manager.sessions[profile.id].transport.connected
            )
            container = window.backend_manager.sessions[profile.id].container_id
            subprocess.run(
                ["docker", "cp", str(filename), container + ":/tmp/studio-vision-fixture.jpg"],
                check=True,
            )
            with (output / f"publisher-{domain}.log").open("w") as log:
                process = subprocess.Popen(
                    [
                        "docker",
                        "exec",
                        container,
                        "bash",
                        "-c",
                        'source "/opt/ros/$ROS_DISTRO/setup.bash" && exec python3 -u -c "$1" "$2"',
                        "fixture",
                        PUBLISH,
                        TOPIC,
                    ],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            processes.append(process)
            wait(
                lambda: TOPIC in window.backend_manager.sessions[profile.id].graph.get("topics", ())
            )
        vision.open_view()
        choose(vision.camera, 0)
        click(button("Start Stream"))
        wait(lambda: vision.latest is not None)
        assert vision.latest.height == 1080 and vision.stream.session.profile.domain_id == 41
        receiver = vision.stream
        choose(vision.detector_choice, vision.detector_choice.findData("hsv"))
        click(button("Start Detection"))
        wait(lambda: vision.detector.processed > 0)
        choose_scene_file(SimpleNamespace(browse=button("Add ONNX…")), args.model.resolve())
        choose(vision.detector_choice, vision.detector_choice.findData("yolo"))
        wait(lambda: {"person", "bus"} <= find_labels())
        assert vision.stream is receiver and receiver.thread.is_alive()
        yolo = [asdict(item.finding) for item in vision.results.detections]
        yolo_inference_ms = vision.results.seconds * 1000
        assert all(0.25 <= item["score"] <= 1 for item in yolo)
        assert vision.estimate is None
        capture("yolo-onnx.png")
        choose(vision.camera, 1)
        wait(lambda: not receiver.thread.is_alive())
        assert not vision.results.detections and vision.view.image.isNull()
        click(button("Start Stream"))
        wait(lambda: vision.latest is not None)
        assert vision.latest.height == 360 and vision.stream.session.profile.domain_id == 42
        choose(vision.detector_choice, vision.detector_choice.findData("aruco"))
        click(button("Start Detection"))
        wait(lambda: "Marker 17" in find_labels())
        (marker,) = (item.finding for item in vision.results.detections)
        assert marker.score is None and len(marker.corners) == 4
        assert vision.estimate is None
        aruco_inference_ms = vision.results.seconds * 1000
        capture("aruco.png")
        result = dict(
            yolo=yolo,
            aruco=asdict(marker),
            yolo_inference_ms=yolo_inference_ms,
            aruco_inference_ms=aruco_inference_ms,
            topic=TOPIC,
            encoding="JPEG CompressedImage",
            backend_domains=[profile.domain_id for profile in profiles],
            detector_switch_preserved_receiver=True,
            camera_switch_cleared_results=True,
            model=str(args.model.resolve()),
            sample=str(args.image.resolve()),
        )
    finally:
        wait(lambda: not window.runner.busy and not window.motion.runner.busy, cleanup=True)
        window.dirty = False
        window.close()
        wait(lambda: window.closed, cleanup=True)
        assert not window.backend_manager.sessions
        assert all(not timer.isActive() for timer in window.findChildren(QTimer))
        for process in processes:
            process.wait(timeout=8)
    result["closed_without_owned_sessions_or_publishers"] = True
    (output / "detector-verification.json").write_text(json.dumps(result, indent=2) + "\n")
    print("Verified ONNX and ArUco on two isolated Humble image streams", flush=True)


if __name__ == "__main__":
    main()
