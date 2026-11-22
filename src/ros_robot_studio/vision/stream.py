"""Latest-frame capture from a local V4L2 device or a ROS backend."""

from __future__ import annotations

import json
import os
import re
import selectors
import struct
import subprocess
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAX_IMAGE = 8 * 1024 * 1024
MAX_METADATA = 64 * 1024
READ_SIZE = 128 * 1024


@dataclass(frozen=True)
class RGBFrame:
    rgb: np.ndarray
    metadata: dict

    @property
    def width(self):
        return self.rgb.shape[1]

    @property
    def height(self):
        return self.rgb.shape[0]

    @property
    def frame_id(self):
        return self.metadata["frame_id"]


def decode_frame(meta, payload):
    encoding = meta["encoding"]
    if not isinstance(meta["stamp_ns"], int) or meta["stamp_ns"] < 0 or not meta["frame_id"]:
        raise ValueError("Invalid ROS image header")
    if encoding == "compressed":
        from PySide6.QtCore import QBuffer, QByteArray, QIODevice
        from PySide6.QtGui import QImage, QImageReader

        buffer = QBuffer()
        buffer.setData(QByteArray(payload))
        buffer.open(QIODevice.OpenModeFlag.ReadOnly)
        reader = QImageReader(buffer)
        size = reader.size()
        if bytes(reader.format()).lower() not in (b"jpeg", b"jpg", b"png"):
            raise ValueError("CompressedImage must contain JPEG or PNG")
        if not (1 <= size.width() <= 1920 and 1 <= size.height() <= 1080):
            raise ValueError("Compressed image exceeds the resolution limit")
        image = reader.read().convertToFormat(QImage.Format.Format_RGB888)
        if image.isNull():
            raise ValueError("Compressed image could not be decoded")
        rgb = np.frombuffer(image.constBits(), np.uint8).reshape(
            image.height(), image.bytesPerLine()
        )
        rgb = rgb[:, : image.width() * 3].reshape(image.height(), image.width(), 3)
    else:
        size = {"rgb8": 3, "bgr8": 3, "mono8": 1, "16UC1": 2}.get(encoding)
        if size is None:
            raise ValueError(f"Unsupported ROS image encoding {encoding!r}")
        width, height, stride = (meta[key] for key in ("width", "height", "step"))
        if not (
            1 <= width <= 1920
            and 1 <= height <= 1080
            and width * size <= stride
            and stride * height == len(payload) <= MAX_IMAGE
        ):
            raise ValueError("Invalid image dimensions, row stride or payload length")
        rows = np.frombuffer(payload, np.uint8).reshape(height, stride)
        if encoding == "16UC1":
            order = ">u2" if meta.get("big_endian") else "<u2"
            depth = rows[:, : width * size].copy(order="C").view(order)
            gray = (depth.astype(np.float32) * (255 / max(1, int(depth.max())))).astype(np.uint8)
            rgb = gray[:, :, None].repeat(3, axis=2)
        else:
            rgb = rows[:, : width * size].reshape(height, width, size)
        if encoding == "mono8":
            rgb = rgb.repeat(3, axis=2)
        elif encoding == "bgr8":
            rgb = rgb[:, :, ::-1]
    rgb = rgb.copy(order="C")
    rgb.setflags(write=False)
    return RGBFrame(rgb, meta)


class PacketReader:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 8:
            header_size, image_size = struct.unpack_from("!II", self.buffer)
            if not 0 < header_size <= MAX_METADATA or image_size > MAX_IMAGE:
                raise ValueError("Invalid RGB transport packet length")
            end = 8 + header_size
            total = end + image_size
            if len(self.buffer) < total:
                break
            meta = json.loads(self.buffer[8:end])
            payload = bytes(self.buffer[end:total])
            del self.buffer[:total]
            yield meta, payload


class CameraStream:
    def __init__(self, camera, session=None, *, capture_depth=False):
        self.camera = camera
        self.session = session
        self.capture_depth = capture_depth
        self.stop_requested = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.error = ""
        self.epoch = 0
        self.count = 0
        self.replaced = 0
        self.thread = threading.Thread(target=self.run, daemon=True, name="Studio RGB receiver")

    def start(self):
        self.thread.start()

    def stop(self):
        self.stop_requested.set()
        with self.lock:
            self.latest = None

    def poll(self):
        with self.lock:
            frame, self.latest = self.latest, None
            return frame, self.error, self.epoch, self.count, self.replaced

    def command(self):
        profile = self.session.profile
        if profile.id != self.camera.backend_id or not self.session.container_id:
            raise ValueError("Camera requires its own connected backend session")
        targets = {self.camera.world_frame, self.camera.base_frame}
        if self.camera.id in ("so101_stereo_left", "so101_stereo_right"):
            targets.update(("robot_base", "so101_work_surface"))
        if self.camera.plane:
            targets.add(self.camera.plane.frame)
        config = self.camera.model_dump(mode="json")
        config.update(
            targets=sorted(targets), clock_topic=profile.clock_topic if profile.use_sim_time else ""
        )
        script = Path(__file__).with_name("ros_receiver.py").read_text()
        return [
            "docker",
            "exec",
            "-i",
            self.session.container_id,
            "bash",
            "-c",
            'source "/opt/ros/$ROS_DISTRO/setup.bash" && exec python3 -u -c "$1" "$2"',
            "studio-rgb",
            script,
            json.dumps(config),
        ]

    def run(self):
        try:
            if self.camera.v4l2_device:
                self.capture_v4l2()
                return
            with tempfile.TemporaryFile() as stderr:
                process = subprocess.Popen(
                    self.command(),
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=stderr,
                    bufsize=0,
                )
                try:
                    self.receive(process)
                    if not self.stop_requested.is_set():
                        stderr.seek(max(0, stderr.seek(0, os.SEEK_END) - 2000))
                        detail = stderr.read().decode(errors="replace").strip()
                        raise ConnectionError(detail or "ROS image receiver disconnected")
                finally:
                    process.stdin.close()
                    process.stdout.close()
                    try:
                        process.wait(timeout=0.8)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            with self.lock:
                self.error = str(exc)
                self.latest = None

    def capture_v4l2(self):
        device = self.camera.v4l2_device
        if self.capture_depth:
            self.capture_z16(device)
            return
        try:
            import cv2
        except ImportError as exc:
            raise OSError("USB camera capture requires the Studio vision extra (OpenCV)") from exc

        camera = self.camera
        if not Path(device).exists():
            raise OSError(f"USB camera not found at {device}. Check its USB port.")
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        try:
            if not capture.isOpened():
                raise OSError(f"Cannot open {device}. Check access and close other camera apps.")
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera.width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera.height)
            capture.set(cv2.CAP_PROP_FPS, camera.fps)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            while not self.stop_requested.is_set():
                ok, bgr = capture.read()
                stamp = time.time_ns()
                if not ok or bgr is None or not bgr.size:
                    raise OSError(f"No frames from {device}. Check the camera connection.")
                self.publish_local(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), stamp)
        except cv2.error as exc:
            raise OSError(f"USB camera capture failed for {device}: {exc}") from exc
        finally:
            capture.release()

    def capture_z16(self, device):
        base = ["v4l2-ctl", "--device", device]
        try:
            probe = subprocess.run(
                [*base, "--try-fmt-video=pixelformat=Z16 "],
                capture_output=True,
                text=True,
                timeout=5,
            )
        except subprocess.TimeoutExpired as exc:
            raise OSError(f"Timed out checking the depth format of {device}") from exc
        dimensions = re.search(r"Width/Height\s*:\s*(\d+)/(\d+)", probe.stdout)
        pixel_format = re.search(r"Pixel Format\s*:\s*'Z16\s*'", probe.stdout)
        bytes_per_line = re.search(r"Bytes per Line\s*:\s*(\d+)", probe.stdout)
        if probe.returncode or not (dimensions and pixel_format and bytes_per_line):
            raise OSError(f"{device} does not expose a V4L2 Z16 depth stream")
        width, height = map(int, dimensions.groups())
        stride = int(bytes_per_line[1])
        size = height * stride
        if not (2 * width <= stride and 0 < size <= MAX_IMAGE):
            raise OSError(f"{device} reported invalid Z16 dimensions or stride")
        arguments = [
            *base,
            f"--set-fmt-video=width={width},height={height},pixelformat=Z16 ",
            "--stream-mmap=3",
            "--stream-to=-",
        ]
        with subprocess.Popen(
            arguments, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        ) as process:
            try:
                with selectors.DefaultSelector() as selector:
                    selector.register(process.stdout, selectors.EVENT_READ)
                    buffer = bytearray()
                    deadline = time.monotonic() + 5
                    while not self.stop_requested.is_set():
                        if not selector.select(timeout=0.1):
                            if time.monotonic() > deadline:
                                raise OSError(f"No Z16 frames from {device}")
                            continue
                        chunk = os.read(process.stdout.fileno(), min(READ_SIZE, size - len(buffer)))
                        if not chunk:
                            raise OSError(f"Z16 stream disconnected at {device}")
                        buffer.extend(chunk)
                        if len(buffer) == size:
                            rows = np.frombuffer(buffer, np.uint8).reshape(height, stride)
                            depth = rows[:, : 2 * width].copy(order="C").view("<u2")
                            self.publish_local(depth, time.time_ns())
                            buffer = bytearray()
                            deadline = time.monotonic() + 5
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        process.kill()

    def publish_local(self, pixels, stamp):
        pixels.setflags(write=False)
        self.publish(
            RGBFrame(
                pixels,
                dict(
                    frame_id=self.camera.optical_frame,
                    epoch=0,
                    stamp_ns=stamp,
                    received_wall_ns=stamp,
                    drops=0,
                    camera_info=None,
                    transforms={},
                ),
            ),
            "",
            0,
        )

    def publish(self, frame, error, epoch):
        with self.lock:
            if self.stop_requested.is_set():
                return
            if epoch != self.epoch:
                self.latest = None
                self.epoch = epoch
            self.error = error
            if frame is not None:
                self.count += 1
                self.replaced += self.latest is not None
                self.latest = frame

    def receive(self, process):
        parser = PacketReader()
        next_lease = 0
        with selectors.DefaultSelector() as selector:
            selector.register(process.stdout, selectors.EVENT_READ)
            while not self.stop_requested.is_set():
                now = time.monotonic()
                if now >= next_lease:
                    os.write(process.stdin.fileno(), b".")
                    next_lease = now + 0.7
                if not selector.select(timeout=0.1):
                    continue
                chunk = os.read(process.stdout.fileno(), READ_SIZE)
                if not chunk:
                    break
                for meta, body in parser.feed(chunk):
                    frame = None
                    error = meta.get("message", "")
                    if meta["kind"] == "image":
                        try:
                            frame = decode_frame(meta, body)
                        except (ValueError, TypeError, KeyError) as exc:
                            error = str(exc)
                    self.publish(frame, error, meta["epoch"])
