"""Bounded binary camera stream in the existing backend's Docker session."""

from __future__ import annotations

import json
import os
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
        channels = {"rgb8": 3, "bgr8": 3, "mono8": 1}.get(encoding)
        if channels is None:
            raise ValueError(f"Unsupported ROS image encoding {encoding!r}")
        width, height, stride = (meta[key] for key in ("width", "height", "step"))
        if not (
            1 <= width <= 1920
            and 1 <= height <= 1080
            and width * channels <= stride
            and stride * height == len(payload) <= MAX_IMAGE
        ):
            raise ValueError("Invalid image dimensions, row stride or payload length")
        # Each supported channel is one byte, so either byte order has identical layout.
        rows = np.frombuffer(payload, np.uint8).reshape(height, stride)
        rgb = rows[:, : width * channels].reshape(height, width, channels)
        if channels == 1:
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
    def __init__(self, camera, session):
        self.camera = camera
        self.session = session
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

    def poll(self):
        with self.lock:
            frame, self.latest = self.latest, None
            return frame, self.error, self.epoch, self.count, self.replaced

    def command(self):
        profile = self.session.profile
        if profile.id != self.camera.backend_id or not self.session.container_id:
            raise ValueError("Camera requires its own connected backend session")
        targets = {self.camera.world_frame, self.camera.base_frame}
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
                    with self.lock:
                        epoch = meta["epoch"]
                        if epoch != self.epoch:
                            self.latest = None
                            self.epoch = epoch
                        self.error = error
                        if frame is not None:
                            self.count += 1
                            self.replaced += self.latest is not None
                            self.latest = frame
