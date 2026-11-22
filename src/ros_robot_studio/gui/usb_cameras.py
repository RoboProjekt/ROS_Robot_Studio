"""Linux USB camera selection and ownership of the launched ROS publisher."""

import os
import struct
from dataclasses import dataclass
from pathlib import Path
from uuid import uuid4

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, QSignalBlocker, QTimer, Signal
from PySide6.QtWidgets import QComboBox, QFormLayout, QPushButton, QWidget

from ros_robot_studio.projects.store import PROJECTS_DIR

RGB_TOPIC = "/so101/usb/camera_1/image_raw"
DEPTH_TOPIC = "/so101/usb/depth/image_raw"
USB_TOPICS = {
    "single": (RGB_TOPIC,),
    "stereo": (
        RGB_TOPIC,
        "/so101/usb/camera_2/image_raw",
        "/so101/usb/stereo/image_raw",
        "/so101/usb/stereo/depth_relative",
    ),
    "depth": (DEPTH_TOPIC,),
}

# VIDIOC_QUERYCAP and the single-planar Linux video capture capability.
QUERY_CAPABILITIES = 0x80685600
CAPABILITIES = struct.Struct("=16s32s32s6I")
VIDEO_CAPTURE = 0x00000001
DEVICE_CAPABILITIES = 0x80000000


@dataclass(frozen=True)
class USBVideoDevice:
    path: str
    label: str
    port: str


def usb_video_devices():
    if not Path("/sys/class/video4linux").is_dir():
        return []
    from fcntl import ioctl

    aliases = {
        alias.resolve(): alias
        for alias in Path("/dev/v4l/by-path").glob("*")
        if alias.is_char_device()
    }
    devices = []
    for node in sorted(Path("/dev").glob("video[0-9]*")):
        entry = Path("/sys/class/video4linux") / node.name
        interface = (entry / "device").resolve()
        port = next(
            (parent for parent in interface.parents if (parent / "idVendor").is_file()), None
        )
        if not port or not node.is_char_device():
            continue
        path = aliases.get(node, node)
        try:
            with open(
                node, "rb", opener=lambda name, flags: os.open(name, flags | os.O_NONBLOCK)
            ) as handle:
                caps = bytearray(CAPABILITIES.size)
                ioctl(handle.fileno(), QUERY_CAPABILITIES, caps)
            _, card, _, _, flags, device_flags, *_ = CAPABILITIES.unpack(caps)
            if not (device_flags if flags & DEVICE_CAPABILITIES else flags) & VIDEO_CAPTURE:
                continue
            name = card.partition(b"\0")[0].decode(errors="replace")
        except PermissionError:
            if not path.name.endswith("video-index0"):
                continue
            name = "USB camera (no access)"
        except OSError:
            continue
        devices.append(
            USBVideoDevice(str(path), f"{name} · USB {port.name} · {node.name}", port.name)
        )
    return devices


class USBCameraControls(QWidget):
    changed = Signal()

    def __init__(self, settings, parent=None):
        super().__init__(parent)
        self.settings = settings
        self.values = dict(settings.get("so101_usb_cameras", {}))
        self.devices = {}
        self.defaults = ()
        self.form = QFormLayout(self)
        self.form.setContentsMargins(0, 0, 0, 0)
        self.mode_field = QComboBox()
        for name, mode in (
            ("Single Camera", "single"),
            ("Double Camera Stereoscopic", "stereo"),
            ("Single Depth Camera", "depth"),
        ):
            self.mode_field.addItem(name, mode)
        self.mode_field.setCurrentIndex(
            max(0, self.mode_field.findData(self.values.get("mode", "stereo")))
        )
        self.form.addRow("Camera mode", self.mode_field)
        self.left = QComboBox()
        self.right = QComboBox()
        self.model = QComboBox()
        self.model.addItem("Select a stereo policy", "")
        self.model.addItem("High · 42–60 cm", "high")
        self.model.addItem("Low · 10–20 cm", "low")
        self.model.setCurrentIndex(
            max(
                0,
                self.model.findData(
                    self.values.get("model", os.environ.get("SO101_STEREO_PROFILE", ""))
                ),
            )
        )
        self.form.addRow("USB camera", self.left)
        self.form.addRow("Right USB camera", self.right)
        self.form.addRow("Stereo policy", self.model)
        self.form.addRow(QPushButton("Refresh USB Cameras", clicked=self.refresh))
        self.mode_field.activated.connect(self.switch_mode)
        for field in (self.left, self.right, self.model):
            field.activated.connect(self.save)

    @property
    def mode(self):
        return self.mode_field.currentData()

    def switch_mode(self):
        self.refresh()
        self.save()

    def refresh(self):
        self.devices = {device.path: device for device in usb_video_devices()}
        fields = (self.left, self.right)
        preset = self.defaults if self.mode == "stereo" else ()
        selected = self.values.get(self.mode, preset)
        available = tuple(self.devices)
        for i, field in enumerate(fields):
            previous = selected[i] if i < len(selected) else ""
            with QSignalBlocker(field):
                field.clear()
                field.addItem("Select a USB camera" if available else "No USB cameras detected", "")
                for device in self.devices.values():
                    field.addItem(device.label, device.path)
                if previous and previous not in self.devices:
                    field.addItem(f"Disconnected · {Path(previous).name}", previous)
                chosen = previous or (available[i] if i < len(available) else "")
                field.setCurrentIndex(max(0, field.findData(chosen)))
        self.form.labelForField(self.left).setText(
            "Left USB camera"
            if self.mode == "stereo"
            else "USB depth camera"
            if self.mode == "depth"
            else "USB camera"
        )
        self.form.setRowVisible(self.right, self.mode == "stereo")
        self.form.setRowVisible(
            self.model, self.mode == "stereo" and not os.environ.get("SO101_STEREO_MODEL")
        )

    def save(self):
        self.values["mode"] = self.mode
        self.values["model"] = self.model.currentData()
        self.values[self.mode] = [self.left.currentData(), self.right.currentData()]
        self.settings.set("so101_usb_cameras", self.values)
        self.changed.emit()

    def configuration(self):
        self.refresh()
        fields = (self.left, self.right) if self.mode == "stereo" else (self.left,)
        paths = tuple(field.currentData() for field in fields)
        if any(path not in self.devices for path in paths):
            count = len(self.devices)
            raise ValueError(f"{count} USB camera input(s) detected. Select connected devices.")
        model = self.model.currentData()
        if self.mode == "stereo":
            if self.devices[paths[0]].port == self.devices[paths[1]].port:
                raise ValueError("Select two different USB cameras for stereo.")
            if not model and not os.environ.get("SO101_STEREO_MODEL"):
                raise ValueError(
                    "Select the stereo policy that matches the camera mounting height."
                )
        return self.mode, paths, model


class USBCameraDriver(QObject):
    failed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.process = QProcess(self)
        self.stopper = QProcess(self)
        self.timer = QTimer(self, singleShot=True, interval=4500)
        self.timer.timeout.connect(self.force_stop)
        self.process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.process_error)
        self.configuration = None
        self.container = ""
        self.stopping = False
        self.output = ""

    @property
    def running(self):
        return self.process.state() != QProcess.ProcessState.NotRunning

    @property
    def idle(self):
        return not self.running and self.stopper.state() == QProcess.ProcessState.NotRunning

    def start(self, mode, paths, model, domain):
        configuration = mode, paths, model, domain
        if not self.idle:
            if self.running and not self.stopping and self.configuration == configuration:
                return
            raise ValueError("Wait for the previous USB camera driver to stop.")
        script = PROJECTS_DIR.parent / "scripts" / "start_so101_usb_cameras.sh"
        if not script.is_file():
            raise ValueError("The SO-101 camera launcher is missing from this Studio checkout.")
        self.configuration = configuration
        self.stopping = False
        self.output = ""
        self.container = "studio-so101-usb-" + uuid4().hex[:12]
        environment = QProcessEnvironment.systemEnvironment()
        for key, value in {
            "SO101_CAMERA_MODE": mode,
            "SO101_CAMERA_CONTAINER": self.container,
            "SO101_STEREO_PROFILE": model or "",
            "ROS_DOMAIN_ID": str(domain),
        }.items():
            environment.insert(key, value)
        self.process.setProcessEnvironment(environment)
        self.process.start("bash", [str(script), *paths])

    def read_output(self):
        output = bytes(self.process.readAllStandardOutput()).decode(errors="replace")
        self.output = (self.output + output)[-3000:]

    def finished(self, *_):
        self.timer.stop()
        self.read_output()
        if not self.stopping:
            lines = self.output.strip().splitlines()
            self.failed.emit(lines[-1] if lines else "The USB camera driver exited.")

    def process_error(self, error):
        if error == QProcess.ProcessError.FailedToStart and not self.stopping:
            self.failed.emit(self.process.errorString())

    def stop(self):
        if self.running and not self.stopping:
            self.stopping = True
            self.process.terminate()
            self.timer.start()

    def force_stop(self):
        if self.running:
            self.stopper.start("docker", ["stop", "--time", "2", self.container])
            self.process.kill()
