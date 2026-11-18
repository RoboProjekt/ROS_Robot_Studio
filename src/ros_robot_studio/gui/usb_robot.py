"""Local serial-port discovery and the SO-101 USB dialog."""

import json
import math
import os
import sys
from pathlib import Path

from PySide6.QtCore import QProcess, Qt, QTimer, Signal
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
)

from ros_robot_studio.backends import so101_usb


def usb_ports():
    """Prefer a stable Linux by-id path when there is one for the port."""
    aliases = {
        str(alias.resolve()): str(alias)
        for alias in sorted(Path("/dev/serial/by-id").glob("*"))
        if alias.exists()
    }
    ports = {}
    for port in QSerialPortInfo.availablePorts():
        location = port.systemLocation()
        if port.hasVendorIdentifier() or location.startswith(("/dev/ttyACM", "/dev/ttyUSB")):
            name = port.description() or "USB serial adapter"
            ports[aliases.get(location, location)] = f"{port.portName()} · {name}"
    return sorted(ports.items(), key=lambda item: item[1])


def lerobot_interpreter():
    installed = Path.home() / "miniforge3/envs/lerobot/bin/python"
    if installed.is_file():
        return str(installed)
    prefix = os.environ.get("CONDA_PREFIX")
    if prefix:
        candidate = Path(prefix) / ("python.exe" if os.name == "nt" else "bin/python")
        if candidate.is_file():
            return str(candidate)
    return sys.executable


def calibration_files():
    cache = Path(os.environ.get("XDG_CACHE_HOME") or Path.home() / ".cache")
    hf_home = Path(os.environ.get("HF_HOME") or cache / "huggingface").expanduser()
    lerobot_home = Path(os.environ.get("HF_LEROBOT_HOME") or hf_home / "lerobot").expanduser()
    root = Path(
        os.environ.get("HF_LEROBOT_CALIBRATION") or lerobot_home / "calibration"
    ).expanduser()
    return sorted(
        {
            path
            for folder in ("so_follower", "so101_follower")
            for path in (root / "robots" / folder).glob("*.json")
            if path.is_file()
        }
    )


def port_access_error(port):
    if os.name == "nt":
        return ""
    if not Path(port).exists():
        return f"{port} is disconnected. Reconnect the adapter and refresh the port list."
    if not os.access(port, os.R_OK | os.W_OK):
        return f"No access to {port}. Check the serial device permissions (usually the dialout group on Linux)."
    return ""


class USBPositionDialog(QDialog):
    feedback = Signal(dict)
    availability_changed = Signal()

    def __init__(self, port, settings, parent=None):
        super().__init__(parent)
        self.setWindowTitle("SO-101 Follower · USB Joint Positions")
        self.port = port
        self.settings = settings
        self.python = settings.get("lerobot_python") or lerobot_interpreter()
        self.calibration = QComboBox()
        self.calibration.addItem("No calibration · raw motor counts", "")
        files = calibration_files()
        for path in files:
            self.add_calibration(path)
        if len(files) == 1:
            self.calibration.setCurrentIndex(1)
        self.browse = QPushButton("Choose calibration file…")
        self.browse.clicked.connect(self.choose_calibration)
        self.status = QLabel(port_access_error(port) or "Serial access available · read only")
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        self.status.setWordWrap(True)
        self.positions = {name: QLabel("—") for name in so101_usb.JOINT_NAMES}

    def add_position_rows(self, layout):
        layout.addRow("LeRobot calibration", self.calibration)
        layout.addRow(self.browse)
        layout.addRow(self.status)
        for name, value in self.positions.items():
            layout.addRow(name, value)

    def add_calibration(self, path):
        self.calibration.addItem(Path(path).stem, str(path))
        self.calibration.setItemData(
            self.calibration.count() - 1, str(path), Qt.ItemDataRole.ToolTipRole
        )

    def choose_calibration(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select SO-101 calibration", "", "JSON (*.json)"
        )
        if path:
            index = self.calibration.findData(path)
            if index < 0:
                self.add_calibration(path)
                index = self.calibration.count() - 1
            self.calibration.setCurrentIndex(index)


class USBJointDialog(USBPositionDialog):
    def __init__(self, port, settings, parent=None, *, allow_jog=False):
        super().__init__(port, settings, parent)
        self.jog_active = False
        self.jog_failed = False
        self.manual_read = False
        self.calibration_verified = False
        self.encoders = {}
        self.target = None
        self.target_start = None
        self.process = QProcess(self)
        self.process.finished.connect(self.read_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.deadline = QTimer(self, singleShot=True, interval=12000)
        self.deadline.timeout.connect(self.read_timed_out)
        self.poll = QTimer(self, singleShot=True, interval=400)
        self.poll.timeout.connect(self.poll_once)
        layout = QFormLayout(self)
        port_label = QLabel(port)
        port_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addRow("Serial port", port_label)
        self.interpreter = QPushButton(self.python)
        self.interpreter.clicked.connect(self.choose_interpreter)
        layout.addRow("LeRobot Python", self.interpreter)
        self.add_position_rows(layout)
        self.arm = None
        if allow_jog:
            self.arm = QCheckBox("Arm is fixed and supported. Enable manual steps.")
            self.arm.toggled.connect(self.update_jog_controls)
            self.arm.toggled.connect(self.arm_changed)
            layout.addRow(self.arm)
            self.joint = QComboBox()
            self.joint.addItems(so101_usb.JOINT_NAMES)
            self.lower = QPushButton("− Step")
            self.upper = QPushButton("+ Step")
            self.lower.clicked.connect(lambda: self.step_joint(-1))
            self.upper.clicked.connect(lambda: self.step_joint(1))
            jog_row = QHBoxLayout()
            for widget in (self.joint, self.lower, self.upper):
                jog_row.addWidget(widget)
            layout.addRow("One motor at a time", jog_row)
            self.motion_status = QLabel()
            self.motion_status.setTextFormat(Qt.TextFormat.PlainText)
            self.motion_status.setWordWrap(True)
            layout.addRow(self.motion_status)
        self.calibration.currentIndexChanged.connect(self.invalidate_reading)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        self.read = buttons.addButton("Read positions", QDialogButtonBox.ButtonRole.ActionRole)
        self.read.clicked.connect(self.read_positions)
        layout.addRow(buttons)
        self.update_jog_controls()

    def showEvent(self, event):
        super().showEvent(event)
        self.read_positions(automatic=True)

    def schedule_poll(self):
        if self.isVisible():
            self.poll.start()

    def arm_changed(self, armed):
        if not armed:
            self.target = None
        self.availability_changed.emit()

    def can_command(self):
        return bool(
            self.arm and self.arm.isChecked() and self.calibration_verified and not self.jog_failed
        )

    def request_target(self, joint, encoder):
        if self.can_command():
            self.target = (joint, encoder)

    def poll_once(self):
        if self.target and self.can_command():
            joint, goal = self.target
            start = self.encoders.get(joint)
            if start is not None and abs(goal - start) > 2:
                self.target_start = (self.target, start)
                self.read_positions(target=self.target, automatic=True)
                return
            self.target = None
        self.read_positions(automatic=True)

    def choose_interpreter(self):
        self.poll.stop()
        path, _ = QFileDialog.getOpenFileName(
            self, "Select LeRobot's Python interpreter", self.python
        )
        if path:
            self.python = path
            self.settings.set("lerobot_python", path)
            self.interpreter.setText(path)
            self.invalidate_reading()
        self.schedule_poll()

    def choose_calibration(self):
        self.poll.stop()
        super().choose_calibration()
        self.schedule_poll()

    def invalidate_reading(self):
        self.calibration_verified = False
        self.target = None
        for value in self.positions.values():
            value.setText("—")
        self.status.setText("Waiting for current motor positions…")
        self.update_jog_controls()
        self.availability_changed.emit()

    def update_jog_controls(self):
        if self.arm:
            ready = self.calibration_verified and self.read.isEnabled()
            self.arm.setEnabled(self.calibration_verified or self.arm.isChecked())
            self.joint.setEnabled(self.arm.isChecked() and not self.jog_active)
            for control in (self.lower, self.upper):
                control.setEnabled(ready and self.arm.isChecked() and not self.jog_failed)

    def step_joint(self, direction):
        if self.can_command():
            self.target = None
            self.read_positions(jog=(self.joint.currentText(), direction))

    def read_positions(self, _checked=False, *, jog=None, target=None, automatic=False):
        if not self.read.isEnabled():
            return
        self.poll.stop()
        command = jog or target
        self.manual_read = not automatic and command is None
        access_error = port_access_error(self.port)
        if access_error:
            self.finish_read(access_error)
            return
        self.jog_active = command is not None
        self.read.setEnabled(False)
        if command:
            for control in (self.interpreter, self.calibration, self.browse):
                control.setEnabled(False)
        self.update_jog_controls()
        if command:
            self.motion_status.setText("Executing one motor step…")
        elif not automatic:
            self.status.setText("Reading motor positions…")
        self.deadline.start()
        args = ["-B", so101_usb.__file__, self.port]
        selected = self.calibration.currentData()
        self.read_context = (self.python, selected)
        if selected:
            args.append(selected)
        if command:
            joint, value = command
            args.extend(("--joint", joint, "--target" if target else "--direction", str(value)))
        self.process.start(self.python, args)

    def process_error(self, error):
        if error == QProcess.ProcessError.FailedToStart:
            self.finish_read("Could not start LeRobot Python. Select its interpreter and retry.")

    def read_timed_out(self):
        if self.jog_active:
            self.motion_status.setText("Waiting for torque release. Keep the arm supported.")
        else:
            self.process.kill()

    def read_finished(self, code, status):
        jog = self.jog_active
        timed_out = not self.deadline.isActive() and not jog
        data = bytes(self.process.readAllStandardOutput())
        error = bytes(self.process.readAllStandardError()).decode(errors="replace").strip()
        if self.read_context != (self.python, self.calibration.currentData()):
            self.finish_read("Waiting for current motor positions…")
            return
        if timed_out:
            self.finish_read("No response. Check the port, arm power, and motor setup.")
            return
        if code or status != QProcess.ExitStatus.NormalExit:
            self.finish_read(
                error
                or (
                    "Manual step interrupted. Support the arm and check motor power."
                    if jog
                    else "Could not read positions from this port."
                )
            )
            return
        try:
            sample = json.loads(data)
            calibrated = sample["calibrated"]
            values = {name: float(sample["positions"][name]) for name in self.positions}
            if not isinstance(calibrated, bool) or not all(
                math.isfinite(v) for v in values.values()
            ):
                raise ValueError("Non-finite reading or invalid calibration flag")
        except (ValueError, TypeError, KeyError):
            self.finish_read("LeRobot returned an invalid position sample.")
            return
        for name, value in values.items():
            unit = ("%" if name == "gripper" else "°") if calibrated else " counts"
            self.positions[name].setText(f"{value:.2f}{unit}" if calibrated else f"{value:g}{unit}")
        if calibrated:
            if jog:
                self.motion_status.setText("Step complete · torque released")
            result = (
                "Calibration matches the motors · live"
                if self.arm
                else "LeRobot calibration matches the motors · read only"
            )
            self.encoders = sample.get("encoders", {})
            if self.target_start:
                target, before = self.target_start
                joint, goal = target
                observed = self.encoders.get(joint)
                if self.target == target and (
                    observed is None or abs(goal - observed) >= abs(goal - before)
                ):
                    self.target = None
                    self.motion_status.setText(
                        "Target cancelled: the joint did not advance toward it."
                    )
        elif self.calibration.currentData():
            result = "Calibration does not match the motors · showing raw counts"
        else:
            result = "Raw motor counts · read only"
        self.finish_read(result, calibrated=calibrated, valid_sample=True)
        if calibrated:
            self.feedback.emit(sample)

    def finish_read(self, text, *, calibrated=False, valid_sample=False):
        self.deadline.stop()
        self.target_start = None
        if not valid_sample:
            for value in self.positions.values():
                value.setText("—")
        if not calibrated:
            self.target = None
            self.encoders = {}
        if self.jog_active and not calibrated:
            self.jog_failed = True
            self.motion_status.setText(
                text[:1200] + " · Use Read positions to verify before another step."
            )
        if self.manual_read and calibrated and self.jog_failed:
            self.jog_failed = False
            self.motion_status.clear()
        self.jog_active = False
        self.calibration_verified = calibrated
        self.status.setText(text[:1200])
        for control in (self.read, self.interpreter, self.calibration, self.browse):
            control.setEnabled(True)
        self.update_jog_controls()
        self.availability_changed.emit()
        self.schedule_poll()

    def done(self, result):
        if self.jog_active:
            return
        self.target = None
        self.poll.stop()
        self.deadline.stop()
        super().done(result)
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.kill()
            self.process.waitForFinished(1000)
