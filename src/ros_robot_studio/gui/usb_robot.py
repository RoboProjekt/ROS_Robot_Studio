"""Local serial-port discovery and a read-only SO-101 position monitor."""

import json
import math
import os
import sys
from pathlib import Path

from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtSerialPort import QSerialPortInfo
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
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


class USBJointDialog(QDialog):
    def __init__(self, port, settings, parent=None):
        super().__init__(parent)
        self.setWindowTitle("SO-101 Follower · USB Joint Positions")
        self.port = port
        self.settings = settings
        self.python = settings.get("lerobot_python") or lerobot_interpreter()
        self.process = QProcess(self)
        self.process.finished.connect(self.read_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.deadline = QTimer(self, singleShot=True, interval=12000)
        self.deadline.timeout.connect(self.read_timed_out)
        layout = QFormLayout(self)
        port_label = QLabel(port)
        port_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addRow("Serial port", port_label)
        self.interpreter = QPushButton(self.python)
        self.interpreter.clicked.connect(self.choose_interpreter)
        layout.addRow("LeRobot Python", self.interpreter)
        self.calibration = QComboBox()
        self.calibration.addItem("No calibration · raw motor counts", "")
        files = calibration_files()
        for path in files:
            self.add_calibration(path)
        if len(files) == 1:
            self.calibration.setCurrentIndex(1)
        layout.addRow("LeRobot calibration", self.calibration)
        self.browse = QPushButton("Choose calibration file…")
        self.browse.clicked.connect(self.choose_calibration)
        layout.addRow(self.browse)
        self.status = QLabel(port_access_error(port) or "Serial access available · read only")
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        self.status.setWordWrap(True)
        layout.addRow(self.status)
        self.positions = {name: QLabel("—") for name in so101_usb.JOINT_NAMES}
        for name, value in self.positions.items():
            layout.addRow(name, value)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        self.read = buttons.addButton("Read positions", QDialogButtonBox.ButtonRole.ActionRole)
        self.read.clicked.connect(self.read_positions)
        layout.addRow(buttons)

    def choose_interpreter(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select LeRobot's Python interpreter", self.python
        )
        if path:
            self.python = path
            self.settings.set("lerobot_python", path)
            self.interpreter.setText(path)

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

    def read_positions(self):
        for value in self.positions.values():
            value.setText("—")
        access_error = port_access_error(self.port)
        if access_error:
            self.status.setText(access_error)
            return
        for control in (self.read, self.interpreter, self.calibration, self.browse):
            control.setEnabled(False)
        self.status.setText("Reading motor positions…")
        self.deadline.start()
        args = ["-B", so101_usb.__file__, self.port]
        if selected := self.calibration.currentData():
            args.append(selected)
        self.process.start(self.python, args)

    def process_error(self, error):
        if error == QProcess.ProcessError.FailedToStart:
            self.finish_read("Could not start LeRobot Python. Select its interpreter and retry.")

    def read_timed_out(self):
        self.process.kill()

    def read_finished(self, code, status):
        timed_out = not self.deadline.isActive()
        data = bytes(self.process.readAllStandardOutput())
        error = bytes(self.process.readAllStandardError()).decode(errors="replace").strip()
        if timed_out:
            self.finish_read("No response. Check the port, arm power, and motor setup.")
            return
        if code or status != QProcess.ExitStatus.NormalExit:
            self.finish_read(error or "Could not read positions from this port.")
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
            result = "LeRobot calibration matches the motors · read only"
        elif self.calibration.currentData():
            result = "Calibration does not match the motors · showing raw counts"
        else:
            result = "Raw motor counts · read only"
        self.finish_read(result)

    def finish_read(self, text):
        self.deadline.stop()
        self.status.setText(text[:1200])
        for control in (self.read, self.interpreter, self.calibration, self.browse):
            control.setEnabled(True)

    def done(self, result):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.process.kill()
            self.process.waitForFinished(1000)
        super().done(result)
