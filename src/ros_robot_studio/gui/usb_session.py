"""A persistent LeRobot process with a modeless dialog for USB setup and readings."""

import json
import math
import time

from PySide6.QtCore import QProcess, Qt, QTimer
from PySide6.QtWidgets import QDialog, QDialogButtonBox, QFormLayout

from ros_robot_studio.backends import so101_usb
from ros_robot_studio.gui.usb_robot import USBPositionDialog, port_access_error


class USBSessionDialog(USBPositionDialog):
    def __init__(self, port, settings, parent=None, *, allow_jog=False):
        super().__init__(port, settings, parent)
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.allow_motor_control = allow_jog
        self.motor_control_requested = self.motor_control_pending = False
        self.motion, self.motion_message = "idle", ""
        self.motion_request = 0
        self.motion_pending = None
        self.armed = self.calibration_verified = self.failed = False
        self.closing = self.restart = False
        self.buffer = bytearray()
        self.last_sample = time.monotonic()
        self.process = QProcess(self)
        self.process.started.connect(self.poll_once)
        self.process.readyReadStandardOutput.connect(self.receive_samples)
        self.process.finished.connect(self.read_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.poll = QTimer(self, interval=300)
        self.poll.timeout.connect(self.poll_once)
        self.calibration.currentIndexChanged.connect(self.calibration_changed)
        layout = QFormLayout(self)
        self.add_position_rows(layout)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Close)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)

    def showEvent(self, event):
        super().showEvent(event)
        self.read_positions()

    def can_change_motor_control(self):
        return (
            self.allow_motor_control
            and (self.calibration_verified or self.armed)
            and not (self.failed or self.closing or self.restart or self.motor_control_pending)
            and self.process.state() == QProcess.ProcessState.Running
        )

    def can_command(self):
        return (
            self.motor_control_requested
            and self.armed
            and self.calibration_verified
            and self.motion == "idle"
            and self.motion_pending is None
            and not (self.closing or self.restart or self.failed)
            and self.process.state() == QProcess.ProcessState.Running
        )

    def send(self, operation, **values):
        if self.process.state() == QProcess.ProcessState.Running:
            message = json.dumps(dict(operation=operation, **values)) + "\n"
            self.process.write(message.encode())

    def set_motor_control(self, enabled, speed, torque):
        if self.motor_control_requested == enabled or not self.can_change_motor_control():
            return
        self.motor_control_requested = enabled
        self.motor_control_pending = True
        if enabled:
            self.send("arm", speed=speed, torque=torque)
        else:
            self.send("disarm")
        self.notify()

    def set_limits(self, speed, torque):
        if self.can_command():
            self.send("limits", speed=speed, torque=torque)

    def request_target(self, joint, encoder):
        if self.can_command():
            self.send("target", joint=joint, position=encoder)

    def request_motion(self, operation, path, speed, torque):
        if operation == "stop_motion":
            if self.motion == "idle" or self.motion_pending is not None:
                return
        elif not self.can_command():
            return
        self.motion_request += 1
        self.motion_pending = self.motion_request
        self.motion_message = "Waiting for USB confirmation…"
        self.send(operation, request_id=self.motion_request, path=path, speed=speed, torque=torque)
        self.notify()

    def poll_once(self):
        if self.closing or self.restart:
            self.send("close")
            return
        self.send("ping")
        if time.monotonic() - self.last_sample > 2:
            self.calibration_verified = self.motor_control_requested = False
            self.motion_pending = None
            self.motor_control_pending = self.motor_control_pending or self.armed
            self.send("disarm")
            self.status.setText("USB feedback is stale. Waiting for the motors to release.")
            self.notify()

    def read_positions(self):
        if self.closing or self.process.state() != QProcess.ProcessState.NotRunning:
            return
        error = port_access_error(self.port)
        if error:
            self.status.setText(error)
            self.notify()
            return
        self.buffer.clear()
        self.calibration_verified = self.failed = False
        self.last_sample = time.monotonic()
        args = ["-u", "-B", so101_usb.__file__, self.port]
        if self.calibration.currentData():
            args.append(self.calibration.currentData())
        args.append("--stream")
        self.status.setText("Connecting to the USB runtime…")
        self.process.start(self.python, args)
        self.poll.start()
        self.notify()

    def calibration_changed(self):
        self.calibration_verified = False
        for value in self.positions.values():
            value.setText("—")
        if self.process.state() == QProcess.ProcessState.NotRunning:
            self.read_positions()
        else:
            self.restart = True
            self.send("close")
            self.status.setText("Checking the selected calibration…")
            self.notify()

    def notify(self):
        selectable = not (
            self.motor_control_requested
            or self.armed
            or self.motor_control_pending
            or self.restart
            or self.closing
        )
        self.calibration.setEnabled(selectable)
        self.browse.setEnabled(selectable)
        self.availability_changed.emit()

    def receive_samples(self):
        self.buffer.extend(bytes(self.process.readAllStandardOutput()))
        if self.restart or self.closing:
            self.buffer.clear()
            return
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            try:
                sample = json.loads(line)
                calibrated = sample["calibrated"]
                armed = sample["armed"]
                values = {name: float(sample["positions"][name]) for name in self.positions}
                if (
                    not isinstance(calibrated, bool)
                    or not isinstance(armed, bool)
                    or not all(math.isfinite(value) for value in values.values())
                ):
                    raise ValueError("Invalid USB sample")
            except (ValueError, TypeError, KeyError):
                self.failed = True
                self.calibration_verified = self.motor_control_requested = False
                self.send("close")
                self.status.setText("Invalid feedback. Closing the USB runtime.")
                self.notify()
                return
            self.last_sample = time.monotonic()
            self.calibration_verified = calibrated
            self.armed = armed
            if not armed or sample.get("motion_ack") == self.motion_pending:
                self.motion_pending = None
            self.motion = sample.get("motion", "idle")
            if self.motion_pending is None:
                self.motion_message = sample.get("motion_message", "")
            if armed == self.motor_control_requested:
                self.motor_control_pending = False
            for name, value in values.items():
                unit = ("%" if name == "gripper" else "°") if calibrated else " counts"
                self.positions[name].setText(f"{value:.2f}{unit}")
            self.status.setText(
                "Live USB · motor position control active"
                if armed
                else "Live USB · calibrated · torque off"
                if calibrated
                else "Live USB · raw counts · matching calibration required"
            )
            self.notify()
            if calibrated:
                self.feedback.emit(sample)

    def read_finished(self, code, status):
        error = bytes(self.process.readAllStandardError()).decode(errors="replace").strip()
        self.failed = self.failed or bool(code or status != QProcess.ExitStatus.NormalExit)
        if error or not self.failed:
            self.status.setText(error or "USB runtime stopped · motor torque released")
        self.finish_process()

    def process_error(self, error):
        if error == QProcess.ProcessError.FailedToStart:
            self.failed = True
            self.status.setText(f"Could not start LeRobot Python ({self.python}).")
            self.finish_process()

    def finish_process(self):
        self.poll.stop()
        self.armed = self.calibration_verified = False
        self.motor_control_requested = self.motor_control_pending = False
        self.motion, self.motion_pending = "idle", None
        self.motion_message = self.status.text()
        if self.closing:
            super().done(QDialog.DialogCode.Rejected)
        elif self.restart:
            self.restart = False
            self.read_positions()
        else:
            self.notify()

    def shutdown(self):
        self.closing = True
        self.hide()
        if self.process.state() == QProcess.ProcessState.NotRunning:
            self.finish_process()
        else:
            self.send("close")

    def done(self, result):
        self.hide()
