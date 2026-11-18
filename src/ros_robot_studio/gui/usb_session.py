"""A persistent LeRobot process for position feedback and slider targets."""

import json
import math
import time

from PySide6.QtCore import QProcess

from ros_robot_studio.backends import so101_usb
from ros_robot_studio.gui.usb_robot import USBJointDialog, port_access_error


class USBSessionDialog(USBJointDialog):
    def __init__(self, *args, **kwargs):
        self.armed = False
        self.closing_result = None
        self.restart = False
        self.buffer = bytearray()
        self.last_sample = time.monotonic()
        super().__init__(*args, **kwargs)
        self.process.readyReadStandardOutput.connect(self.receive_samples)
        self.poll.setInterval(300)
        if self.arm:
            self.arm.setText("Arm is fixed and supported. Enable motor position control.")

    def can_command(self):
        return self.armed and super().can_command() and self.closing_result is None

    def send(self, operation, **values):
        if self.process.state() == QProcess.ProcessState.Running:
            message = json.dumps(dict(operation=operation, **values)) + "\n"
            self.process.write(message.encode())

    def arm_changed(self, armed):
        self.jog_active = self.process.state() == QProcess.ProcessState.Running
        self.send("arm" if armed else "disarm")
        self.update_jog_controls()
        self.availability_changed.emit()

    def request_target(self, joint, encoder):
        if self.can_command():
            self.send("target", joint=joint, position=encoder)
            self.motion_status.setText(f"Position target sent · {joint}")

    def step_joint(self, direction):
        joint = self.joint.currentText()
        if self.can_command() and joint in self.encoders:
            ranges = self.ranges[joint]
            target = max(
                ranges[0], min(ranges[1], self.encoders[joint] + direction * so101_usb.JOG_COUNTS)
            )
            self.request_target(joint, target)

    def poll_once(self):
        if self.closing_result is not None:
            self.send("close")
            self.schedule_poll()
            return
        self.send("ping")
        if time.monotonic() - self.last_sample > 2:
            self.calibration_verified = False
            self.armed = False
            self.send("disarm")
            self.status.setText("USB feedback is stale. Waiting for the motors to release.")
            self.availability_changed.emit()
            self.update_jog_controls()
        self.schedule_poll()

    def read_positions(self, _checked=False, **_):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.send("ping")
            return
        error = port_access_error(self.port)
        if error:
            self.status.setText(error)
            return
        self.buffer.clear()
        self.calibration_verified = self.armed = False
        self.jog_failed = False
        self.closing_result = None
        self.read_context = (self.python, self.calibration.currentData())
        args = ["-u", "-B", so101_usb.__file__, self.port]
        if self.calibration.currentData():
            args.append(self.calibration.currentData())
        args.append("--stream")
        self.read.setEnabled(False)
        self.last_sample = time.monotonic()
        self.process.start(self.python, args)
        self.status.setText("Connecting to the USB runtime…")
        self.schedule_poll()

    def invalidate_reading(self):
        super().invalidate_reading()
        self.restart = True
        self.send("close")
        self.interpreter.setEnabled(False)
        self.browse.setEnabled(False)

    def update_jog_controls(self):
        super().update_jog_controls()
        if self.arm:
            ready = self.can_command()
            self.lower.setEnabled(ready)
            self.upper.setEnabled(ready)
        for control in (self.interpreter, self.calibration, self.browse):
            control.setEnabled(not self.armed and not self.jog_active)

    def receive_samples(self):
        self.buffer.extend(bytes(self.process.readAllStandardOutput()))
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer = bytearray(rest)
            try:
                sample = json.loads(line)
                calibrated = sample["calibrated"]
                armed = sample["armed"]
                values = {name: float(sample["positions"][name]) for name in self.positions}
                encoders = {name: int(sample["encoders"][name]) for name in self.positions}
                if (
                    not isinstance(calibrated, bool)
                    or not isinstance(armed, bool)
                    or not all(math.isfinite(value) for value in values.values())
                ):
                    raise ValueError("Invalid USB sample")
            except (ValueError, TypeError, KeyError):
                self.send("close")
                self.status.setText("Invalid feedback. Closing the USB runtime.")
                self.calibration_verified = False
                self.availability_changed.emit()
                return
            self.last_sample = time.monotonic()
            self.calibration_verified = calibrated
            self.armed = armed
            self.jog_active = False
            self.encoders = encoders
            self.ranges = sample.get("ranges", {})
            if self.read_context != (self.python, self.calibration.currentData()):
                continue
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
            self.read.setEnabled(True)
            self.update_jog_controls()
            self.availability_changed.emit()
            if self.closing_result is not None:
                self.send("close")
            if calibrated:
                self.feedback.emit(sample)
        self.schedule_poll()

    def read_finished(self, code, status):
        self.poll.stop()
        self.armed = self.calibration_verified = self.jog_active = False
        error = bytes(self.process.readAllStandardError()).decode(errors="replace").strip()
        self.jog_failed = bool(code or status != QProcess.ExitStatus.NormalExit)
        self.status.setText(error or "USB runtime stopped · motor torque released")
        self.read.setEnabled(True)
        self.update_jog_controls()
        self.availability_changed.emit()
        if self.closing_result is not None:
            result = self.closing_result
            self.closing_result = None
            super().done(result)
        elif self.restart:
            self.restart = False
            self.read_positions()

    def process_error(self, error):
        if error == QProcess.ProcessError.FailedToStart:
            self.poll.stop()
            self.calibration_verified = self.armed = self.jog_active = False
            self.read.setEnabled(True)
            self.status.setText("Could not start LeRobot Python. Select its interpreter and retry.")
            self.update_jog_controls()
            self.availability_changed.emit()
            if self.closing_result is not None:
                result = self.closing_result
                self.closing_result = None
                super().done(result)

    def done(self, result):
        if self.process.state() != QProcess.ProcessState.NotRunning:
            self.closing_result = result
            self.send("close")
            self.status.setText("Closing USB connection and releasing motor torque…")
            return
        self.poll.stop()
        super().done(result)
