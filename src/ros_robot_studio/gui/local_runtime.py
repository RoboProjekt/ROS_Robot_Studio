"""Attach USB or local MuJoCo feedback to the same state used by ROS and the viewer."""

import math

from PySide6.QtCore import QObject, QSignalBlocker, QTimer

from ros_robot_studio.backends.local_mujoco import LocalMujoco
from ros_robot_studio.backends.so101_usb import JOG_SPEED, JOINT_NAMES, USB_TORQUE_RANGE
from ros_robot_studio.gui.usb_session import USBSessionDialog


class LocalJointRuntime(QObject):
    def __init__(self, main):
        super().__init__(main)
        self.main = main
        self.profile = self.lease = self.dialog = self.mujoco = None
        self.retiring_usb = None
        self.ranges = {}
        self.error = ""
        self.timer = QTimer(self, interval=30)
        self.timer.timeout.connect(self.step_mujoco)
        self.controls = main.robot_panel.joint_controls
        self.motor_control = self.controls.usb_motor_control
        self.motor_control.toggled.connect(self.set_usb_motor_control)
        self.controls.usb_speed.valueChanged.connect(self.set_usb_limits)
        self.controls.usb_torque.valueChanged.connect(self.set_usb_limits)
        main.backend_manager.changed.connect(self.check_selected_profile)

    def check_selected_profile(self):
        manager = self.main.backend_manager
        if self.profile and (
            manager.selected_id != self.profile.id
            or manager.profiles.get(self.profile.id) is not self.profile
        ):
            self.main.start_offline()

    def enter(self, profile, backend, description):
        main = self.main
        if main.mode != "offline":
            raise ValueError("Disconnect the active runtime before starting another one.")
        if not profile.enabled:
            raise ValueError("Backend profile is disabled.")
        main.motion.stop()
        main.detach_transport()
        self.profile = profile
        self.error = ""
        main.mode = "local"
        self.lease = main.state_manager.activate_source(backend, description)
        main.mode_status.setText(description)
        main.monitor.mode.setText(description)
        main.monitor.source.setCurrentIndex(0)
        main.monitor.container.setText("Local runtime")
        main.robot_panel.profile.setEnabled(False)
        main.connection_status("Waiting for live joint feedback")
        main.busy_changed(main.runner.busy)
        main.update_joint_controls()

    def start_usb(self, profile, port, allow_jog):
        if self.retiring_usb:
            raise ValueError("Wait for the previous USB connection to release torque.")
        if self.dialog and self.profile.id == profile.id and self.dialog.port == port:
            self.dialog.show()
            self.dialog.raise_()
            return
        if self.main.project.robot.model_id != "so101_follower" or (
            set(self.main.model.default_configuration()) != set(JOINT_NAMES)
        ):
            raise ValueError("Load the catalog SO-101 Follower model before connecting USB.")
        self.enter(profile, "so101_usb", "SO-101 · USB")
        self.controls.usb_speed.setValue(JOG_SPEED)
        self.controls.usb_torque.setValue(USB_TORQUE_RANGE[1])
        dialog = USBSessionDialog(port, self.main.settings, self.main, allow_jog=allow_jog)
        self.dialog = dialog
        dialog.feedback.connect(lambda sample: self.usb_feedback(dialog, sample))
        dialog.availability_changed.connect(lambda: self.usb_availability(dialog))
        dialog.finished.connect(lambda _: self.usb_closed(dialog))
        self.update_usb_motor_control()
        dialog.show()

    def set_usb_motor_control(self, enabled):
        if self.dialog:
            self.dialog.set_motor_control(
                enabled, self.controls.usb_speed.value(), self.controls.usb_torque.value()
            )
        self.update_usb_motor_control()

    def set_usb_limits(self):
        if self.dialog:
            self.dialog.set_limits(
                self.controls.usb_speed.value(), self.controls.usb_torque.value()
            )

    def update_usb_motor_control(self):
        dialog = self.dialog
        available = dialog is not None and dialog.allow_motor_control
        ready = available and dialog.can_change_motor_control()
        with QSignalBlocker(self.motor_control):
            self.motor_control.setChecked(available and dialog.motor_control_requested)
            self.motor_control.setEnabled(ready)
        self.motor_control.setVisible(available)
        self.controls.usb_limits.setEnabled(ready)
        self.controls.usb_limits.setVisible(available)

    def usb_closed(self, dialog):
        if self.dialog is dialog:
            self.dialog = None
            self.main.start_offline()
        if self.retiring_usb is dialog:
            self.retiring_usb = None
            if dialog.failed:
                self.main.transport_error(dialog.status.text())
        dialog.deleteLater()

    def usb_availability(self, dialog):
        if self.dialog is dialog:
            self.update_usb_motor_control()
            if not dialog.calibration_verified:
                self.invalidate(dialog.status.text())
            self.main.update_joint_controls()

    def usb_feedback(self, dialog, sample):
        if self.dialog is not dialog:
            return
        try:
            ranges = sample["ranges"]
            encoders = sample["encoders"]
            bounds = self.main.model.effective_joint_bounds
            positions = {}
            for name in JOINT_NAMES:
                low, high = ranges[name]
                lo, hi = bounds[name]
                raw = encoders[name]
                if not (0 <= low < high <= 4095 and low <= raw <= high and math.isfinite(lo + hi)):
                    raise ValueError(f"{name}: encoder feedback is outside calibrated travel.")
                if name == "gripper":
                    positions[name] = lo + (raw - low) / (high - low) * (hi - lo)
                else:
                    positions[name] = math.radians(float(sample["positions"][name]))
            self.ranges = ranges
            self.publish(positions, source="SO-101 calibrated USB")
        except (ValueError, TypeError, KeyError) as exc:
            self.invalidate(str(exc))

    def start_mujoco(self, profile, scene):
        if self.mujoco and self.profile.id == profile.id:
            return
        if (
            profile.model_signature
            and profile.model_signature != self.main.state_manager.snapshot().model_signature
        ):
            raise ValueError("MuJoCo profile is bound to a different robot model.")
        backend = LocalMujoco(
            self.main.model, scene, profile.joint_mapping, self.main.independent_positions()
        )
        self.enter(profile, "mujoco", "MuJoCo · local simulation")
        self.mujoco = backend
        self.timer.start()
        self.step_mujoco()

    def step_mujoco(self):
        try:
            positions, velocities = self.mujoco.step()
            self.publish(positions, velocities=velocities, source="MuJoCo joint state")
        except (ValueError, RuntimeError) as exc:
            self.timer.stop()
            self.invalidate(str(exc))

    def publish(self, positions, *, source, velocities=None):
        if self.main.state_manager.update(
            self.lease, positions, velocities=velocities, source=source
        ):
            if self.error or self.main.ros_status == "Waiting for live joint feedback":
                self.error = ""
                self.main.connection_status("Live joint feedback")
            self.main.update_joint_controls()

    def invalidate(self, error):
        if self.profile and error != self.error:
            self.error = error
            self.main.state_manager.invalidate(error)
            self.main.update_joint_controls()
            self.main.connection_status(error)

    def command_joints(self):
        if not self.profile or self.main.state_manager.stale(2.0):
            return set()
        if self.dialog and self.dialog.can_command():
            return set(self.ranges)
        return set(self.mujoco.actuators) if self.mujoco and self.timer.isActive() else set()

    def target(self, joint, position):
        if joint not in self.command_joints():
            return
        if self.mujoco:
            self.mujoco.target(joint, position)
        else:
            lo, hi = self.main.model.effective_joint_bounds[joint]
            low, high = self.ranges[joint]
            bounded = max(lo, min(hi, position))
            if joint == "gripper":
                encoder = low + (bounded - lo) / (hi - lo) * (high - low)
            else:
                encoder = (low + high) / 2 + math.degrees(bounded) * 4095 / 360
            self.dialog.request_target(joint, round(max(low, min(high, encoder))))

    def stop(self):
        self.timer.stop()
        dialog = self.dialog
        self.profile = self.lease = self.dialog = self.mujoco = None
        self.ranges = {}
        self.update_usb_motor_control()
        if dialog:
            self.retiring_usb = dialog
            dialog.shutdown()

    def close_ready(self):
        if self.retiring_usb:
            return False
        if self.dialog:
            self.stop()
            return False
        return True
