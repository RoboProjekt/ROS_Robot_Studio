"""Framed pose, teach-in, task-step and pick/place dialogs; no execution transport."""

from __future__ import annotations

import math
import threading

import numpy as np
from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
    QWizard,
    QWizardPage,
)

from ros_robot_studio.gui.joint_controls import JointControls
from ros_robot_studio.gui.widgets import button, label
from ros_robot_studio.gui.workers import TaskRunner
from ros_robot_studio.motion.pick_place import PickPlaceGenerator, PickPlaceRequest
from ros_robot_studio.motion.schema import CartesianPose, SavedPosition, TaskStep
from ros_robot_studio.robots.urdf import pose_rpy


def number(value=0, minimum=-100000, maximum=100000, decimals=4, suffix=""):
    spin = QDoubleSpinBox()
    spin.setMinimumHeight(30)
    spin.setDecimals(decimals)
    spin.setRange(minimum, maximum)
    spin.setValue(value)
    spin.setSuffix(suffix)
    spin.setStyleSheet("QDoubleSpinBox { padding: 4px; }")
    return spin


class PoseEditor(QWidget):
    def __init__(self, chain, joints, pose=None, parent=None):
        super().__init__(parent)
        self.chain = chain
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        row = QFormLayout()
        self.frame = QComboBox()
        self.frame.addItems(chain.model.links)
        self.frame.setCurrentText(pose.frame if pose else chain.base_link)
        self.end = QComboBox()
        self.end.addItems(chain.model.links)
        self.end.setCurrentText(pose.end_effector if pose else chain.end_effector)
        row.addRow("Reference frame", self.frame)
        row.addRow("End effector", self.end)
        layout.addLayout(row)
        self.mode = QComboBox()
        self.mode.addItem("Position Only", "position_only")
        self.mode.addItem("Position + axis direction", "axis")
        self.mode.addItem("Full Pose · 6D", "full_pose")
        if not chain.analyze().full_pose_available:
            self.mode.model().item(2).setEnabled(False)
            self.mode.setItemData(
                2,
                "This chain has fewer than six independent Cartesian dimensions",
                Qt.ItemDataRole.ToolTipRole,
            )
        self.mode.setCurrentIndex(
            max(0, self.mode.findData(pose.mode if pose else "position_only"))
        )
        self.axis = QComboBox()
        self.axis.addItems(["X", "Y", "Z"])
        self.axis.setCurrentText(pose.axis if pose else "Z")
        modes = QHBoxLayout()
        modes.addWidget(self.mode, 1)
        modes.addWidget(QLabel("Tool axis"))
        modes.addWidget(self.axis)
        layout.addLayout(modes)
        current = pose_rpy(chain.forward_kinematics(joints))
        values = (*pose.position, *pose.rpy) if pose else current
        grid = QGridLayout()
        self.values = []
        for index, (name, value) in enumerate(zip(["X", "Y", "Z", "Roll", "Pitch", "Yaw"], values)):
            spin = number(
                value * 1000 if index < 3 else math.degrees(value),
                suffix=" mm" if index < 3 else " °",
            )
            grid.addWidget(QLabel(name), index // 3 * 2, index % 3)
            grid.addWidget(spin, index // 3 * 2 + 1, index % 3)
            self.values.append(spin)
        layout.addLayout(grid)
        self.mode.currentIndexChanged.connect(self.update_mode)
        self.update_mode()

    def update_mode(self, *_):
        orientation = self.mode.currentData() != "position_only"
        for spin in self.values[3:]:
            spin.setEnabled(orientation)
        self.axis.setEnabled(self.mode.currentData() == "axis")

    def pose(self):
        return CartesianPose(
            frame=self.frame.currentText(),
            end_effector=self.end.currentText(),
            position=tuple(s.value() / 1000 for s in self.values[:3]),
            rpy=tuple(math.radians(s.value()) for s in self.values[3:]),
            mode=self.mode.currentData(),
            axis=self.axis.currentText(),
        )

    def set_pose(self, pose):
        self.frame.setCurrentText(pose.frame)
        self.end.setCurrentText(pose.end_effector)
        self.mode.setCurrentIndex(self.mode.findData(pose.mode))
        self.axis.setCurrentText(pose.axis)
        for index, (spin, value) in enumerate(zip(self.values, (*pose.position, *pose.rpy))):
            spin.setValue(value * 1000 if index < 3 else math.degrees(value))


class PositionDialog(QDialog):
    def __init__(self, chain, current, position=None, kind="cartesian", parent=None):
        super().__init__(parent)
        self.chain = chain
        self.current = current
        self.original = position
        self.setWindowTitle("Teach / edit position")
        self.resize(570, 580)
        layout = QVBoxLayout(self)
        self.name = QLineEdit(position.name if position else "New position")
        layout.addWidget(QLabel("Position name"))
        layout.addWidget(self.name)
        self.kind = QComboBox()
        self.kind.addItem("Joint position · bound to this robot", "joint")
        self.kind.addItem("Cartesian pose · resolve via IK", "cartesian")
        self.kind.setCurrentIndex(self.kind.findData(position.kind if position else kind))
        layout.addWidget(self.kind)
        self.stack = QStackedWidget()
        self.joints = JointControls()
        matching = (
            not position
            or position.kind != "joint"
            or position.model_signature == chain.engine.signature
        )
        values = position.joints if position and position.kind == "joint" and matching else current
        self.joints.set_model(chain.model, values)
        self.stack.addWidget(self.joints)
        self.cartesian = PoseEditor(
            chain, current, position.pose if position and position.kind == "cartesian" else None
        )
        self.stack.addWidget(self.cartesian)
        layout.addWidget(self.stack)
        self.reteach = QCheckBox("Explicitly re-teach this joint position for the current robot")
        self.reteach.setVisible(not matching)
        layout.addWidget(self.reteach)
        self.kind.currentIndexChanged.connect(self.stack.setCurrentIndex)
        self.stack.setCurrentIndex(self.kind.currentIndex())
        layout.addWidget(
            label(
                "Values are offline configuration. Joint positions cannot be silently applied to another robot.",
                "subtitle",
                True,
            )
        )
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.validate_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)
        self.matching = matching

    def position(self):
        if self.kind.currentData() == "joint":
            if not self.matching and not self.reteach.isChecked():
                raise ValueError("Confirm re-teaching before changing the robot binding")
            data = dict(
                name=self.name.text().strip(),
                kind="joint",
                model_signature=self.chain.engine.signature,
                joints=dict(self.joints.values),
            )
        else:
            data = dict(name=self.name.text().strip(), kind="cartesian", pose=self.cartesian.pose())
        if self.original:
            data["id"] = self.original.id
        return SavedPosition(**data)

    def validate_accept(self):
        try:
            self.position()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid position", str(exc))
            return
        self.accept()


class StepDialog(QDialog):
    ACTIONS = [
        ("Move to Joint Position", "move_joint"),
        ("Move to Cartesian Pose", "move_cartesian"),
        ("Open Gripper", "open_gripper"),
        ("Close Gripper", "close_gripper"),
        ("Wait", "wait"),
        ("Move Relative · endpoint IK", "move_relative"),
        ("Home", "home"),
    ]

    def __init__(self, chain, positions, current, step=None, parent=None):
        super().__init__(parent)
        self.chain = chain
        self.original = step
        self.positions = positions
        self.setWindowTitle("Configure task step")
        self.resize(530, 520)
        value = step or TaskStep(action="move_joint")
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.name = QLineEdit(value.name)
        self.action = QComboBox()
        for title, identifier in self.ACTIONS:
            self.action.addItem(title, identifier)
        self.action.setCurrentIndex(self.action.findData(value.action))
        form.addRow("Name", self.name)
        form.addRow("Action", self.action)
        self.target = QComboBox()
        form.addRow("Saved position", self.target)
        self.duration = number(value.duration, 0, 3600, suffix=" s")
        form.addRow("Minimum move time / wait", self.duration)
        self.frame = QComboBox()
        self.frame.addItems(chain.model.links)
        self.frame.setCurrentText(value.reference_frame or chain.base_link)
        form.addRow("Relative reference frame", self.frame)
        layout.addLayout(form)
        self.relative = []
        grid = QGridLayout()
        for index, (name, val) in enumerate(
            zip(
                ["dX · mm", "dY · mm", "dZ · mm", "dRoll · °", "dPitch · °", "dYaw · °"],
                (*value.relative_xyz, *value.relative_rpy),
            )
        ):
            widget = number(val * 1000 if index < 3 else math.degrees(val))
            self.relative.append(widget)
            grid.addWidget(QLabel(name), index // 3 * 2, index % 3)
            grid.addWidget(widget, index // 3 * 2 + 1, index % 3)
        layout.addLayout(grid)
        self.gripper_values = {}
        grip = QFormLayout()
        for name in chain.gripper_joints:
            joint = chain.engine.joints[name]
            factor = 1 if joint.kind == "prismatic" else 180 / math.pi
            widget = number(
                value.gripper_values.get(name, current[name]) * factor,
                joint.lower * factor if joint.lower is not None else -180,
                joint.upper * factor if joint.upper is not None else 180,
                suffix=" m" if joint.kind == "prismatic" else " °",
            )
            self.gripper_values[name] = (widget, factor)
            grip.addRow(name, widget)
        layout.addLayout(grip)
        self.attach = QCheckBox("Attach virtual object on close (nonphysical)")
        self.attach.setChecked(value.attach_object)
        self.release = QCheckBox("Release virtual object on open")
        self.release.setChecked(value.release_object)
        layout.addWidget(self.attach)
        layout.addWidget(self.release)
        layout.addWidget(
            label(
                "Moves use synchronized joint-space curves. Relative motion solves an endpoint; a straight Cartesian path is not available.",
                "warning",
                True,
            )
        )
        self.action.currentIndexChanged.connect(self.update_action)
        self.update_action()
        index = self.target.findData(value.position_id)
        if index >= 0:
            self.target.setCurrentIndex(index)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.validate_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def update_action(self, *_):
        action = self.action.currentData()
        self.target.clear()
        self.target.addItem(
            "Default virtual home" if action == "home" else "Select position…", None
        )
        for p in self.positions.positions:
            if (action in ("home", "move_joint") and p.kind == "joint") or (
                action == "move_cartesian" and p.kind == "cartesian"
            ):
                self.target.addItem(p.name, p.id)
        self.target.setEnabled(action in ("move_joint", "move_cartesian", "home"))
        self.frame.setEnabled(action == "move_relative")
        for widget in self.relative:
            widget.setEnabled(action == "move_relative")
        for widget, _ in self.gripper_values.values():
            widget.setEnabled(action in ("open_gripper", "close_gripper"))
        self.attach.setEnabled(action == "close_gripper")
        self.release.setEnabled(action == "open_gripper")

    def step(self):
        action = self.action.currentData()
        if action in ("move_joint", "move_cartesian") and not self.target.currentData():
            raise ValueError("Select a saved position")
        data = dict(
            name=self.name.text().strip(),
            action=action,
            position_id=self.target.currentData(),
            duration=self.duration.value(),
            relative_xyz=tuple(s.value() / 1000 for s in self.relative[:3]),
            relative_rpy=tuple(math.radians(s.value()) for s in self.relative[3:]),
            reference_frame=self.frame.currentText(),
            gripper_values={n: s.value() / factor for n, (s, factor) in self.gripper_values.items()}
            if action in ("open_gripper", "close_gripper")
            else {},
            attach_object=action == "close_gripper" and self.attach.isChecked(),
            release_object=action == "open_gripper" and self.release.isChecked(),
        )
        if self.original:
            data["id"] = self.original.id
        return TaskStep(**data)

    def validate_accept(self):
        try:
            self.step()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid step", str(exc))
            return
        self.accept()


class PositionPicker(QWidget):
    def __init__(self, chain, current, positions, preferred, parent=None):
        super().__init__(parent)
        self.chain, self.current, self.positions = chain, current, positions
        layout = QVBoxLayout(self)
        self.source = QComboBox()
        self.source.addItems(
            ["Current Cartesian pose", "Saved position", "Cartesian input", "Joint input"]
        )
        layout.addWidget(self.source)
        self.stack = QStackedWidget()
        current_label = label(
            "The current virtual end-effector pose is captured. You can also use a saved pose or edit joint/Cartesian values.",
            "subtitle",
            True,
        )
        self.stack.addWidget(current_label)
        self.saved = QComboBox()
        for p in positions.positions:
            self.saved.addItem(p.name, p.id)
        for i in range(self.saved.count()):
            if preferred.casefold() in self.saved.itemText(i).casefold():
                self.saved.setCurrentIndex(i)
                self.source.setCurrentIndex(1)
                break
        self.stack.addWidget(self.saved)
        self.cartesian = PoseEditor(chain, current)
        self.stack.addWidget(self.cartesian)
        self.joints = JointControls()
        self.joints.set_model(chain.model, current)
        self.stack.addWidget(self.joints)
        layout.addWidget(self.stack)
        self.source.currentIndexChanged.connect(self.stack.setCurrentIndex)
        self.stack.setCurrentIndex(self.source.currentIndex())

    def position(self, name):
        source = self.source.currentIndex()
        if source == 0:
            return self.positions.capture_cartesian(name, self.chain, self.current)
        if source == 1:
            if not self.saved.currentData():
                raise ValueError("No saved position is available")
            return self.positions.get(self.saved.currentData()).model_copy(deep=True)
        if source == 2:
            return SavedPosition(name=name, kind="cartesian", pose=self.cartesian.pose())
        return self.positions.capture_joint(name, self.chain, self.joints.values)


class PickPlaceWizard(QWizard):
    def __init__(
        self, chain, current, positions, settings, options, solver, on_generated, parent=None
    ):
        super().__init__(parent)
        self.resize(700, 740)
        self.setWindowTitle("Pick-and-Place Wizard · offline only")
        self.setWizardStyle(QWizard.WizardStyle.ModernStyle)
        self.chain, self.current, self.positions = chain, dict(current), positions
        self.settings, self.options, self.solver = settings, options, solver
        self.preview_callback = None
        self.on_generated = on_generated
        self.result_data = None
        self.cancel_event = threading.Event()
        self.runner = TaskRunner(self)
        self.runner.progress.connect(self.progress)
        self.runner.failed.connect(self.failed)
        self.runner.busy_changed.connect(self.busy_changed)
        page, body = self.page(
            "1 / Current robot", f"{chain.model.name} · {chain.base_link} → {chain.end_effector}"
        )
        self.name = QLineEdit("Pick and Place")
        body.addWidget(QLabel("Task name"))
        body.addWidget(self.name)
        body.addWidget(
            label(
                "The current confirmed arm/gripper mapping is used. Collision checking not available – preview only.",
                "warning",
                True,
            )
        )
        self.addPage(page)
        page, body = self.page(
            "2 / Pick position",
            "Capture the current pose, choose a taught position or edit Cartesian/joint values.",
        )
        self.pick = PositionPicker(chain, current, positions, "Pick")
        body.addWidget(self.pick)
        self.addPage(page)
        page, body = self.page(
            "3 / Place position",
            "Choose a different reachable location. Every generated waypoint will be solved and checked.",
        )
        self.place = PositionPicker(chain, current, positions, "Place")
        body.addWidget(self.place)
        self.addPage(page)
        page, body = self.page(
            "4 / Virtual gripper", "Values are URDF angles/metres, never LeRobot motor percentages."
        )
        self.grip = {}
        form = QFormLayout()
        for name in chain.gripper_joints:
            joint = chain.engine.joints[name]
            factor = 1 if joint.kind == "prismatic" else 180 / math.pi
            lo = joint.lower if joint.lower is not None else -math.pi
            hi = joint.upper if joint.upper is not None else math.pi
            opened = number(
                current[name] * factor,
                lo * factor,
                hi * factor,
                suffix=" m" if factor == 1 else " °",
            )
            closed = number(
                current[name] * factor,
                lo * factor,
                hi * factor,
                suffix=" m" if factor == 1 else " °",
            )
            form.addRow(name + " · open", opened)
            form.addRow(name + " · close", closed)
            self.grip[name] = (opened, closed, factor, lo, hi)
        body.addLayout(form)
        body.addWidget(button("Suggest URDF extrema · virtual preview only", self.suggest_gripper))
        self.confirm = QCheckBox(
            "I confirm these virtual open/close values; no hardware calibration is implied"
        )
        body.addWidget(self.confirm)
        self.dwell = number(0.25, 0, 60, suffix=" s")
        body.addWidget(QLabel("Optional dwell"))
        body.addWidget(self.dwell)
        self.addPage(page)
        page, body = self.page(
            "5 / Approach, lift and retreat",
            "Offsets are suggestions, not generally valid or collision-free paths.",
        )
        form = QFormLayout()
        self.frame = QComboBox()
        self.frame.addItems(chain.model.links)
        self.frame.setCurrentText(chain.base_link)
        form.addRow("Offset reference frame", self.frame)
        scale = max(sum(float(np.linalg.norm(j.xyz)) for j in chain.path), 0.2)
        suggested = scale * 0.025
        self.approach = number(suggested * 1000, 0, 1000, suffix=" mm")
        self.lift = number(suggested * 1.5 * 1000, 0, 1000, suffix=" mm")
        self.retreat = number(suggested * 1000, 0, 1000, suffix=" mm")
        self.directions = []
        for name, spin in [
            ("Pre-pick/pre-place offset", self.approach),
            ("Lift", self.lift),
            ("Retreat", self.retreat),
        ]:
            form.addRow(name, spin)
            row = QWidget()
            grid = QHBoxLayout(row)
            grid.setContentsMargins(0, 0, 0, 0)
            vector = []
            for axis, val in zip("XYZ", (0, 0, 1)):
                grid.addWidget(QLabel(axis))
                value = number(val, -1, 1, 3)
                grid.addWidget(value)
                vector.append(value)
            form.addRow("Direction", row)
            self.directions.append(vector)
        body.addLayout(form)
        self.addPage(page)
        page, body = self.page(
            "6 / Generate and validate",
            "A failed waypoint keeps the task uncreated; go back and adjust the inputs.",
        )
        self.generate_button = button(
            "Generate all positions and validate IK / trajectory", self.generate, True
        )
        body.addWidget(self.generate_button)
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        body.addWidget(self.output)
        self.addPage(page)
        page, body = self.page(
            "7 / Preview",
            "Preview in the shared 3D workspace, then Finish to retain the validated task in the editor.",
        )
        self.summary = label("No task generated yet.", "subtitle", True)
        body.addWidget(self.summary)
        body.addWidget(button("Preview in shared 3D viewer", self.preview))
        body.addWidget(
            label(
                "Cube attachment is a visual convention. Forces, contact, stability and collisions are not simulated.",
                "warning",
                True,
            )
        )
        self.addPage(page)
        self.accepted.connect(self.finish_generation)
        # Editing any source invalidates the complete validation, including while a worker runs.
        self.revision = 0
        for widget in self.findChildren(QDoubleSpinBox):
            widget.valueChanged.connect(self.invalidate)
        for widget in self.findChildren(QComboBox):
            widget.currentIndexChanged.connect(self.invalidate)
        for widget in self.findChildren(QLineEdit):
            widget.textEdited.connect(self.invalidate)
        for picker in (self.pick, self.place):
            picker.joints.configuration_changed.connect(self.invalidate)
        self.confirm.toggled.connect(self.invalidate)

    def invalidate(self, *_):
        self.revision += 1
        self.result_data = None
        self.summary.setText("Inputs changed. Generate and validate again.")
        if self.runner.busy:
            self.cancel_event.set()

    def page(self, title, subtitle):
        page = QWizardPage()
        page.setTitle(title)
        page.setSubTitle(subtitle)
        return page, QVBoxLayout(page)

    def suggest_gripper(self):
        for opened, closed, factor, lo, hi in self.grip.values():
            opened.setValue(hi * factor)
            closed.setValue(lo * factor)

    def generate(self):
        if self.runner.busy:
            return
        try:
            request = PickPlaceRequest(
                self.name.text().strip(),
                self.pick.position("Pick"),
                self.place.position("Place"),
                {n: v[0].value() / v[2] for n, v in self.grip.items()},
                {n: v[1].value() / v[2] for n, v in self.grip.items()},
                self.frame.currentText(),
                self.approach.value() / 1000,
                self.lift.value() / 1000,
                self.retreat.value() / 1000,
                *(tuple(s.value() for s in vector) for vector in self.directions),
                dwell=self.dwell.value(),
                gripper_confirmed=self.confirm.isChecked(),
            )
        except ValueError as exc:
            self.failed(str(exc))
            return
        self.cancel_event.clear()
        self.result_data = None
        self.running_revision = self.revision
        self.runner.run(
            lambda progress: PickPlaceGenerator(self.chain, self.options, self.solver).generate(
                request, self.current, self.positions, self.settings, self.cancel_event, progress
            ),
            self.generated,
        )

    def generated(self, result):
        if self.running_revision != self.revision:
            self.failed("Inputs changed; stale result discarded")
            return
        self.result_data = result
        positions, task, plan = result
        message = f"{len(positions)} positions · {len(task.steps)} steps · {plan.duration:.2f} s\nAll endpoints and joint-space segments kinematically validated.\nCollision checking not available – preview only."
        self.output.appendPlainText(message)
        self.summary.setText(message)

    def progress(self, text):
        self.output.appendPlainText(text)

    def failed(self, text):
        self.output.appendPlainText("ERROR: " + text)

    def busy_changed(self, busy):
        self.generate_button.setEnabled(not busy)
        for which in (
            QWizard.WizardButton.NextButton,
            QWizard.WizardButton.BackButton,
            QWizard.WizardButton.FinishButton,
        ):
            self.button(which).setEnabled(not busy)

    def validateCurrentPage(self):
        if self.currentId() in (5, 6) and self.result_data is None:
            self.failed("Generate a valid task before proceeding.")
            return False
        return not self.runner.busy

    def preview(self):
        if self.result_data and self.preview_callback:
            self.preview_callback(self.result_data[2])

    def finish_generation(self):
        if self.result_data:
            self.on_generated(self.result_data)

    def reject(self):
        if self.runner.busy:
            self.cancel_event.set()
            return
        super().reject()
