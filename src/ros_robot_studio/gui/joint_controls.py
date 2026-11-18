"""Joint controls and measured feedback in the active model's SI units."""

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QLabel,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.backends.so101_usb import JOG_SPEED, USB_SPEED_RANGE, USB_TORQUE_RANGE
from ros_robot_studio.robots.urdf import RobotModel


class JointControls(QWidget):
    configuration_changed = Signal(dict)
    target_requested = Signal(str, float)
    error = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model: RobotModel | None = None
        self.values = {}
        self.controls = {}
        self.readouts = {}
        self.live = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setMinimumHeight(206)
        self.scroll.setMaximumHeight(220)
        self.scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        layout.addWidget(self.scroll)
        self.usb_motor_control = QCheckBox(
            "Arm is fixed and supported. Enable motor position control."
        )
        self.usb_motor_control.hide()
        layout.addWidget(self.usb_motor_control, alignment=Qt.AlignmentFlag.AlignLeft)
        self.usb_limits = QWidget()
        limits_grid = QGridLayout(self.usb_limits)
        limits_grid.setContentsMargins(0, 0, 0, 0)
        limits_grid.setColumnStretch(1, 1)
        self.usb_speed = self.add_limit_slider(
            limits_grid, 0, "USB Speed", USB_SPEED_RANGE, JOG_SPEED, " steps/s", "#74bcff", 6
        )
        self.usb_torque = self.add_limit_slider(
            limits_grid,
            1,
            "USB Max torque",
            USB_TORQUE_RANGE,
            USB_TORQUE_RANGE[1],
            " %",
            "#ffbb68",
            2,
        )
        self.usb_speed.setToolTip("SO-101 movement speed in motor encoder steps per second.")
        self.usb_torque.setToolTip("Requested torque ceiling. Each motor retains any lower limit.")
        self.usb_limits.hide()
        layout.addWidget(self.usb_limits)

    @staticmethod
    def add_limit_slider(layout, row, name, bounds, default, unit, color, radius):
        slider = QSlider(Qt.Orientation.Horizontal)
        slider.setRange(*bounds)
        slider.setValue(default)
        slider.setTracking(False)
        slider.setAccessibleName(name)
        slider.setStyleSheet(
            f"QSlider::sub-page:horizontal:enabled {{ background: {color}; border-radius: 2px; }}"
            f"QSlider::handle:horizontal:enabled {{ background: {color}; border-radius: {radius}px; }}"
            "QSlider::handle:horizontal:disabled { background: #647084; }"
        )
        title = QLabel(name)
        title.setBuddy(slider)
        value = QLabel(f"{default}{unit}")
        value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        slider.sliderMoved.connect(lambda number: value.setText(f"{number}{unit}"))
        slider.valueChanged.connect(lambda number: value.setText(f"{number}{unit}"))
        layout.addWidget(title, row, 0)
        layout.addWidget(slider, row, 1)
        layout.addWidget(value, row, 2)
        return slider

    def set_model(self, model: RobotModel, values=None):
        self.model = model
        body = QWidget()
        grid = QGridLayout(body)
        grid.setContentsMargins(0, 0, 5, 0)
        grid.setHorizontalSpacing(9)
        grid.setVerticalSpacing(5)
        self.controls, self.readouts = {}, {}
        row = 0
        # Topological order matches the visible physical chain, not XML declaration order.
        order, links = [], [model.root]
        for parent in links:
            for joint in model.joints:
                if joint.parent == parent:
                    order.append(joint)
                    links.append(joint.child)
        for joint in order:
            name = QLabel(joint.name)
            name.setToolTip(
                f"{joint.kind}\n{joint.parent} → {joint.child}\nAxis: {joint.axis}\nLimits (SI): {joint.lower} … {joint.upper}"
            )
            grid.addWidget(name, row, 0)
            if not joint.movable or joint.mimic:
                note = QLabel(f"coupled to {joint.mimic}" if joint.mimic else joint.kind)
                grid.addWidget(note, row, 1, 1, 3)
                self.readouts[joint.name] = note
                row += 1
                continue
            lower = joint.lower if joint.lower is not None else -math.pi
            upper = joint.upper if joint.upper is not None else math.pi
            factor = 1 if joint.kind == "prismatic" else 180 / math.pi
            unit = "m" if joint.kind == "prismatic" else "°"
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(0, 10000)
            slider.setMinimumWidth(90)
            slider.setEnabled(upper > lower)
            spin = QDoubleSpinBox()
            spin.setStyleSheet("QDoubleSpinBox { padding: 3px; }")
            spin.setDecimals(5)
            spin.setRange(lower * factor, upper * factor)
            spin.setSingleStep(0.001 if joint.kind == "prismatic" else 0.5)
            spin.setKeyboardTracking(not self.live)
            spin.editingFinished.connect(lambda editor=spin: editor.lineEdit().setModified(False))
            spin.setSuffix(" " + unit)
            spin.setMinimumWidth(118)
            limit = QLabel(f"{lower * factor:.1f} … {upper * factor:.1f} {unit}")
            limit.setObjectName("subtitle")
            if joint.kind == "continuous":
                spin.setRange(-1e9, 1e9)
                limit.setText("continuous · slider ±180°")
                spin.setToolTip(
                    "The URDF has no physical limit. Numeric values retain unwrapped turns; the slider spans ±180°."
                )
            grid.addWidget(slider, row, 1)
            grid.addWidget(spin, row, 2)
            grid.addWidget(limit, row, 3)
            self.controls[joint.name] = (slider, spin, lower, upper, factor)
            slider.valueChanged.connect(
                lambda value, name=joint.name: self._edit(name, slider_value=value)
            )
            spin.valueChanged.connect(
                lambda value, name=joint.name: self._edit(name, display_value=value)
            )
            row += 1
        grid.setColumnStretch(1, 1)
        self.scroll.setWidget(body)
        self.set_configuration(values if values else model.default_configuration())

    def set_configuration(self, values, *, measured=False):
        resolved = self.model.configuration(values, check_limits=not measured)
        self.values = {name: resolved[name] for name in self.controls}
        for name, (slider, spin, lower, upper, factor) in self.controls.items():
            value = resolved[name]
            for control in (slider, spin):
                control.blockSignals(True)
            if next(j for j in self.model.joints if j.name == name).kind != "continuous":
                spin.setRange(min(lower, value) * factor, max(upper, value) * factor)
            if not self.live or not slider.isSliderDown():
                slider.setValue(
                    round((value - lower) / (upper - lower) * 10000) if upper > lower else 0
                )
            if not self.live or not spin.lineEdit().isModified():
                spin.setValue(value * factor)
            for control in (slider, spin):
                control.blockSignals(False)
        for joint in self.model.joints:
            if joint.mimic:
                self.readouts[joint.name].setText(
                    f"coupled to {joint.mimic} · {resolved[joint.name]:.5f} SI"
                )

    def _edit(self, name, slider_value=None, display_value=None):
        _, _, lower, upper, factor = self.controls[name]
        value = (
            lower + slider_value / 10000 * (upper - lower)
            if slider_value is not None
            else display_value / factor
        )
        if self.model and next(j for j in self.model.joints if j.name == name).kind != "continuous":
            value = max(lower, min(upper, value))
        if self.live:
            self.target_requested.emit(name, value)
            return
        updated = dict(self.values, **{name: value})
        try:
            self.set_configuration(updated)
        except ValueError as exc:
            self.set_configuration(self.values)
            self.error.emit(str(exc))
            return
        self.configuration_changed.emit(dict(self.values))

    def reset_defaults(self):
        if self.model and not self.live:
            try:
                self.set_configuration(self.model.default_configuration())
            except ValueError as exc:
                self.error.emit(str(exc))
                return
            self.configuration_changed.emit(dict(self.values))

    def set_live_controls(self, live, enabled):
        mode_changed = self.live != live
        self.live = live
        for name, (slider, spin, lower, upper, _) in self.controls.items():
            editable = not live or name in enabled
            slider.setEnabled(editable and upper > lower)
            spin.setEnabled(editable)
            if mode_changed:
                spin.setKeyboardTracking(not live)
