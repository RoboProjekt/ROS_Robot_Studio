"""Offline-only joint editing in SI units, derived entirely from the active URDF."""

import math

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QLabel,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.robots.urdf import RobotModel


class JointControls(QWidget):
    configuration_changed = Signal(dict)
    error = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.model: RobotModel | None = None
        self.values = {}
        self.controls = {}
        self.readouts = {}
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setMinimumHeight(206)
        self.scroll.setMaximumHeight(220)
        self.scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        layout.addWidget(self.scroll)

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
            spin.setSuffix(" " + unit)
            spin.setMinimumWidth(118)
            limit = QLabel(f"{lower * factor:.1f} … {upper * factor:.1f} {unit}")
            limit.setObjectName("subtitle")
            if joint.kind == "continuous":
                limit.setText("continuous · ±180° preview")
                spin.setToolTip(
                    "The URDF has no physical limit. The offline control uses a ±180° display interval."
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

    def set_configuration(self, values):
        resolved = self.model.configuration(values)
        self.values = {name: resolved[name] for name in self.controls}
        for name, (slider, spin, lower, upper, factor) in self.controls.items():
            value = resolved[name]
            for control in (slider, spin):
                control.blockSignals(True)
            slider.setValue(
                round((value - lower) / (upper - lower) * 10000) if upper > lower else 0
            )
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
        updated = dict(self.values, **{name: max(lower, min(upper, value))})
        try:
            self.set_configuration(updated)
        except ValueError as exc:
            self.set_configuration(self.values)
            self.error.emit(str(exc))
            return
        self.configuration_changed.emit(dict(self.values))

    def reset_defaults(self):
        if self.model:
            try:
                self.set_configuration(self.model.default_configuration())
            except ValueError as exc:
                self.error.emit(str(exc))
                return
            self.configuration_changed.emit(dict(self.values))
