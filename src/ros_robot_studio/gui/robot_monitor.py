"""Shared state telemetry; model-derived names, explicit missing values and bounded plots."""

import time

import pyqtgraph as pg
from PySide6.QtCore import QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QHeaderView,
    QListWidget,
    QListWidgetItem,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.gui.plots import close_plot
from ros_robot_studio.gui.robot_view import RobotView
from ros_robot_studio.gui.widgets import label
from ros_robot_studio.ros_bridge.transport import SampleBuffer


class StateSignals(QObject):
    changed = Signal(object)


class RobotStatePanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        body = QVBoxLayout(self)
        self.overview = label("Waiting for robot state…", "subtitle", True)
        body.addWidget(self.overview)
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(
            ["JOINT / SI", "POSITION", "VELOCITY", "EFFORT", "STATUS"]
        )
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.verticalHeader().setDefaultSectionSize(25)
        self.table.setMinimumHeight(190)
        self.table.setMaximumHeight(245)
        robot_row = QHBoxLayout()
        robot_row.addWidget(self.table, 3)
        self.view = RobotView()
        self.view.setMinimumSize(300, 190)
        self.view.setMaximumHeight(245)
        robot_row.addWidget(self.view, 2)
        body.addLayout(robot_row)
        row = QHBoxLayout()
        self.fields = QListWidget()
        self.fields.setMaximumWidth(280)
        self.fields.itemChanged.connect(self.selection_changed)
        row.addWidget(self.fields)
        self.plot = pg.PlotWidget(background="#111b29")
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.addLegend()
        self.plot.setLabel("bottom", "Source time before latest state", units="s")
        self.plot.setLabel("left", "Selected quantities · SI units")
        row.addWidget(self.plot, 1)
        body.addLayout(row, 1)
        self.details = label(
            "Positions: rad/m · velocities: rad/s or m/s · efforts: Nm/N when supplied.",
            "subtitle",
            True,
        )
        body.addWidget(self.details)
        self.window_seconds = 30
        self.buffer = SampleBuffer()
        self.curves = {}
        self.state = None
        self.instance = None
        self.paused = False
        self.latest_time = 0.0
        self.source_key = None
        self.time_basis = None
        self.available = set()
        self.restore_fields = []

    def shutdown(self):
        self.fields.blockSignals(True)
        self.curves.clear()
        self.state = None
        close_plot(self.plot)

    def showEvent(self, event):
        super().showEvent(event)
        QTimer.singleShot(0, self.view.reset_camera)

    def reset(self):
        self.buffer.clear()
        self.latest_time = 0.0
        for curve in self.curves.values():
            curve.setData([], [])

    def selection_changed(self, *_):
        selected = [
            self.fields.item(i).text()
            for i in range(self.fields.count())
            if self.fields.item(i).checkState() == Qt.CheckState.Checked
        ][:8]
        for name in list(self.curves):
            if name not in selected:
                self.plot.removeItem(self.curves.pop(name))
        for index, name in enumerate(selected):
            if name not in self.curves:
                self.curves[name] = self.plot.plot(
                    pen=pg.mkPen(pg.intColor(index, 8), width=2), name=name
                )

    def receive(self, state, model):
        key = (state.instance, state.backend)
        if key != self.source_key:
            self.reset()
            self.source_key = key
            self.time_basis = None
            self.fields.clear()
            self.available = set()
            self.plot.clear()
            self.curves = {}
        if self.view.model is not model:
            self.view.set_model(model)
        if state.valid:
            self.view.set_joints({n: state.positions[n] for n in model.default_configuration()})
            self.view.set_end_effector(state.end_effector)
        self.state = state
        self.instance = state.instance
        fields = state.fields()
        # Show unavailable quantities as selectable empty curves, never invented zero measurements.
        names = (
            [
                f"{kind}.{joint}"
                for kind in ("position", "velocity", "effort", "acceleration")
                for joint in state.joint_names
            ]
            + [f"tool.{axis}" for axis in ("x", "y", "z", "roll", "pitch", "yaw")]
            + ["execution.progress"]
        )
        if not self.fields.count():
            self.fields.blockSignals(True)
            for index, name in enumerate(names):
                item = QListWidgetItem(name, self.fields)
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                selected = (
                    name in self.restore_fields
                    if self.restore_fields
                    else index < min(3, len(state.joint_names))
                )
                item.setCheckState(Qt.CheckState.Checked if selected else Qt.CheckState.Unchecked)
            self.fields.blockSignals(False)
            self.selection_changed()
        self.table.setRowCount(len(state.joint_names))
        by_name = {j.name: j for j in model.joints}
        for row, name in enumerate(state.joint_names):
            joint = by_name[name]
            angular = joint.kind != "prismatic"

            def value(values, unit):
                return (
                    f"{values[name]:+.5f} {unit}"
                    if values is not None and name in values
                    else "N/A"
                )

            values = [
                name,
                value(state.positions, "rad" if angular else "m"),
                value(state.velocities, "rad/s" if angular else "m/s"),
                value(state.efforts, "Nm" if angular else "N"),
                "OK" if state.valid and name in state.positions else "MISSING / INVALID",
            ]
            for col, text in enumerate(values):
                self.table.setItem(row, col, QTableWidgetItem(text))
        t = state.source_timestamp if state.source_timestamp is not None else state.timestamp
        if not self.paused and state.valid:
            basis = state.source_timestamp is not None
            if self.time_basis is not None and self.time_basis != basis:
                self.reset()
            self.time_basis = basis
            # A clock reset starts a new plot epoch instead of drawing time backwards.
            if t < self.latest_time:
                self.reset()
            if not self.buffer.samples or t > self.latest_time:
                self.latest_time = t
                self.buffer.append(t, fields)
        self.details.setText(
            f"{state.reference_frame} → {state.end_effector} · "
            + (
                "XYZ " + ", ".join(f"{v:.5f}" for v in state.pose[:3]) + " m"
                if state.pose
                else "Pose unavailable"
            )
            + "\n"
            + state.diagnostic
        )
        self.render(self.window_seconds)

    def render(self, window, stale=False):
        if self.plot.plotItem is None:
            return
        self.window_seconds = window
        if self.state:
            state = self.state
            samples = self.buffer.samples
            frequency = (
                (len(samples) - 1) / (samples[-1][0] - samples[0][0])
                if len(samples) > 1 and samples[-1][0] > samples[0][0]
                else 0
            )
            self.overview.setText(
                f"{state.name} · Model: {state.robot_id} · {state.revision[:12]}\nBackend: {state.backend} · Source: {state.source} · Execution: {state.execution_status}\nState: {'STALE / INVALID' if stale or not state.valid else 'VALID'} · {frequency:.1f} Hz · timestamp {state.source_timestamp if state.source_timestamp is not None else state.timestamp:.6f} · receive age {time.monotonic() - state.timestamp:.2f} s"
            )
        for name, curve in self.curves.items():
            curve.setData(*self.buffer.series(name, self.latest_time, window))
        self.plot.setXRange(-window, 0, padding=0)
