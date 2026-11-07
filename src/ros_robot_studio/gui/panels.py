"""Presentation panels. Project/runtime orchestration lives in MainWindow."""

from __future__ import annotations

import json
import time

import pyqtgraph as pg
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPlainTextEdit,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QTreeWidget,
    QTreeWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.gui.robot_view import RobotView
from ros_robot_studio.gui.widgets import button, card, label, page_header
from ros_robot_studio.ros_bridge.transport import SampleBuffer, numeric_fields


class RobotPanel(QWidget):
    changed = Signal()
    import_requested = Signal()
    example_requested = Signal()

    def __init__(self, registry, parent=None):
        super().__init__(parent)
        self.registry = registry
        self.loading = False
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 22, 24, 18)
        layout.addWidget(
            page_header(
                "Workspace / 01",
                "Build your robot.",
                "Define its structure, inspect the kinematic chain and prepare your interfaces.",
                "KINEMATIC PREVIEW",
            )
        )
        splitter = QSplitter()
        self.view = RobotView()
        preview, view_layout = card(
            "Robot workspace", "Interactive structure · drag to orbit · scroll to zoom"
        )
        view_layout.addWidget(self.view, 1)
        self.structure_note = label("No meshes · no physical hardware connected", "subtitle", True)
        view_layout.addWidget(self.structure_note)
        splitter.addWidget(preview)
        details = QWidget()
        side = QVBoxLayout(details)
        side.setContentsMargins(0, 0, 0, 0)
        identity, identity_layout = card("Project definition")
        form = QFormLayout()
        self.name = QLineEdit()
        self.description = QTextEdit()
        self.description.setMaximumHeight(65)
        self.profile = QComboBox()
        for plugin in registry.by_kind("robot"):
            self.profile.addItem(plugin.name, plugin.id)
        self.end_effector = QComboBox()
        self.end_effector.setEditable(True)
        for title, widget in [
            ("Project name", self.name),
            ("Description", self.description),
            ("Robot profile", self.profile),
            ("End effector", self.end_effector),
        ]:
            form.addRow(title, widget)
        identity_layout.addLayout(form)
        row = QHBoxLayout()
        row.addWidget(button("↥  Import URDF / Xacro", self.import_requested.emit, True))
        row.addWidget(button("Load example", self.example_requested.emit))
        identity_layout.addLayout(row)
        side.addWidget(identity)
        structure, structure_layout = card("Kinematic chain")
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Link / Joint", "Type", "Limits"])
        self.tree.header().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.tree.setMinimumHeight(170)
        structure_layout.addWidget(self.tree)
        side.addWidget(structure, 1)
        splitter.addWidget(details)
        splitter.setSizes([730, 500])
        layout.addWidget(splitter, 1)
        integrations, integration_layout = card(
            "Interfaces & capabilities",
            "Execution of hardware drivers, controllers and IK is planned. Selections are saved as configuration.",
        )
        row = QHBoxLayout()
        self.hardware, self.controller, self.ik = QComboBox(), QComboBox(), QComboBox()
        for title, widget, kind in [
            ("Hardware driver", self.hardware, "hardware"),
            ("Controller", self.controller, "controller"),
            ("IK solver", self.ik, "ik"),
        ]:
            column = QVBoxLayout()
            column.addWidget(label(title, "subtitle"))
            widget.addItem("None selected", None)
            for plugin in registry.by_kind(kind):
                widget.addItem(plugin.name + " · planned", plugin.id)
            column.addWidget(widget)
            row.addLayout(column, 1)
        integration_layout.addLayout(row)
        layout.addWidget(integrations)
        for widget in (self.name,):
            widget.textChanged.connect(self.emit_changed)
        self.description.textChanged.connect(self.emit_changed)
        self.end_effector.currentTextChanged.connect(self.emit_changed)
        for widget in (self.profile, self.hardware, self.controller, self.ik):
            widget.currentIndexChanged.connect(self.emit_changed)
        self.profile.currentIndexChanged.connect(self.profile_note)

    def emit_changed(self, *_):
        if not self.loading:
            self.changed.emit()

    def profile_note(self, *_):
        plugin = self.registry.plugins.get(self.profile.currentData())
        if plugin:
            self.structure_note.setText(plugin.description)

    def set_project(self, project, model):
        self.loading = True
        self.name.setText(project.name)
        self.description.setPlainText(project.description)
        for widget, value in [
            (self.profile, project.robot.profile),
            (self.hardware, project.robot.hardware),
            (self.controller, project.robot.controller),
            (self.ik, project.robot.ik_solver),
        ]:
            index = widget.findData(value)
            if index < 0:
                widget.addItem(f"{value} · manifest missing", value)
                index = widget.count() - 1
            widget.setCurrentIndex(index)
        self.set_model(model)
        self.end_effector.setCurrentText(project.robot.end_effector)
        self.loading = False
        self.profile_note()

    def set_model(self, model):
        self.view.set_model(model)
        self.tree.clear()
        self.end_effector.clear()
        self.end_effector.addItem("")
        self.end_effector.addItems(model.links)
        items = {model.root: QTreeWidgetItem(self.tree, [model.root, "root link", ""])}
        queue = [model.root]
        for parent in queue:
            for joint in model.joints:
                if joint.parent == parent:
                    limits = (
                        f"{joint.lower:g} … {joint.upper:g}" if joint.lower is not None else "—"
                    )
                    node = QTreeWidgetItem(items[parent], [joint.name, joint.kind, limits])
                    node.setToolTip(
                        0,
                        f"{joint.parent} → {joint.child}\nOrigin xyz: {joint.xyz}\nRPY: {joint.rpy}\nAxis: {joint.axis}",
                    )
                    items[joint.child] = QTreeWidgetItem(node, [joint.child, "link", ""])
                    queue.append(joint.child)
        self.tree.expandAll()
        if any(j.kind in ("floating", "planar") for j in model.joints):
            self.structure_note.setText(
                "Floating/planar joints shown at zero pose. Mesh rendering is planned."
            )

    def update_compatibility(self, distro):
        self.loading = True
        for widget in (self.profile, self.hardware, self.controller, self.ik):
            for index in range(widget.count()):
                plugin = self.registry.plugins.get(widget.itemData(index))
                if plugin:
                    state = (
                        "Not compatible / unverified"
                        if distro not in plugin.distributions
                        else "Planned, not implemented"
                        if plugin.status == "planned"
                        else "Available and installed"
                    )
                    widget.setItemText(index, plugin.name + " · " + state)
                    widget.setItemData(index, plugin.description, Qt.ItemDataRole.ToolTipRole)
        self.loading = False

    def collect(self, project):
        project.name = self.name.text().strip()
        project.description = self.description.toPlainText()
        project.robot.profile = self.profile.currentData()
        project.robot.end_effector = self.end_effector.currentText()
        project.robot.hardware = self.hardware.currentData()
        project.robot.controller = self.controller.currentData()
        project.robot.ik_solver = self.ik.currentData()


class SensorPanel(QWidget):
    add_requested = Signal()
    edit_requested = Signal(int)
    remove_requested = Signal(int)
    monitor_requested = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 22, 24, 18)
        layout.addWidget(
            page_header(
                "Workspace / 02",
                "Connect the senses.",
                "Manage data sources, reference frames and measurement thresholds.",
                "PROJECT CONFIGURATION",
            )
        )
        row = QHBoxLayout()
        row.addWidget(button("＋  Add sensor", self.add_requested.emit, True))
        row.addWidget(
            button("Edit sensor", lambda: self.edit_requested.emit(self.table.currentRow()))
        )
        row.addWidget(button("Remove", lambda: self.remove_requested.emit(self.table.currentRow())))
        row.addStretch()
        row.addWidget(button("View topic live →", self.monitor_selected))
        layout.addLayout(row)
        frame, body = card(
            "Sensor sources",
            "Select a sensor to inspect its frame transform. Double-click to configure.",
        )
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels(
            ["NAME", "TYPE", "TOPIC", "PARENT FRAME", "TF", "ENABLED"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.cellDoubleClicked.connect(lambda row, _: self.edit_requested.emit(row))
        self.table.itemSelectionChanged.connect(self.show_details)
        body.addWidget(self.table)
        layout.addWidget(frame, 1)
        offsets, body = card(
            "Frame transform",
            "Translation in metres · rotation in radians · offsets are stored, not broadcast as TF",
        )
        self.details = label("Add or select a sensor to inspect its transform.", "", True)
        body.addWidget(self.details)
        layout.addWidget(offsets)
        layout.addWidget(
            label(
                "Sensor thresholds support observation and warnings. They are not certified safety functions and must never replace protective hardware or a safety controller.",
                "warning",
                True,
            )
        )
        layout.addWidget(
            label(
                "PLANNED   Camera calibration · Hand-eye calibration · YOLO · Tracking · AI integrations",
                "subtitle",
                True,
            )
        )
        self.sensors = []

    def set_sensors(self, sensors):
        self.sensors = sensors
        self.table.setRowCount(len(sensors))
        for row, sensor in enumerate(sensors):
            for column, value in enumerate(
                [
                    sensor.name,
                    sensor.kind,
                    sensor.topic,
                    sensor.parent_frame,
                    sensor.transform.kind,
                    "Yes" if sensor.enabled else "No",
                ]
            ):
                self.table.setItem(row, column, QTableWidgetItem(value))
        if sensors:
            self.table.selectRow(0)
        else:
            self.details.setText("No sensors configured.")

    def show_details(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.sensors):
            sensor = self.sensors[row]
            xyz, rpy = sensor.transform.translation, sensor.transform.rotation
            self.details.setText(
                f"{sensor.name}  /  {sensor.parent_frame}\n\nX  {xyz[0]:.4f}    Y  {xyz[1]:.4f}    Z  {xyz[2]:.4f} m\nRoll  {rpy[0]:.4f}    Pitch  {rpy[1]:.4f}    Yaw  {rpy[2]:.4f} rad\n\n{sensor.quantity} · {sensor.unit or 'unitless'} · configured {sensor.sample_rate:g} Hz · {sensor.transform.kind} transform\nLimits: {sensor.lower_limit} … {sensor.upper_limit} · warning: {sensor.warning}"
            )

    def monitor_selected(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.sensors):
            self.monitor_requested.emit(self.sensors[row].topic)


class MonitoringPanel(QWidget):
    topic_selected = Signal(str, str)
    refresh_requested = Signal()
    changed = Signal()
    COLORS = [
        "#55daba",
        "#75aaff",
        "#efba6d",
        "#cf8cf0",
        "#f18da6",
        "#b4df7b",
        "#6cd7ec",
        "#dcdfe5",
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.topic = ""
        self.fields = {}
        self.curves = {}
        self.buffer = SampleBuffer()
        self.started = time.monotonic()
        self.frozen_at = None
        self.last_received = None
        self.restore_fields = ["data"]
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 22, 24, 18)
        layout.addWidget(
            page_header(
                "Workspace / 03",
                "Signals, in real time.",
                "Explore the ROS graph and follow the values that matter.",
                "LIVE TELEMETRY",
            )
        )
        self.mode = label("OFFLINE DEMO · All values below are simulated.", "warning")
        layout.addWidget(self.mode)
        splitter = QSplitter()
        explorer, explorer_body = card("ROS graph explorer")
        row = QHBoxLayout()
        self.search = QLineEdit()
        self.search.setPlaceholderText("Search topics or nodes…")
        self.search.textChanged.connect(self.filter_graph)
        row.addWidget(self.search)
        row.addWidget(button("↻", self.refresh_requested.emit))
        explorer_body.addLayout(row)
        self.topics = QTreeWidget()
        self.topics.setHeaderLabels(["TOPIC", "TYPE"])
        self.topics.header().setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        self.topics.currentItemChanged.connect(self.select_topic)
        explorer_body.addWidget(self.topics, 2)
        explorer_body.addWidget(label("NODES", "eyebrow"))
        self.nodes = QListWidget()
        explorer_body.addWidget(self.nodes, 1)
        self.topic_details = label(
            "Publisher/subscriber details appear on selection.", "subtitle", True
        )
        explorer_body.addWidget(self.topic_details)
        splitter.addWidget(explorer)
        right = QWidget()
        body = QVBoxLayout(right)
        body.setContentsMargins(0, 0, 0, 0)
        chart, chart_body = card(
            "Live plot", "Select numeric fields below · up to 8 curves · bounded to 6,000 samples"
        )
        controls = QHBoxLayout()
        self.pause = button("Ⅱ  Pause", self.toggle_pause)
        controls.addWidget(self.pause)
        controls.addWidget(button("Reset", self.reset))
        controls.addStretch()
        controls.addWidget(label("Window", "subtitle"))
        self.window = QSpinBox()
        self.window.setRange(5, 300)
        self.window.setValue(30)
        self.window.setSuffix(" s")
        self.window.valueChanged.connect(self.changed.emit)
        controls.addWidget(self.window)
        chart_body.addLayout(controls)
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget(background="#111b29")
        self.plot.showGrid(x=True, y=True, alpha=0.15)
        self.plot.setLabel("bottom", "Time before now", units="s")
        self.plot.setLabel("left", "Topic value")
        self.plot.addLegend(offset=(15, 15))
        self.plot.setMinimumHeight(235)
        chart_body.addWidget(self.plot, 1)
        body.addWidget(chart, 3)
        data, data_body = card("Message inspector")
        data_row = QHBoxLayout()
        self.field_list = QListWidget()
        self.field_list.setMaximumWidth(240)
        self.field_list.itemChanged.connect(self.field_changed)
        self.message_text = QPlainTextEdit("Waiting for a message…")
        self.message_text.setReadOnly(True)
        self.message_text.setMaximumBlockCount(200)
        data_row.addWidget(self.field_list, 1)
        data_row.addWidget(self.message_text, 2)
        data_body.addLayout(data_row)
        body.addWidget(data, 2)
        splitter.addWidget(right)
        splitter.setSizes([350, 900])
        layout.addWidget(splitter, 1)
        diagnostics, diagnostics_body = card("Diagnostics")
        row = QHBoxLayout()
        self.connection = label("Offline demo", "subtitle", True)
        self.container = label("Container: none", "subtitle", True)
        self.last_message = label("Last message: —", "subtitle", True)
        self.errors = label("No transport errors", "subtitle", True)
        for widget in (self.connection, self.container, self.last_message, self.errors):
            row.addWidget(widget, 1)
        diagnostics_body.addLayout(row)
        layout.addWidget(diagnostics)

    def set_graph(self, graph):
        if "topic_info" in graph:
            if graph["topic_info"] == self.topic:
                details = getattr(self, "_topic_info", {})
                details.update(
                    {key: graph[key] for key in ("publishers", "subscribers") if key in graph}
                )
                self._topic_info = details
                self.topic_details.setText(
                    "\n".join(
                        f"{key.title()}: {', '.join(details.get(key, [])) or 'unavailable'}"
                        for key in ("publishers", "subscribers")
                    )
                )
            return
        if "topics" in graph:
            previous = self.topic
            pairs = list(zip(graph["topics"], graph.get("types", [])))
            existing = [
                (self.topics.topLevelItem(i).text(0), self.topics.topLevelItem(i).text(1))
                for i in range(self.topics.topLevelItemCount())
            ]
            if pairs != existing:
                self.topics.blockSignals(True)
                self.topics.clear()
                selected = None
                for topic, message_type in pairs:
                    item = QTreeWidgetItem(self.topics, [topic, message_type])
                    if topic == previous:
                        selected = item
                self.topics.blockSignals(False)
                if selected:
                    self.topics.setCurrentItem(selected)
                elif self.topics.topLevelItemCount():
                    self.topics.setCurrentItem(self.topics.topLevelItem(0))
        if "nodes" in graph:
            self.nodes.clear()
            self.nodes.addItems(graph["nodes"])
        self.filter_graph()

    def filter_graph(self):
        query = self.search.text().lower()
        for i in range(self.topics.topLevelItemCount()):
            item = self.topics.topLevelItem(i)
            item.setHidden(query not in (item.text(0) + item.text(1)).lower())
        for i in range(self.nodes.count()):
            item = self.nodes.item(i)
            item.setHidden(query not in item.text().lower())

    def select_topic(self, current, previous):
        if not current:
            return
        if current.text(0) != self.topic:
            self.topic = current.text(0)
            self.reset()
            self.field_list.clear()
            self._topic_info = {}
            self.topic_details.setText("Querying publisher / subscriber information…")
        self.topic_selected.emit(current.text(0), current.text(1))
        self.changed.emit()

    def choose_topic(self, topic):
        for i in range(self.topics.topLevelItemCount()):
            if self.topics.topLevelItem(i).text(0) == topic:
                self.topics.setCurrentItem(self.topics.topLevelItem(i))
                return True
        return False

    def reset(self):
        self.buffer.clear()
        self.last_received = None
        self.started = time.monotonic()
        self.frozen_at = None
        self.pause.setText("Ⅱ  Pause")
        self.plot.clear()
        self.curves.clear()
        self.field_list.clear()
        self.fields = {}
        self.message_text.setPlainText("Waiting for a message…")

    def toggle_pause(self):
        self.frozen_at = None if self.frozen_at is not None else time.monotonic()
        self.pause.setText("▶  Resume" if self.frozen_at is not None else "Ⅱ  Pause")

    def receive(self, topic, message):
        if topic != self.topic:
            return
        now = time.monotonic()
        self.last_received = now
        fields = numeric_fields(message)
        if self.frozen_at is None:
            self.buffer.append(now, fields)
            self.fields = fields
            self.message_text.setPlainText(
                json.dumps(message, indent=2, ensure_ascii=False)[:18000]
            )
        if not self.field_list.count() and fields:
            self.field_list.blockSignals(True)
            for index, name in enumerate(fields):
                item = QListWidgetItem(name, self.field_list)
                item.setFlags(item.flags() | Qt.ItemFlag.ItemIsUserCheckable)
                checked = name in self.restore_fields or (
                    not any(f in fields for f in self.restore_fields) and index < 3
                )
                item.setCheckState(Qt.CheckState.Checked if checked else Qt.CheckState.Unchecked)
            self.field_list.blockSignals(False)
            self.field_changed(notify=False)

    def selected_fields(self):
        return [
            self.field_list.item(i).text()
            for i in range(self.field_list.count())
            if self.field_list.item(i).checkState() == Qt.CheckState.Checked
        ][:8]

    def field_changed(self, *_, notify=True):
        selected = self.selected_fields()
        self.restore_fields = selected
        for name in list(self.curves):
            if name not in selected:
                self.plot.removeItem(self.curves.pop(name))
        for index, name in enumerate(selected):
            if name not in self.curves:
                self.curves[name] = self.plot.plot(
                    pen=pg.mkPen(self.COLORS[index % 8], width=2), name=name
                )
        if notify:
            self.changed.emit()

    def render(self):
        now = self.frozen_at if self.frozen_at is not None else time.monotonic()
        for name, curve in self.curves.items():
            x, y = self.buffer.series(name, now, self.window.value())
            curve.setData(x, y)
        self.plot.setXRange(-self.window.value(), 0, padding=0)
        if self.last_received:
            self.last_message.setText(
                f"Last message: {time.monotonic() - self.last_received:.1f}s ago\n{len(self.buffer.samples)} buffered samples"
            )

    def restore_monitoring(self, monitoring):
        self.window.setValue(monitoring.window_seconds)
        self.restore_fields = list(monitoring.fields)
        self.reset()
        self.choose_topic(monitoring.topic)

    def clear_graph(self):
        self.topics.clear()
        self.nodes.clear()
        self.topic = ""
        self.field_list.clear()
        self.reset()


class RuntimePanel(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 22, 24, 18)
        layout.addWidget(
            page_header(
                "Workspace / 04",
                "Your runtime, under control.",
                "Isolated ROS environments. A single workspace. No system-wide ROS required.",
                "DOCKER ENGINE",
            )
        )
        row = QHBoxLayout()
        project, body = card("Current project")
        self.project_info = label("", "", True)
        body.addWidget(self.project_info)
        self.compatibility = button("Check compatibility")
        self.set_default = button("Save selected ROS as project default")
        self.export = button("Export project…")
        body.addWidget(self.compatibility)
        body.addWidget(self.set_default)
        body.addWidget(self.export)
        row.addWidget(project, 1)
        runtime, body = card("Runtime operations")
        self.docker_status = label("Checking Docker…", "", True)
        self.container_status = label("No container started", "subtitle", True)
        body.addWidget(self.docker_status)
        body.addWidget(self.container_status)
        controls = QHBoxLayout()
        self.start, self.stop, self.restart = (
            button("▶  Start ROS demo", primary=True),
            button("■  Stop"),
            button("↻  Restart"),
        )
        for control in (self.start, self.stop, self.restart):
            controls.addWidget(control)
        body.addLayout(controls)
        more = QHBoxLayout()
        self.refresh, self.logs, self.verify = (
            button("Refresh status"),
            button("Fetch logs"),
            button("Verify ROS demo"),
        )
        for control in (self.refresh, self.logs, self.verify):
            more.addWidget(control)
        body.addLayout(more)
        row.addWidget(runtime, 2)
        layout.addLayout(row)
        images, body = card(
            "ROS distribution library",
            "Official bases verified upstream · runtime builds use live amd64 digests · each distribution is isolated",
        )
        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(
            ["DISTRIBUTION", "UBUNTU", "BASE IMAGE", "SUPPORT", "LOCAL STATUS"]
        )
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.table.setMaximumHeight(170)
        body.addWidget(self.table)
        controls = QHBoxLayout()
        self.install = button("＋  Install selected runtime", primary=True)
        self.remove = button("Uninstall selected runtime…")
        self.setup = button("Open setup assistant")
        controls.addWidget(self.install)
        controls.addWidget(self.remove)
        controls.addStretch()
        controls.addWidget(self.setup)
        body.addLayout(controls)
        layout.addWidget(images)
        logs, body = card(
            "Runtime activity", "Build progress, container output and actionable errors"
        )
        self.output = QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.setMaximumBlockCount(1000)
        body.addWidget(self.output)
        layout.addWidget(logs, 1)
        self.progress = label("Ready", "subtitle")
        layout.addWidget(self.progress)
