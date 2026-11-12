"""Shared Sensor Configuration and dockable Camera & Vision panel."""

from __future__ import annotations

import json
import math
import time
from pathlib import Path
from uuid import uuid4

from PySide6.QtCore import QObject, QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen
from PySide6.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDialog,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.backends.isaac import IsaacSimInstallationManager, IsaacSimSceneManager
from ros_robot_studio.gui.camera_dialogs import (
    CalibrationDialog,
    CameraDialog,
    FormDialog,
    values,
    vector_fields,
)
from ros_robot_studio.gui.widgets import button, label
from ros_robot_studio.motion.positions import VisionPoseProvider
from ros_robot_studio.vision.detection import (
    ARUCO_DICTIONARIES,
    DETECTORS,
    DetectionModelManager,
    DetectorSettings,
    ObjectDetectionManager,
)
from ros_robot_studio.vision.geometry import Intrinsics, transform_matrix
from ros_robot_studio.vision.profiles import CalibrationRecord, CameraManager, WorkPlane
from ros_robot_studio.vision.results import VisionResultManager
from ros_robot_studio.vision.stream import CameraStream


def combo(items):
    field = QComboBox()
    for name, value in items:
        field.addItem(name, value)
    return field


def actions(layout, entries):
    row = QHBoxLayout()
    for name, callback in entries:
        row.addWidget(button(name, callback))
    row.addStretch()
    layout.addLayout(row)


class RGBView(QWidget):
    clicked = Signal(float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 220)
        self.image = QImage()
        self.frame = None
        self.detections = ()
        self.selected = -1
        self.reference = None
        self.area = QRectF()

    def set_frame(self, frame=None, detections=(), selected=-1, reference=None):
        if self.frame is not frame:
            self.frame = frame
            self.image = (
                QImage()
                if frame is None
                else QImage(
                    frame.rgb.data,
                    frame.width,
                    frame.height,
                    frame.rgb.strides[0],
                    QImage.Format.Format_RGB888,
                ).copy()
            )
        self.detections, self.selected, self.reference = detections, selected, reference
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor("#111b29"))
        if self.image.isNull():
            painter.setPen(QColor("#cad6e4"))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "Waiting for RGB frames")
            self.area = QRectF()
            return
        width, height = self.image.width(), self.image.height()
        scale = min(self.width() / width, self.height() / height)
        self.area = QRectF(
            (self.width() - width * scale) / 2,
            (self.height() - height * scale) / 2,
            width * scale,
            height * scale,
        )
        painter.drawImage(self.area, self.image)
        painter.translate(self.area.topLeft())
        for index, detection in enumerate(self.detections):
            item = detection.finding
            x1, y1, x2, y2 = item.box
            painter.setPen(QPen(QColor("#ffdc73" if index == self.selected else "#54f4cd"), 2))
            painter.drawRect(QRectF(x1 * scale, y1 * scale, (x2 - x1) * scale, (y2 - y1) * scale))
            score = "" if item.score is None else f" {item.score:.2f}"
            painter.drawText(QPointF(x1 * scale + 3, max(13, y1 * scale - 4)), item.label + score)
            painter.drawEllipse(QPointF(item.center[0] * scale, item.center[1] * scale), 3, 3)
            if item.corners:
                corners = [QPointF(x * scale, y * scale) for x, y in item.corners]
                for start, end in zip(corners, corners[1:] + corners[:1]):
                    painter.drawLine(start, end)
        if self.reference:
            painter.setPen(QPen(QColor("#ffdc73"), 2))
            x, y = (value * scale for value in self.reference)
            painter.drawLine(QPointF(x - 6, y), QPointF(x + 6, y))
            painter.drawLine(QPointF(x, y - 6), QPointF(x, y + 6))

    def mousePressEvent(self, event):
        if self.area.contains(event.position()) and not self.image.isNull():
            self.clicked.emit(
                (event.position().x() - self.area.x()) * self.image.width() / self.area.width(),
                (event.position().y() - self.area.y()) * self.image.height() / self.area.height(),
            )
        super().mousePressEvent(event)


class DetectorDialog(FormDialog):
    def __init__(self, parent, config):
        super().__init__("Detection settings", parent)
        self.fps = QSpinBox()
        self.fps.setRange(1, 30)
        self.fps.setValue(config.fps)
        self.threshold = QLineEdit(str(config.threshold))
        self.classes = QLineEdit(", ".join(config.classes))
        self.intervals = QLineEdit(json.dumps(config.hsv))
        self.area = QLineEdit(str(config.area))
        self.dictionary = combo((name, name) for name in ARUCO_DICTIONARIES)
        self.dictionary.setCurrentIndex(max(0, self.dictionary.findData(config.aruco_dictionary)))
        for name, field in (
            ("Inference FPS limit", self.fps),
            ("YOLO score threshold", self.threshold),
            ("Class names (comma-separated)", self.classes),
            ("HSV intervals (JSON)", self.intervals),
            ("Minimum contour area (px²)", self.area),
            ("ArUco dictionary", self.dictionary),
        ):
            self.form.addRow(name, field)
        self.message.setText("Each HSV interval is [H min, H max, S min, S max, V min, V max].")

    def data(self):
        return DetectorSettings(
            fps=self.fps.value(),
            threshold=float(self.threshold.text()),
            classes=[name.strip() for name in self.classes.text().split(",") if name.strip()],
            hsv=json.loads(self.intervals.text()),
            area=float(self.area.text()),
            aruco_dictionary=self.dictionary.currentData(),
        )


class DraftDialog(FormDialog):
    def __init__(self, parent, frame):
        super().__init__("Pick target draft", parent)
        self.confirm = combo(
            (("Set explicit tool orientation and offset", False), ("Confirm values", True))
        )
        rpy, self.rpy = vector_fields((0, 0, 0), low=-360, high=360)
        offset, self.offset = vector_fields((0, 0, 0), low=-2, high=2)
        self.form.addRow("Tool roll, pitch, yaw (°)", rpy)
        self.form.addRow("Grasp offset (m)", offset)
        self.form.addRow("Reference frame", QLabel(frame))
        self.form.addRow(self.confirm)

    def data(self):
        if not self.confirm.currentData():
            raise ValueError("Confirm the explicit orientation and grasp offset")
        return tuple(map(math.radians, values(self.rpy))), values(self.offset)


class VisionWorkspace(QObject):
    def __init__(self, main):
        super().__init__(main)
        self.main = main
        self.cameras = CameraManager()
        self.detector = ObjectDetectionManager()
        self.results = VisionResultManager()
        self.models = DetectionModelManager(main.settings)
        self.pose_provider = VisionPoseProvider({})
        self.stream = None
        self.latest = None
        self.displayed = None
        self.paused = False
        self.last_arrival = 0.0
        self.last_detection = 0.0
        self.epoch = 0
        self.manual = None
        self.estimate = None
        self.selection = 0
        self.pending_authoring = None
        self.requested_detector = None
        self.rate_time = 0.0
        self.rate_counts = (0, 0, 0)
        self.rates = (0.0, 0.0, 0.0)
        self.display_count = 0
        self.build_camera_config()
        self.build_view()
        self.refresh_models()
        self.load(main.project)
        main.backend_manager.profiles_changed.connect(self.refresh)
        main.isaac_processes.changed.connect(self.scene_finished)
        main.isaac_processes.error.connect(self.scene_error)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)

    def build_camera_config(self):
        body = self.main.sensor_panel.layout()
        body.addWidget(label("Cameras", "eyebrow"))
        self.camera_table = QTableWidget(0, 8)
        self.camera_table.setHorizontalHeaderLabels(
            [
                "Camera",
                "Backend",
                "Domain",
                "RGB topic",
                "CameraInfo",
                "Optical frame",
                "Calibration",
                "Stream",
            ]
        )
        self.camera_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.camera_table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.camera_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.ResizeToContents
        )
        self.camera_table.setMinimumHeight(100)
        self.camera_table.setMaximumHeight(180)
        self.camera_table.cellClicked.connect(self.select_row)
        body.addWidget(self.camera_table)
        actions(
            body,
            (
                ("Add Camera…", lambda: self.edit()),
                ("Configure…", lambda: self.edit(current=True)),
                ("Remove", self.remove),
                ("Add Camera to Isaac Sim…", lambda: self.edit(author=True)),
            ),
        )
        actions(
            body,
            (
                ("Connect camera", self.connect),
                ("Disconnect camera", self.stop),
                ("Calibrate…", self.calibrate),
                ("Open Camera View", self.open_view),
            ),
        )
        self.config_status = label("", "subtitle", True)
        body.addWidget(self.config_status)

    def build_view(self):
        self.panel = QWidget()
        self.panel.setObjectName("CameraVision")
        layout = QVBoxLayout(self.panel)
        self.camera = QComboBox()
        self.camera.activated.connect(lambda _: self.select(self.camera.currentData()))
        layout.addWidget(self.camera)
        self.status = label("", "subtitle", True)
        layout.addWidget(self.status)
        actions(
            layout,
            (
                ("Start Stream", self.start),
                ("Stop Stream", self.stop),
                ("Pause / resume display", self.pause_display),
                ("Snapshot…", self.snapshot),
            ),
        )
        self.view = RGBView(self.panel)
        self.view.clicked.connect(self.pick_pixel)
        layout.addWidget(self.view, 1)
        row = QFormLayout()
        self.detector_choice = combo((definition[0], key) for key, definition in DETECTORS.items())
        self.detector_choice.activated.connect(self.change_detector)
        row.addRow("Object detection", self.detector_choice)
        self.model = QComboBox()
        self.model.activated.connect(self.change_detector)
        row.addRow("Local ONNX model", self.model)
        layout.addLayout(row)
        actions(
            layout,
            (
                ("Add ONNX…", self.add_model),
                ("Remove ONNX", self.remove_model),
                ("Settings…", self.detector_settings),
                ("Start Detection", self.start_detector),
                ("Stop Detection", self.stop_detector),
            ),
        )
        self.detector_status = label("", "subtitle", True)
        layout.addWidget(self.detector_status)
        self.objects = QTableWidget(0, 5)
        self.objects.setHorizontalHeaderLabels(
            ["ID", "Class", "Score / quality", "Pixel X", "Pixel Y"]
        )
        self.objects.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.objects.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.objects.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.objects.setMaximumHeight(110)
        self.objects.cellClicked.connect(self.select_object)
        layout.addWidget(self.objects)
        self.reference = combo((("Bounding-box center", "center"), ("Bottom midpoint", "bottom")))
        self.reference.activated.connect(self.reference_changed)
        self.output_frame = combo(
            (
                ("Robot base", "base_frame"),
                ("World", "world_frame"),
                ("Camera optical frame", "optical_frame"),
            )
        )
        self.output_frame.activated.connect(self.update_estimate)
        form = QFormLayout()
        form.addRow("Reference pixel", self.reference)
        form.addRow("Output frame", self.output_frame)
        layout.addLayout(form)
        self.position = label(
            "Select a detection to estimate a point on the work plane.", "subtitle", True
        )
        layout.addWidget(self.position)
        actions(layout, (("Calibrate…", self.calibrate), ("Use as Target Draft…", self.draft)))

    def profile(self):
        return self.cameras.profiles.get(self.cameras.selected_id)

    def backend(self, camera=None):
        camera = camera or self.profile()
        return self.main.backend_manager.profiles.get(camera.backend_id) if camera else None

    def load(self, project):
        self.stop()
        self.cameras.load(project.cameras)
        self.refresh()

    def refresh(self):
        self.camera.blockSignals(True)
        self.camera.clear()
        self.camera_table.setRowCount(len(self.cameras.profiles))
        for row, camera in enumerate(self.cameras.profiles.values()):
            backend = self.backend(camera)
            self.camera.addItem(camera.name, camera.id)
            for column, value in enumerate(
                (
                    camera.name,
                    backend.name if backend else "Missing backend",
                    str(backend.domain_id) if backend else "—",
                    camera.rgb_topic,
                    camera.info_topic,
                    camera.optical_frame,
                    "Saved record" if camera.calibration else "Live CameraInfo + TF2",
                    "Selected" if camera.id == self.cameras.selected_id else "Idle",
                )
            ):
                self.camera_table.setItem(row, column, QTableWidgetItem(value))
            if camera.id == self.cameras.selected_id:
                self.camera_table.selectRow(row)
        self.camera.setCurrentIndex(self.camera.findData(self.cameras.selected_id))
        self.camera.blockSignals(False)
        self.show_status()

    def show_status(self, message=""):
        camera, backend = self.profile(), self.backend()
        prefix = (
            f"{backend.name} · {backend.id} · ROS domain {backend.domain_id}"
            if backend
            else "Camera has no configured backend"
            if camera
            else "Add an RGB camera in Sensor Configuration"
        )
        self.status.setText(prefix + ("\n" + message if message else ""))
        if camera:
            row = self.camera.findData(camera.id)
            item = self.camera_table.item(row, 7)
            if item:
                item.setText(message.split("\n")[0].split(" · ")[0] if message else "Idle")

    def select_row(self, row, _column):
        self.select(tuple(self.cameras.profiles)[row])

    def select(self, identity):
        if identity != self.cameras.selected_id:
            self.stop()
            self.cameras.select(identity)
            self.refresh()

    def open_view(self):
        self.main.tabs.setCurrentIndex(self.main.tabs.panels.index(self.panel))

    def edit(self, current=False, author=False):
        camera = self.profile() if current else None
        backend = camera.backend_id if camera else self.main.backend_manager.selected_id
        dialog = CameraDialog(
            self.main,
            self.main.backend_manager.profiles,
            backend,
            self.main.model.root,
            camera,
            author,
        )

        def refresh_topics():
            current = dialog.rgb.currentText()
            dialog.rgb.clear()
            session = self.main.backend_manager.sessions.get(dialog.backend.currentData())
            if session:
                for topic, encoding in self.cameras.discover(session.graph).items():
                    dialog.rgb.addItem(topic, encoding)
            dialog.rgb.setEditText(current)

        if not author:
            dialog.backend.currentIndexChanged.connect(refresh_topics)
            dialog.rgb.activated.connect(
                lambda index: dialog.encoding.setCurrentText(dialog.rgb.itemData(index) or "raw")
            )
            refresh_topics()
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        try:
            if author:
                self.author(dialog.result_data)
            else:
                self.save_camera(dialog.result_data)
        except (ValueError, OSError) as exc:
            self.config_status.setText(str(exc))

    def save_camera(self, camera):
        self.cameras.save(camera)
        self.stop()
        self.main.project.cameras = list(self.cameras.profiles.values())
        self.main.mark_dirty()
        self.refresh()

    def remove(self):
        camera = self.profile()
        if camera:
            self.stop()
            self.cameras.remove(camera.id)
            self.main.project.cameras = list(self.cameras.profiles.values())
            self.main.mark_dirty()
            self.refresh()

    def author(self, camera, destination=None):
        profile = self.backend(camera)
        if not profile or profile.backend_type != "isaac_sim" or self.pending_authoring:
            raise ValueError("Choose an Isaac backend and finish current scene work first")
        header = self.main.simulation_header
        local = header.local(profile.id)
        source = local.get("scene_path")
        if not source:
            raise ValueError("Choose the source USD in the Simulation File field first")
        if self.main.runner.busy or self.main.isaac_processes.processes:
            raise ValueError("Finish the current operation and stop the owned Isaac instance first")
        installations = IsaacSimInstallationManager().detect([local.get("installation_path")])
        if not installations:
            raise ValueError("Configure an Isaac Sim 5.1 installation in Backend Connections")
        installation = installations[0]
        if destination is None:
            destination, _ = QFileDialog.getSaveFileName(
                self.main,
                "Save derived RGB scene",
                "SO101_RGB_Vision_Demo.usd",
                "USD (*.usd *.usda *.usdc)",
            )
        if not destination:
            return
        destination = Path(destination).expanduser().resolve()
        if not destination.suffix:
            destination = destination.with_suffix(".usd")
        existing = self.cameras.profiles
        trial = CameraManager()
        trial.load(existing.values())
        trial.save(camera)
        project_id = self.main.project.id
        model, positions = self.main.model, self.main.independent_positions()
        folder = self.main.model_manager.root.parent / "simulations" / profile.id / uuid4().hex
        self.config_status.setText("Building an independent RGB scene in Isaac Sim…")
        self.pending_authoring = (profile.id, project_id, camera, destination)

        def prepared(request):
            try:
                if (
                    self.main.project.id != project_id
                    or self.main.backend_manager.profiles.get(profile.id) is not profile
                ):
                    raise ValueError("Project or Isaac backend changed during scene preparation")
                self.main.isaac_processes.start(
                    profile, installation, request, headless=True, build_only=True
                )
            except (ValueError, OSError) as exc:
                self.scene_error(profile.id, str(exc))

        self.main.runner.run(
            lambda _: IsaacSimSceneManager().prepare(
                profile, model, folder, positions, source, camera=camera, vision_output=destination
            ),
            prepared,
            lambda message: self.scene_error(profile.id, message),
        )

    def scene_error(self, identity, message):
        if self.pending_authoring and self.pending_authoring[0] == identity:
            self.pending_authoring = None
            self.config_status.setText(message)

    def scene_finished(self, identity):
        if not self.pending_authoring or self.pending_authoring[0] != identity:
            return
        processes = self.main.isaac_processes
        if identity in processes.processes:
            return
        _, project_id, camera, destination = self.pending_authoring
        self.pending_authoring = None
        report = processes.ready_reports.get(identity, {})
        if not destination.is_file() or report.get("scene") != str(destination):
            self.config_status.setText("Isaac did not validate the derived scene")
            return
        if project_id != self.main.project.id:
            self.config_status.setText(f"Saved {destination}; the active project changed")
            return
        item = next(
            (value for value in report.get("rgb_cameras", ()) if value["id"] == camera.id), None
        )
        if not item:
            self.config_status.setText("Isaac did not report the authored RGB camera")
            return
        if item.get("plane"):
            camera = camera.model_copy(update={"plane": WorkPlane.model_validate(item["plane"])})
        try:
            self.main.backend_manager.select_profile(identity)
            self.main.simulation_header.assign_scene(str(destination))
            self.save_camera(camera)
            self.config_status.setText(f"Saved {destination}\nCamera prim {item['prim']}")
        except (ValueError, OSError) as exc:
            self.config_status.setText(str(exc))

    def connect(self):
        backend = self.backend()
        if not backend:
            self.show_status("Select a camera with a ROS backend")
            return
        try:
            self.main.backend_workspace.connect(backend.id)
        except (ValueError, RuntimeError) as exc:
            self.show_status(str(exc))

    def start(self):
        camera = self.profile()
        session = self.main.backend_manager.sessions.get(camera.backend_id) if camera else None
        if (
            not session
            or session.cancelled
            or not session.transport
            or not session.transport.connected
        ):
            self.show_status("Connect the camera's ROS backend first")
            return
        if self.stream and self.stream.thread.is_alive():
            if self.stream.stop_requested.is_set():
                self.show_status("Wait for the previous receiver to stop")
            return
        self.clear_frame()
        self.stream = CameraStream(camera, session)
        self.stream.start()
        self.epoch, self.display_count = 0, 0
        self.rate_counts, self.rates = (0, 0, 0), (0.0, 0.0, 0.0)
        self.rate_time = time.monotonic()
        self.show_status("Waiting for RGB frames")
        self.timer.setInterval(max(15, round(1000 / camera.display_fps)))
        self.timer.start()

    def clear_frame(self):
        self.latest = self.displayed = None
        self.last_arrival = 0
        self.paused = False
        self.clear_results()

    def clear_results(self):
        self.results.clear()
        self.last_detection = 0
        self.manual = self.estimate = None
        self.pose_provider.estimates.clear()
        self.objects.setRowCount(0)
        self.position.setText("Select a detection to estimate a point on the work plane.")
        self.render()

    def stop(self):
        if self.stream:
            self.stream.stop()
        self.stop_detector()
        self.clear_frame()
        self.show_status("Stream stopped")
        if hasattr(self, "timer"):
            self.timer.stop()

    def pause_display(self):
        self.paused = not self.paused
        if not self.paused:
            self.displayed = None

    def snapshot(self):
        if self.view.image.isNull():
            self.show_status("Wait for a frame before saving a snapshot")
            return
        path, _ = QFileDialog.getSaveFileName(
            self.main, "Save RGB snapshot", "camera.png", "PNG (*.png)"
        )
        if path and not self.view.image.save(path, "PNG"):
            self.show_status("Snapshot could not be saved")

    def refresh_models(self):
        previous = self.model.currentData()
        self.model.clear()
        self.model.addItem("Select a model…", None)
        for path in self.models.paths():
            self.model.addItem(Path(path).name, path)
        self.model.setCurrentIndex(max(0, self.model.findData(previous)))

    def add_model(self):
        path, _ = QFileDialog.getOpenFileName(
            self.main, "Select YOLOv5n v7 ONNX", "", "ONNX (*.onnx)"
        )
        if path:
            try:
                selected = self.models.add(path)
            except (ValueError, OSError) as exc:
                self.detector_status.setText(str(exc))
            else:
                self.refresh_models()
                self.model.setCurrentIndex(self.model.findData(str(selected)))
                self.change_detector()

    def remove_model(self):
        if self.model.currentData():
            self.models.remove(self.model.currentData())
            self.refresh_models()
            self.change_detector()

    def settings_value(self):
        return DetectorSettings.model_validate(self.main.settings.get("vision_detection", {}))

    def detector_settings(self):
        dialog = DetectorDialog(self.main, self.settings_value())
        if dialog.exec() == QDialog.DialogCode.Accepted:
            self.main.settings.set("vision_detection", dialog.result_data.model_dump(mode="json"))
            self.change_detector()

    def change_detector(self, *_):
        active = self.detector.running or self.requested_detector is not None
        self.stop_detector()
        if active and self.detector_choice.currentData() != "disabled":
            self.start_detector()
        else:
            self.detector_status.setText(
                self.detector.availability(self.detector_choice.currentData())
            )

    def start_detector(self):
        camera = self.profile()
        if not camera or not self.stream or self.stream.stop_requested.is_set():
            self.detector_status.setText("Start a camera stream first")
            return
        model = self.model.currentData()
        self.requested_detector = (
            self.detector_choice.currentData(),
            Path(model) if model else None,
        )
        self.begin_detector()

    def begin_detector(self):
        if self.requested_detector is None or (
            self.detector.thread and self.detector.thread.is_alive()
        ):
            return
        detector_id, model = self.requested_detector
        self.requested_detector = None
        try:
            self.detector.start(detector_id, self.settings_value(), model, self.profile())
            self.detector_status.setText("Initializing detector…")
        except ValueError as exc:
            self.detector_status.setText(str(exc))

    def stop_detector(self):
        self.requested_detector = None
        self.detector.stop()
        self.clear_results()
        self.detector_status.setText("Detection stopped")

    def tick(self):
        stream = self.stream
        if not stream:
            return
        session = self.main.backend_manager.sessions.get(stream.camera.backend_id)
        if session is not stream.session or session.cancelled or not session.transport.connected:
            self.stop()
            self.show_status("ROS backend disconnected")
            return
        now = time.monotonic()
        frame, error, epoch, received, replaced = stream.poll()
        if self.epoch != epoch:
            self.clear_frame()
            self.epoch = epoch
        self.begin_detector()
        if frame is not None:
            self.latest = frame
            self.last_arrival = now
            self.detector.submit(frame)
        if not stream.thread.is_alive():
            self.stop()
            self.show_status(error or "ROS image receiver disconnected")
            return
        if self.latest is None or now - self.last_arrival > 1.5:
            if self.latest is not None:
                self.clear_frame()
            self.show_status(
                error or "Waiting for RGB frames (paused, absent or incompatible publisher)"
            )
            return
        result, failure, detected, inference_drops = self.detector.poll()
        if failure:
            self.detector_status.setText(failure)
        if result and not self.paused and result[0].metadata["epoch"] == self.epoch:
            self.results.update(result)
            self.last_detection = now
            self.populate_objects()
        if not self.paused and self.results.frame is not None and now - self.last_detection > 1.5:
            self.clear_results()
        annotated = self.results.frame if now - self.last_detection < 0.25 else None
        chosen = annotated if annotated is not None else self.latest
        if not self.paused and self.displayed is not chosen:
            self.displayed = chosen
            self.display_count += 1
            self.render()
        elapsed = now - self.rate_time
        if elapsed >= 1:
            counts = (received, self.display_count, detected)
            self.rates = tuple(
                max(0, end - start) / elapsed for start, end in zip(self.rate_counts, counts)
            )
            self.rate_counts, self.rate_time = counts, now
        metadata = self.latest.metadata
        age = max(0, (time.time_ns() - metadata["received_wall_ns"]) / 1e6)
        rate, display, detection = self.rates
        self.show_status(
            f"RGB {self.latest.width} × {self.latest.height} · {rate:.1f} FPS received · {display:.1f} FPS display"
            f"{' · display paused' if self.paused else ''}\n"
            f"DDS receive → UI {age:.0f} ms · drops {metadata['drops'] + replaced} · "
            f"ROS stamp {metadata['stamp_ns'] / 1e9:.6f} s"
        )
        if self.detector.running and not failure and self.results.frame is not None:
            self.detector_status.setText(
                f"{detection:.1f} detection FPS · {self.results.seconds * 1000:.1f} ms inference · "
                f"{inference_drops} input replacements · {len(self.detector.classes)} classes"
            )

    def render(self):
        same = self.displayed is self.results.frame
        self.view.set_frame(
            self.displayed,
            self.results.detections if same else (),
            self.selection,
            self.estimate.pixel if same and self.estimate else self.manual if same else None,
        )

    def populate_objects(self):
        objects = self.results.detections
        self.objects.setRowCount(len(objects))
        self.selection = min(self.selection, max(0, len(objects) - 1))
        for row, detection in enumerate(objects):
            found = detection.finding
            score = "—" if found.score is None else f"{found.score:.3f} ({found.score_kind})"
            for column, value in enumerate(
                (
                    detection.id,
                    found.label,
                    score,
                    f"{found.center[0]:.1f}",
                    f"{found.center[1]:.1f}",
                )
            ):
                self.objects.setItem(row, column, QTableWidgetItem(value))
        if objects:
            self.objects.selectRow(self.selection)
        self.manual = None
        self.update_estimate()

    def select_object(self, row, _column):
        self.selection = row
        self.reference_changed()

    def reference_changed(self, *_):
        self.manual = None
        self.update_estimate()

    def pick_pixel(self, x, y):
        if self.displayed is self.results.frame and self.results.detections:
            self.paused = True
            self.manual = (x, y)
            self.update_estimate()

    def update_estimate(self, *_):
        self.estimate = None
        self.pose_provider.estimates.clear()
        camera = self.profile()
        if not self.results.detections or not camera:
            self.position.setText("No 2D detection.")
        else:
            found = self.results.detections[self.selection]
            output = getattr(camera, self.output_frame.currentData())
            try:
                self.estimate = self.results.estimate(
                    camera, found.finding, output, self.reference.currentData(), self.manual
                )
            except (ValueError, ImportError) as exc:
                self.position.setText(
                    f"2D detection · {found.finding.label}\nPlanar position unavailable: {exc}"
                )
            else:
                p = self.estimate
                self.position.setText(
                    f"{p.method} · {p.frame_id} · pixel ({p.pixel[0]:.1f}, {p.pixel[1]:.1f})"
                    f"\nXYZ {p.position[0]:+.4f}, {p.position[1]:+.4f}, {p.position[2]:+.4f} m"
                    "\nCalibrated plane estimate · full 3D object pose unavailable"
                )
            try:
                base_estimate = self.results.estimate(
                    camera,
                    found.finding,
                    camera.base_frame,
                    self.reference.currentData(),
                    self.manual,
                )
            except (ValueError, ImportError):
                pass
            else:
                self.pose_provider.estimates[found.id] = base_estimate
        self.render()

    def calibration(self):
        camera, frame = self.profile(), self.latest
        if not camera or frame is None:
            return None
        meta = frame.metadata
        try:
            info = meta["camera_info"]
            Intrinsics.from_info(info, frame)
            translation, rotation = meta["transforms"][camera.world_frame]
            transform_matrix(translation, rotation)
            return CalibrationRecord(
                stamp_ns=meta["stamp_ns"],
                optical_frame=frame.frame_id,
                reference_frame=camera.world_frame,
                resolution=(frame.width, frame.height),
                intrinsics=tuple(info["k"]),
                distortion=tuple(info["d"]),
                distortion_model=info["model"],
                translation=translation,
                rotation_xyzw=rotation,
            )
        except (KeyError, TypeError, ValueError):
            return None

    def calibrate(self):
        camera = self.profile()
        if camera:
            dialog = CalibrationDialog(self.main, camera, self.calibration())
            if dialog.exec() == QDialog.DialogCode.Accepted:
                plane, record = dialog.result_data
                self.save_camera(
                    camera.model_copy(
                        update={"plane": plane, "calibration": record or camera.calibration}
                    )
                )

    def draft(self):
        chain = self.main.motion.chain
        if not chain or not self.pose_provider.estimates:
            self.position.setText(
                "A current calibrated base-frame estimate is required for a draft."
            )
            return
        target_id, estimate = next(iter(self.pose_provider.estimates.items()))
        if chain.base_link != estimate.frame_id:
            self.position.setText("The estimate frame differs from the configured IK base.")
            return
        dialog = DraftDialog(self.main, estimate.frame_id)
        was_paused = self.paused
        self.paused = True
        try:
            accepted = dialog.exec() == QDialog.DialogCode.Accepted
        finally:
            self.paused = was_paused
        if accepted and self.main.motion.chain is chain:
            try:
                rpy, offset = dialog.result_data
                pose = self.pose_provider.draft(target_id, chain.end_effector, rpy, offset)
            except ValueError as exc:
                self.position.setText(str(exc))
                return
            self.main.motion.pose_editor.set_pose(pose)
            self.main.robot_panel.detail_tabs.setCurrentWidget(self.main.motion.kin_panel)
            self.main.tabs.setCurrentIndex(0)

    def close_ready(self):
        self.stop()
        return not any(
            thread and thread.is_alive()
            for thread in (self.stream.thread if self.stream else None, self.detector.thread)
        )
