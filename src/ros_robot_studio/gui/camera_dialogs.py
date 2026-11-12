"""Camera placement and calibrated work-plane dialogs."""

from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.gui.widgets import button
from ros_robot_studio.vision.geometry import usd_rpy
from ros_robot_studio.vision.profiles import CameraMount, CameraProfile, WorkPlane


def vector_fields(values, *, low=-100, high=100, digits=4):
    row = QHBoxLayout()
    inputs = []
    for axis, value in zip(("X", "Y", "Z"), values):
        field = QDoubleSpinBox()
        field.setRange(low, high)
        field.setDecimals(digits)
        field.setValue(value)
        row.addWidget(QLabel(axis))
        row.addWidget(field)
        inputs.append(field)
    body = QWidget()
    body.setLayout(row)
    row.setContentsMargins(0, 0, 0, 0)
    return body, inputs


def values(fields):
    return tuple(field.value() for field in fields)


class FormDialog(QDialog):
    def __init__(self, title, parent):
        super().__init__(parent)
        self.setWindowTitle(title)
        self.resize(640, 460)
        layout = QVBoxLayout(self)
        self.form = QFormLayout()
        layout.addLayout(self.form)
        self.message = QLabel()
        self.message.setWordWrap(True)
        layout.addWidget(self.message)
        self.buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        self.buttons.accepted.connect(self.validate_input)
        self.buttons.rejected.connect(self.reject)
        layout.addWidget(self.buttons)

    def validate_input(self):
        try:
            self.result_data = self.data()
        except (ValueError, ImportError) as exc:
            self.message.setText(str(exc))
        else:
            self.accept()


class CameraDialog(FormDialog):
    def __init__(self, parent, profiles, backend, model_root, original=None, author=False):
        super().__init__("Add RGB Camera to Isaac Sim" if author else "RGB Camera", parent)
        self.original = original
        self.author = author
        self.name = QLineEdit(original.name if original else "SO-101 Overhead Camera")
        self.backend = QComboBox()
        for profile in profiles.values():
            if not author or profile.backend_type == "isaac_sim":
                self.backend.addItem(f"{profile.name} · Domain {profile.domain_id}", profile.id)
        self.backend.setCurrentIndex(max(0, self.backend.findData(backend)))
        self.rgb = QComboBox()
        self.rgb.setEditable(True)
        self.rgb.addItem(original.rgb_topic if original else "/so101/camera/image_raw")
        self.info = QLineEdit(original.info_topic if original else "/so101/camera/camera_info")
        self.encoding = QComboBox()
        self.encoding.addItems(["raw", "compressed"])
        self.encoding.setCurrentText(original.encoding if original else "raw")
        self.link = QLineEdit(original.camera_frame if original else "so101_camera_link")
        self.optical = QLineEdit(
            original.optical_frame if original else "so101_camera_optical_frame"
        )
        self.world = QLineEdit(original.world_frame if original else "world")
        self.base = QLineEdit(original.base_frame if original else model_root)
        self.size = QComboBox()
        for width, height in ((320, 240), (640, 480), (1280, 720), (1920, 1080)):
            self.size.addItem(f"{width} × {height}", (width, height))
        resolution = (original.width, original.height) if original else (640, 480)
        if self.size.findData(resolution) < 0:
            self.size.addItem(f"{resolution[0]} × {resolution[1]}", resolution)
        self.size.setCurrentIndex(self.size.findData(resolution))
        self.fps = QSpinBox()
        self.fps.setRange(1, 60)
        self.fps.setValue(original.fps if original else 20)
        self.display = QSpinBox()
        self.display.setRange(1, 60)
        self.display.setValue(original.display_fps if original else 20)
        for label, field in (
            ("Camera name", self.name),
            ("ROS backend", self.backend),
            ("RGB topic", self.rgb),
            ("CameraInfo topic", self.info),
            ("Image transport", self.encoding),
            ("Camera link", self.link),
            ("ROS optical frame", self.optical),
            ("World frame", self.world),
            ("Robot base frame", self.base),
            ("Render resolution", self.size),
            ("Render FPS (nominal)", self.fps),
            ("Maximum display FPS", self.display),
        ):
            self.form.addRow(label, field)
        self.mount = None
        if author or (original and original.mount):
            self.mount = QComboBox()
            self.mount.addItems(
                ["World fixed", "Robot base fixed (prepared)", "End-effector (prepared)"]
            )
            previous = original.mount if original else None
            field, self.xyz = vector_fields(previous.position if previous else (0.23, -0.18, 0.68))
            orientation, self.rpy = vector_fields(
                previous.usd_rpy_deg if previous else (0, 0, 0),
                low=-360,
                high=360,
                digits=5,
            )
            target, self.target = vector_fields((0.23, -0.18, 0))
            self.fov = QDoubleSpinBox()
            self.fov.setRange(6, 159)
            self.fov.setValue(previous.horizontal_fov if previous else 70)
            self.demo = QCheckBox("Add red test cube to the Studio mounting table")
            self.demo.setChecked(previous.demo_cube if previous else True)
            self.form.addRow("Mounting", self.mount)
            self.form.addRow("Position in world (m)", field)
            self.form.addRow("USD roll, pitch, yaw (°)", orientation)
            self.form.addRow("Horizontal FOV (°)", self.fov)
            self.form.addRow("Look at world point (m)", target)
            self.form.addRow(
                button("Apply look-at", self.look_at), button("Preview pose", self.preview)
            )
            self.form.addRow(self.demo)
            self.message.setText(
                "Placement proposal. CameraInfo and TF are verified from the running scene."
            )
        if author:
            self.encoding.setEnabled(False)
            self.buttons.button(QDialogButtonBox.StandardButton.Save).setText("Create camera…")

    def look_at(self):
        try:
            rpy = usd_rpy(values(self.xyz), values(self.target))
        except (ValueError, ImportError) as exc:
            self.message.setText(str(exc))
            return
        for spin, angle in zip(self.rpy, rpy):
            spin.setValue(angle)
        self.preview()

    def preview(self):
        import numpy as np

        from ros_robot_studio.robots.urdf import origin_transform

        matrix = origin_transform(values(self.xyz), np.radians(values(self.rpy)))
        forward = -matrix[:3, 2]
        self.message.setText(
            "Camera points along world direction " + ", ".join(f"{value:+.3f}" for value in forward)
        )

    def data(self):
        if not self.backend.currentData():
            raise ValueError("Add or choose a ROS backend profile first")
        mount = None
        if self.mount:
            if self.mount.currentIndex():
                raise ValueError("This scene authoring supports the world-fixed mount")
            mount = CameraMount(
                reference_frame=self.world.text().strip(),
                position=values(self.xyz),
                usd_rpy_deg=values(self.rpy),
                horizontal_fov=self.fov.value(),
                demo_cube=self.demo.isChecked(),
            )
        width, height = self.size.currentData()
        data = dict(
            name=self.name.text().strip(),
            backend_id=self.backend.currentData(),
            rgb_topic=self.rgb.currentText().strip(),
            info_topic=self.info.text().strip(),
            camera_frame=self.link.text().strip(),
            optical_frame=self.optical.text().strip(),
            world_frame=self.world.text().strip(),
            base_frame=self.base.text().strip(),
            encoding="raw" if self.author else self.encoding.currentText(),
            width=width,
            height=height,
            fps=self.fps.value(),
            display_fps=self.display.value(),
            mount=mount,
        )
        if self.original:
            old = self.original.model_dump()
            change = any(
                data[key] != old[key] for key in data if key not in ("name", "display_fps")
            )
            data.update(
                id=self.original.id,
                plane=self.original.plane,
                calibration=None if change else self.original.calibration,
            )
        return CameraProfile(**data)


class CalibrationDialog(FormDialog):
    def __init__(self, parent, camera, record):
        super().__init__("Camera Calibration · Work Plane", parent)
        self.record = record
        plane = camera.plane
        self.name = QLineEdit(plane.name if plane else "Measured work plane")
        self.frame = QLineEdit(plane.frame if plane else camera.world_frame)
        point, self.point = vector_fields(plane.point if plane else (0, 0, 0))
        normal, self.normal = vector_fields(plane.normal if plane else (0, 0, 1), low=-1, high=1)
        self.confirmed = QCheckBox(
            "I verified this plane against the scene or a physical measurement"
        )
        self.confirmed.setChecked(plane is not None)
        self.bounds = QLineEdit(
            " ".join(str(v) for limit in plane.bounds for v in limit)
            if plane and plane.bounds
            else ""
        )
        self.form.addRow("Name", self.name)
        self.form.addRow("Reference frame", self.frame)
        self.form.addRow("Point on plane (m)", point)
        self.form.addRow("Normal", normal)
        self.form.addRow("Bounds (optional, min XYZ then max XYZ)", self.bounds)
        self.form.addRow(self.confirmed)
        self.store = QCheckBox("Store the current CameraInfo and TF2 calibration record")
        self.store.setEnabled(record is not None)
        self.form.addRow(self.store)
        if record:
            k = record.intrinsics
            self.message.setText(
                f"{record.resolution[0]} × {record.resolution[1]} · "
                f"fx {k[0]:.3f}, fy {k[4]:.3f}, cx {k[2]:.3f}, cy {k[5]:.3f}\n"
                f"{record.optical_frame} → {record.reference_frame} · {record.stamp_ns} ns"
            )
        else:
            self.message.setText("Waiting for valid live CameraInfo and TF2.")

    def data(self):
        if not self.confirmed.isChecked():
            raise ValueError("Confirm a measured or scene-derived work plane")
        bounds = None
        if self.bounds.text().strip():
            numbers = tuple(map(float, self.bounds.text().split()))
            if len(numbers) != 6:
                raise ValueError("Enter six bounds values")
            bounds = (numbers[:3], numbers[3:])
        plane = WorkPlane(
            name=self.name.text().strip(),
            frame=self.frame.text().strip(),
            point=values(self.point),
            normal=values(self.normal),
            bounds=bounds,
        )
        return plane, self.record if self.store.isChecked() else None
