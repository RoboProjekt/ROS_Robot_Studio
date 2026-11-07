from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QVBoxLayout,
)

from ros_robot_studio.core.models import Sensor, Transform
from ros_robot_studio.gui.widgets import label


class SensorDialog(QDialog):
    def __init__(self, sensor: Sensor | None = None, frames=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Configure sensor")
        self.resize(560, 650)
        self.sensor = sensor
        value = sensor or Sensor(name="New sensor")
        layout = QVBoxLayout(self)
        layout.addWidget(label("Sensor & frame", "title"))
        layout.addWidget(
            label(
                "Offsets are in metres and radians. Dynamic TF requires a publisher.",
                "subtitle",
                True,
            )
        )
        form = QFormLayout()
        self.name = QLineEdit(value.name)
        self.kind = QComboBox()
        self.kind.addItems(
            [
                "RGB camera",
                "Depth camera",
                "Force / torque",
                "Pressure",
                "Motor current",
                "Estimated torque",
                "Joint encoder",
                "Generic numeric",
            ]
        )
        self.kind.setCurrentText(value.kind)
        self.topic = QLineEdit(value.topic)
        self.frame = QComboBox()
        self.frame.setEditable(True)
        self.frame.addItems(frames or ["base_link"])
        self.frame.setCurrentText(value.parent_frame)
        self.tf_kind = QComboBox()
        self.tf_kind.addItems(["static", "dynamic"])
        self.tf_kind.setCurrentText(value.transform.kind)
        self.driver = QLineEdit(value.driver)
        self.driver.setReadOnly(True)
        self.unit, self.quantity = QLineEdit(value.unit), QLineEdit(value.quantity)
        self.rate = self.number(value.sample_rate, 0.1, 10000)
        for title, widget in [
            ("Name", self.name),
            ("Sensor type", self.kind),
            ("Driver", self.driver),
            ("ROS topic", self.topic),
            ("Parent frame", self.frame),
            ("Transform", self.tf_kind),
            ("Quantity", self.quantity),
            ("Unit", self.unit),
            ("Sample rate · Hz", self.rate),
        ]:
            form.addRow(title, widget)
        layout.addLayout(form)
        offsets = QGridLayout()
        self.offsets = []
        for index, (name, val) in enumerate(
            zip(
                ["X · m", "Y · m", "Z · m", "Roll · rad", "Pitch · rad", "Yaw · rad"],
                value.transform.translation + value.transform.rotation,
            )
        ):
            spin = self.number(val, -10000, 10000)
            offsets.addWidget(QLabel(name), index // 3 * 2, index % 3)
            offsets.addWidget(spin, index // 3 * 2 + 1, index % 3)
            self.offsets.append(spin)
        layout.addLayout(offsets)
        limits = QFormLayout()
        self.limits = []
        for name, val in [
            ("Lower limit", value.lower_limit),
            ("Upper limit", value.upper_limit),
            ("Warning threshold", value.warning),
        ]:
            edit = QLineEdit("" if val is None else str(val))
            edit.setPlaceholderText("Optional")
            limits.addRow(name, edit)
            self.limits.append(edit)
        layout.addLayout(limits)
        self.enabled = QCheckBox("Sensor enabled")
        self.enabled.setChecked(value.enabled)
        layout.addWidget(self.enabled)
        layout.addWidget(
            label(
                "Monitoring thresholds are not certified safety functions. TF offsets are stored as configuration; publishing TF and calibration are planned.",
                "warning",
                True,
            )
        )
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.validate_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    @staticmethod
    def number(value, low, high):
        spin = QDoubleSpinBox()
        spin.setRange(low, high)
        spin.setDecimals(4)
        spin.setValue(value)
        return spin

    def result_sensor(self):
        offsets = [spin.value() for spin in self.offsets]
        limits = [float(edit.text()) if edit.text().strip() else None for edit in self.limits]
        data = dict(
            name=self.name.text().strip(),
            kind=self.kind.currentText(),
            topic=self.topic.text().strip(),
            driver=self.driver.text(),
            parent_frame=self.frame.currentText().strip(),
            unit=self.unit.text(),
            quantity=self.quantity.text(),
            sample_rate=self.rate.value(),
            enabled=self.enabled.isChecked(),
            transform=Transform(
                kind=self.tf_kind.currentText(),
                translation=tuple(offsets[:3]),
                rotation=tuple(offsets[3:]),
            ),
            lower_limit=limits[0],
            upper_limit=limits[1],
            warning=limits[2],
        )
        if self.sensor:
            data["id"] = self.sensor.id
        return Sensor(**data)

    def validate_accept(self):
        try:
            self.result_sensor()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid sensor configuration", str(exc))
            return
        self.accept()
