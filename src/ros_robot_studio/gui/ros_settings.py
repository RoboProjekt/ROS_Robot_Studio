"""Explicit simulation/DDS configuration, separate from robot and task definitions."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMessageBox,
    QScrollArea,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.core.models import ExecutionConfig, RuntimeConfig


class ROSSettingsDialog(QDialog):
    def __init__(self, project, model, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ROS simulation / DDS configuration")
        self.resize(700, 760)
        self.project = project
        outer = QVBoxLayout(self)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        content = QWidget()
        scroll.setWidget(content)
        outer.addWidget(scroll)
        body = QVBoxLayout(content)
        form = QFormLayout()
        r = project.runtime
        e = project.execution
        self.profile = QComboBox()
        self.profile.addItems(["managed_simulation", "external_simulation"])
        self.profile.setCurrentText(e.profile)
        self.adapter = QComboBox()
        self.adapter.addItems(["follow_joint_trajectory", "joint_state_command"])
        self.adapter.setCurrentText(e.external_adapter)
        self.domain = QSpinBox()
        self.domain.setRange(0, 232)
        self.domain.setValue(r.domain_id)
        self.port = QSpinBox()
        self.port.setRange(1024, 65535)
        self.port.setValue(r.port)
        self.network = QComboBox()
        self.network.addItems(["bridge", "host"])
        self.network.setCurrentText(r.network)
        self.discovery = QComboBox()
        self.discovery.addItems(["subnet", "localhost"])
        self.discovery.setCurrentText(r.discovery)
        self.namespace = QLineEdit(r.namespace)
        self.state_topic = QLineEdit(e.state_topic)
        self.command_topic = QLineEdit(e.command_topic)
        self.action = QLineEdit(e.action)
        self.clock_topic = QLineEdit(e.clock_topic)
        self.tf_topic = QLineEdit(e.tf_topic)
        self.clock = QCheckBox("Use /clock simulation time (external adapter)")
        self.clock.setChecked(e.use_sim_time)
        self.udp = QCheckBox("Fast DDS over UDP (shared-memory transport disabled)")
        self.udp.setChecked(r.fastdds_udp)
        for title, widget in [
            ("Simulation profile", self.profile),
            ("External adapter", self.adapter),
            ("ROS_DOMAIN_ID", self.domain),
            ("Middleware", QLabel("rmw_fastrtps_cpp")),
            ("Networking", self.network),
            ("Discovery", self.discovery),
            ("ROS namespace", self.namespace),
            ("Loopback rosbridge port", self.port),
            ("JointState feedback", self.state_topic),
            ("JointState command", self.command_topic),
            ("FollowJointTrajectory action", self.action),
            ("Simulation clock topic", self.clock_topic),
            ("TF topic", self.tf_topic),
        ]:
            form.addRow(title, widget)
        body.addLayout(form)
        body.addWidget(self.clock)
        body.addWidget(self.udp)
        body.addWidget(QLabel("Joint mapping · URDF → ROS names (SI units, no calibration)"))
        joints = [j.name for j in model.joints if j.movable]
        self.mapping = QTableWidget(len(joints), 2)
        self.mapping.setHorizontalHeaderLabels(["URDF joint", "ROS joint"])
        self.mapping.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for i, name in enumerate(joints):
            self.mapping.setItem(i, 0, QTableWidgetItem(name))
            self.mapping.item(i, 0).setFlags(
                self.mapping.item(i, 0).flags() & ~Qt.ItemFlag.ItemIsEditable
            )
            self.mapping.setItem(i, 1, QTableWidgetItem(e.joint_mapping.get(name, name)))
        body.addWidget(self.mapping)
        self.confirm = QCheckBox(
            "I confirm the external endpoint is a simulation, with no robot hardware connected"
        )
        self.confirm.setChecked(e.external_simulation_confirmed)
        body.addWidget(self.confirm)
        self.host_confirm = QCheckBox(
            "Allow host DDS networking for this explicit external simulation profile"
        )
        self.host_confirm.setChecked(r.external_network_confirmed)
        body.addWidget(self.host_confirm)
        warning = QLabel(
            "Hardware execution remains disabled. A visible ROS topic does not prove simulation. Host mode exposes DDS discovery; rosbridge remains bound to 127.0.0.1. Lyrical/Isaac compatibility is unverified."
        )
        warning.setWordWrap(True)
        body.addWidget(warning)
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Save | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.validate)
        buttons.rejected.connect(self.reject)
        outer.addWidget(buttons)

    def values(self):
        r = self.project.runtime.model_dump()
        r.update(
            domain_id=self.domain.value(),
            port=self.port.value(),
            network=self.network.currentText(),
            namespace=self.namespace.text().strip(),
            discovery=self.discovery.currentText(),
            fastdds_udp=self.udp.isChecked(),
            external_network_confirmed=self.host_confirm.isChecked(),
        )
        e = self.project.execution.model_dump()
        e.update(
            profile=self.profile.currentText(),
            external_adapter=self.adapter.currentText(),
            clock_topic=self.clock_topic.text().strip(),
            tf_topic=self.tf_topic.text().strip(),
            state_topic=self.state_topic.text().strip(),
            command_topic=self.command_topic.text().strip(),
            action=self.action.text().strip(),
            joint_mapping={
                self.mapping.item(i, 0).text(): self.mapping.item(i, 1).text().strip()
                for i in range(self.mapping.rowCount())
            },
            use_sim_time=self.clock.isChecked(),
            external_simulation_confirmed=self.confirm.isChecked(),
        )
        if r["network"] == "host" and (
            e["profile"] != "external_simulation"
            or not r["external_network_confirmed"]
            or not e["external_simulation_confirmed"]
        ):
            raise ValueError("Host networking requires explicitly confirmed external simulation")
        if e["profile"] == "external_simulation" and not e["external_simulation_confirmed"]:
            raise ValueError("Confirm the simulation-only external endpoint")
        if not all(e["joint_mapping"].values()):
            raise ValueError("Joint mapping contains empty names")
        return RuntimeConfig.model_validate(r), ExecutionConfig.model_validate(e)

    def validate(self):
        try:
            self.values()
        except ValueError as exc:
            QMessageBox.warning(self, "Invalid ROS settings", str(exc))
            return
        self.accept()
