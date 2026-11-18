"""Record and replay hand-guided SO-101 motion from the Positions tab."""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QSlider,
    QWidget,
)

from ros_robot_studio.backends.so101_replay import DEFAULT_HAND_TORQUE, HAND_TORQUE
from ros_robot_studio.gui.widgets import label
from ros_robot_studio.projects.store import PROJECTS_DIR

REPLAYS_DIR = PROJECTS_DIR.with_name("Replays")


class USBMotionPanel(QWidget):
    requested = Signal(str, str)

    def __init__(self):
        super().__init__()
        self.mode = "idle"
        self.ready = self.passive_ready = self.pending = False
        self.runtime_state = None
        self.recorded_path = None
        self.title = QLineEdit(placeholderText="Recording title")
        self.title.setAccessibleName("Recording title")
        self.title.setMinimumWidth(0)
        self.title.textChanged.connect(self.update_buttons)
        self.record = QPushButton("Record")
        self.record.setToolTip("Guide the arm by hand with a torque limit of at most 25%.")
        self.record.clicked.connect(self.record_clicked)
        self.record_torque = QSlider(Qt.Orientation.Horizontal)
        self.record_torque.setRange(0, HAND_TORQUE)
        self.record_torque.setValue(DEFAULT_HAND_TORQUE)
        self.record_torque.setMaximumWidth(160)
        self.record_torque.setAccessibleName("Recording torque")
        self.record_torque.setToolTip("0% releases motor torque. Support the arm before recording.")
        torque_label = label("Recording torque")
        torque_label.setBuddy(self.record_torque)
        torque_value = label(f"{DEFAULT_HAND_TORQUE} %")
        self.record_torque.valueChanged.connect(lambda value: torque_value.setText(f"{value} %"))
        self.record_torque.valueChanged.connect(self.update_buttons)
        torque_row = QHBoxLayout()
        for widget in (torque_label, self.record_torque, torque_value):
            torque_row.addWidget(widget)
        torque_row.addStretch()
        self.replays = QComboBox()
        self.replays.setAccessibleName("Recorded motion")
        self.replays.setPlaceholderText("Select a recording")
        self.replays.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        self.replays.currentIndexChanged.connect(self.update_buttons)
        self.replay = QPushButton("Replay")
        self.replay.setToolTip("Clear the robot's movement area before replay.")
        self.replay.clicked.connect(self.replay_clicked)
        self.status = label("", "subtitle", True)
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setColumnStretch(0, 1)
        grid.addWidget(self.title, 0, 0)
        grid.addWidget(self.record, 0, 1)
        grid.addLayout(torque_row, 1, 0, 1, 2)
        grid.addWidget(self.replays, 2, 0)
        grid.addWidget(self.replay, 2, 1)
        grid.addWidget(self.status, 3, 0, 1, 2)
        self.hide()

    def refresh_files(self, selected=None):
        selected = selected or self.replays.currentData()
        self.replays.clear()
        for path in sorted(REPLAYS_DIR.glob("*.csv"), key=lambda path: path.name.casefold()):
            if path.is_file():
                self.replays.addItem(path.stem, str(path))
        self.replays.setCurrentIndex(self.replays.findData(selected))

    def set_runtime(
        self, available, ready=False, mode="idle", pending=False, message="", *, passive_ready=False
    ):
        state = (available, ready, mode, pending, message, passive_ready)
        if state == self.runtime_state:
            return
        self.runtime_state = state
        if mode != self.mode and self.mode == "record":
            self.refresh_files(self.recorded_path)
        if available and self.isHidden():
            self.refresh_files()
        self.mode, self.ready, self.pending = mode, ready, pending
        self.passive_ready = passive_ready
        self.setVisible(available)
        self.status.setText(
            message
            or ("" if ready else "Enable USB motor control for powered recording or replay.")
        )
        self.update_buttons()

    def update_buttons(self):
        idle = self.mode == "idle"
        editable = idle and not self.pending
        self.title.setEnabled(editable)
        self.record_torque.setEnabled(editable)
        self.replays.setEnabled(editable)
        self.record.setText("Stop recording" if self.mode == "record" else "Record")
        self.replay.setText("Stop replay" if self.mode == "replay" else "Replay")
        can_record = self.ready or (self.passive_ready and self.record_torque.value() == 0)
        self.record.setEnabled(
            not self.pending
            and (
                self.mode == "record"
                or (editable and can_record and bool(self.title.text().strip()))
            )
        )
        self.replay.setEnabled(
            not self.pending
            and (
                self.mode == "replay"
                or (editable and self.ready and self.replays.currentIndex() >= 0)
            )
        )

    def record_clicked(self):
        if self.mode == "record":
            self.requested.emit("stop_motion", "")
            return
        title = self.title.text().strip()
        if (
            not title
            or title.startswith(".")
            or any(character in '/\\:*?"<>|' or ord(character) < 32 for character in title)
        ):
            self.status.setText("Enter a file title without path separators or special characters.")
            return
        if title.lower().endswith(".csv"):
            title = title[:-4].rstrip()
        if not title:
            self.status.setText("Enter a recording title.")
            return
        path = REPLAYS_DIR / (title + ".csv")
        try:
            if path.exists():
                self.status.setText("A recording already uses this title. Enter a new title.")
                return
            REPLAYS_DIR.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            self.status.setText(str(exc))
            return
        self.recorded_path = str(path)
        self.requested.emit("record", self.recorded_path)

    def replay_clicked(self):
        if self.mode == "replay":
            self.requested.emit("stop_motion", "")
        elif self.replays.currentData():
            self.requested.emit("replay", self.replays.currentData())
