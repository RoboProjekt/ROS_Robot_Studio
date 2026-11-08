"""One reusable, nonmodal diagnostics surface for all Docker entry points."""

import json

from PySide6.QtWidgets import QDialog, QHBoxLayout, QLabel, QPlainTextEdit, QPushButton, QVBoxLayout


class DockerDiagnosticsDialog(QDialog):
    def __init__(self, main):
        super().__init__(main)
        self.setWindowTitle("Docker runtime diagnostics")
        self.resize(720, 540)
        layout = QVBoxLayout(self)
        self.summary = QLabel()
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.details = QPlainTextEdit()
        self.details.setReadOnly(True)
        layout.addWidget(self.details)
        row = QHBoxLayout()
        for title, callback in [
            ("Recheck access", main.refresh_runtime),
            ("Setup assistant", main.open_wizard),
            ("Close", self.hide),
        ]:
            button = QPushButton(title)
            button.clicked.connect(callback)
            row.addWidget(button)
        layout.addLayout(row)
        self.update_status(main.docker_status)

    def update_status(self, status):
        self.summary.setText(status.message)
        self.details.setPlainText(json.dumps(status.diagnostics, indent=2))
