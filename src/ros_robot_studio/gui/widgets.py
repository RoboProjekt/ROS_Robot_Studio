from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget


def label(text: str, role: str = "", wrap=False) -> QLabel:
    item = QLabel(text)
    item.setObjectName(role)
    item.setWordWrap(wrap)
    return item


def button(text: str, callback=None, primary=False) -> QPushButton:
    item = QPushButton(text)
    if primary:
        item.setObjectName("primary")
    if callback:
        item.clicked.connect(callback)
    return item


def card(title: str, subtitle: str = ""):
    frame = QFrame()
    frame.setObjectName("card")
    layout = QVBoxLayout(frame)
    layout.setContentsMargins(18, 16, 18, 16)
    layout.setSpacing(12)
    if title:
        layout.addWidget(label(title, "section"))
    if subtitle:
        layout.addWidget(label(subtitle, "subtitle", True))
    return frame, layout


def page_header(eyebrow: str, title: str, subtitle: str, badge: str = ""):
    widget = QWidget()
    layout = QHBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 15)
    text = QVBoxLayout()
    text.setSpacing(6)
    text.addWidget(label(eyebrow.upper(), "eyebrow"))
    text.addWidget(label(title, "title"))
    text.addWidget(label(subtitle, "subtitle", True))
    layout.addLayout(text, 1)
    if badge:
        layout.addWidget(label(badge, "badge"), 0, Qt.AlignmentFlag.AlignTop)
    return widget
