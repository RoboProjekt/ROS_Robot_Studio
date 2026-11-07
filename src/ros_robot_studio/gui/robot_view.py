"""Software-rendered, interactive 3D kinematic structure. No GPU or mesh dependency."""

import math

import numpy as np
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen
from PySide6.QtWidgets import QWidget

from ros_robot_studio.robots.urdf import RobotModel


class RobotView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 300)
        self.model: RobotModel | None = None
        self.values = {}
        self.yaw, self.pitch, self.zoom = -0.75, 0.42, 1.0
        self.last_pos = None
        self.setToolTip(
            "Drag to orbit · Scroll to zoom · Double-click to reset view. Kinematic structure only; meshes are not rendered."
        )

    def set_model(self, model):
        self.model = model
        self.values = {}
        self.update()

    def set_joints(self, values):
        self.values = values
        self.update()

    def mousePressEvent(self, event):
        self.last_pos = event.position()

    def mouseMoveEvent(self, event):
        if self.last_pos is not None:
            delta = event.position() - self.last_pos
            self.yaw += delta.x() * 0.008
            self.pitch = max(-1.2, min(1.2, self.pitch + delta.y() * 0.008))
            self.last_pos = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        self.last_pos = None

    def mouseDoubleClickEvent(self, event):
        self.yaw, self.pitch, self.zoom = -0.75, 0.42, 1.0
        self.update()

    def wheelEvent(self, event):
        self.zoom = max(0.2, min(5, self.zoom * (1.12 if event.angleDelta().y() > 0 else 0.89)))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor("#111b29"))
        if self.model is None:
            painter.setPen(QColor("#a5b6cb"))
            painter.drawText(
                self.rect(), Qt.AlignmentFlag.AlignCenter, "Import a URDF to explore its structure"
            )
            return
        positions = self.model.positions(self.values)
        array = np.array(list(positions.values()))
        span = max(float(np.ptp(array, axis=0).max()), 0.3)
        center = (array.max(axis=0) + array.min(axis=0)) * 0.5
        scale = min(self.width(), self.height()) * 0.64 / span * self.zoom
        c, s, cp, sp = (
            math.cos(self.yaw),
            math.sin(self.yaw),
            math.cos(self.pitch),
            math.sin(self.pitch),
        )

        def project(point):
            x, y, z = np.array(point) - center
            side, depth = x * c - y * s, x * s + y * c
            return QPointF(
                self.width() * 0.48 + side * scale,
                self.height() * 0.5 - (z * cp - depth * sp) * scale,
            )

        painter.setPen(QPen(QColor("#26364a"), 1))
        extent = span * 0.9
        for i in range(-5, 6):
            t = i * extent / 5
            painter.drawLine(project((t, -extent, 0)), project((t, extent, 0)))
            painter.drawLine(project((-extent, t, 0)), project((extent, t, 0)))
        # Root axes in actual model coordinates.
        origin = positions[self.model.root]
        for direction, color, name in [
            ((1, 0, 0), "#f88383", "X"),
            ((0, 1, 0), "#77d5a0", "Y"),
            ((0, 0, 1), "#76acff", "Z"),
        ]:
            tip = origin + np.array(direction) * span * 0.25
            painter.setPen(QPen(QColor(color), 2))
            painter.drawLine(project(origin), project(tip))
            painter.drawText(project(tip) + QPointF(4, -4), name)
        for joint in self.model.joints:
            start, end = project(positions[joint.parent]), project(positions[joint.child])
            painter.setPen(
                QPen(QColor("#1f5d59"), 20, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            )
            painter.drawLine(start, end)
            painter.setPen(
                QPen(QColor("#62d9c1"), 10, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap)
            )
            painter.drawLine(start, end)
        painter.setFont(QFont("DejaVu Sans", 9))
        for name, position in positions.items():
            point = project(position)
            painter.setBrush(QColor("#152839"))
            painter.setPen(QPen(QColor("#a2f3df"), 2))
            painter.drawEllipse(point, 8, 8)
            painter.setPen(QColor("#bacbdd"))
            painter.drawText(point + QPointF(14, 0), name)
        painter.setPen(QColor("#788fa9"))
        painter.drawText(
            18, self.height() - 20, "KINEMATIC STRUCTURE   /   metres     ·     Drag to orbit"
        )
        painter.setPen(QColor("#53d9bc"))
        painter.drawText(
            18, 25, f"{len(self.model.links)} LINKS    {len(self.model.joints)} JOINTS"
        )
