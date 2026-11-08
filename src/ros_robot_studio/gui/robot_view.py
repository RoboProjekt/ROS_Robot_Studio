"""Existing structure viewer augmented by an optional PyQtGraph OpenGL STL scene."""

import math
from itertools import product

import numpy as np
from PySide6.QtCore import QPointF, Qt, QTimer, Signal
from PySide6.QtGui import (
    QColor,
    QFont,
    QMatrix4x4,
    QOffscreenSurface,
    QOpenGLContext,
    QPainter,
    QPen,
    QSurfaceFormat,
    QVector3D,
    QVector4D,
)
from PySide6.QtWidgets import QApplication, QVBoxLayout, QWidget

from ros_robot_studio.robots.geometry import visual_key
from ros_robot_studio.robots.urdf import RobotModel


class RobotView(QWidget):
    renderer_changed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 230)
        self.gl_view = None
        self.mesh_items = []
        self.show_meshes = True
        self.show_axes = True
        self.end_effector = ""
        self.renderer = "Kinematic structure"
        self.camera_center = np.zeros(3)
        self.camera_span = 0.5
        self.pan_offset = QPointF()
        self.drag_button = None
        self.gl_layout = QVBoxLayout(self)
        self.gl_layout.setContentsMargins(0, 0, 0, 0)
        self.model: RobotModel | None = None
        self.values = {}
        self.yaw, self.pitch, self.zoom = -0.75, 0.42, 1.0
        self.last_pos = None
        self.setToolTip(
            "Left drag: orbit · Right/middle drag: pan · Scroll: zoom · Double-click: reset. Meshes use original URDF visual origins, material colors and scale."
        )

    def set_model(self, model):
        self.model = model
        self.values = model.default_configuration()
        self._configure_scene()
        self.reset_camera()
        QTimer.singleShot(0, self.reset_camera)
        self.update()

    def set_joints(self, values):
        self.model.configuration(values)
        self.values = dict(values)
        self._update_mesh_transforms()
        self.update()

    def mousePressEvent(self, event):
        self.last_pos = event.position()
        self.drag_button = event.button()

    def mouseMoveEvent(self, event):
        if self.last_pos is not None:
            delta = event.position() - self.last_pos
            if self.drag_button in (Qt.MouseButton.RightButton, Qt.MouseButton.MiddleButton):
                self.pan_offset += delta
            else:
                self.yaw += delta.x() * 0.008
                self.pitch = max(-1.2, min(1.2, self.pitch + delta.y() * 0.008))
            self.last_pos = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        self.last_pos = None

    def mouseDoubleClickEvent(self, event):
        self.reset_camera()

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
        span = self.camera_span
        center = self.camera_center
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
                self.width() * 0.48 + side * scale + self.pan_offset.x(),
                self.height() * 0.5 - (z * cp - depth * sp) * scale + self.pan_offset.y(),
            )

        painter.setPen(QPen(QColor("#26364a"), 1))
        extent = span * 0.9
        for i in range(-5, 6):
            t = i * extent / 5
            painter.drawLine(project((t, -extent, 0)), project((t, extent, 0)))
            painter.drawLine(project((-extent, t, 0)), project((extent, t, 0)))
        # Root axes in actual model coordinates.
        origin = positions[self.model.root]
        for direction, color, name in (
            [
                ((1, 0, 0), "#f88383", "X"),
                ((0, 1, 0), "#77d5a0", "Y"),
                ((0, 0, 1), "#76acff", "Z"),
            ]
            if self.show_axes
            else []
        ):
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
        painter.drawText(18, self.height() - 20, self.renderer + "   /   metres")
        painter.setPen(QColor("#53d9bc"))
        painter.drawText(
            18, 25, f"{len(self.model.links)} LINKS    {len(self.model.joints)} JOINTS"
        )

    def _opengl_available(self):
        if QApplication.platformName() in ("offscreen", "minimal"):
            return False
        context = QOpenGLContext()
        fmt = QSurfaceFormat()
        fmt.setVersion(2, 1)
        fmt.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
        context.setFormat(fmt)
        surface = QOffscreenSurface()
        surface.setFormat(fmt)
        surface.create()
        available = context.create() and context.makeCurrent(surface)
        if available:
            context.doneCurrent()
        surface.destroy()
        return available

    def _configure_scene(self):
        if self.gl_view:
            for item, _ in self.mesh_items:
                self.gl_view.removeItem(item)
            self.mesh_items.clear()
        if not self.model.meshes or not self.show_meshes:
            if self.gl_view:
                self.gl_view.hide()
            self.renderer = "Kinematic structure · no mesh rendering"
            self.renderer_changed.emit(self.renderer)
            return
        if self.gl_view is None:
            if not self._opengl_available():
                self.renderer = "Kinematic structure · OpenGL unavailable"
                self.renderer_changed.emit(self.renderer)
                return
            import pyqtgraph.opengl as gl

            self.gl_view = OrbitMeshView(self.reset_camera, self)
            self.gl_view.setBackgroundColor("#111b29")
            self.gl_layout.addWidget(self.gl_view)
            self.grid_item = gl.GLGridItem(color=(65, 85, 110, 100))
            self.gl_view.addItem(self.grid_item)
            self.root_axes = gl.GLAxisItem()
            self.ee_axes = gl.GLAxisItem()
            self.gl_view.addItem(self.root_axes)
            self.gl_view.addItem(self.ee_axes)
        import pyqtgraph.opengl as gl

        for index, visual in enumerate(self.model.visuals):
            geometry = self.model.meshes.get(visual_key(visual, index))
            if geometry is None:
                continue
            item = gl.GLMeshItem(
                vertexes=geometry.vertices,
                faces=geometry.faces,
                color=visual.color,
                smooth=False,
                shader=studio_shader(),
                drawEdges=False,
                glOptions="opaque",
            )
            self.gl_view.addItem(item)
            self.mesh_items.append((item, visual))
        self.gl_view.show()
        self.renderer = f"OpenGL meshes · {len(self.mesh_items)}/{len(self.model.visuals)} visuals · original geometry"
        self._update_mesh_transforms()
        self.set_axes_visible(self.show_axes)
        self.renderer_changed.emit(self.renderer)

    def _update_mesh_transforms(self):
        if not self.gl_view or not self.model:
            return
        transforms = self.model.transforms(self.values)
        for item, visual in self.mesh_items:
            matrix = self.model.visual_transform(visual, transforms)
            item.setTransform(QMatrix4x4(*matrix.flatten().tolist()))
        if self.end_effector in transforms:
            self.ee_axes.setTransform(QMatrix4x4(*transforms[self.end_effector].flatten().tolist()))
        self.gl_view.update()

    def set_end_effector(self, frame):
        self.end_effector = frame
        self._update_mesh_transforms()
        self.set_axes_visible(self.show_axes)

    def set_meshes_visible(self, enabled):
        self.show_meshes = enabled
        if self.model:
            self._configure_scene()
        self.update()

    def set_axes_visible(self, enabled):
        self.show_axes = enabled
        if self.gl_view:
            self.root_axes.setVisible(enabled)
            self.ee_axes.setVisible(enabled and self.end_effector in self.model.links)
        self.update()

    def reset_camera(self):
        self.yaw, self.pitch, self.zoom = -0.75, 0.42, 1.0
        self.pan_offset = QPointF()
        if not self.model:
            return
        transforms = self.model.transforms(self.values)
        bounds = [np.array([matrix[:3, 3] for matrix in transforms.values()])]
        for index, visual in enumerate(self.model.visuals):
            geometry = self.model.meshes.get(visual_key(visual, index))
            if geometry is not None:
                matrix = self.model.visual_transform(visual, transforms)
                vertices = geometry.vertices @ matrix[:3, :3].T + matrix[:3, 3]
                bounds.append(
                    np.array(list(product(*zip(vertices.min(axis=0), vertices.max(axis=0)))))
                )
        points = np.concatenate(bounds)
        self.camera_center = (points.min(axis=0) + points.max(axis=0)) * 0.5
        self.camera_span = max(float(np.ptp(points, axis=0).max()), 0.1)
        if self.gl_view:
            self.gl_view.setCameraPosition(
                pos=QVector3D(*self.camera_center),
                distance=self.camera_span * 1.65,
                elevation=25,
                azimuth=55,
            )
            # PyQtGraph's field of view is horizontal; account for a wide, short panel.
            # Project a bounding box through the actual camera matrix to avoid clipping.
            self.gl_view.setCameraPosition(distance=1.0)
            camera = self.gl_view.viewMatrix()
            corners = [camera.map(QVector3D(*point)) for point in points]
            tangent = math.tan(math.radians(self.gl_view.opts["fov"]) / 2)
            aspect = max(self.gl_view.width(), 1) / max(self.gl_view.height(), 1)
            distance = max(
                max(abs(corner.x()) / tangent, abs(corner.y()) * aspect / tangent)
                + corner.z()
                + 1.0
                for corner in corners
            )
            self.gl_view.setCameraPosition(distance=max(distance * 1.12, self.camera_span * 0.2))
            self.grid_item.setSize(x=self.camera_span * 2.5, y=self.camera_span * 2.5)
            self.grid_item.setSpacing(x=self.camera_span / 5, y=self.camera_span / 5)
            self.root_axes.setSize(*([self.camera_span * 0.18] * 3))
            self.ee_axes.setSize(*([self.camera_span * 0.13] * 3))
        self.update()


# Loaded lazily by RobotView, so headless environments never create an OpenGL widget.
def OrbitMeshView(reset_callback, parent):
    import pyqtgraph.opengl as gl

    class View(gl.GLViewWidget):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            surface_format = self.format()
            surface_format.setVersion(2, 1)
            surface_format.setProfile(QSurfaceFormat.OpenGLContextProfile.CompatibilityProfile)
            surface_format.setSamples(4)
            self.setFormat(surface_format)

        def projectionMatrix(self, region=None):
            matrix = super().projectionMatrix(region)
            # Robot-scale clipping preserves depth precision for fine overlapping STL details.
            near, far = self.opts["distance"] * 0.03, self.opts["distance"] * 20
            matrix.setRow(
                2, QVector4D(0, 0, -(far + near) / (far - near), -2 * far * near / (far - near))
            )
            return matrix

        def mouseDoubleClickEvent(self, event):
            reset_callback()

        def mouseMoveEvent(self, event):
            if event.buttons() & Qt.MouseButton.RightButton:
                difference = event.position() - self.mousePos
                self.mousePos = event.position()
                self.pan(difference.x(), difference.y(), 0, relative="view-upright")
            else:
                super().mouseMoveEvent(event)

    return View(parent=parent)


def studio_shader():
    """Camera-relative fill lighting; original URDF material colors are retained."""
    from pyqtgraph.opengl import shaders

    if "studio_lit" not in shaders.ShaderProgram.names:
        shaders.ShaderProgram(
            "studio_lit",
            [
                shaders.VertexShader("""
                varying vec3 normal;
                void main() {
                    normal = normalize(gl_NormalMatrix * gl_Normal);
                    gl_FrontColor = gl_Color;
                    gl_BackColor = gl_Color;
                    gl_Position = ftransform();
                }
            """),
                shaders.FragmentShader("""
                varying vec3 normal;
                void main() {
                    vec3 n = normalize(normal);
                    if (!gl_FrontFacing) n = -n;
                    float key = max(dot(n, normalize(vec3(0.4, -0.3, 1.0))), 0.0);
                    float fill = max(dot(n, normalize(vec3(-0.6, 0.4, 0.5))), 0.0);
                    gl_FragColor = vec4(gl_Color.rgb * (0.40 + 0.45 * key + 0.15 * fill), gl_Color.a);
                }
            """),
            ],
        )
    return "studio_lit"
