"""Split tab workspace. Panels keep their identity and top-level OpenGL window."""

from __future__ import annotations

from copy import deepcopy

from PySide6.QtCore import QPoint, QRect, Qt, Signal
from PySide6.QtGui import QColor, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QLayout,
    QScrollArea,
    QSplitter,
    QTabBar,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)


class WorkspaceGroup(QTabWidget):
    """Expose original panels while Qt hosts their persistent scroll pages."""

    def __init__(self, workspace):
        super().__init__(workspace)
        self.workspace = workspace

    def addTab(self, panel, title):
        return super().addTab(self.workspace.pages[panel], title)

    def widget(self, index):
        page = super().widget(index)
        return self.workspace.originals.get(page)

    def currentWidget(self):
        return self.workspace.originals.get(super().currentWidget())

    def indexOf(self, panel):
        return super().indexOf(self.workspace.pages.get(panel))

    def setCurrentWidget(self, panel):
        super().setCurrentWidget(self.workspace.pages[panel])


class DropOverlay(QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.preview = QRect()
        self.hide()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setPen(QPen(QColor("#54b9ff"), 3))
        painter.setBrush(QColor(60, 160, 240, 65))
        painter.drawRect(self.preview.adjusted(2, 2, -2, -2))
        painter.setPen(QColor("#ffffff"))
        painter.drawText(self.preview, Qt.AlignmentFlag.AlignCenter, "Drop to split / tab here")


class WorkspaceTabBar(QTabBar):
    def __init__(self, group, workspace):
        super().__init__(group)
        self.group, self.workspace = group, workspace
        self.origin = None
        self.dragging = False
        self.panel = None
        self.previous = None
        self.setElideMode(Qt.TextElideMode.ElideRight)
        self.setExpanding(False)
        self.setToolTip("Drag a tab to an edge to split; drop in the center to merge tabs.")

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.origin = event.globalPosition().toPoint()
            self.previous = self.group.currentWidget()
            self.panel = self.group.widget(self.tabAt(event.position().toPoint()))
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if (
            self.origin is not None
            and self.panel is not None
            and event.buttons() & Qt.MouseButton.LeftButton
        ):
            point = event.globalPosition().toPoint()
            if (point - self.origin).manhattanLength() >= QApplication.startDragDistance():
                self.dragging = True
                self.workspace.preview_drop(point)
                return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        dragging, panel = self.dragging, self.panel
        self.dragging = False
        self.origin = None
        self.panel = None
        self.workspace.overlay.hide()
        super().mouseReleaseEvent(event)
        if dragging and panel is not None:
            self.workspace.drop_panel(panel, event.globalPosition().toPoint(), self.previous)


class Workspace(QWidget):
    layout_changed = Signal()
    VERSION = 2

    def __init__(self, parent=None):
        super().__init__(parent)
        self.panels = []
        self.pages = {}
        self.originals = {}
        self.titles = {}
        self.groups = []
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(0, 0, 0, 0)
        self.staging = QWidget(self)
        self.staging.hide()
        self.root = self.new_group()
        self.body.addWidget(self.root)
        self.overlay = DropOverlay(self)

    def new_group(self):
        group = WorkspaceGroup(self)
        group.setTabBar(WorkspaceTabBar(group, self))
        group.setDocumentMode(True)
        group.setMinimumSize(190, 150)
        group.currentChanged.connect(lambda *_: self.layout_changed.emit())
        self.groups.append(group)
        return group

    def addTab(self, panel, title):
        self.panels.append(panel)
        self.titles[panel] = title
        page = QScrollArea(self.staging)
        page.setFrameShape(QFrame.Shape.NoFrame)
        page.setWidgetResizable(True)
        # A small split scrolls the content instead of compressing layouts below
        # the minimum heights of the existing viewers, labels and controls.
        panel.layout().setSizeConstraint(QLayout.SizeConstraint.SetMinimumSize)
        page.setWidget(panel)
        self.pages[panel] = page
        self.originals[page] = panel
        return self.groups[0].addTab(panel, title)

    def count(self):
        return len(self.panels)

    def widget(self, index):
        return self.panels[index]

    def group_for(self, panel):
        return next((g for g in self.groups if g.indexOf(panel) >= 0), None)

    def setCurrentIndex(self, index):
        panel = self.panels[index]
        self.group_for(panel).setCurrentWidget(panel)

    def currentIndex(self):
        focused = QApplication.focusWidget()
        for i, panel in enumerate(self.panels):
            if focused and (panel == focused or panel.isAncestorOf(focused)):
                return i
        return self.panels.index(self.groups[0].currentWidget())

    def drop_location(self, global_point):
        for group in self.groups:
            point = group.mapFromGlobal(global_point)
            if group.isVisible() and group.rect().contains(point):
                w, h = group.width(), group.height()
                distances = {
                    "left": point.x() / w,
                    "right": (w - point.x()) / w,
                    "top": point.y() / h,
                    "bottom": (h - point.y()) / h,
                }
                edge = min(distances, key=distances.get)
                # The tab strip is a center target, enabling tab reordering/merging.
                if group.tabBar().geometry().contains(point):
                    edge = "center"
                elif distances[edge] > 0.24:
                    edge = "center"
                return group, edge
        return None, None

    def preview_drop(self, global_point):
        group, edge = self.drop_location(global_point)
        if group is None:
            self.overlay.hide()
            return
        rect = QRect(group.mapTo(self, QPoint()), group.size())
        if edge == "left":
            rect.setWidth(rect.width() // 2)
        elif edge == "right":
            rect.setLeft(rect.center().x())
        elif edge == "top":
            rect.setHeight(rect.height() // 2)
        elif edge == "bottom":
            rect.setTop(rect.center().y())
        self.overlay.setGeometry(self.rect())
        self.overlay.preview = rect
        self.overlay.show()
        self.overlay.raise_()
        self.overlay.update()

    def replace(self, old, new):
        parent = old.parentWidget()
        if isinstance(parent, QSplitter):
            index = parent.indexOf(old)
            old.setParent(self.staging)
            parent.insertWidget(index, new)
        else:
            self.body.removeWidget(old)
            old.setParent(self.staging)
            self.body.addWidget(new)
            self.root = new
        new.show()

    def drop_panel(self, panel, global_point, previous=None):
        target, edge = self.drop_location(global_point)
        if target is None:
            return
        source = self.group_for(panel)
        if target is source and edge == "center":
            index = target.tabBar().tabAt(target.tabBar().mapFromGlobal(global_point))
            if index >= 0:
                target.tabBar().moveTab(target.indexOf(panel), index)
            self.layout_changed.emit()
            return
        if target is source and source.count() == 1:
            return
        if edge != "center":
            split = QSplitter(
                Qt.Orientation.Horizontal if edge in ("left", "right") else Qt.Orientation.Vertical,
                self,
            )
            split.setChildrenCollapsible(False)
            destination = self.new_group()
            self.replace(target, split)
            for widget in (
                (destination, target) if edge in ("left", "top") else (target, destination)
            ):
                split.addWidget(widget)
                widget.show()
            split.setSizes([max(1, split.width() // 2)] * 2)
        else:
            destination = target
        source.removeTab(source.indexOf(panel))
        destination.addTab(panel, self.titles[panel])
        destination.setCurrentWidget(panel)
        panel.show()
        if previous is not panel and source.indexOf(previous) >= 0:
            source.setCurrentWidget(previous)
        if source.count() == 0:
            self.remove_group(source)
        self.layout_changed.emit()

    def remove_group(self, group):
        self.groups.remove(group)
        parent = group.parentWidget()
        group.setParent(self.staging)
        group.deleteLater()
        while isinstance(parent, QSplitter) and parent.count() == 1:
            child = parent.widget(0)
            ancestor = parent.parentWidget()
            self.replace(parent, child)
            parent.deleteLater()
            parent = ancestor

    def state(self):
        def node(widget):
            if isinstance(widget, QSplitter):
                return {
                    "split": "horizontal"
                    if widget.orientation() == Qt.Orientation.Horizontal
                    else "vertical",
                    "sizes": widget.sizes(),
                    "children": [node(widget.widget(i)) for i in range(widget.count())],
                }
            return {
                "tabs": [self.panels.index(widget.widget(i)) for i in range(widget.count())],
                "current": self.panels.index(widget.currentWidget()),
            }

        return {"version": self.VERSION, "tree": node(self.root)}

    def restore(self, state):
        if state == self.state():
            return True
        seen = []

        def validate(node, depth=0):
            if not isinstance(node, dict) or depth > 6:
                raise ValueError("Invalid workspace tree")
            if "tabs" in node:
                tabs = node["tabs"]
                if (
                    not isinstance(tabs, list)
                    or not tabs
                    or any(type(i) is not int or i not in range(len(self.panels)) for i in tabs)
                ):
                    raise ValueError("Unknown panel")
                if node.get("current") not in tabs:
                    raise ValueError("Invalid current panel")
                seen.extend(tabs)
            else:
                children, sizes = node["children"], node["sizes"]
                if (
                    node["split"] not in ("horizontal", "vertical")
                    or len(children) < 2
                    or len(children) != len(sizes)
                    or any(type(n) is not int or n < 0 for n in sizes)
                ):
                    raise ValueError("Invalid split")
                for child in children:
                    validate(child, depth + 1)

        try:
            version = state.get("version")
            if version not in (1, self.VERSION):
                return False
            validate(state["tree"])
            if version == 1 and len(self.panels) == 5 and sorted(seen) == list(range(4)):
                state = deepcopy(state)
                first = state["tree"]
                while "children" in first:
                    first = first["children"][0]
                first["tabs"].append(4)
                seen.append(4)
            if sorted(seen) != list(range(len(self.panels))):
                return False
        except (ValueError, KeyError, TypeError, AttributeError):
            return False
        old = self.root
        for panel in self.panels:
            group = self.group_for(panel)
            group.removeTab(group.indexOf(panel))
            self.pages[panel].setParent(self.staging)
        self.groups = []

        def build(node):
            if "tabs" in node:
                group = self.new_group()
                for index in node["tabs"]:
                    panel = self.panels[index]
                    group.addTab(panel, self.titles[panel])
                group.setCurrentWidget(self.panels[node["current"]])
                return group
            split = QSplitter(
                Qt.Orientation.Horizontal
                if node["split"] == "horizontal"
                else Qt.Orientation.Vertical,
                self,
            )
            split.setChildrenCollapsible(False)
            for child in node["children"]:
                split.addWidget(build(child))
            split.setSizes(node["sizes"])
            return split

        self.root = build(state["tree"])
        self.body.removeWidget(old)
        old.setParent(self.staging)
        old.deleteLater()
        self.body.addWidget(self.root)
        self.root.show()
        self.layout_changed.emit()
        return True

    def reset(self):
        self.restore(
            {"version": self.VERSION, "tree": {"tabs": list(range(len(self.panels))), "current": 0}}
        )
