"""Repeated real widget/worker destruction, including already queued ROS deliveries.

No Docker daemon is used here; ROS packets are injected into the actual transport.
Use verify_ros_runtime.py separately for native DDS/action acceptance.
"""

import argparse
import gc
import json
import tempfile
import time
from importlib.metadata import version
from pathlib import Path
from unittest.mock import MagicMock

import shiboken6
from PySide6.QtCore import QCoreApplication, QEvent, QObject, Qt, QTimer, Signal
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport
from ros_robot_studio.runtime.catalog import catalog
from ros_robot_studio.runtime.manager import RuntimeStatus


class PendingPackets(QObject):
    message = Signal(str, dict)
    state = Signal(object)


def verify(cycles, robot, report_path=None):
    app = QApplication.instance() or QApplication([])
    app.setStyle("Fusion")
    # QApplication lazily owns one Qt::Desktop helper until application shutdown.
    # See docs/QT_WIDGET_INVESTIGATION.md; never delete this Qt-internal object.
    baseline = len(
        [w for w in QApplication.allWidgets() if w.windowType() != Qt.WindowType.Desktop]
    )
    desktop_pointer = None
    desktop_counts = []
    errors = []
    destroyed = 0
    opengl_cycles = 0
    started = time.monotonic()

    def wait(predicate):
        deadline = time.monotonic() + 20
        while not predicate():
            app.processEvents()
            QTest.qWait(5)
            assert not errors, errors
            assert time.monotonic() < deadline, "Lifecycle phase timed out"

    with tempfile.TemporaryDirectory(prefix="studio-gui-lifecycle-") as temp:
        for cycle in range(cycles):
            manager = MagicMock()
            manager.specs = catalog()
            manager.status.return_value = RuntimeStatus(False, "Lifecycle test: no Docker")
            window = MainWindow(
                Settings(Path(temp) / f"settings-{cycle}.json"), manager, show_wizard=False
            )
            window.runner.failed.disconnect()
            window.runner.failed.connect(errors.append)
            window.show_error = errors.append
            window.motion.runner.failed.connect(errors.append)
            window.show()
            wait(
                lambda: not window.runner.busy
                and not window.motion.runner.busy
                and not window.initial_model_check_pending
            )
            project, model, _ = pick_place_demo(window.model_manager, prefer_so101=robot == "so101")
            if robot == "so101":
                assert project.robot.model_id == "so101_follower", "Install original SO-101 first"
            window.project, window.model = project, model
            window.apply_project()
            wait(lambda: not window.motion.runner.busy)
            window.tabs.setCurrentIndex(2)
            QTest.qWait(30)
            monitor = window.monitor.robot_state
            # Exercise positions and analytic velocities in the same PlotWidget.
            field = "velocity." + window.state_manager.snapshot().joint_names[0]
            for i in range(monitor.fields.count()):
                if monitor.fields.item(i).text() == field:
                    monitor.fields.item(i).setCheckState(Qt.CheckState.Checked)
            window.motion.compile_task(True)
            wait(lambda: not window.motion.runner.busy)
            QTest.qWait(90)
            assert window.state_manager.snapshot().velocities is not None
            assert monitor.curves and len(monitor.buffer.samples) > 1
            window.tabs.setCurrentIndex(0)
            window.tabs.setCurrentIndex(2)
            window.motion.pause()
            window.motion.pause()
            window.motion.stop()
            # The earlier reproduction also created and dismissed a QWizard.
            window.motion.open_wizard()
            wizard = window.motion.wizard
            wizard.suggest_gripper()
            wizard.confirm.setChecked(True)
            wizard.generate()
            wait(lambda: not wizard.runner.busy)
            assert wizard.result_data and wizard.result_data[2].valid
            wizard.reject()
            opengl_cycles += window.robot_panel.view.gl_view is not None
            if app.platformName() == "xcb":
                from OpenGL.GL import glGetError, glIsProgram

                for view in (window.robot_panel.view, monitor.view):
                    assert view.gl_view is not None, "Native acceptance requires OpenGL"
                    assert not view.gl_view.grabFramebuffer().isNull()
                    view.gl_view.makeCurrent()
                    assert glIsProgram(view.mesh_shader.program()), "Shader belongs to old context"
                    assert glGetError() == 0, "OpenGL rendering failed"
                    view.gl_view.doneCurrent()

            transport = RosbridgeTransport("ws://127.0.0.1:1", window)
            window.mode = "ros"
            window.ros_lease = window.state_manager.activate_source(
                "ros_simulation", "/joint_states"
            )
            window.attach_transport(transport)
            state = window.state_manager.snapshot()
            message = {
                "name": list(state.positions),
                "position": list(state.positions.values()),
                "velocity": [0.0] * len(state.positions),
            }
            transport._receive(
                json.dumps({"op": "publish", "topic": "/joint_states", "msg": message})
            )
            assert transport.latest  # A real transport flush timer is still pending.
            packets = PendingPackets()
            packets.message.connect(window.receive_message, Qt.ConnectionType.QueuedConnection)
            packets.state.connect(window.robot_state_updated, Qt.ConnectionType.QueuedConnection)
            packets.message.emit("/joint_states", message)
            packets.state.emit(state)
            QTimer.singleShot(0, window.update_live)
            QTimer.singleShot(0, window.refresh_runtime)
            QTimer.singleShot(0, window.refresh_model_catalog)
            QTimer.singleShot(0, window.ros_start_failed)
            window.ros_start_deadline.start(20000)
            pools = [window.runner.pool, window.motion.runner.pool, wizard.runner.pool]
            timers = window.findChildren(QTimer)
            plots = [window.monitor.plot, monitor.plot]
            window.dirty = False
            assert window.close() and window.closed
            assert window.close()  # Repeated close must not close a PlotItem twice.
            after_close = window.state_manager.snapshot()
            for pool in pools:
                assert pool.waitForDone(3000)
                assert pool.activeThreadCount() == 0
            assert all(not timer.isActive() for timer in timers)
            app.processEvents()
            assert window.state_manager.snapshot() is after_close, "Late packet changed closed UI"
            assert not window.runner.busy and not window.motion.runner.busy
            assert all(plot.plotItem is None for plot in plots)
            # PlotWidget.close detaches its parent; explicitly delete the closed widgets too.
            for widget in (*plots, window):
                if shiboken6.isValid(widget):
                    widget.deleteLater()
            packets.deleteLater()
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            assert not shiboken6.isValid(window)
            assert not shiboken6.isValid(transport)
            assert not shiboken6.isValid(wizard)
            assert all(not shiboken6.isValid(plot) for plot in plots)
            destroyed += 1
            window = transport = wizard = monitor = plots = pools = timers = packets = None
            widget = None
            gc.collect()
            app.processEvents()
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            desktops = [
                w for w in QApplication.allWidgets() if w.windowType() == Qt.WindowType.Desktop
            ]
            desktop_counts.append(len(desktops))
            assert len(desktops) <= 1
            if desktops:
                helper = desktops[0]
                assert helper.metaObject().className() == "QWidget"
                assert helper.parent() is None and not helper.isVisible() and not helper.children()
                pointer = shiboken6.getCppPointer(helper)[0]
                desktop_pointer = pointer if desktop_pointer is None else desktop_pointer
                assert pointer == desktop_pointer
            application_widgets = [
                w for w in QApplication.allWidgets() if w.windowType() != Qt.WindowType.Desktop
            ]
            assert len(application_widgets) <= baseline, [
                (type(w).__name__, w.objectName(), type(w.parent()).__name__)
                for w in application_widgets
            ]
            assert not errors, errors
            print(f"CYCLE {cycle + 1}/{cycles}: destroyed widgets, timers and workers", flush=True)

    report = dict(
        robot=robot,
        cycles=cycles,
        destroyed_windows=destroyed,
        opengl_cycles=opengl_cycles,
        qt_platform=app.platformName(),
        pyside6=version("PySide6"),
        shiboken6=version("shiboken6"),
        pyqtgraph=version("pyqtgraph"),
        numpy=version("numpy"),
        queued_ros_packets="injected into actual transport; not a DDS test",
        pending_events_ignored=True,
        widget_leaks=0,
        expected_qt_desktop_widget_counts=desktop_counts,
        qt_desktop_identity_stable=True,
        workers_joined=True,
        elapsed_seconds=round(time.monotonic() - started, 2),
    )
    if report_path:
        report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycles", type=int, default=20)
    parser.add_argument("--robot", choices=("generic", "so101"), default="so101")
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    assert args.cycles > 0
    verify(args.cycles, args.robot, args.report)
