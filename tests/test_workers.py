"""Queued work must target a live GUI QObject, including already posted results."""

import shiboken6
from PySide6.QtCore import QCoreApplication, QEvent, QObject, QThread

from ros_robot_studio.gui.workers import TaskRunner


def test_queued_completion_is_discarded_after_receiver_destruction(qapp):
    parent = QObject()
    runner = TaskRunner(parent)
    received = []
    runner.run(lambda _: "complete", received.append)
    # Finish the worker without delivering its queued GUI callbacks yet.
    assert runner.pool.waitForDone(2000)
    assert runner.busy and not received
    parent.deleteLater()
    QCoreApplication.sendPostedEvents(parent, QEvent.Type.DeferredDelete)
    assert not shiboken6.isValid(parent) and not shiboken6.isValid(runner)
    qapp.processEvents()
    assert received == []


def test_results_errors_and_serial_callbacks_run_in_gui_thread(qapp, qtbot):
    parent = QObject()
    runner = TaskRunner(parent)
    results = []
    threads = []

    def success(value):
        results.append(value)
        threads.append(QThread.currentThread())

    def failure(message):
        results.append(message)
        threads.append(QThread.currentThread())

    def fail(_):
        raise ValueError("expected worker failure")

    runner.run(lambda _: "first", success)
    runner.run(fail, error=failure)
    qtbot.waitUntil(lambda: not runner.busy)
    assert results == ["first", "expected worker failure"]
    assert threads and all(thread == qapp.thread() for thread in threads)
    assert not runner.outcomes and not runner.callbacks
    parent.deleteLater()
