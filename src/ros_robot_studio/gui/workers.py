"""Serial background execution with GUI-thread signal delivery and backpressure."""

import logging
import time

from PySide6.QtCore import QObject, QRunnable, Qt, QThreadPool, Signal, Slot

log = logging.getLogger(__name__)


class WorkerSignals(QObject):
    done = Signal(object)
    error = Signal(str)
    progress = Signal(str)
    finished = Signal()


class Worker(QRunnable):
    def __init__(self, function):
        super().__init__()
        self.function = function
        self.signals = WorkerSignals()
        self.last_progress = 0.0

    def progress(self, text):
        # Docker can emit thousands of progress events; cap queued GUI signals at 10 Hz.
        now = time.monotonic()
        if now - self.last_progress > 0.1 or "error" in text.lower() or "ready" in text.lower():
            self.last_progress = now
            self.signals.progress.emit(text)

    def run(self):
        try:
            self.signals.done.emit(self.function(self.progress))
        except Exception as exc:
            log.exception("Background operation failed")
            self.signals.error.emit(str(exc))
        finally:
            self.signals.finished.emit()


class TaskRunner(QObject):
    busy_changed = Signal(bool)
    progress = Signal(str)
    failed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pool = QThreadPool(self)
        self.pool.setMaxThreadCount(1)
        self.tasks = set()
        self.outcomes = {}
        self.callbacks = {}

    @property
    def busy(self):
        return bool(self.tasks)

    def run(self, function, done=lambda result: None, error=None):
        worker = Worker(function)
        self.tasks.add(worker)
        self.callbacks[worker.signals] = (worker, done, error or self.failed.emit)
        # Real receiver slots give queued events a QObject lifetime and GUI-thread
        # context. Qt drops them when this runner is destroyed, including events
        # already queued before the parent window was deleted.
        worker.signals.done.connect(self._result, Qt.ConnectionType.QueuedConnection)
        worker.signals.error.connect(self._error, Qt.ConnectionType.QueuedConnection)
        worker.signals.progress.connect(self._progress, Qt.ConnectionType.QueuedConnection)
        worker.signals.finished.connect(self._finished, Qt.ConnectionType.QueuedConnection)
        self.busy_changed.emit(True)
        self.pool.start(worker)

    @Slot(object)
    def _result(self, result):
        job = self.callbacks.get(self.sender())
        if job:
            self.outcomes[job[0]] = (job[1], result)

    @Slot(str)
    def _error(self, message):
        job = self.callbacks.get(self.sender())
        if job:
            self.outcomes[job[0]] = (job[2], message)

    @Slot(str)
    def _progress(self, message):
        self.progress.emit(message)

    @Slot()
    def _finished(self):
        job = self.callbacks.pop(self.sender(), None)
        if job:
            self._finish(job[0])

    def _finish(self, worker):
        self.tasks.discard(worker)
        self.busy_changed.emit(self.busy)
        callback, value = self.outcomes.pop(worker, (None, None))
        if callback:
            callback(value)
