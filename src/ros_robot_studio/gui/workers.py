"""Serial background execution with GUI-thread signal delivery and backpressure."""

import logging
import time

from PySide6.QtCore import QObject, QRunnable, QThreadPool, Signal

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

    @property
    def busy(self):
        return bool(self.tasks)

    def run(self, function, done=lambda result: None, error=None):
        worker = Worker(function)
        self.tasks.add(worker)
        worker.signals.done.connect(done)
        worker.signals.error.connect(error or self.failed.emit)
        worker.signals.progress.connect(self.progress.emit)
        worker.signals.finished.connect(lambda: self._finish(worker))
        self.busy_changed.emit(True)
        self.pool.start(worker)

    def _finish(self, worker):
        self.tasks.discard(worker)
        self.busy_changed.emit(self.busy)
