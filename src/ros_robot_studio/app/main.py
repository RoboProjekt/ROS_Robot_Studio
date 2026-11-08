import argparse
import json
import logging
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from ros_robot_studio.core.settings import Settings, xdg_path
from ros_robot_studio.gui.main_window import MainWindow


class JsonFormatter(logging.Formatter):
    def format(self, record):
        return json.dumps(
            {
                "time": self.formatTime(record),
                "level": record.levelname,
                "logger": record.name,
                "message": record.getMessage(),
            },
            ensure_ascii=False,
        )


def main():
    parser = argparse.ArgumentParser(description="ROS Robot Studio desktop workbench")
    parser.add_argument("--demo", action="store_true", help="Skip setup and start the offline demo")
    parser.add_argument("--project", type=Path)
    parser.add_argument(
        "--robot", help="Select a catalog robot; downloads still require confirmation"
    )
    parser.add_argument(
        "--screenshot", type=Path, help="Save a demo screenshot, then exit (for QA)"
    )
    parser.add_argument(
        "--smoke-test", action="store_true", help="Start GUI, process events, and exit"
    )
    args = parser.parse_args()
    log_path = xdg_path("state") / "logs" / "studio.jsonl"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handler = RotatingFileHandler(log_path, maxBytes=2_000_000, backupCount=3)
    handler.setFormatter(JsonFormatter())
    logging.basicConfig(level=logging.INFO, handlers=[handler])
    app = QApplication(sys.argv[:1])
    app.setApplicationName("ROS Robot Studio")
    app.setOrganizationName("ROS Robot Studio")
    app.setStyle("Fusion")
    window = MainWindow(
        Settings(), show_wizard=not (args.demo or args.screenshot or args.smoke_test)
    )
    window.show()
    if args.robot:

        def choose_robot():
            if window.runner.busy:
                QTimer.singleShot(100, choose_robot)
                return
            index = window.robot_panel.profile.findData(args.robot)
            if index < 0:
                window.show_error("Unknown robot model: " + args.robot)
            else:
                window.robot_panel.profile.setCurrentIndex(index)

        QTimer.singleShot(100, choose_robot)
    if args.project:

        def open_selected_project():
            if window.runner.busy or window.initial_model_check_pending:
                QTimer.singleShot(100, open_selected_project)
                return
            window.open_path(args.project)

        QTimer.singleShot(100, open_selected_project)
    if args.screenshot:

        def capture():
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(args.screenshot))
            window.tabs.setCurrentIndex(2)
            QTimer.singleShot(1200, capture_monitor)

        def capture_monitor():
            window.grab().save(
                str(args.screenshot.with_name(args.screenshot.stem + "-monitoring.png"))
            )
            window.dirty = False
            app.quit()

        QTimer.singleShot(2200, capture)
    if args.smoke_test:

        def finish_smoke():
            window.dirty = False
            app.quit()

        QTimer.singleShot(1200, finish_smoke)
    return app.exec()
