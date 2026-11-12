"""Native visible vision acceptance with real Humble and Isaac Sim."""

import argparse
import ctypes
import json
import re
import subprocess
import time
from collections import Counter
from dataclasses import asdict
from hashlib import sha256
from pathlib import Path
from unittest.mock import patch

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QDialogButtonBox, QFileDialog, QPushButton
from verify_gui_usability import choose, choose_scene_file, drag_tab

from ros_robot_studio.backends.isaac import IsaacSimInstallationManager
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.camera_dialogs import CalibrationDialog, CameraDialog
from ros_robot_studio.gui.main_window import MainWindow
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.schema import MotionTask, TaskStep
from ros_robot_studio.projects.store import save_project


def click(button):
    assert button.isEnabled(), button.text()
    QTest.mouseClick(button, Qt.MouseButton.LeftButton)


def toolkit_space(app, sdk_pid, output):
    tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
    windows = []
    for line in tree.splitlines():
        match = re.search(r'\b(0x[0-9a-f]+) "(.*Isaac Sim[^"]*)"', line, re.I)
        if not match:
            continue
        xid = match[1]
        properties = subprocess.check_output(["xprop", "-id", xid, "_NET_WM_PID"], text=True)
        if properties.rstrip().endswith("= " + str(sdk_pid)):
            windows.append(int(xid, 16))
    assert len(windows) == 1, (windows, tree[-2000:])
    window = windows[0]
    x11 = ctypes.CDLL("libX11.so.6")
    xtest = ctypes.CDLL("libXtst.so.6")
    x11.XOpenDisplay.restype = ctypes.c_void_p
    display = ctypes.c_void_p(x11.XOpenDisplay(None))
    assert display.value
    try:
        x11.XRaiseWindow(display, ctypes.c_ulong(window))
        x11.XSetInputFocus(display, ctypes.c_ulong(window), 2, ctypes.c_ulong(0))
        x11.XSync(display, 0)
        QTest.qWait(300)
        focus = ctypes.c_ulong()
        revert = ctypes.c_int()
        x11.XGetInputFocus(display, ctypes.byref(focus), ctypes.byref(revert))
        assert focus.value == window, (focus.value, window)
        if output is not None:
            assert app.primaryScreen().grabWindow(window).save(str(output))
        key = x11.XKeysymToKeycode(display, ctypes.c_ulong(0x20))
        assert key
        for pressed in (1, 0):
            assert xtest.XTestFakeKeyEvent(display, key, pressed, ctypes.c_ulong(0))
        x11.XSync(display, 0)
    finally:
        x11.XCloseDisplay(display)
    return hex(window)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("out", type=Path)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = Path("simulation_scenes/IsaacSim/So-101_demo/scene.usd").resolve()
    scene_path = out / "SO101_RGB_Vision_Demo.usd"
    project_path = out / "SO101_RGB_Vision_Demo.studio.yaml"
    hashes = {p: sha256(p.read_bytes()).hexdigest() for p in source.parent.rglob("*.usd")}
    app = QApplication([])
    assert app.platformName() == "xcb"
    app.setStyle("Fusion")
    errors = []
    settings = Settings(out / "settings.json")
    logs = (out / "isaac-process.log").open("w")

    def open_window():
        window = MainWindow(settings, show_wizard=False)
        window.runner.failed.disconnect()
        window.runner.failed.connect(errors.append)
        window.backend_manager.failed.disconnect()
        window.backend_manager.failed.connect(errors.append)
        window.ros_execution.error.connect(errors.append)
        window.isaac_processes.error.connect(
            lambda identity, message: errors.append((identity, message))
        )
        window.isaac_processes.log.connect(
            lambda identity, message: (logs.write(message), logs.flush())
        )
        window.show_error = errors.append
        window.resize(1500, 1000)
        window.show()
        return window

    w = open_window()

    def wait(predicate, timeout=85, cleanup=False):
        until = time.monotonic() + timeout
        while not predicate():
            QTest.qWait(20)
            assert cleanup or not errors, errors
            assert time.monotonic() < until, dict(
                errors=errors,
                status=w.simulation_header.status.text(),
                camera=w.vision.status.text(),
                detection=w.vision.detector_status.text(),
                setup=w.vision.config_status.text(),
                execution=w.ros_execution.status,
                task=w.motion.task_status.text(),
            )

    def close_window():
        w.dirty = False
        w.close()
        wait(lambda: w.closed, timeout=45, cleanup=True)
        assert not w.backend_manager.sessions and not w.isaac_processes.processes
        assert all(not t.isActive() for t in w.findChildren(QTimer))

    def button(parent, text):
        return next(b for b in parent.findChildren(QPushButton) if b.text() == text)

    def capture(name):
        assert w.grab().save(str(out / name))

    def active_camera_dialog():
        dialog = w.findChild(CameraDialog)
        click(button(dialog, "Apply look-at"))
        assert dialog.grab().save(str(out / "camera-setup-wizard.png"))
        click(dialog.buttons.button(QDialogButtonBox.StandardButton.Save))

    def wait_idle():
        wait(
            lambda: not w.initial_model_check_pending
            and not w.runner.busy
            and not w.motion.runner.busy
        )

    try:
        wait_idle()
        assert w.docker_status.available, w.docker_status.diagnostics
        w.project, w.model, _ = pick_place_demo(w.model_manager)
        w.apply_project()
        wait_idle()
        installation = IsaacSimInstallationManager().detect()[0]
        profile = w.backend_manager.add(
            new_profile(
                "isaac_sim",
                distribution="humble",
                network_confirmed=True,
                simulation_confirmed=True,
                read_only=False,
                execution_enabled=True,
                connection_mode="local_installation",
                discovery="localhost",
            )
        )
        header = w.simulation_header
        choose(header.runtime, header.runtime.findData(profile.id))
        header.save_local(
            profile.id, dict(installation_path=str(installation.path), headless=False)
        )
        choose_scene_file(header, source)
        QTimer.singleShot(250, w, active_camera_dialog)
        with patch.object(QFileDialog, "getSaveFileName", return_value=(str(scene_path), "USD")):
            button(w.sensor_panel, "Add Camera to Isaac Sim…").click()
        wait(
            lambda: w.vision.profile() and scene_path.is_file() and not w.isaac_processes.processes
        )
        save_project(w.collect(), project_path)
        print("Derived USD authored via Studio", flush=True)
        close_window()
        w = open_window()
        wait_idle()
        w.open_path(project_path, confirmed=True)
        wait(
            lambda: w.vision.profile()
            and w.vision.profile().backend_id == profile.id
            and not w.runner.busy
        )
        header = w.simulation_header
        assert header.scene.currentData() == str(scene_path), header.scene.currentData()
        assert not header.auto_play.isChecked()
        click(header.start)
        wait(lambda: profile.id in w.isaac_processes.ready_reports)
        wait(
            lambda: profile.id in w.backend_manager.sessions
            and w.backend_manager.sessions[profile.id].transport is not None
            and w.backend_manager.sessions[profile.id].transport.connected
        )
        session = w.backend_manager.sessions[profile.id]
        scene = w.isaac_processes.ready_reports[profile.id]
        sdk_pid = scene["simulator_process_id"]
        vision = w.vision
        vision.open_view()
        click(button(vision.panel, "Start Stream"))
        QTest.qWait(1800)
        assert vision.latest is None and not session.clock_seen
        window_id = toolkit_space(app, sdk_pid, out / "isaac-native-paused.png")
        wait(
            lambda: w.ros_execution.ready()
            and vision.latest
            and vision.latest.metadata["transforms"].get("base_link")
        )
        wait(lambda: vision.profile().rgb_topic in session.graph["topics"])
        capture("rgb-live.png")
        snapshot = out / "rgb-snapshot.png"
        with patch.object(QFileDialog, "getSaveFileName", return_value=(str(snapshot), "PNG")):
            click(button(vision.panel, "Snapshot…"))
        saved = QImage(str(snapshot))
        assert (saved.width(), saved.height()) == (640, 480)
        print("Native Play, discovered RGB, CameraInfo, TF2 and rendered image", flush=True)
        choose(vision.detector_choice, vision.detector_choice.findData("hsv"))
        click(button(vision.panel, "Start Detection"))
        wait(lambda: vision.results.detections and vision.estimate)
        frame = vision.results.frame
        hsv_inference_ms = vision.results.seconds * 1000
        camera = vision.profile()
        found = vision.results.detections[0].finding
        world = vision.results.estimate(camera, found, camera.world_frame)
        base = vision.results.estimate(camera, found, camera.base_frame)
        bottom = vision.results.estimate(camera, found, camera.world_frame, "bottom")
        truth = scene["rgb_cameras"][0]["evaluation_contact"]
        error = float(np.linalg.norm(np.asarray(world.position) - truth))
        assert error < 0.065, (world, truth, error)
        w.tabs.pages[vision.panel].ensureWidgetVisible(vision.position)
        capture("hsv-planar-position.png")

        def store_calibration():
            dialog = w.findChild(CalibrationDialog)
            click(dialog.store)
            assert dialog.grab().save(str(out / "camera-calibration.png"))
            click(dialog.buttons.button(QDialogButtonBox.StandardButton.Save))

        previous_receiver = vision.stream.thread
        wait(lambda: vision.calibration() is not None)
        QTimer.singleShot(200, w, store_calibration)
        click(button(vision.panel, "Calibrate…"))
        assert vision.profile().calibration is not None
        wait(lambda: not previous_receiver.is_alive())
        save_project(w.collect(), project_path)
        w.tabs.pages[vision.panel].ensureWidgetVisible(vision.status)
        click(button(vision.panel, "Start Stream"))
        click(button(vision.panel, "Start Detection"))
        wait(lambda: vision.estimate is not None)
        toolkit_space(app, sdk_pid, out / "isaac-camera-scene.png")
        wait(lambda: vision.latest is None and not vision.results.detections, timeout=15)
        toolkit_space(app, sdk_pid, None)
        wait(lambda: w.ros_execution.ready() and vision.estimate)
        w.activateWindow()
        w.raise_()
        drag_tab(w.tabs, vision.panel, w.tabs.group_for(w.robot_panel), "right")
        assert w.tabs.group_for(vision.panel) is not w.tabs.group_for(w.robot_panel)
        w.tabs.setCurrentIndex(0)
        assert vision.panel.isVisible() and w.robot_panel.isVisible()
        capture("camera-beside-robot.png")
        motion = w.motion
        w.robot_panel.detail_tabs.setCurrentWidget(motion.program_panel)
        motion.program_tabs.setCurrentIndex(1)
        w.tabs.pages[w.robot_panel].ensureWidgetVisible(motion.play_button)
        initial = w.state_manager.snapshot()
        q = {name: initial.positions[name] for name in w.model.default_configuration()}
        q["shoulder_pan"] += 0.10
        task = MotionTask(
            name="Vision verification: shoulder motion",
            steps=[
                TaskStep(
                    action="move_joint",
                    name="Shoulder pan",
                    joint_values=q,
                    model_signature=initial.model_signature,
                    duration=3.0,
                )
            ],
        )
        motion.tasks.append(task)
        motion.refresh_tasks(task.id)
        counts = Counter()
        heartbeat, latency, observed_fps = [], [], []
        last_frame = None

        def observe(topic, message):
            if topic == profile.state_topic and message.get("position"):
                counts["joint_states"] += 1

        def sample():
            nonlocal last_frame
            heartbeat.append(time.monotonic())
            sample_frame = vision.latest
            if sample_frame is not None and sample_frame is not last_frame:
                last_frame = sample_frame
                latency.append((time.time_ns() - sample_frame.metadata["received_wall_ns"]) / 1e6)
            if vision.rates[0] > 0:
                observed_fps.append(vision.rates)

        session.transport.message.connect(observe)
        heartbeat_timer = QTimer(w)
        heartbeat_timer.setInterval(25)
        heartbeat_timer.timeout.connect(sample)
        heartbeat_timer.start()
        QTest.qWait(500)
        click(motion.play_button)
        wait(lambda: w.ros_execution.status == "executing")
        begin_frames = vision.stream.count
        begin_detections = vision.detector.processed
        begin_states = counts["joint_states"]
        wait(lambda: w.ros_execution.status == "succeeded", timeout=45)
        during = dict(
            rgb_frames=vision.stream.count - begin_frames,
            detections=vision.detector.processed - begin_detections,
            joint_states=counts["joint_states"] - begin_states,
        )
        assert min(during.values()) > 1, during
        final = w.state_manager.snapshot()
        assert abs(final.positions["shoulder_pan"] - q["shoulder_pan"]) <= profile.goal_tolerance
        wait(lambda: w.monitor.robot_state.state.sequence >= final.sequence)
        assert w.monitor.robot_state.state.backend_id == profile.id
        QTest.qWait(4500)
        heartbeat_timer.stop()
        rss = next(
            line
            for line in Path("/proc/self/status").read_text().splitlines()
            if line.startswith("VmRSS:")
        )
        report = dict(
            platform=app.platformName(),
            version=str(installation.version),
            scene=str(scene_path),
            project=str(project_path),
            prim=scene["rgb_cameras"][0]["prim"],
            source_hashes={str(p.relative_to(source.parent)): h for p, h in hashes.items()},
            domain=profile.domain_id,
            window=window_id,
            resolution=[frame.width, frame.height],
            topics=[camera.rgb_topic, camera.info_topic],
            frames=[
                camera.camera_frame,
                camera.optical_frame,
                camera.world_frame,
                camera.base_frame,
            ],
            calibration=frame.metadata["camera_info"],
            plane=camera.plane.model_dump(),
            transforms=frame.metadata["transforms"],
            truth_world=truth,
            center_world=asdict(world),
            center_base=asdict(base),
            bottom_world=asdict(bottom),
            center_error_m=error,
            bottom_error_m=float(np.linalg.norm(np.asarray(bottom.position) - truth)),
            detection=asdict(found),
            hsv_inference_ms=hsv_inference_ms,
            calibration_saved=True,
            snapshot_verified=True,
            motion=during,
            shoulder_delta_rad=final.positions["shoulder_pan"] - initial.positions["shoulder_pan"],
            rates_median_fps=np.median(observed_fps, axis=0).tolist(),
            latency_ms=dict(
                p50=float(np.median(latency)),
                p95=float(np.percentile(latency, 95)),
                samples=len(latency),
            ),
            max_gui_heartbeat_gap_ms=max(np.diff(heartbeat)) * 1000,
            studio_rss=rss,
            stream_counters=vision.stream.poll()[3:],
            paused_and_cleared=True,
            graph_discovered=True,
        )
        print("Planar localization and concurrent SO-101 goal verified", flush=True)
        receiver, detector, container = (
            vision.stream.thread,
            vision.detector.thread,
            session.container_id,
        )
        toolkit_space(app, sdk_pid, None)
        wait(lambda: vision.latest is None, timeout=15)
        previous_epoch = vision.epoch
        first_expected_error = len(errors)
        command = [
            "docker",
            "exec",
            container,
            "bash",
            "-c",
            'source "/opt/ros/$ROS_DISTRO/setup.bash" && '
            "ros2 topic pub --once --wait-matching-subscriptions 2 /clock "
            'rosgraph_msgs/msg/Clock "{clock: {sec: 0, nanosec: 0}}"',
        ]
        reset = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        try:
            wait(lambda: reset.poll() is not None, timeout=18, cleanup=True)
            assert reset.returncode == 0, reset.communicate()[0]
            wait(
                lambda: vision.epoch > previous_epoch and session.requires_reconnect,
                timeout=10,
                cleanup=True,
            )
            assert (
                vision.latest is None and not vision.results.detections and vision.estimate is None
            )
            assert all(
                "clock reset" in str(error).lower() for error in errors[first_expected_error:]
            )
            del errors[first_expected_error:]
            report["clock_reset_cleared_vision_epoch"] = True
        finally:
            if reset.poll() is None:
                reset.terminate()
            reset.communicate(timeout=5)
        w.activateWindow()
        w.raise_()
        click(button(vision.panel, "Stop Stream"))
        wait(lambda: not receiver.is_alive() and not detector.is_alive())
        assert vision.view.image.isNull() and not vision.results.detections
        header.disconnect_selected()
        w.isaac_processes.stop(profile.id)
        wait(
            lambda: not w.isaac_processes.processes and profile.id not in w.backend_manager.sessions
        )
        assert not Path(f"/proc/{sdk_pid}").exists()
        assert subprocess.run(["docker", "inspect", container], capture_output=True).returncode != 0
        assert all(sha256(p.read_bytes()).hexdigest() == h for p, h in hashes.items())
        report["source_unchanged"] = True
        close_window()
        report["closed_without_owned_simulator_container_receiver_detector_or_timers"] = True
        (out / "verification.json").write_text(json.dumps(report, indent=2) + "\n")
    finally:
        if not w.closed:
            close_window()
        logs.close()


if __name__ == "__main__":
    main()
