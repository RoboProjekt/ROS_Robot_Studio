"""Stereo and language planning controls in the Camera & Vision workspace."""

import time
from uuid import uuid4

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

from ros_robot_studio.gui.robot_view import RobotView
from ros_robot_studio.gui.workers import TaskRunner
from ros_robot_studio.vision.geometry import transform_matrix
from ros_robot_studio.vision.moveit import DEFAULT_CONTAINER, make_request, resolve_ik, run_plan
from ros_robot_studio.vision.orchestrator import (
    DEFAULT_MODEL,
    DEFAULT_URL,
    Proposal,
    chat,
    mock_reply,
    probe,
    target_geometry,
    validate_reply,
)
from ros_robot_studio.vision.stereo import (
    MAX_AGE_NS,
    WORK_FRAME,
    OpenCVStereo,
    StereoPairer,
    perceive,
    stereo_profiles,
)
from ros_robot_studio.vision.stream import CameraStream

SNAPSHOT_TIMEOUT = 120


def annotate(rgb, objects):
    import cv2

    result = rgb.copy()
    for obj in objects:
        x, y, width, height = obj["bbox"]
        color = (100, 255, 120) if obj["position_valid"] else (255, 210, 75)
        cv2.rectangle(result, (x, y), (x + width, y + height), color, 1)
        cv2.putText(
            result,
            obj["id"],
            (x, max(12, y - 4)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            color,
            1,
            cv2.LINE_AA,
        )
    return result


def table_height(frame, camera):
    transforms = frame.metadata["transforms"]
    if not {"so101_work_surface", "robot_base", camera.base_frame} <= transforms.keys():
        return None
    base = transform_matrix(*transforms["robot_base"])
    if not np.allclose(base, transform_matrix(*transforms[camera.base_frame]), atol=1e-5):
        return None
    matrix = base @ np.linalg.inv(transform_matrix(*transforms["so101_work_surface"]))
    if matrix[2, 2] < 0.99:
        return None
    return float(matrix[2, 3])


class StereoWorkspace(QWidget):
    def __init__(self, vision, estimator_factory=OpenCVStereo):
        super().__init__()
        self.vision = vision
        self.main = vision.main
        self.estimator_factory = estimator_factory
        self.processing = TaskRunner(self)
        self.requests = TaskRunner(self)
        self.pairer = StereoPairer()
        self.streams = ()
        self.retired = []
        self.session = None
        self.epochs = None
        self.latest = self.proposal = self.planned = None
        self.generation = 0
        self.paused = False
        self.build()
        self.timer = QTimer(self, interval=30)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    def build(self):
        layout = QVBoxLayout(self)
        form = QFormLayout()
        self.baseline = QDoubleSpinBox(minimum=40, maximum=120, value=80, suffix=" mm")
        self.distance = QDoubleSpinBox(minimum=30, maximum=60, value=45, suffix=" cm")
        form.addRow("Stereo baseline", self.baseline)
        form.addRow("Working distance", self.distance)
        layout.addLayout(form)
        self.register = QPushButton("Register Stereo Cameras", clicked=self.register_cameras)
        self.scene = QPushButton("Build Derived Isaac USD…", clicked=self.build_scene)
        self.toggle = QPushButton("Start Stereo", clicked=self.toggle_stream)
        self.pause = QPushButton("Pause / resume", clicked=self.pause_stream)
        actions = QHBoxLayout()
        for control in (self.register, self.scene, self.toggle, self.pause):
            actions.addWidget(control)
        layout.addLayout(actions)
        self.view = QComboBox()
        self.view.addItems(["Left", "Right", "Stereo / Disparity"])
        self.view.currentIndexChanged.connect(self.render)
        layout.addWidget(self.view)
        self.image = QLabel(alignment=Qt.AlignmentFlag.AlignCenter)
        self.image.setMinimumHeight(160)
        layout.addWidget(self.image, 1)
        self.status = QLabel("Connect an Isaac Humble backend in Domain 1.", wordWrap=True)
        self.status.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.status)
        self.objects = QLabel(wordWrap=True)
        self.objects.setTextFormat(Qt.TextFormat.PlainText)
        layout.addWidget(self.objects)
        layout.addWidget(QLabel("Work surface · +Y in robot_base (base_link) = left"))
        options = QFormLayout()
        self.url = QLineEdit(self.main.settings.get("stereo_ollama_url", DEFAULT_URL))
        self.model = QLineEdit(self.main.settings.get("stereo_ollama_model", DEFAULT_MODEL))
        self.container = QLineEdit(
            self.main.settings.get("stereo_moveit_container", DEFAULT_CONTAINER)
        )
        options.addRow("Ollama URL", self.url)
        options.addRow("Model", self.model)
        options.addRow("MoveIt container", self.container)
        layout.addLayout(options)
        self.task = QLineEdit(
            placeholderText="Place the red cube left of the blue bowl.", maxLength=2000
        )
        layout.addWidget(self.task)
        self.mode = QComboBox()
        self.mode.addItems(
            [
                "Ollama",
                "Mock · Success",
                "Mock · Ambiguous",
                "Mock · Unknown ID",
                "Mock · Invalid JSON",
            ]
        )
        check = QPushButton("Check Ollama", clicked=self.probe)
        ask = QPushButton("Interpret Task", clicked=self.ask)
        plan = QPushButton("Plan Only", clicked=self.plan)
        self.path = QPushButton("Preview Path…", clicked=self.preview_path)
        self.path.setEnabled(False)
        command_row = QHBoxLayout()
        for control in (self.mode, check, ask, plan, self.path):
            command_row.addWidget(control)
        layout.addLayout(command_row)
        self.preview = QPlainTextEdit(readOnly=True)
        self.preview.setMaximumHeight(135)
        layout.addWidget(self.preview)

    def backend(self, *, active=False):
        manager = self.main.backend_manager
        identity = manager.active_id if active else manager.selected_id
        profile = manager.profiles.get(identity)
        if not profile or (profile.backend_type, profile.distribution, profile.domain_id) != (
            "isaac_sim",
            "humble",
            1,
        ):
            raise ValueError("Select an Isaac Sim profile with Humble and Domain 1.")
        if self.main.project.robot.model_id != "so101_follower":
            raise ValueError("Load the SO-101 Follower catalog model.")
        return profile

    def register_cameras(self):
        try:
            profile = self.backend()
            if self.streams:
                self.toggle_stream()
            for camera in stereo_profiles(profile.id, self.main.model.root):
                existing = self.vision.cameras.profiles.get(camera.id)
                if not existing or existing.backend_id != profile.id:
                    self.vision.save_camera(camera)
            self.status.setText("Stereo topics registered for the selected backend.")
        except (ValueError, OSError) as exc:
            self.status.setText(str(exc))

    def build_scene(self):
        try:
            profile = self.backend()
            cameras = stereo_profiles(
                profile.id,
                self.main.model.root,
                self.baseline.value() / 1000,
                self.distance.value() / 100,
            )
            self.vision.author(cameras)
        except (ValueError, OSError) as exc:
            self.status.setText(str(exc))

    def toggle_stream(self):
        if self.streams:
            self.stop()
            return
        try:
            profile = self.backend(active=True)
            session = self.main.backend_manager.sessions.get(profile.id)
            if (
                not session
                or session.cancelled
                or not session.transport
                or not session.transport.connected
            ):
                raise ValueError("Connect and activate the Isaac ROS backend first.")
            pair = [
                self.vision.cameras.profiles.get(f"so101_stereo_{side}")
                for side in ("left", "right")
            ]
            if any(camera is None or camera.backend_id != profile.id for camera in pair):
                raise ValueError("Register both stereo cameras on this backend first.")
            self.estimator = self.estimator_factory()
            self.session = session
            self.streams = tuple(CameraStream(camera, session) for camera in pair)
            self.epochs = (0, 0)
            for stream in self.streams:
                stream.start()
            self.toggle.setText("Stop Stereo")
            self.status.setText("Waiting for timestamp-matched stereo frames.")
        except (ValueError, OSError, ImportError) as exc:
            self.status.setText(str(exc))

    def invalidate(self):
        self.generation += 1
        self.pairer.clear()
        self.latest = self.proposal = None
        self.clear_plan()
        self.image.clear()
        self.objects.clear()

    def stop(self):
        for stream in self.streams:
            stream.stop()
        self.retired.extend(stream.thread for stream in self.streams)
        self.streams = ()
        self.session = None
        self.paused = False
        self.invalidate()
        self.toggle.setText("Start Stereo")
        self.status.setText("Stereo stream stopped.")

    def close_ready(self):
        self.stop()
        self.retired = [thread for thread in self.retired if thread.is_alive()]
        return not (self.retired or self.processing.busy or self.requests.busy)

    def pause_stream(self):
        if self.streams:
            self.paused = not self.paused
            self.invalidate()
            self.status.setText(
                "Stereo paused." if self.paused else "Waiting for fresh stereo frames."
            )

    def tick(self):
        if not self.streams:
            return
        manager = self.main.backend_manager
        session = self.session
        if (
            manager.active_id != session.profile.id
            or manager.sessions.get(session.profile.id) is not session
            or session.cancelled
            or session.transport is None
            or not session.transport.connected
        ):
            self.stop()
            self.status.setText("Stereo backend changed or disconnected.")
            return
        readings = [stream.poll() for stream in self.streams]
        epochs = tuple(reading[2] for reading in readings)
        if epochs != self.epochs:
            self.invalidate()
            self.epochs = epochs
            return
        for stream, reading in zip(self.streams, readings, strict=True):
            if not stream.thread.is_alive():
                self.stop()
                self.status.setText(reading[1] or "Stereo receiver stopped.")
                return
        if self.paused:
            return
        now = time.time_ns()
        if self.latest and now - self.latest[2] > MAX_AGE_NS:
            self.invalidate()
            self.status.setText("Waiting for fresh stereo frames.")
        for side, reading in enumerate(readings):
            if reading[0] is not None:
                previous = self.pairer.last[side]
                stamp = reading[0].metadata["stamp_ns"]
                if previous is not None and stamp < previous:
                    self.invalidate()
                    return
                self.pairer.push(side, reading[0], now)
        if self.processing.busy:
            return
        pair = self.pairer.pair(now)
        if pair:
            estimator, camera, generation = self.estimator, self.streams[0].camera, self.generation
            receipt = min(frame.metadata["received_wall_ns"] for frame in pair)
            self.processing.run(
                lambda _: (perceive(estimator, *pair, "robot_base"), table_height(pair[0], camera)),
                lambda outcome: self.accept_frame(generation, outcome, receipt),
                lambda error: self.error(generation, error),
            )

    def error(self, generation, error):
        if self.generation == generation:
            self.status.setText(error)

    def accept_frame(self, generation, outcome, receipt):
        if generation == self.generation and time.time_ns() - receipt <= MAX_AGE_NS:
            result, height = outcome
            self.latest = result, height, receipt
            self.status.setText(
                result.depth_error or f"Paired stereo at {result.stamp_ns / 1e9:.3f} s"
            )
            self.objects.setText(
                " · ".join(
                    f"{obj['id']} {np.round(obj['xyz'], 3).tolist() if obj['position_valid'] else 'depth unavailable'}"
                    for obj in result.objects
                )
            )
            self.render()

    def render(self):
        if self.latest is None:
            return
        import cv2

        result = self.latest[0]
        index = self.view.currentIndex()
        if index == 2:
            raw = cv2.applyColorMap(
                np.uint8(np.clip(result.disparity * 3, 0, 255)), cv2.COLORMAP_TURBO
            )
            raw[result.disparity <= 0] = 0
            rgb = cv2.cvtColor(raw, cv2.COLOR_BGR2RGB)
        else:
            rgb = annotate(result.left, result.objects) if index == 0 else result.right
        image = QImage(
            rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format.Format_RGB888
        )
        self.image.setPixmap(
            QPixmap.fromImage(image).scaled(
                self.image.size(),
                Qt.AspectRatioMode.KeepAspectRatio,
                Qt.TransformationMode.FastTransformation,
            )
        )

    def capture(self):
        if self.latest is None or time.time_ns() - self.latest[2] > MAX_AGE_NS or self.paused:
            raise ValueError("A fresh stereo scene is required.")
        result, height, _ = self.latest
        return result, {
            "scene_id": uuid4().hex,
            "stamp_ns": result.stamp_ns,
            "frame": "robot_base",
            "unit": "m",
            "work_surface": WORK_FRAME,
            "table_z": height,
            "objects": result.objects,
        }

    def endpoint(self):
        url, model = self.url.text().strip(), self.model.text().strip()
        self.main.settings.set("stereo_ollama_url", url)
        self.main.settings.set("stereo_ollama_model", model)
        return url, model

    def probe(self):
        if self.requests.busy:
            return
        url, model = self.endpoint()
        self.requests.run(
            lambda _: probe(url, model),
            lambda result: self.preview.setPlainText(
                f"Ollama server {result[0]} · {model} {'available' if result[1] else 'missing'}"
            ),
            self.preview.setPlainText,
        )

    def ask(self):
        if self.requests.busy:
            return
        self.proposal = None
        self.clear_plan()
        try:
            result, snapshot = self.capture()
            task = self.task.text().strip()
            if not task:
                raise ValueError("Enter a task.")
            generation = self.generation
            mode = self.mode.currentText()
            url, model = self.endpoint()

            def run(_):
                if mode.startswith("Mock"):
                    raw = mock_reply(mode.split(" · ")[-1], snapshot)
                else:
                    import cv2

                    image = annotate(result.left, snapshot["objects"])
                    ok, jpeg = cv2.imencode(".jpg", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                    if not ok:
                        raise ValueError("Could not encode the scene image.")
                    raw = chat(url, model, task, jpeg.tobytes(), snapshot)
                return validate_reply(raw, snapshot, task)

            def accepted(reply):
                if generation != self.generation:
                    self.preview.setPlainText("Discarded response from a stale stereo session.")
                    return
                if not isinstance(reply, Proposal):
                    self.preview.setPlainText(f"[{mode}] {task}\n{reply.status}: {reply.message}")
                    return
                summary = (
                    f"[{mode}] {task}\n{reply.object_id} {reply.relation} {reply.reference_id}"
                )
                try:
                    geometry = target_geometry(reply, snapshot)
                    geometry["reference_id"] = reply.reference_id
                    self.proposal = generation, time.monotonic(), snapshot, geometry, summary
                    self.preview.setPlainText(
                        summary + f"\nDrop XYZ (m) {geometry['place_center']}"
                    )
                except ValueError as exc:
                    self.preview.setPlainText(summary + "\nPlanning blocked: " + str(exc))

            self.preview.setPlainText(f"[{mode}] Waiting for proposal…")
            self.requests.run(
                run, accepted, lambda error: self.preview.setPlainText(f"[{mode}] {error}")
            )
        except (ValueError, OSError) as exc:
            self.preview.setPlainText(str(exc))

    def live_state(self, snapshot):
        _, live = self.capture()
        current = {obj["id"]: obj for obj in live["objects"]}
        if set(current) != {obj["id"] for obj in snapshot["objects"]} or live["table_z"] is None:
            raise ValueError("Work-surface observations changed. Interpret the task again.")
        if abs(live["table_z"] - snapshot["table_z"]) > 0.005:
            raise ValueError("The work surface moved. Interpret the task again.")
        for obj in snapshot["objects"]:
            other = current.get(obj["id"])
            if (
                not other
                or other["position_valid"] != obj["position_valid"]
                or (
                    obj["position_valid"]
                    and np.linalg.norm(np.array(obj["xyz"]) - other["xyz"]) > 0.02
                )
            ):
                raise ValueError("Stereo objects changed. Interpret the task again.")
        profile = self.backend(active=True)
        state = self.main.state_manager.backend_snapshot(profile.id)
        if (
            not state
            or not state.valid
            or time.monotonic() - state.timestamp > profile.stale_timeout
        ):
            raise ValueError("A fresh measured SO-101 start state is required.")
        return profile, dict(state.positions)

    def plan(self):
        if self.requests.busy:
            return
        self.clear_plan()
        try:
            if not self.proposal:
                raise ValueError("Interpret a task with valid object positions first.")
            generation, when, snapshot, geometry, summary = self.proposal
            if generation != self.generation or time.monotonic() - when > SNAPSHOT_TIMEOUT:
                raise ValueError("Scene snapshot expired. Reinterpret the task.")
            profile, initial = self.live_state(snapshot)
            request = make_request(geometry, snapshot, initial, profile.domain_id)
            model = self.main.model
            name = self.container.text().strip()
            self.main.settings.set("stereo_moveit_container", name)

            def done(result):
                try:
                    if generation != self.generation or time.monotonic() - when > SNAPSHOT_TIMEOUT:
                        raise ValueError("The plan's scene is stale.")
                    _, current = self.live_state(snapshot)
                    if set(initial) != set(current) or any(
                        abs(initial[j] - current[j]) > 0.01 for j in initial
                    ):
                        raise ValueError("The robot moved after planning started.")
                    stages = [
                        f"{stage['name']} · {len(stage['points'])} points · "
                        f"{stage['points'][-1]['t']:.2f} s"
                        for stage in result["stages"]
                    ]
                    self.preview.setPlainText(
                        summary
                        + f"\nDrop XYZ (m) {geometry['place_center']}"
                        + "\nMoveIt Plan Only\n"
                        + "\n".join(stages)
                    )
                    self.planned = snapshot, initial, model, result["stages"]
                    self.path.setEnabled(True)
                except ValueError as exc:
                    self.preview.setPlainText(str(exc))

            self.preview.setPlainText(summary + "\nMoveIt Plan Only…")
            self.requests.run(
                lambda _: run_plan(name, resolve_ik(request, model)),
                done,
                lambda error: self.preview.setPlainText(summary + "\nMoveIt Plan Only: " + error),
            )
        except (ValueError, OSError) as exc:
            self.preview.setPlainText(str(exc))

    def clear_plan(self):
        self.planned = None
        self.path.setEnabled(False)

    def preview_path(self):
        if self.planned is None:
            return
        snapshot, initial, model, stages = self.planned
        try:
            _, state = self.live_state(snapshot)
            if set(state) != set(initial) or any(
                abs(state[j] - initial[j]) > 0.01 for j in initial
            ):
                raise ValueError("The plan start state changed. Plan again.")
        except ValueError as exc:
            self.clear_plan()
            self.preview.setPlainText(str(exc))
            return
        dialog = QDialog(self, windowTitle="MoveIt Plan Only · SO-101")
        layout = QVBoxLayout(dialog)
        robot = RobotView(dialog)
        robot.set_model(model)
        layout.addWidget(robot)
        caption = QLabel()
        layout.addWidget(caption)
        frames, joints = [], initial.copy()
        for stage in stages:
            for point in stage["points"]:
                joints.update(zip(stage["joint_names"], point["positions"], strict=True))
                frames.append((stage["name"], point["t"], joints.copy()))
        slider = QSlider(Qt.Orientation.Horizontal, minimum=0, maximum=len(frames) - 1)
        layout.addWidget(slider)

        def show_point(index):
            stage, seconds, positions = frames[index]
            caption.setText(f"{stage} · {seconds:.2f} s · {index + 1}/{len(frames)} samples")
            robot.set_joints(positions, measured=True)

        slider.valueChanged.connect(show_point)
        show_point(0)
        dialog.resize(620, 470)
        dialog.exec()
