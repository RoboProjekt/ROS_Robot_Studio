"""CSV teach-in and timed playback on the SO-101 USB position loop."""

import csv
import hashlib
import json
import math
from pathlib import Path
from queue import Empty, SimpleQueue
from threading import Event, Thread

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
HAND_TORQUE = 25
HAND_DEADBAND = 2
REPLAY_TOLERANCE = 12
CSV_COLUMNS = (
    "time_s",
    *(f"{name}_deg" for name in JOINT_NAMES[:-1]),
    "gripper_pct",
    *(f"{name}_ticks" for name in JOINT_NAMES),
    "calibration_sha256",
)


def calibration_key(calibration):
    data = [
        (motor.id, motor.drive_mode, motor.homing_offset, motor.range_min, motor.range_max)
        for motor in (calibration[name] for name in JOINT_NAMES)
    ]
    return hashlib.sha256(json.dumps(data).encode()).hexdigest()


class Recording:
    def __init__(self, path, calibration):
        self.path = Path(path)
        self.calibration = calibration
        self.key = calibration_key(calibration)
        self.start = None
        self.stream = self.path.open("x", encoding="utf-8", newline="")
        try:
            self.writer = csv.writer(self.stream)
            self.writer.writerow(CSV_COLUMNS)
        except BaseException:
            self.close()
            raise

    def append(self, now, encoders):
        if self.start is None:
            self.start = now
        angles = []
        for name in JOINT_NAMES[:-1]:
            limits = self.calibration[name]
            midpoint = (limits.range_min + limits.range_max) / 2
            angles.append((encoders[name] - midpoint) * 360 / 4095)
        gripper = self.calibration["gripper"]
        opening = (
            100
            * (encoders["gripper"] - gripper.range_min)
            / (gripper.range_max - gripper.range_min)
        )
        if gripper.drive_mode:
            opening = 100 - opening
        self.writer.writerow(
            (
                f"{now - self.start:.6f}",
                *angles,
                opening,
                *(encoders[name] for name in JOINT_NAMES),
                self.key,
            )
        )
        self.stream.flush()

    def close(self):
        self.stream.close()
        if self.start is None:
            self.path.unlink(missing_ok=True)


def load_frames(path, calibration, cancelled=None):
    """Validate the entire recording before any replay targets are sent."""
    key = calibration_key(calibration)
    frames = []
    last_time = -1.0
    with Path(path).open(encoding="utf-8", newline="") as stream:
        rows = csv.reader(stream)
        if tuple(next(rows, ())) != CSV_COLUMNS:
            raise ValueError("Expected a Studio SO-101 replay CSV.")
        for row in rows:
            if cancelled is not None and cancelled.is_set():
                return []
            if len(row) != len(CSV_COLUMNS) or row[-1] != key:
                raise ValueError("The recording requires its original SO-101 calibration.")
            try:
                elapsed = float(row[0])
                encoders = tuple(int(value) for value in row[-7:-1])
            except ValueError as exc:
                raise ValueError(f"Invalid replay value on CSV row {rows.line_num}.") from exc
            if not math.isfinite(elapsed) or elapsed <= last_time or (not frames and elapsed != 0):
                raise ValueError(f"Invalid replay timestamp on CSV row {rows.line_num}.")
            for name, value in zip(JOINT_NAMES, encoders, strict=True):
                limits = calibration[name]
                if not max(0, limits.range_min) <= value <= min(4095, limits.range_max):
                    raise ValueError(
                        f"{name} is outside calibrated travel on CSV row {rows.line_num}."
                    )
            frames.append((elapsed, encoders))
            last_time = elapsed
    if not frames:
        raise ValueError("The replay has no joint samples.")
    return frames


def pace_frames(frames, speed):
    """Stretch intervals that require more speed than the USB limit allows."""
    previous_time, previous_encoders = frames[0]
    playback_time = 0.0
    for index in range(1, len(frames)):
        elapsed, encoders = frames[index]
        distance = max(
            abs(value - prior) for value, prior in zip(encoders, previous_encoders, strict=True)
        )
        playback_time += max(elapsed - previous_time, distance / speed)
        frames[index] = playback_time, encoders
        previous_time, previous_encoders = elapsed, encoders


class USBMotion:
    def __init__(self, control):
        self.control = control
        self.recording = None
        self.frames = None
        self.loading = None
        self.message = ""
        self.ack = None

    @property
    def mode(self):
        if self.recording:
            return "record"
        if self.frames:
            return "replay"
        return "idle"

    @property
    def active(self):
        return self.mode != "idle" or self.loading is not None

    def requested(self, command, encoders):
        operation = command["operation"]
        request_id = command["request_id"]
        if operation == "stop_motion":
            self.ack = request_id
            self.stop(encoders)
            return
        if not self.control.armed or self.active:
            self.ack = request_id
            self.message = "Enable motor control and stop the current activity first."
            return
        path = command["path"]
        speed, torque = command["speed"], command["torque"]
        calibration = self.control.bus.calibration
        try:
            self.control.validate_limits(speed, torque)
            if operation == "record":
                self.recording = Recording(path, calibration)
        except (OSError, ValueError) as exc:
            self.ack = request_id
            self.message = str(exc)
            return
        if operation == "replay":
            self.prepare_replay(path, calibration.copy(), request_id, speed, torque)
            return
        self.ack = request_id
        self.control.set_limits(speed, min(torque, HAND_TORQUE))
        self.goal = encoders.copy()
        self.control.target_positions(self.goal)
        self.message = "Recording… guide the arm by hand."

    def prepare_replay(self, path, calibration, request_id, speed, torque):
        results = SimpleQueue()
        cancelled = Event()

        def read():
            try:
                frames = load_frames(path, calibration, cancelled)
                if frames:
                    pace_frames(frames, speed)
                    results.put((frames, ""))
            except (OSError, ValueError, csv.Error) as exc:
                results.put((None, str(exc)))

        self.loading = (request_id, speed, torque, results, cancelled)
        Thread(target=read, daemon=True).start()

    def start_loaded_replay(self, encoders, now):
        request_id, speed, torque, results, _ = self.loading
        try:
            frames, error = results.get_nowait()
        except Empty:
            return
        self.loading = None
        self.ack = request_id
        if error:
            self.message = error
            return
        self.control.set_limits(speed, torque)
        self.frames = frames
        self.speed = speed
        self.index = 1
        self.started = None
        self.set_replay_goal(self.frames[0][1], encoders, now)
        self.message = "Moving to the recorded start pose…"

    def set_replay_goal(self, frame, encoders, now):
        self.goal = dict(zip(JOINT_NAMES, frame, strict=True))
        self.control.target_positions(self.goal)
        distance = self.distance(encoders)
        self.deadline = now + 3 + 2 * distance / self.speed

    def distance(self, encoders):
        return max(abs(self.goal[name] - encoders[name]) for name in JOINT_NAMES)

    def sample(self, sample, now):
        encoders = sample["encoders"]
        if self.loading:
            self.start_loaded_replay(encoders, now)
        elif self.recording:
            updated = {}
            for name in JOINT_NAMES:
                self.control.validate_target(name, encoders[name])
                if abs(encoders[name] - self.goal[name]) > HAND_DEADBAND:
                    updated[name] = encoders[name]
            self.control.target_positions(updated)
            self.goal.update(updated)
            self.recording.append(now, encoders)
        elif self.frames:
            self.replay_sample(encoders, now)

    def replay_sample(self, encoders, now):
        if self.started is None:
            if self.distance(encoders) <= REPLAY_TOLERANCE:
                self.started = now
                self.message = "Replaying…"
            elif now >= self.deadline:
                self.stop(encoders, "Start pose was not reached. Holding current position.")
            return

        frame = None
        while self.index < len(self.frames) and self.frames[self.index][0] <= now - self.started:
            frame = self.frames[self.index][1]
            self.index += 1
        if frame is not None:
            self.set_replay_goal(frame, encoders, now)
        if self.index == len(self.frames):
            if self.distance(encoders) <= REPLAY_TOLERANCE:
                self.clear("Replay complete. Holding the final pose.")
            elif now >= self.deadline:
                self.stop(encoders, "Final pose was not reached. Holding current position.")

    def stop(self, encoders, message=None):
        if self.recording:
            self.clear("Recording saved. Holding the last pose.")
        elif self.frames:
            self.control.target_positions(encoders)
            self.clear(message or "Replay stopped. Holding current position.")

    def clear(self, message=""):
        if self.loading:
            self.loading[-1].set()
            self.loading = None
        self.frames = None
        recording, self.recording = self.recording, None
        self.message = message
        if recording:
            recording.close()
