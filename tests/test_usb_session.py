import sys
import time
from types import SimpleNamespace

import pytest
from PySide6.QtCore import QProcess

from ros_robot_studio.backends.so101_replay import USBMotion, load_frames
from ros_robot_studio.backends.so101_usb import JOINT_NAMES, USBControlSession
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.usb_session import USBSessionDialog


class MotorBus:
    def __init__(self):
        self.calibration = {
            name: SimpleNamespace(
                id=index, drive_mode=0, homing_offset=0, range_min=0, range_max=4095
            )
            for index, name in enumerate(JOINT_NAMES, 1)
        }
        self.is_calibrated = True
        self.values = {
            "Present_Position": dict.fromkeys(JOINT_NAMES, 2048),
            "Torque_Enable": dict.fromkeys(JOINT_NAMES, 0),
            "Goal_Speed": dict.fromkeys(JOINT_NAMES, 0),
            "Torque_Limit": dict.fromkeys(JOINT_NAMES, 1000),
            "Max_Torque_Limit": dict.fromkeys(JOINT_NAMES, 1000),
            "Operating_Mode": dict.fromkeys(JOINT_NAMES, 0),
            "Goal_Position": {},
        }
        self.writes = []

    def sync_read(self, register, **_):
        return dict(self.values[register])

    def read(self, register, joint, **_):
        return self.values[register][joint]

    def write(self, register, joint, value, **_):
        self.writes.append((register, joint, value))
        self.values[register][joint] = value

    def sync_write(self, register, values, **_):
        for joint, value in values.items():
            self.write(register, joint, value)


def test_motor_session_holds_targets_until_disarmed_and_validates_limits():
    bus = MotorBus()
    session = USBControlSession(bus)
    session.arm(speed=60, torque=35)
    assert session.armed
    assert all(value == 1 for value in bus.values["Torque_Enable"].values())
    assert bus.values["Goal_Position"] == bus.values["Present_Position"]
    assert set(bus.values["Goal_Speed"].values()) == {60}
    assert set(bus.values["Torque_Limit"].values()) == {350}
    session.target("shoulder_pan", 2100)
    assert bus.values["Goal_Position"]["shoulder_pan"] == 2100
    assert all(value == 1 for value in bus.values["Torque_Enable"].values())
    with pytest.raises(ValueError, match="outside"):
        session.target("shoulder_pan", 9000)
    session.release()
    assert not session.armed and not session.limits_to_restore
    assert all(value == 0 for value in bus.values["Torque_Enable"].values())
    assert all(value == 0 for value in bus.values["Goal_Speed"].values())
    assert set(bus.values["Torque_Limit"].values()) == {1000}
    with pytest.raises(ValueError, match="Enable"):
        session.target("shoulder_pan", 2050)


def test_partial_arm_failure_releases_every_motor_it_attempted_to_enable(monkeypatch):
    bus = MotorBus()
    session = USBControlSession(bus)
    original = bus.write

    def fail(register, joint, value, **kwargs):
        original(register, joint, value, **kwargs)
        if (register, joint, value) == ("Torque_Enable", "elbow_flex", 1):
            raise OSError("Lost acknowledgement")

    monkeypatch.setattr(bus, "write", fail)
    with pytest.raises(OSError, match="acknowledgement"):
        session.arm(torque=40)
    assert not session.owned and not session.limits_to_restore
    assert all(value == 0 for value in bus.values["Torque_Enable"].values())
    assert set(bus.values["Torque_Limit"].values()) == {1000}


def test_live_limits_preserve_lower_motor_limits_and_restore_original_values():
    bus = MotorBus()
    bus.values["Max_Torque_Limit"]["gripper"] = 500
    bus.values["Torque_Limit"]["wrist_roll"] = 450
    bus.values["Goal_Speed"]["elbow_flex"] = 31
    original_torque = bus.values["Torque_Limit"].copy()
    original_speed = bus.values["Goal_Speed"].copy()
    session = USBControlSession(bus)
    with pytest.raises(ValueError, match="Enable motor control"):
        session.set_limits(60, 30)
    with pytest.raises(ValueError, match="Speed"):
        session.arm(speed=0)
    assert not bus.writes

    session.arm(torque=30)
    session.set_limits(speed=120, torque=70)
    assert set(bus.values["Goal_Speed"].values()) == {120}
    assert bus.values["Torque_Limit"]["shoulder_pan"] == 700
    assert bus.values["Torque_Limit"]["gripper"] == 500
    assert bus.values["Torque_Limit"]["wrist_roll"] == 450
    assert not any(register == "Max_Torque_Limit" for register, *_ in bus.writes)
    writes = len(bus.writes)
    session.set_limits(speed=120, torque=70)
    assert len(bus.writes) == writes
    with pytest.raises(ValueError, match="Maximum torque"):
        session.set_limits(120, 101)
    assert len(bus.writes) == writes
    session.release()
    assert bus.values["Goal_Speed"] == original_speed
    assert bus.values["Torque_Limit"] == original_torque


def test_usb_session_can_close_while_its_process_is_starting(qtbot, tmp_path, monkeypatch):
    port = tmp_path / "ttyACM0"
    port.touch()
    helper = tmp_path / "start.py"
    stopped = tmp_path / "stopped"
    helper.write_text(
        "import json, sys\nfrom pathlib import Path\n"
        "for line in sys.stdin:\n"
        " if json.loads(line)['operation']=='close':\n"
        f"  Path({str(stopped)!r}).touch()\n"
        "  break\n"
    )
    monkeypatch.setattr("ros_robot_studio.backends.so101_usb.__file__", str(helper))
    settings = Settings(tmp_path / "settings.json")
    settings.set("lerobot_python", sys.executable)
    dialog = USBSessionDialog(str(port), settings)
    qtbot.addWidget(dialog)
    dialog.show()
    dialog.shutdown()
    qtbot.waitUntil(stopped.exists)
    qtbot.waitUntil(lambda: dialog.process.state() == QProcess.ProcessState.NotRunning)


def test_hand_recording_holds_measured_positions_and_replays_with_csv_timing(tmp_path, qtbot):
    bus = MotorBus()
    control = USBControlSession(bus)
    control.arm(speed=120, torque=80)
    motion = USBMotion(control)
    path = tmp_path / "Demonstration.csv"
    current = bus.values["Present_Position"]

    def sample(now):
        motion.sample({"positions": dict.fromkeys(JOINT_NAMES, 0), "encoders": current}, now)

    motion.requested(
        dict(operation="record", request_id=1, path=path, speed=120, torque=80, record_torque=10),
        current,
    )
    assert motion.ack == 1 and motion.mode == "record"
    assert set(bus.values["Torque_Limit"].values()) == {100}
    sample(10.0)
    current["shoulder_pan"] = 2049
    sample(10.1)
    assert bus.values["Goal_Position"]["shoulder_pan"] == 2048
    current["shoulder_pan"] = 2098
    sample(10.2)
    sample(10.3)
    assert bus.values["Goal_Position"]["shoulder_pan"] == 2098
    motion.requested(dict(operation="stop_motion", request_id=2), current)
    frames = load_frames(path, bus.calibration)
    assert [elapsed for elapsed, _ in frames] == [0, 0.1, 0.2, 0.3]
    assert [positions[0] for _, positions in frames] == [2048, 2049, 2098, 2098]
    assert control.armed and set(bus.values["Torque_Limit"].values()) == {800}

    command = dict(operation="replay", request_id=3, path=path, speed=120, torque=75)
    motion.requested(command, current)

    def loaded():
        sample(20.0)
        return motion.ack == 3

    qtbot.waitUntil(loaded)
    assert motion.mode == "replay" and bus.values["Goal_Position"]["shoulder_pan"] == 2048
    assert set(bus.values["Torque_Limit"].values()) == {750}
    current["shoulder_pan"] = 2048
    sample(21.0)
    sample(21.15)
    assert motion.goal["shoulder_pan"] == 2049
    sample(21.25)
    assert motion.goal["shoulder_pan"] == 2049
    sample(21.55)
    assert motion.goal["shoulder_pan"] == 2098
    assert 2098 <= bus.values["Goal_Position"]["shoulder_pan"] <= 2120
    current["shoulder_pan"] = 2098
    sample(21.65)
    assert motion.mode == "idle" and control.armed

    bus.calibration["shoulder_pan"].homing_offset += 1
    motion.requested(dict(command, request_id=4), current)
    writes = len(bus.writes)

    def rejected():
        sample(time.monotonic())
        return motion.ack == 4

    qtbot.waitUntil(rejected)
    assert motion.mode == "idle" and "original" in motion.message
    assert len(bus.writes) == writes


@pytest.mark.parametrize("initially_armed", [False, True])
def test_zero_torque_recording_keeps_motors_off(tmp_path, initially_armed):
    bus = MotorBus()
    control = USBControlSession(bus)
    if initially_armed:
        control.arm(speed=120, torque=80)
    motion = USBMotion(control)
    path = tmp_path / "Free motion.csv"
    current = bus.values["Present_Position"]
    motion.requested(
        dict(operation="record", request_id=1, path=path, speed=120, torque=80, record_torque=0),
        current,
    )
    assert motion.mode == "record" and not control.armed
    assert set(bus.values["Torque_Enable"].values()) == {0}
    writes = len(bus.writes)
    motion.sample({"positions": current, "encoders": current}, 10)
    current["shoulder_pan"] = 2200
    motion.sample({"positions": current, "encoders": current}, 10.01)
    motion.stop(current)
    assert len(bus.writes) == writes and not control.armed and motion.mode == "idle"
    assert load_frames(path, bus.calibration)[-1][1][0] == 2200


def test_start_pose_grace_tracks_progress_then_reports_stalled_joint():
    bus = MotorBus()
    control = USBControlSession(bus)
    control.arm(speed=120, torque=45)
    motion = USBMotion(control)
    motion.speed = 120
    motion.frames = [(0.0, (2150, 2048, 2048, 2048, 2048, 2048))]
    motion.started = None
    current = bus.values["Present_Position"]
    motion.set_replay_goal(motion.frames[0][1], current, 0, confirm=True)
    current["shoulder_pan"] = 2098
    motion.replay_sample(current, 5.9)
    assert motion.mode == "replay" and motion.deadline > 11
    motion.replay_sample(current, 12)
    assert motion.mode == "idle" and control.armed
    assert "shoulder_pan, 4.57° (52 ticks) off at 45% torque" in motion.message
    assert bus.values["Goal_Position"]["shoulder_pan"] == 2098


@pytest.mark.parametrize("starting", [True, False])
@pytest.mark.parametrize("recovers", [True, False])
def test_replay_pid_corrects_test2_elbow_error_or_stops_at_the_deadline(starting, recovers):
    bus = MotorBus()
    current = bus.values["Present_Position"]
    current["elbow_flex"] = 1052
    control = USBControlSession(bus)
    control.arm(speed=24, torque=100)
    motion = USBMotion(control)
    motion.speed = 24
    motion.frames = [(0.0, (2048, 2048, 1023, 2048, 2048, 2048))]
    motion.started = None if starting else 0.0
    motion.index = 1
    motion.set_replay_goal(motion.frames[0][1], current, 0.0, confirm=True)
    motion.replay_sample(current, 0.05)
    assert motion.mode == "replay" and motion.goal["elbow_flex"] == 1023
    first_target = bus.values["Goal_Position"]["elbow_flex"]
    assert 1001 <= first_target < 1023
    assert set(bus.values["Torque_Limit"].values()) == {1000}
    assert set(bus.values["Goal_Speed"].values()) == {24}
    for step in range(2, 122):
        if recovers:
            # Emulate a loaded joint that settles 29 ticks above its commanded position.
            current["elbow_flex"] = bus.values["Goal_Position"]["elbow_flex"] + 29
        motion.replay_sample(current, step * 0.05)
        if motion.mode == "idle" or (starting and motion.started is not None):
            break
    if recovers:
        assert abs(current["elbow_flex"] - 1023) <= 22
        if starting:
            assert motion.started is not None and motion.message == "Replaying…"
        else:
            assert motion.mode == "idle" and "Replay complete" in motion.message
    else:
        assert motion.mode == "idle" and "2.55° (29 ticks)" in motion.message
        assert bus.values["Goal_Position"]["elbow_flex"] == 1052


def test_replay_does_not_correct_past_calibration_limit():
    bus = MotorBus()
    bus.calibration["elbow_flex"].range_min = 1023
    current = bus.values["Present_Position"]
    current["elbow_flex"] = 1052
    control = USBControlSession(bus)
    control.arm()
    motion = USBMotion(control)
    motion.speed = 24
    motion.frames = [(0.0, (2048, 2048, 1023, 2048, 2048, 2048))]
    motion.started = None
    motion.set_replay_goal(motion.frames[0][1], current, 0.0)
    motion.replay_sample(current, 0.5)
    assert bus.values["Goal_Position"]["elbow_flex"] == 1023
    motion.replay_sample(current, 6.0)
    assert motion.mode == "idle"
    assert bus.values["Goal_Position"]["elbow_flex"] == 1052
    assert all(
        value >= 1023
        for register, name, value in bus.writes
        if register == "Goal_Position" and name == "elbow_flex"
    )


def test_replay_pid_compensates_the_measured_shoulder_offset_at_start():
    bus = MotorBus()
    current = bus.values["Present_Position"]
    current["shoulder_lift"] = 3071
    control = USBControlSession(bus)
    control.arm(speed=60, torque=65)
    motion = USBMotion(control)
    motion.speed = 60
    frame = (2048, 3017, 2048, 2048, 2048, 2048)
    motion.frames = [(0.0, frame)]
    motion.started = None
    motion.index = 1
    motion.set_replay_goal(frame, current, 0.0, confirm=True)
    for step in range(1, 121):
        motion.replay_sample(current, step * 0.05)
        if motion.started is not None or motion.mode == "idle":
            break
        commanded = bus.values["Goal_Position"]["shoulder_lift"]
        assert 3017 - 45 <= commanded <= 3017
        # Reproduce an offset that leaves a 39-tick error with the old 22-tick limit.
        current["shoulder_lift"] = commanded + 61

    assert motion.started is not None
    assert abs(current["shoulder_lift"] - 3017) <= 22
    assert set(bus.values["Torque_Limit"].values()) == {650}
    assert set(bus.values["Goal_Speed"].values()) == {60}
