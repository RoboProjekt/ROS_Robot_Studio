import sys
from types import SimpleNamespace

import pytest
from PySide6.QtCore import QProcess

from ros_robot_studio.backends.so101_usb import JOINT_NAMES, USBControlSession
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.usb_session import USBSessionDialog


class MotorBus:
    def __init__(self):
        self.calibration = dict.fromkeys(JOINT_NAMES, SimpleNamespace(range_min=0, range_max=4095))
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
