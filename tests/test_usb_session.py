import json
import sys
from types import SimpleNamespace

import pytest

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
    session.arm()
    assert session.armed
    assert all(value == 1 for value in bus.values["Torque_Enable"].values())
    assert bus.values["Goal_Position"] == bus.values["Present_Position"]
    session.target("shoulder_pan", 2100)
    assert bus.values["Goal_Position"]["shoulder_pan"] == 2100
    assert all(value == 1 for value in bus.values["Torque_Enable"].values())
    with pytest.raises(ValueError, match="outside"):
        session.target("shoulder_pan", 9000)
    session.release()
    assert not session.armed and not session.speeds
    assert all(value == 0 for value in bus.values["Torque_Enable"].values())
    assert all(value == 0 for value in bus.values["Goal_Speed"].values())
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
        session.arm()
    assert not session.owned and not session.speeds
    assert all(value == 0 for value in bus.values["Torque_Enable"].values())


def test_usb_session_sends_targets_and_closes_gracefully(qtbot, tmp_path, monkeypatch):
    port = tmp_path / "ttyACM0"
    port.touch()
    helper = tmp_path / "stream.py"
    log = tmp_path / "commands.jsonl"
    helper.write_text(
        "import sys, json\nfrom pathlib import Path\n"
        f"names = {JOINT_NAMES!r}\n"
        "values = dict.fromkeys(names, 2048)\narmed = False\n"
        "def emit():\n"
        " print(json.dumps(dict(calibrated=True, positions=values, encoders=values, "
        "ranges=dict.fromkeys(names, [0,4095]), armed=armed)), flush=True)\n"
        "emit()\nfor line in sys.stdin:\n"
        f" with Path({str(log)!r}).open('a') as target: target.write(line)\n"
        " command=json.loads(line)\n"
        " if command['operation']=='close': break\n"
        " if command['operation']=='arm': armed=True\n"
        " if command['operation']=='disarm': armed=False\n"
        " if command['operation']=='target': values[command['joint']]=command['position']\n"
        " emit()\n"
    )
    monkeypatch.setattr("ros_robot_studio.backends.so101_usb.__file__", str(helper))
    settings = Settings(tmp_path / "settings.json")
    settings.set("lerobot_python", sys.executable)
    dialog = USBSessionDialog(str(port), settings, allow_jog=True)
    qtbot.addWidget(dialog)
    dialog.show()
    qtbot.waitUntil(dialog.arm.isEnabled)
    dialog.request_target("shoulder_pan", 2100)
    assert not dialog.can_command()
    dialog.arm.setChecked(True)
    qtbot.waitUntil(dialog.can_command)
    dialog.request_target("shoulder_pan", 2100)
    qtbot.waitUntil(lambda: dialog.encoders.get("shoulder_pan") == 2100)
    assert dialog.arm.isChecked() and dialog.armed
    dialog.arm.setChecked(False)
    qtbot.waitUntil(lambda: not dialog.armed)
    dialog.close()
    qtbot.waitUntil(lambda: not dialog.isVisible())
    operations = [json.loads(line)["operation"] for line in log.read_text().splitlines()]
    assert operations.count("target") == 1
    assert operations[-1] == "close"


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
    dialog.close()
    qtbot.waitUntil(lambda: not dialog.isVisible())
    assert stopped.exists()
