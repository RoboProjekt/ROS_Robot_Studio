import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from ros_robot_studio.backends.so101_usb import JOG_SPEED, JOINT_NAMES, jog_joint, read_positions
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.usb_robot import USBJointDialog, calibration_files, port_access_error
from ros_robot_studio.projects.store import PROJECTS_DIR, load_project


def test_serial_read_uses_matching_calibration_and_preserves_torque(monkeypatch, tmp_path):
    bus = Mock(is_calibrated=True)
    bus.sync_read.return_value = dict.fromkeys(JOINT_NAMES, 123)
    factory = Mock(return_value=bus)
    monkeypatch.setitem(
        sys.modules,
        "lerobot.motors",
        SimpleNamespace(
            Motor=Mock(),
            MotorCalibration=SimpleNamespace,
            MotorNormMode=SimpleNamespace(DEGREES="deg", RANGE_0_100="pct"),
            SerialMotorsBus=factory,
        ),
    )
    path = tmp_path / "SO101.json"
    path.write_text(
        json.dumps(
            {
                name: {"id": index, "range_min": 0, "range_max": 4095}
                for index, name in enumerate(JOINT_NAMES, 1)
            }
        )
    )
    for calibrated in (True, False):
        bus.reset_mock()
        bus.is_calibrated = calibrated
        assert read_positions("/dev/ttyACM0", path)["calibrated"] is calibrated
        bus.connect.assert_called_once_with()
        bus.sync_read.assert_called_once_with("Present_Position", normalize=calibrated, num_retry=1)
        bus.disconnect.assert_called_once_with(disable_torque=False)
        bus.write.assert_not_called()
        bus.sync_write.assert_not_called()


@pytest.mark.parametrize("allow_jog", [False, True])
def test_usb_dialog_detects_calibration_and_reads_positions(
    qtbot, tmp_path, monkeypatch, allow_jog
):
    calibration = tmp_path / "robots/so_follower/SO101.json"
    calibration.parent.mkdir(parents=True)
    calibration.write_text("{}")
    monkeypatch.setenv("HF_LEROBOT_CALIBRATION", str(tmp_path))
    assert calibration_files() == [calibration]
    port = tmp_path / "ttyACM0"
    assert "disconnected" in port_access_error(str(port))
    port.touch()
    helper = tmp_path / "helper.py"
    sample = json.dumps({"calibrated": True, "positions": dict.fromkeys(JOINT_NAMES, 12.5)})
    arguments = tmp_path / "arguments.json"
    helper.write_text(
        "import sys, json\nfrom pathlib import Path\n"
        f"Path({str(arguments)!r}).write_text(json.dumps(sys.argv[1:]))\nprint({sample!r})\n"
    )
    monkeypatch.setattr("ros_robot_studio.backends.so101_usb.__file__", str(helper))
    settings = Settings(tmp_path / "settings.json")
    settings.set("lerobot_python", sys.executable)
    dialog = USBJointDialog(str(port), settings, allow_jog=allow_jog)
    qtbot.addWidget(dialog)
    assert dialog.calibration.currentData() == str(calibration)
    if allow_jog:
        assert not dialog.arm.isEnabled() and not dialog.upper.isEnabled()
    else:
        assert dialog.arm is None
    dialog.read.click()
    qtbot.waitUntil(lambda: dialog.read.isEnabled())
    assert dialog.positions["shoulder_pan"].text() == "12.50°"
    assert dialog.positions["gripper"].text() == "12.50%"
    assert json.loads(arguments.read_text()) == [str(port), str(calibration)]
    if allow_jog:
        assert dialog.arm.isEnabled() and not dialog.upper.isEnabled()
        dialog.arm.setChecked(True)
        dialog.joint.setCurrentText("wrist_flex")
        dialog.upper.click()
        qtbot.waitUntil(lambda: dialog.read.isEnabled())
        assert json.loads(arguments.read_text())[-4:] == [
            "--joint",
            "wrist_flex",
            "--direction",
            "1",
        ]
        assert dialog.motion_status.text() == "Step complete · torque released"
        dialog.calibration.setCurrentIndex(0)
        assert dialog.arm.isChecked() and not dialog.upper.isEnabled()


def test_live_usb_reads_keep_confirmation_and_stop_when_dialog_closes(qtbot, tmp_path, monkeypatch):
    port = tmp_path / "ttyACM0"
    port.touch()
    counter = tmp_path / "count"
    helper = tmp_path / "helper.py"
    helper.write_text(
        "import json, sys\nfrom pathlib import Path\n"
        "if '--joint' in sys.argv: sys.exit('Simulated step error')\n"
        f"counter = Path({str(counter)!r})\n"
        "count = int(counter.read_text()) + 1 if counter.exists() else 1\n"
        "counter.write_text(str(count))\n"
        f"print(json.dumps(dict(calibrated=True, positions=dict.fromkeys({JOINT_NAMES!r}, count))))\n"
    )
    monkeypatch.setattr("ros_robot_studio.backends.so101_usb.__file__", str(helper))
    settings = Settings(tmp_path / "settings.json")
    settings.set("lerobot_python", sys.executable)
    dialog = USBJointDialog(str(port), settings, allow_jog=True)
    qtbot.addWidget(dialog)
    dialog.poll.setInterval(70)
    dialog.show()
    qtbot.waitUntil(dialog.arm.isEnabled)
    first = dialog.positions["shoulder_pan"].text()
    dialog.arm.setChecked(True)
    qtbot.waitUntil(lambda: dialog.positions["shoulder_pan"].text() != first)
    assert dialog.arm.isChecked()

    port.unlink()
    qtbot.waitUntil(lambda: "disconnected" in dialog.status.text())
    assert dialog.arm.isChecked() and not dialog.upper.isEnabled()
    assert dialog.positions["shoulder_pan"].text() == "—"
    port.touch()
    qtbot.waitUntil(dialog.upper.isEnabled)

    dialog.upper.click()
    qtbot.waitUntil(lambda: "Simulated step error" in dialog.motion_status.text())
    qtbot.waitUntil(lambda: dialog.calibration_verified and dialog.read.isEnabled())
    assert dialog.arm.isChecked() and not dialog.upper.isEnabled()
    dialog.read.click()
    qtbot.waitUntil(dialog.upper.isEnabled)
    assert dialog.arm.isChecked()

    dialog.close()
    final_count = counter.read_text()
    qtbot.wait(180)
    assert not dialog.poll.isActive() and counter.read_text() == final_count


def test_manual_project_uses_the_offline_model_and_no_vision_or_ros_execution():
    project = load_project(PROJECTS_DIR / "SO-101-Real.studio.yaml")
    reference = load_project(PROJECTS_DIR.parent / "examples/so101/so101.studio.yaml")
    assert project.robot.urdf == reference.robot.urdf
    assert not project.sensors
    (profile,) = project.backends
    assert profile.id == project.active_backend
    assert profile.usb_joint_jog and profile.read_only and not profile.execution_enabled


def test_jog_only_writes_to_selected_motor_and_releases_torque():
    bus = Mock()
    bus.read.side_effect = [0, 0, 100, 0, 100, 105, 0]
    jog_joint(bus, "elbow_flex", 1, SimpleNamespace(range_min=0, range_max=105))
    assert [call.args for call in bus.write.call_args_list] == [
        ("Goal_Speed", "elbow_flex", JOG_SPEED),
        ("Goal_Position", "elbow_flex", 100),
        ("Torque_Enable", "elbow_flex", 1),
        ("Goal_Position", "elbow_flex", 105),
        ("Torque_Enable", "elbow_flex", 0),
        ("Goal_Speed", "elbow_flex", 0),
    ]


def test_jog_releases_torque_on_failed_target_write():
    bus = Mock()
    bus.read.side_effect = [0, 0, 100, 0, 100, 0]
    bus.write.side_effect = [None, None, None, OSError("No acknowledgement"), None, None]
    with pytest.raises(OSError, match="No acknowledgement"):
        jog_joint(bus, "wrist_roll", 1, SimpleNamespace(range_min=0, range_max=4095))
    assert bus.write.call_args_list[-2].args == ("Torque_Enable", "wrist_roll", 0)


@pytest.mark.parametrize(
    ("readings", "error"),
    [
        ([1], "position mode"),
        ([0, 1], "already has torque enabled"),
        ([0, 0, 105], "calibrated limit"),
        ([0, 0, 106], "outside"),
    ],
)
def test_jog_rejects_unsafe_initial_state_without_writing(readings, error):
    bus = Mock()
    bus.read.side_effect = readings
    with pytest.raises(ValueError, match=error):
        jog_joint(bus, "gripper", 1, SimpleNamespace(range_min=0, range_max=105))
    bus.write.assert_not_called()
