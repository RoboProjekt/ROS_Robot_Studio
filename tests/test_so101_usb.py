import json
import sys
from types import SimpleNamespace
from unittest.mock import Mock

from ros_robot_studio.backends.so101_usb import JOINT_NAMES, read_positions
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.usb_robot import USBJointDialog, calibration_files, port_access_error


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


def test_usb_dialog_detects_calibration_and_reads_positions(qtbot, tmp_path, monkeypatch):
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
    helper.write_text(f"print({sample!r})")
    monkeypatch.setattr("ros_robot_studio.backends.so101_usb.__file__", str(helper))
    settings = Settings(tmp_path / "settings.json")
    settings.set("lerobot_python", sys.executable)
    dialog = USBJointDialog(str(port), settings)
    qtbot.addWidget(dialog)
    assert dialog.calibration.currentData() == str(calibration)
    dialog.read.click()
    qtbot.waitUntil(lambda: dialog.read.isEnabled())
    assert dialog.positions["shoulder_pan"].text() == "12.50°"
    assert dialog.positions["gripper"].text() == "12.50%"
