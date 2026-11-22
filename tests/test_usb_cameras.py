from pathlib import Path

import pytest

from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.usb_cameras import USBCameraControls, USBVideoDevice


def test_usb_selection_handles_hotplug_and_keeps_left_right_roles(qtbot, tmp_path, monkeypatch):
    a = USBVideoDevice("/dev/v4l/by-path/a-video-index0", "Camera A", "1-1")
    b = USBVideoDevice("/dev/v4l/by-path/b-video-index0", "Camera B", "1-2")
    cameras = [a]
    monkeypatch.setattr("ros_robot_studio.gui.usb_cameras.usb_video_devices", lambda: list(cameras))
    settings = Settings(Path(tmp_path) / "settings.json")
    controls = USBCameraControls(settings)
    qtbot.addWidget(controls)
    controls.defaults = (a.path, b.path)
    controls.refresh()
    with pytest.raises(ValueError, match="1 USB camera"):
        controls.configuration()

    cameras.append(b)
    controls.model.setCurrentIndex(controls.model.findData("high"))
    assert controls.configuration() == ("stereo", (a.path, b.path), "high")
    controls.right.setCurrentIndex(controls.right.findData(a.path))
    controls.save()
    with pytest.raises(ValueError, match="different USB cameras"):
        controls.configuration()

    controls.mode_field.setCurrentIndex(controls.mode_field.findData("single"))
    controls.switch_mode()
    assert not controls.form.isRowVisible(controls.right)
    assert controls.configuration()[0:2] == ("single", (a.path,))
