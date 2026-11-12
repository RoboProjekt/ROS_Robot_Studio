"""Exercise vision validation, diagnostics and local configuration through Qt controls."""

import json
from unittest.mock import patch

import numpy as np
import pytest
from PySide6.QtCore import Qt
from PySide6.QtWidgets import QDialog, QDialogButtonBox

from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.settings import Settings
from ros_robot_studio.gui.camera_dialogs import CalibrationDialog, CameraDialog
from ros_robot_studio.vision.detection import (
    DetectionModelManager,
    DetectorSettings,
    ObjectDetectionManager,
)
from ros_robot_studio.vision.geometry import Intrinsics
from ros_robot_studio.vision.profiles import CalibrationRecord, CameraMount, CameraProfile
from ros_robot_studio.vision.stream import RGBFrame


def test_camera_settings_edit_preserves_calibration_until_geometry_changes(qtbot):
    profile = new_profile("isaac_sim")
    camera = CameraProfile(
        name="Overhead", backend_id=profile.id, mount=CameraMount(position=(0, 0, 1))
    )
    record = CalibrationRecord(
        stamp_ns=10,
        optical_frame=camera.optical_frame,
        reference_frame=camera.world_frame,
        resolution=(640, 480),
        intrinsics=(480, 0, 320, 0, 480, 240, 0, 0, 1),
        distortion=(),
        distortion_model="plumb_bob",
        translation=(0, 0, 1),
        rotation_xyzw=(1, 0, 0, 0),
    )
    camera.calibration = record
    dialog = CameraDialog(None, {profile.id: profile}, profile.id, "base_link", camera)
    qtbot.addWidget(dialog)
    dialog.show()
    dialog.name.selectAll()
    qtbot.keyClicks(dialog.name, "Renamed")
    assert dialog.data().calibration == record
    dialog.xyz[2].setValue(0.9)
    qtbot.mouseClick(
        dialog.buttons.button(QDialogButtonBox.StandardButton.Save), Qt.MouseButton.LeftButton
    )
    assert dialog.result() == QDialog.DialogCode.Accepted
    assert dialog.result_data.name == "Renamed" and dialog.result_data.calibration is None


def test_calibration_dialog_requires_confirmation_and_persists_frame(qtbot):
    camera = CameraProfile(name="RGB", backend_id="test")
    dialog = CalibrationDialog(None, camera, None)
    qtbot.addWidget(dialog)
    dialog.show()
    save = dialog.buttons.button(QDialogButtonBox.StandardButton.Save)
    qtbot.mouseClick(save, Qt.MouseButton.LeftButton)
    assert "Confirm" in dialog.message.text()
    assert not dialog.store.isEnabled()
    dialog.point[2].setValue(-0.005)
    qtbot.mouseClick(dialog.confirmed, Qt.MouseButton.LeftButton)
    qtbot.mouseClick(save, Qt.MouseButton.LeftButton)
    assert dialog.result() == QDialog.DialogCode.Accepted
    plane, record = dialog.result_data
    assert record is None and plane.frame == "world" and plane.point == (0, 0, -0.005)


def test_unavailable_detector_and_local_model_registry(tmp_path):
    manager = ObjectDetectionManager()
    assert manager.availability("disabled") == "Disabled"
    assert "not configured" in manager.availability("custom")
    with patch("importlib.util.find_spec", return_value=None):
        assert manager.availability("yolo") == "Missing cv2, onnxruntime"
    settings = Settings(tmp_path / "settings.json")
    models = DetectionModelManager(settings)
    filename = tmp_path / "weights.onnx"
    with pytest.raises(ValueError, match="existing ONNX"):
        models.add(filename)
    filename.write_bytes(b"test fixture")
    assert models.add(filename) == filename
    models.add(filename)
    assert models.paths() == (str(filename),)
    assert json.loads(settings.path.read_text())["vision_onnx_models"] == [str(filename)]
    models.remove(filename)
    assert not models.paths() and filename.is_file()


@pytest.mark.parametrize("missing", [True, False])
def test_yolo_invalid_model_reports_failure_and_worker_exits(missing, tmp_path, qtbot):
    pytest.importorskip("cv2")
    pytest.importorskip("onnxruntime")
    filename = tmp_path / "invalid.onnx"
    if not missing:
        filename.write_bytes(b"invalid ONNX")
    manager = ObjectDetectionManager()
    camera = CameraProfile(name="RGB", backend_id="test")
    manager.start("yolo", DetectorSettings(), filename, camera)
    qtbot.waitUntil(lambda: not manager.thread.is_alive())
    result, error, processed, _ = manager.poll()
    assert result is None and processed == 0
    assert "existing YOLOv5n" in error if missing else "invalid.onnx" in error


def test_camera_info_checks_frame_distortion_and_crop():
    frame = RGBFrame(np.zeros((20, 20, 3), np.uint8), dict(frame_id="camera_optical"))
    info = dict(
        width=20,
        height=20,
        frame_id="camera_optical",
        k=[12, 0, 10, 0, 12, 10, 0, 0, 1],
        d=[],
        model="plumb_bob",
    )
    np.testing.assert_allclose(Intrinsics.from_info(info, frame).ray((10, 10)), [0, 0, 1])
    for updates, reason in [
        ({"frame_id": "camera_link"}, "optical frame"),
        ({"d": [0.25, 0.5]}, "distortion coefficients"),
        ({"roi": True}, "cropped"),
    ]:
        with pytest.raises(ValueError, match=reason):
            Intrinsics.from_info(info | updates, frame)
