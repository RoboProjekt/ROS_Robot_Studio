"""Replaceable, CPU-oriented RGB detectors and a one-frame inference worker."""

from __future__ import annotations

import ast
import importlib.util
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import numpy as np
from pydantic import Field, model_validator

from ros_robot_studio.core.validation import Model


class DetectorSettings(Model):
    fps: int = Field(default=10, ge=1, le=30)
    threshold: float = Field(default=0.25, ge=0.01, le=1)
    classes: list[str] = Field(default_factory=list)
    hsv: list[tuple[int, int, int, int, int, int]] = Field(
        default_factory=lambda: [(0, 12, 100, 255, 50, 255), (168, 179, 100, 255, 50, 255)],
        min_length=1,
        max_length=16,
    )
    area: float = Field(default=100, ge=1, le=2_073_600)
    aruco_dictionary: str = "DICT_4X4_50"

    @model_validator(mode="after")
    def valid_ranges(self):
        for hl, hh, sl, sh, vl, vh in self.hsv:
            if not (0 <= hl <= hh <= 179 and 0 <= sl <= sh <= 255 and 0 <= vl <= vh <= 255):
                raise ValueError("Invalid HSV interval. H is 0–179, S and V are 0–255.")
        return self


@dataclass(frozen=True)
class Finding:
    class_id: int
    label: str
    score: float | None
    box: tuple[float, float, float, float]
    score_kind: str
    corners: tuple = ()

    @property
    def center(self):
        x1, y1, x2, y2 = self.box
        return (x1 + x2) / 2, (y1 + y2) / 2


@dataclass(frozen=True)
class Detection:
    id: str
    detector_id: str
    camera_id: str
    backend_id: str
    stamp_ns: int
    finding: Finding
    valid: bool = True


class ObjectDetector(Protocol):
    def initialize(self, configuration: DetectorSettings, model: Path | None): ...
    def detect(self, image: np.ndarray) -> list[Finding]: ...
    def get_classes(self) -> tuple[str, ...]: ...
    def get_capabilities(self) -> tuple[str, ...]: ...
    def shutdown(self): ...


class HSVDetector:
    def initialize(self, configuration, model=None):
        import cv2

        cv2.setNumThreads(1)
        self.cv = cv2
        self.config = configuration

    def get_classes(self):
        return ("Color region",)

    def get_capabilities(self):
        return ("2D bounding box", "mask fill ratio")

    def detect(self, image):
        if not image.size:
            return []
        cv = self.cv
        hsv = cv.cvtColor(image, cv.COLOR_RGB2HSV)
        mask = np.zeros(image.shape[:2], np.uint8)
        for hl, hh, sl, sh, vl, vh in self.config.hsv:
            cv.bitwise_or(mask, cv.inRange(hsv, (hl, sl, vl), (hh, sh, vh)), dst=mask)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        result = []
        for contour in sorted(contours, key=cv.contourArea, reverse=True)[:100]:
            if cv.contourArea(contour) < self.config.area:
                break
            x, y, w, h = cv.boundingRect(contour)
            score = cv.countNonZero(mask[y : y + h, x : x + w]) / (w * h)
            result.append(Finding(0, "Color region", score, (x, y, x + w, y + h), "mask fill"))
        return result

    def shutdown(self):
        pass


ARUCO_DICTIONARIES = ("DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250", "DICT_APRILTAG_36h11")


class ArucoDetector:
    def initialize(self, configuration, model=None):
        import cv2

        cv2.setNumThreads(1)
        if not hasattr(cv2, "aruco"):
            raise ValueError("Install OpenCV contrib for ArUco")
        if configuration.aruco_dictionary not in ARUCO_DICTIONARIES:
            raise ValueError("Unknown ArUco dictionary")
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(getattr(aruco, configuration.aruco_dictionary))
        self.detector = aruco.ArucoDetector(dictionary, aruco.DetectorParameters())
        self.classes = tuple(f"Marker {i}" for i in range(len(dictionary.bytesList)))

    def get_classes(self):
        return self.classes

    def get_capabilities(self):
        return ("Marker ID", "2D corners")

    def detect(self, image):
        if not image.size:
            return []
        corners, ids, _ = self.detector.detectMarkers(image)
        if ids is None:
            return []
        results = []
        for points, identity in zip(corners[:100], ids.flat):
            points = points.reshape(4, 2)
            box = tuple(map(float, (*points.min(axis=0), *points.max(axis=0))))
            results.append(
                Finding(
                    int(identity),
                    f"Marker {identity}",
                    None,
                    box,
                    "none",
                    tuple((float(x), float(y)) for x, y in points),
                )
            )
        return results

    def shutdown(self):
        self.detector = None


class DetectionModelManager:
    """Local references only. Loading and validation are performed on the inference worker."""

    def __init__(self, settings):
        self.settings = settings

    def paths(self):
        return tuple(self.settings.get("vision_onnx_models", ()))

    def add(self, path):
        path = Path(path).expanduser().resolve()
        if (
            not path.is_file()
            or path.suffix.lower() != ".onnx"
            or not 0 < path.stat().st_size <= 128 * 1024 * 1024
        ):
            raise ValueError("Select an existing ONNX file no larger than 128 MiB")
        self.settings.set(
            "vision_onnx_models", list(dict.fromkeys([str(path), *self.paths()]))[:15]
        )
        return path

    def remove(self, path):
        self.settings.set(
            "vision_onnx_models", [value for value in self.paths() if value != str(path)]
        )


class YoloV5Detector:
    """YOLOv5n v7 raw 640x640, 80 classes, xywh with objectness and class probabilities."""

    def initialize(self, configuration, model):
        import cv2
        import onnxruntime as ort

        if model is None or not model.is_file():
            raise ValueError("Choose an existing YOLOv5n v7 ONNX model")
        cv2.setNumThreads(1)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        session = ort.InferenceSession(str(model), options, providers=["CPUExecutionProvider"])
        inputs, outputs = session.get_inputs(), session.get_outputs()
        if len(inputs) != 1 or len(outputs) != 1:
            raise ValueError("Expected a raw YOLOv5n v7 detection export with one input and output")
        tensor, output = inputs[0], outputs[0]
        if (
            tensor.shape != [1, 3, 640, 640]
            or output.shape != [1, 25200, 85]
            or (
                tensor.type not in ("tensor(float)", "tensor(float16)")
                or output.type != tensor.type
            )
        ):
            raise ValueError(
                "Unsupported ONNX contract. Expected YOLOv5n v7 640 RGB and raw [1,25200,85]"
            )
        try:
            names = ast.literal_eval(session.get_modelmeta().custom_metadata_map.get("names", ""))
            if isinstance(names, dict):
                names = [names[index] for index in range(80)]
            if len(names) != 80 or not all(isinstance(name, str) and name for name in names):
                raise ValueError
        except (ValueError, SyntaxError, KeyError, TypeError) as exc:
            raise ValueError(
                "The supported ONNX export must provide 80 class labels in names metadata"
            ) from exc
        self.session, self.tensor, self.labels, self.cv = session, tensor, tuple(names), cv2
        self.configuration = configuration
        self.dtype = np.float16 if tensor.type == "tensor(float16)" else np.float32

    def get_classes(self):
        return self.labels

    def get_capabilities(self):
        return ("YOLOv5n v7 COCO", "CPU", "class-aware NMS")

    def detect(self, image):
        if not image.size:
            return []
        h, w = image.shape[:2]
        scale = min(640 / h, 640 / w)
        scaled_w, scaled_h = round(w * scale), round(h * scale)
        x, y = (640 - scaled_w) // 2, (640 - scaled_h) // 2
        padded = np.full((640, 640, 3), 114, np.uint8)
        padded[y : y + scaled_h, x : x + scaled_w] = self.cv.resize(image, (scaled_w, scaled_h))
        data = np.ascontiguousarray(padded.transpose(2, 0, 1)[None], dtype=self.dtype) / self.dtype(
            255
        )
        raw = self.session.run(None, {self.tensor.name: data})[0][0].astype(np.float32)
        if raw.shape != (25200, 85) or not np.isfinite(raw).all():
            raise ValueError("Invalid YOLO output values")
        classes = raw[:, 5:].argmax(axis=1)
        scores = raw[:, 4] * raw[np.arange(len(raw)), 5 + classes]
        keep = scores >= self.configuration.threshold
        if self.configuration.classes:
            keep &= np.isin(
                classes,
                [i for i, name in enumerate(self.labels) if name in self.configuration.classes],
            )
        indexes = np.flatnonzero(keep)
        indexes = indexes[np.argsort(scores[indexes])[::-1][:2000]]
        result = []
        for class_id in np.unique(classes[indexes]):
            selected = indexes[classes[indexes] == class_id]
            center = raw[selected, :4].copy()
            center[:, :2] -= center[:, 2:] / 2
            valid = np.flatnonzero(np.all(center[:, 2:] > 0, axis=1))
            selected, center = selected[valid], center[valid]
            indices = self.cv.dnn.NMSBoxes(
                center.tolist(), scores[selected].tolist(), self.configuration.threshold, 0.45
            )
            for index in np.asarray(indices, dtype=int).reshape(-1):
                bx, by, bw, bh = center[index]
                x1, x2 = np.clip((np.array([bx, bx + bw]) - x) / scale, 0, w)
                y1, y2 = np.clip((np.array([by, by + bh]) - y) / scale, 0, h)
                if x2 > x1 and y2 > y1:
                    result.append(
                        Finding(
                            int(class_id),
                            self.labels[class_id],
                            float(scores[selected[index]]),
                            (float(x1), float(y1), float(x2), float(y2)),
                            "model score",
                        )
                    )
        return sorted(result, key=lambda item: item.score, reverse=True)[:100]

    def shutdown(self):
        self.session = None


DETECTORS = {
    "disabled": ("Disabled", None, ()),
    "hsv": ("Color / HSV Detection", HSVDetector, ("cv2",)),
    "aruco": ("ArUco Marker Detection", ArucoDetector, ("cv2",)),
    "yolo": ("YOLO ONNX", YoloV5Detector, ("cv2", "onnxruntime")),
    "custom": ("Custom Detector…", None, ()),
}


class ObjectDetectionManager:
    def __init__(self):
        self.condition = threading.Condition()
        self.pending = None
        self.result = None
        self.error = ""
        self.running = False
        self.thread = None
        self.drops = 0
        self.processed = 0
        self.classes = ()

    @staticmethod
    def availability(detector_id):
        _, implementation, dependencies = DETECTORS[detector_id]
        if not implementation:
            return (
                "Disabled" if detector_id == "disabled" else "Custom detectors are not configured"
            )
        missing = [name for name in dependencies if importlib.util.find_spec(name) is None]
        return "Missing " + ", ".join(missing) if missing else ""

    def start(self, detector_id, settings, model, camera):
        if self.thread and self.thread.is_alive():
            raise ValueError("Wait for the previous detector to stop")
        error = self.availability(detector_id)
        if error:
            raise ValueError(error)
        self.running, self.error, self.result, self.pending = True, "", None, None
        self.drops, self.processed, self.classes = 0, 0, ()
        self.thread = threading.Thread(
            target=self.run,
            args=(detector_id, settings, model, camera),
            daemon=True,
            name="Studio RGB inference",
        )
        self.thread.start()

    def submit(self, frame):
        with self.condition:
            if self.running:
                self.drops += self.pending is not None
                self.pending = frame
                self.condition.notify()

    def poll(self):
        with self.condition:
            result, self.result = self.result, None
            return result, self.error, self.processed, self.drops

    def stop(self):
        with self.condition:
            self.running, self.pending, self.result = False, None, None
            self.condition.notify()

    def run(self, detector_id, settings, model, camera):
        detector = DETECTORS[detector_id][1]()
        try:
            detector.initialize(settings, model)
            self.classes = detector.get_classes()
            next_inference = 0
            while True:
                with self.condition:
                    while self.running:
                        wait = next_inference - time.monotonic()
                        if self.pending is not None and wait <= 0:
                            break
                        self.condition.wait(max(wait, 0.001) if self.pending is not None else None)
                    if not self.running:
                        return
                    frame, self.pending = self.pending, None
                started = time.monotonic()
                findings = detector.detect(frame.rgb)
                timestamp = frame.metadata["stamp_ns"]
                detections = tuple(
                    Detection(
                        f"{frame.metadata['sequence']}:{index}",
                        detector_id,
                        camera.id,
                        camera.backend_id,
                        timestamp,
                        finding,
                    )
                    for index, finding in enumerate(findings)
                    if not settings.classes or finding.label in settings.classes
                )
                with self.condition:
                    if self.running:
                        self.processed += 1
                        self.result = (frame, detections, time.monotonic() - started)
                next_inference = started + 1 / settings.fps
        except Exception as exc:
            with self.condition:
                self.error = str(exc)
        finally:
            detector.shutdown()
            self.stop()
