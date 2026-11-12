"""Portable camera configuration bound to the project's existing backend IDs."""

from math import isfinite
from typing import Literal
from uuid import uuid4

from pydantic import Field, model_validator

from ros_robot_studio.core.validation import Model

Vec3 = tuple[float, float, float]


class WorkPlane(Model):
    name: str = Field(default="Work plane", min_length=1, max_length=100)
    frame: str = "world"
    point: Vec3
    normal: Vec3 = (0, 0, 1)
    bounds: tuple[Vec3, Vec3] | None = None

    @model_validator(mode="after")
    def validate_plane(self):
        if not self.frame or any(c.isspace() for c in self.frame):
            raise ValueError("Work plane needs a valid reference frame")
        if (
            not all(isfinite(v) for v in (*self.point, *self.normal))
            or max(abs(v) for v in self.normal) < 1e-9
        ):
            raise ValueError("Work plane needs a finite point and nonzero normal")
        if self.bounds and any(
            not (isfinite(lo) and isfinite(hi) and lo <= hi) for lo, hi in zip(*self.bounds)
        ):
            raise ValueError("Invalid work area bounds")
        return self


class CameraMount(Model):
    kind: Literal["world_fixed", "robot_base_fixed", "end_effector"] = "world_fixed"
    reference_frame: str = "world"
    position: Vec3
    usd_rpy_deg: Vec3 = (0, 0, 0)
    horizontal_fov: float = Field(default=70, gt=5, lt=160)
    demo_cube: bool = False

    @model_validator(mode="after")
    def finite_pose(self):
        if not all(isfinite(v) for v in (*self.position, *self.usd_rpy_deg)):
            raise ValueError("Camera placement must be finite")
        return self


class CalibrationRecord(Model):
    stamp_ns: int = Field(gt=0)
    optical_frame: str
    reference_frame: str
    resolution: tuple[int, int]
    intrinsics: tuple[float, ...] = Field(min_length=9, max_length=9)
    distortion: tuple[float, ...] = Field(max_length=16)
    distortion_model: str
    translation: Vec3
    rotation_xyzw: tuple[float, float, float, float]
    source: Literal["CameraInfo + TF2"] = "CameraInfo + TF2"


class CameraProfile(Model):
    id: str = Field(
        default_factory=lambda: "camera_" + uuid4().hex, pattern=r"^[A-Za-z_][A-Za-z0-9_]{0,79}$"
    )
    name: str = Field(min_length=1, max_length=100)
    backend_id: str = Field(min_length=1, max_length=80)
    rgb_topic: str = "/so101/camera/image_raw"
    info_topic: str = "/so101/camera/camera_info"
    encoding: Literal["raw", "compressed"] = "raw"
    camera_frame: str = "so101_camera_link"
    optical_frame: str = "so101_camera_optical_frame"
    world_frame: str = "world"
    base_frame: str = "base_link"
    width: int = Field(default=640, ge=16, le=1920)
    height: int = Field(default=480, ge=16, le=1080)
    fps: int = Field(default=20, ge=1, le=60)
    display_fps: int = Field(default=20, ge=1, le=60)
    mount: CameraMount | None = None
    plane: WorkPlane | None = None
    calibration: CalibrationRecord | None = None

    @model_validator(mode="after")
    def coherent(self):
        topics = (self.rgb_topic, self.info_topic)
        if len(set(topics)) != len(topics) or any(
            not topic.startswith("/") or len(topic) < 2 or any(c.isspace() for c in topic)
            for topic in topics
        ):
            raise ValueError("Use two distinct absolute ROS camera topics")
        frames = (self.camera_frame, self.optical_frame, self.world_frame, self.base_frame)
        if len(set(frames)) != len(frames) or any(
            not frame or frame.startswith("/") or any(c.isspace() for c in frame)
            for frame in frames
        ):
            raise ValueError("Use distinct TF frame names without a leading slash")
        return self


class CameraManager:
    def __init__(self):
        self.profiles = {}
        self.selected_id = None

    def load(self, profiles):
        self.profiles = {profile.id: profile for profile in profiles}
        self.selected_id = next(iter(self.profiles), None)

    def select(self, identity):
        if identity is not None and identity not in self.profiles:
            raise ValueError("Unknown camera")
        self.selected_id = identity

    def save(self, profile):
        for other in self.profiles.values():
            if other.id == profile.id or other.backend_id != profile.backend_id:
                continue
            if {other.rgb_topic, other.info_topic} & {profile.rgb_topic, profile.info_topic}:
                raise ValueError("Camera topics must be unique within this backend")
            if {other.camera_frame, other.optical_frame} & {
                profile.camera_frame,
                profile.optical_frame,
            }:
                raise ValueError("Camera frames must be unique within this backend")
        self.profiles[profile.id] = profile
        self.select(profile.id)

    def remove(self, identity):
        self.profiles.pop(identity)
        if self.selected_id == identity:
            self.selected_id = next(iter(self.profiles), None)

    @staticmethod
    def discover(graph):
        topics = dict(zip(graph.get("topics", ()), graph.get("types", ())))
        types = {
            "sensor_msgs/msg/Image": "raw",
            "sensor_msgs/Image": "raw",
            "sensor_msgs/msg/CompressedImage": "compressed",
            "sensor_msgs/CompressedImage": "compressed",
        }
        return {name: types[kind] for name, kind in topics.items() if kind in types}
