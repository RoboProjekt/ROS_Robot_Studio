"""Versioned, distribution-neutral project contracts. No Qt, Docker or ROS imports."""

from __future__ import annotations

from typing import Any, Literal
from uuid import uuid4

from pydantic import Field, model_validator

from ros_robot_studio.backends.profiles import BackendProfile
from ros_robot_studio.core.validation import Model as Model
from ros_robot_studio.motion.schema import (
    KinematicSettings,
    MotionSettings,
    MotionTask,
    SavedPosition,
)
from ros_robot_studio.vision.profiles import CameraManager, CameraProfile

Distro = Literal["humble", "jazzy", "lyrical"]


class Transform(Model):
    kind: Literal["static", "dynamic"] = "static"
    translation: tuple[float, float, float] = (0, 0, 0)
    rotation: tuple[float, float, float] = (0, 0, 0)


class Sensor(Model):
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str = Field(min_length=1, max_length=120)
    kind: Literal[
        "RGB camera",
        "Depth camera",
        "Force / torque",
        "Pressure",
        "Motor current",
        "Estimated torque",
        "Joint encoder",
        "Generic numeric",
    ] = "Generic numeric"
    driver: str = "ros-topic"
    topic: str = "/studio/sine"
    parent_frame: str = "base_link"
    transform: Transform = Field(default_factory=Transform)
    quantity: str = "value"
    unit: str = ""
    sample_rate: float = Field(default=20, gt=0, le=10000)
    lower_limit: float | None = None
    upper_limit: float | None = None
    warning: float | None = None
    enabled: bool = True

    @model_validator(mode="after")
    def limits_valid(self) -> Sensor:
        if self.lower_limit is not None and self.upper_limit is not None:
            if self.lower_limit > self.upper_limit:
                raise ValueError("Lower sensor limit must not exceed upper limit")
        if not self.topic.startswith("/"):
            raise ValueError("ROS topic must start with /")
        return self


class SimulationAsset(Model):
    format: Literal["usd"] = "usd"
    path: str | None = None


class SimulationSettings(Model):
    backend: Literal["isaac_sim"] = "isaac_sim"
    status: Literal["planned"] = "planned"
    asset: SimulationAsset = Field(default_factory=SimulationAsset)
    joint_mapping: Literal["not_configured"] = "not_configured"


class Robot(Model):
    profile: str = "generic-arm"
    model_id: str | None = Field(default=None, pattern=r"^[a-z][a-z0-9_-]{1,64}$")
    model_version: str | None = None
    source_revision: str | None = Field(default=None, pattern=r"^[a-f0-9]{40}$")
    joint_positions: dict[str, float] = Field(default_factory=dict)
    joint_mapping_status: Literal["not_calibrated"] = "not_calibrated"
    simulation: SimulationSettings = Field(default_factory=SimulationSettings)
    urdf: str | None = None
    source_xacro: str | None = None
    hardware: str | None = None
    controller: str | None = None
    ik_solver: str | None = None
    end_effector: str = ""

    @model_validator(mode="after")
    def model_reference_complete(self):
        if self.model_id and (not self.model_version or not self.source_revision or not self.urdf):
            raise ValueError(
                "Catalog robots require model version, source revision and URDF reference"
            )
        return self


class Monitoring(Model):
    window_seconds: int = Field(default=30, ge=5, le=300)
    update_hz: int = Field(default=20, ge=1, le=30)
    fields: list[str] = Field(default_factory=lambda: ["data"])
    topic: str = "/joint_states"
    source: Literal["robot_state", "signals"] = "robot_state"
    robot_fields: list[str] = Field(default_factory=list)


class ExecutionConfig(Model):
    profile: Literal["managed_simulation", "external_simulation"] = "managed_simulation"
    external_adapter: Literal["follow_joint_trajectory", "joint_state_command"] = (
        "follow_joint_trajectory"
    )
    clock_topic: str = "/clock"
    tf_topic: str = "/tf"
    state_topic: str = "/joint_states"
    command_topic: str = "/joint_command"
    action: str = "/joint_trajectory_controller/follow_joint_trajectory"
    joint_mapping: dict[str, str] = Field(default_factory=dict)
    external_simulation_confirmed: bool = False
    use_sim_time: bool = False
    stale_timeout: float = Field(default=1.0, gt=0, le=10)
    goal_tolerance: float = Field(default=0.002, gt=0, le=0.1)
    start_tolerance: float = Field(default=0.01, gt=0, le=0.1)
    hardware_execution_enabled: Literal[False] = False

    @model_validator(mode="after")
    def valid_mapping(self):
        if len(self.joint_mapping.values()) != len(set(self.joint_mapping.values())):
            raise ValueError("Joint mapping must be one-to-one")
        for topic in (
            self.state_topic,
            self.command_topic,
            self.action,
            self.clock_topic,
            self.tf_topic,
        ):
            if not topic.startswith("/") or any(c.isspace() for c in topic):
                raise ValueError("ROS interfaces must be absolute names without spaces")
        return self


class RuntimeConfig(Model):
    port: int = Field(default=9090, ge=1024, le=65535)
    memory_mb: int = Field(default=2048, ge=512, le=32768)
    cpu_count: float = Field(default=2, ge=0.25, le=32)
    domain_id: int = Field(default=42, ge=0, le=232)
    middleware: Literal["rmw_fastrtps_cpp"] = "rmw_fastrtps_cpp"
    network: Literal["bridge", "host"] = "bridge"
    namespace: str = Field(
        default="", pattern=r"^(/?[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*)?$"
    )
    discovery: Literal["subnet", "localhost"] = "subnet"
    fastdds_udp: bool = True
    external_network_confirmed: bool = False
    overrides: dict[str, dict[str, Any]] = Field(default_factory=dict)


class Project(Model):
    backends: list[BackendProfile] = Field(default_factory=list)
    cameras: list[CameraProfile] = Field(default_factory=list)
    active_backend: str | None = None
    format_version: Literal[3] = 3
    kinematics: KinematicSettings = Field(default_factory=KinematicSettings)
    positions: list[SavedPosition] = Field(default_factory=list)
    tasks: list[MotionTask] = Field(default_factory=list)
    motion: MotionSettings = Field(default_factory=MotionSettings)
    execution_backend: Literal["offline", "ros2_simulation"] = "offline"
    execution: ExecutionConfig = Field(default_factory=ExecutionConfig)
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str = Field(default="Untitled robot", min_length=1, max_length=120)
    description: str = ""
    default_ros: Distro = "humble"
    robot: Robot = Field(default_factory=Robot)
    sensors: list[Sensor] = Field(default_factory=list)
    monitoring: Monitoring = Field(default_factory=Monitoring)
    runtime: RuntimeConfig = Field(default_factory=RuntimeConfig)

    @model_validator(mode="after")
    def unique_sensors(self) -> Project:
        backend_ids = [p.id for p in self.backends]
        if len(set(backend_ids)) != len(backend_ids):
            raise ValueError("Backend IDs must be unique")
        if self.active_backend is not None and self.active_backend not in backend_ids:
            raise ValueError("Active backend must reference an existing profile")
        for library, title in [
            (self.positions, "position"),
            (self.tasks, "task"),
            (self.cameras, "camera"),
        ]:
            ids = [item.id for item in library]
            if len(ids) != len(set(ids)):
                raise ValueError(f"{title} IDs must be unique")
        cameras = CameraManager()
        for camera in self.cameras:
            cameras.save(camera)
        ids = [s.id for s in self.sensors]
        if len(ids) != len(set(ids)):
            raise ValueError("Sensor IDs must be unique")
        return self
