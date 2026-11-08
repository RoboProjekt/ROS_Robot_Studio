"""Versioned, distribution-neutral project contracts. No Qt, Docker or ROS imports."""

from __future__ import annotations

from typing import Any, Literal
from uuid import uuid4

from pydantic import BaseModel, ConfigDict, Field, model_validator

Distro = Literal["humble", "jazzy", "lyrical"]


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", validate_assignment=True, allow_inf_nan=False)


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
    topic: str = "/studio/sine"


class RuntimeConfig(Model):
    port: int = Field(default=9090, ge=1024, le=65535)
    memory_mb: int = Field(default=2048, ge=512, le=32768)
    cpu_count: float = Field(default=2, ge=0.25, le=32)
    overrides: dict[str, dict[str, Any]] = Field(default_factory=dict)


class Project(Model):
    format_version: Literal[2] = 2
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
        ids = [s.id for s in self.sensors]
        if len(ids) != len(set(ids)):
            raise ValueError("Sensor IDs must be unique")
        return self
