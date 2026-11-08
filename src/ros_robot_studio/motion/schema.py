"""Distribution-independent, serializable teaching and motion program contracts."""

from typing import Literal
from uuid import uuid4

from pydantic import Field, model_validator

from ros_robot_studio.core.validation import Model

IKMode = Literal["position_only", "axis", "full_pose"]


class IKOptions(Model):
    position_tolerance: float = Field(default=0.0001, gt=0, le=0.01)
    orientation_tolerance: float = Field(default=0.001, gt=0, le=0.2)
    position_weight: float = Field(default=1.0, gt=0, le=1000)
    orientation_weight: float = Field(default=0.15, gt=0, le=1000)
    starts: int = Field(default=4, ge=1, le=16)
    seed: int = 42
    max_evaluations: int = Field(default=250, ge=1, le=2000)
    time_limit: float = Field(default=10, gt=0, le=120)


class KinematicSettings(Model):
    model_signature: str = ""
    base_link: str = ""
    end_effector: str = ""
    active_joints: list[str] = Field(default_factory=list)
    gripper_joints: list[str] = Field(default_factory=list)
    confirmed: bool = False
    solver: str = "auto"
    mode: IKMode = "position_only"
    options: IKOptions = Field(default_factory=IKOptions)


class CartesianPose(Model):
    frame: str = Field(min_length=1)
    end_effector: str = Field(min_length=1)
    position: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0, 0, 0)
    mode: IKMode = "position_only"
    axis: Literal["X", "Y", "Z"] = "Z"


class SavedPosition(Model):
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str = Field(min_length=1, max_length=100)
    kind: Literal["joint", "cartesian"]
    model_signature: str = ""
    joints: dict[str, float] = Field(default_factory=dict)
    pose: CartesianPose | None = None

    @model_validator(mode="after")
    def payload_valid(self):
        if self.kind == "joint" and (
            not self.model_signature or not self.joints or self.pose is not None
        ):
            raise ValueError(
                "Joint positions need a robot signature and joint values, not a cartesian payload"
            )
        if self.kind == "cartesian" and (self.pose is None or self.joints):
            raise ValueError("Cartesian positions need an explicit framed pose, not joint values")
        return self


class MotionSettings(Model):
    speed_scale: float = Field(default=0.25, gt=0, le=1)
    demo_revolute_velocity: float = Field(default=0.35, gt=0, le=5)
    demo_prismatic_velocity: float = Field(default=0.05, gt=0, le=1)
    acceleration_limits: dict[str, float] = Field(default_factory=dict)
    minimum_duration: float = Field(default=0.1, ge=0.01, le=60)

    @model_validator(mode="after")
    def positive_accelerations(self):
        if any(value <= 0 for value in self.acceleration_limits.values()):
            raise ValueError("Acceleration limits must be positive")
        return self


class TaskStep(Model):
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str = ""
    action: Literal[
        "move_joint",
        "move_cartesian",
        "open_gripper",
        "close_gripper",
        "wait",
        "move_relative",
        "home",
    ]
    position_id: str | None = None
    target_pose: CartesianPose | None = None
    joint_values: dict[str, float] = Field(default_factory=dict)
    model_signature: str = ""
    gripper_values: dict[str, float] = Field(default_factory=dict)
    duration: float = Field(default=0, ge=0, le=3600)
    relative_xyz: tuple[float, float, float] = (0, 0, 0)
    relative_rpy: tuple[float, float, float] = (0, 0, 0)
    reference_frame: str = ""
    attach_object: bool = False
    release_object: bool = False


class VirtualObject(Model):
    name: str = "Virtual cube"
    frame: str = Field(min_length=1)
    position: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0, 0, 0)
    size: float = Field(default=0.02, gt=0, le=1)


class MotionTask(Model):
    schema_version: Literal[1] = 1
    id: str = Field(default_factory=lambda: str(uuid4()))
    name: str = Field(min_length=1, max_length=100)
    steps: list[TaskStep] = Field(default_factory=list)
    home_position_id: str | None = None
    settings: MotionSettings = Field(default_factory=MotionSettings)
    virtual_object: VirtualObject | None = None
    extensions: dict[str, str] = Field(default_factory=dict)

    @model_validator(mode="after")
    def unique_steps(self):
        ids = [step.id for step in self.steps]
        if len(ids) != len(set(ids)):
            raise ValueError("Task step IDs must be unique")
        return self
