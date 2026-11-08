"""Portable backend configuration. Machine paths live exclusively in local Settings."""

import json
from pathlib import Path, PurePosixPath
from typing import Literal
from uuid import uuid4

from pydantic import Field, model_validator

from ros_robot_studio.backends.domains import MAX_DOMAIN, validate_domain
from ros_robot_studio.core.validation import Model

BackendType = Literal["ros_demo", "isaac_sim", "real_robot", "mujoco", "gazebo", "generic_ros"]
DEFAULTS = json.loads(
    (Path(__file__).resolve().parents[1] / "data/backends/defaults.json").read_text()
)


class BackendProfile(Model):
    id: str = Field(
        default_factory=lambda: "backend_" + uuid4().hex, pattern=r"^[A-Za-z_][A-Za-z0-9_-]{0,79}$"
    )
    name: str = Field(min_length=1, max_length=120)
    backend_type: BackendType = "ros_demo"
    domain_id: int = Field(default=DEFAULTS["ros_demo"]["domain_id"], ge=0, le=MAX_DOMAIN)
    distribution: Literal["humble", "jazzy", "lyrical"] = "humble"
    middleware: Literal["rmw_fastrtps_cpp"] = "rmw_fastrtps_cpp"
    connection_mode: Literal["local_installation", "existing", "remote"] = "existing"
    network: Literal["bridge", "host"] = "bridge"
    network_confirmed: bool = False
    discovery: Literal["subnet", "localhost"] = "subnet"
    port: int = Field(default=0, ge=0, le=65535)
    enabled: bool = True
    execution_enabled: bool = False
    read_only: bool = True
    simulation_confirmed: bool = False
    shared_domain_confirmed: bool = False
    auto_start: bool = False
    robot_model_id: str | None = None
    model_revision: str = ""
    model_signature: str = ""
    joint_mapping: dict[str, str] = Field(default_factory=dict)
    state_topic: str = "/joint_states"
    command_topic: str = "/joint_command"
    action: str = "/joint_trajectory_controller/follow_joint_trajectory"
    clock_topic: str = "/clock"
    tf_topic: str = "/tf"
    controller_interface: Literal["follow_joint_trajectory", "joint_state_command"] = (
        "follow_joint_trajectory"
    )
    use_sim_time: bool = False
    stale_timeout: float = Field(default=1.0, gt=0, le=10)
    goal_tolerance: float = Field(default=0.002, gt=0, le=0.1)
    # Portable logical references; never local executable or installation paths.
    installation_ref: str = ""
    scene_ref: str = ""

    @model_validator(mode="after")
    def coherent(self):
        validate_domain(self.domain_id)
        if self.port and self.port < 1024:
            raise ValueError("Bridge port must be zero (automatic) or 1024..65535")
        if self.backend_type == "real_robot" and (not self.read_only or self.execution_enabled):
            raise ValueError("Real Robot is always read only; hardware execution is locked")
        if len(set(self.joint_mapping.values())) != len(self.joint_mapping):
            raise ValueError("Joint mapping must be one-to-one")
        if any(
            not k or not v or any(c.isspace() for c in k + v) for k, v in self.joint_mapping.items()
        ):
            raise ValueError("Joint mapping names must be nonempty without whitespace")
        for name in (
            self.state_topic,
            self.command_topic,
            self.action,
            self.clock_topic,
            self.tf_topic,
        ):
            if not name.startswith("/") or any(c.isspace() for c in name):
                raise ValueError("ROS interfaces must be absolute names without whitespace")
        for reference in (self.installation_ref, self.scene_ref):
            if reference and (
                PurePosixPath(reference).is_absolute()
                or ".." in PurePosixPath(reference).parts
                or "\\" in reference
                or ":" in reference
            ):
                raise ValueError("Backend references must be portable logical/relative references")
        return self

    def runtime_config(self, port):
        from ros_robot_studio.core.models import RuntimeConfig

        return RuntimeConfig(
            port=port,
            domain_id=self.domain_id,
            middleware=self.middleware,
            network=self.network,
            discovery=self.discovery,
            external_network_confirmed=self.network_confirmed,
        )

    def execution_config(self):
        from ros_robot_studio.core.models import ExecutionConfig

        return ExecutionConfig(
            profile="managed_simulation"
            if self.backend_type == "ros_demo"
            else "external_simulation",
            external_adapter=self.controller_interface,
            state_topic=self.state_topic,
            command_topic=self.command_topic,
            action=self.action,
            clock_topic=self.clock_topic,
            tf_topic=self.tf_topic,
            joint_mapping=self.joint_mapping,
            use_sim_time=self.use_sim_time,
            external_simulation_confirmed=self.simulation_confirmed,
            stale_timeout=self.stale_timeout,
            goal_tolerance=self.goal_tolerance,
        )


def new_profile(backend_type="ros_demo", **values):
    default = DEFAULTS[backend_type]
    simulation = backend_type == "ros_demo"
    data = dict(
        name=default["name"],
        backend_type=backend_type,
        domain_id=default["domain_id"],
        execution_enabled=simulation,
        read_only=not simulation,
        simulation_confirmed=simulation,
    )
    if backend_type == "isaac_sim":
        data.update(controller_interface="joint_state_command", use_sim_time=True, network="host")
    data.update(values)
    return BackendProfile(**data)
