"""Built-in plugin host. Declarative manifests never execute third-party code."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

from ros_robot_studio.plugins.registry import PluginRegistry
from ros_robot_studio.runtime.robot_config import simulation_config


@dataclass(frozen=True)
class BackendCapabilities:
    available: frozenset[str]
    execution_permitted: bool
    reason: str


class ROSBackendPlugin(ABC):
    @abstractmethod
    def validate_installation(self, local_settings): ...

    @abstractmethod
    def get_connection_requirements(self, profile): ...

    @abstractmethod
    def get_execution_capabilities(self, profile, graph, state, controller, clock_seen): ...

    def configure_joint_mapping(self, profile, model):
        names = {j.name for j in model.joints if j.movable}
        mapping = profile.joint_mapping or {n: n for n in names}
        if set(mapping) != names or len(set(mapping.values())) != len(mapping):
            raise ValueError("Configure a complete, unique URDF → ROS joint mapping")
        return mapping

    def prepare_runtime(self, profile, model, positions, runtime):
        config = profile.execution_config()
        payload = simulation_config(model, positions, config, runtime)
        if profile.read_only or not profile.execution_enabled:
            payload["profile"] = "read_only"
        payload["backend_id"] = profile.id
        return payload

    def discover_robot_interfaces(self, graph):
        return dict(zip(graph.get("topics", []), graph.get("types", [])))


class BuiltinROSBackendPlugin(ROSBackendPlugin):
    def __init__(self, manifest):
        self.manifest = manifest
        self.backend_type = manifest.parameters["backend_type"]
        self.prepared = manifest.status != "implemented"
        self.simulation_execution = manifest.parameters["execution_policy"] == "simulation"

    def validate_installation(self, local_settings):
        if self.backend_type == "isaac_sim":
            from ros_robot_studio.backends.isaac import IsaacSimInstallationManager

            path = local_settings.get("installation_path", "")
            return IsaacSimInstallationManager().validate(path) if path else None
        return None

    def get_connection_requirements(self, profile):
        if self.prepared:
            raise ValueError(
                f"{self.manifest.name}: Not Implemented; use a configured Generic ROS backend for an existing ROS adapter"
            )
        if profile.distribution not in self.manifest.distributions:
            raise ValueError(f"{self.manifest.name} does not support {profile.distribution}")
        if profile.backend_type != "ros_demo" and profile.network != "host":
            raise ValueError(
                "External host/remote DDS requires the explicit host-network profile; private bridge mode is reserved for isolated ROS Demo"
            )
        if profile.network == "host" and not profile.network_confirmed:
            raise ValueError("Confirm host DDS networking before connecting")
        if profile.execution_enabled and (
            profile.read_only or not self.simulation_execution or not profile.simulation_confirmed
        ):
            raise ValueError(
                "Execution requires a permitted plugin and an explicitly confirmed simulation profile"
            )
        return ["Docker runtime", profile.middleware, f"ROS domain {profile.domain_id}"]

    def get_execution_capabilities(self, profile, graph, state, controller, clock_seen):
        topics = self.discover_robot_interfaces(graph)
        available = set()
        typed_state = topics.get(profile.state_topic) in (
            "sensor_msgs/msg/JointState",
            "sensor_msgs/JointState",
        )
        if state and state.valid and typed_state:
            available.add("joint_state_feedback")
            if state.efforts is not None:
                available.add("joint_effort_feedback")
        if topics.get(profile.tf_topic) in ("tf2_msgs/msg/TFMessage", "tf2_msgs/TFMessage"):
            available.add("tf")
        if clock_seen:
            available.add("simulation_clock")
        if any(t in ("sensor_msgs/msg/Image", "sensor_msgs/Image") for t in topics.values()):
            available.add("camera_topics")
        if any(
            t in ("geometry_msgs/msg/WrenchStamped", "geometry_msgs/WrenchStamped")
            for t in topics.values()
        ):
            available.add("force_torque_feedback")
        discovered_controller = topics.get(profile.action.rstrip("/") + "/_action/status") in (
            "action_msgs/msg/GoalStatusArray",
            "action_msgs/GoalStatusArray",
        )
        if controller or discovered_controller:
            available.add("trajectory_controller")
        permitted = bool(
            self.simulation_execution
            and not self.prepared
            and profile.backend_type != "real_robot"
            and profile.execution_enabled
            and not profile.read_only
            and profile.simulation_confirmed
            and typed_state
            and state
            and state.valid
            and controller
            and (not profile.use_sim_time or clock_seen)
        )
        if permitted:
            available.update(
                ("joint_position_control", "joint_trajectory_execution", "gripper_control")
            )
        reason = (
            "Simulation Ready"
            if permitted
            else "Read Only"
            if profile.read_only or profile.backend_type == "real_robot"
            else "Waiting for typed feedback, controller and configured clock"
        )
        return BackendCapabilities(frozenset(available), permitted, reason)


class BackendPluginRegistry:
    def __init__(self, registry=None):
        self.registry = registry or PluginRegistry()
        self.plugins = {
            m.parameters["backend_type"]: BuiltinROSBackendPlugin(m)
            for m in self.registry.by_kind("backend")
        }

    def get(self, backend_type):
        if backend_type not in self.plugins:
            raise ValueError("Unknown backend plugin: " + backend_type)
        return self.plugins[backend_type]
