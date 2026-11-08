"""Portable simulation contract, with the original URDF copied verbatim into an owned runtime."""

from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.projects.store import EXAMPLE_URDF
from ros_robot_studio.robots.urdf import parse_urdf


def simulation_config(model=None, positions=None, execution=None, runtime=None):
    model = model or parse_urdf(EXAMPLE_URDF)
    if not model.source_path:
        raise ValueError("A source URDF is required for ROS simulation")
    values = positions if positions is not None else model.default_configuration()
    model.configuration(values)
    from ros_robot_studio.core.models import ExecutionConfig

    execution = execution or ExecutionConfig()
    from uuid import uuid4

    return dict(
        schema_version=1,
        session=uuid4().hex,
        profile=execution.profile,
        urdf=model.source_path.read_text(),
        initial=dict(values),
        model_signature=UniversalKinematicsEngine(model).signature,
        namespace=runtime.namespace if runtime else "",
        clock_topic=execution.clock_topic,
        tf_topic=execution.tf_topic,
        state_topic=execution.state_topic,
        action=execution.action,
        start_tolerance=execution.start_tolerance,
        joint_mapping=execution.joint_mapping,
        use_sim_time=execution.use_sim_time,
        external_adapter=execution.external_adapter,
        external_simulation_confirmed=execution.external_simulation_confirmed,
        goal_tolerance=execution.goal_tolerance,
        command_topic=execution.command_topic,
    )
