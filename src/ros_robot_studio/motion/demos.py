"""Validated demonstration data. Model-specific taught seeds are examples, never solver logic."""

from ros_robot_studio.core.models import Project, Robot
from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.motion.pick_place import PickPlaceGenerator, PickPlaceRequest
from ros_robot_studio.motion.positions import PositionManager
from ros_robot_studio.motion.schema import IKOptions, KinematicSettings, MotionSettings
from ros_robot_studio.projects.store import DATA
from ros_robot_studio.robots.geometry import load_robot


def pick_place_demo(models=None, prefer_so101=True, progress=lambda _: None):
    spec = models.spec("so101_follower") if models and prefer_so101 else None
    if spec and models.status(spec.id).state == "installed":
        model = load_robot(models.resolve(spec.reference), require_meshes=True)
        robot = Robot(
            profile=spec.plugin_id or "custom-urdf",
            model_id=spec.id,
            model_version=spec.model_version,
            source_revision=spec.source_revision,
            urdf=spec.reference,
            end_effector=spec.default_end_effector,
        )
        grippers = spec.gripper_joints
        # Reviewed example joint configurations, verified again through this exact URDF.
        pick = {"shoulder_pan": 0.2, "shoulder_lift": -0.4, "elbow_flex": 0.6, "wrist_flex": -0.2}
        place = dict(pick, shoulder_pan=-0.4, shoulder_lift=-0.3)
        name = "SO101_PickAndPlace_Demo"
    else:
        source = DATA / "examples" / "generic_pick_arm.urdf"
        model = load_robot(source)
        robot = Robot(profile="custom-urdf", urdf=str(source), end_effector="tool0")
        grippers = ["finger_slide"]
        pick = {"axis_x": -0.08, "axis_y": -0.05, "axis_z": 0.1}
        place = {"axis_x": 0.08, "axis_y": 0.06, "axis_z": 0.1}
        name = "Generic_PickAndPlace_Demo"
    engine = UniversalKinematicsEngine(model)
    chain = engine.create_chain(model.root, robot.end_effector, gripper_joints=grippers)
    current = engine.seed()
    pick = engine.seed(pick)
    place = engine.seed(place)
    positions = PositionManager()
    positions.add(positions.capture_joint("Pick", chain, pick))
    positions.add(positions.capture_joint("Place", chain, place))
    opened = {n: engine.bounds()[n][1] for n in grippers}
    closed = {n: engine.bounds()[n][0] for n in grippers}
    options = IKOptions(starts=4)
    settings = MotionSettings(speed_scale=0.4)
    request = PickPlaceRequest(
        "Pick and Place",
        positions.positions[0],
        positions.positions[1],
        opened,
        closed,
        model.root,
        0.015,
        0.02,
        0.015,
        gripper_confirmed=True,
    )
    created, task, plan = PickPlaceGenerator(chain, options).generate(
        request, current, positions, settings, progress=progress
    )
    for p in created:
        positions.add(p)
    robot.joint_positions = current
    config = KinematicSettings(
        model_signature=engine.signature,
        base_link=model.root,
        end_effector=chain.end_effector,
        active_joints=list(chain.active_joints),
        gripper_joints=list(grippers),
        confirmed=True,
        options=options,
    )
    return (
        Project(
            name=name,
            description="Validated offline joint-space example; virtual attachment, no contact or collision physics.",
            robot=robot,
            kinematics=config,
            positions=positions.positions,
            tasks=[task],
            motion=settings,
        ),
        model,
        plan,
    )
