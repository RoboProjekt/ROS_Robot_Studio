import math
from pathlib import Path

import numpy as np
import pytest

from ros_robot_studio.core.models import Project
from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.motion.backend import OfflinePreviewBackend
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.pick_place import PickPlaceGenerator, PickPlaceRequest
from ros_robot_studio.motion.positions import (
    ManualPoseProvider,
    PositionManager,
    SavedPoseProvider,
    TFFramePoseProvider,
    VisionPoseProvider,
)
from ros_robot_studio.motion.program import TaskCompiler
from ros_robot_studio.motion.schema import (
    CartesianPose,
    MotionSettings,
    MotionTask,
    SavedPosition,
    TaskStep,
)
from ros_robot_studio.motion.trajectory import TrajectoryGenerator
from ros_robot_studio.projects.store import DATA, load_project, portable_copy, save_project


@pytest.fixture
def demo():
    return pick_place_demo(prefer_so101=False)


def test_cubic_peaks_and_bounds(demo):
    p, m, plan = demo
    e = plan.chain.engine
    q = e.seed()
    end = e.seed({"axis_x": 0.2, "axis_z": 0.2})
    t = TrajectoryGenerator(e).generate(q, end, MotionSettings(speed_scale=0.5))
    assert t.sample(0) == q and t.sample(t.duration) == end
    for joint in m.joints:
        if joint.name in q:
            assert t.peak_velocity(joint.name) <= joint.velocity * 0.5 + 1e-9
            assert t.peak_acceleration(joint.name) <= joint.acceleration + 1e-9
    assert t.sample(t.duration / 2)["axis_x"] == pytest.approx(0.1)
    with pytest.raises(ValueError):
        TrajectoryGenerator(e).generate(q, dict(end, axis_x=1))


def test_continuous_shortest_and_missing_limits():
    e = UniversalKinematicsEngine.load_robot(DATA / "examples/generic_six_axis.urdf")
    name = next(j.name for j in e.model.joints if j.kind == "continuous")
    q = e.seed({name: math.pi - 0.05})
    end = e.seed({name: -math.pi + 0.05})
    t = TrajectoryGenerator(e).generate(q, end)
    assert t.end[name] - t.start[name] == pytest.approx(0.1)
    for fraction in np.linspace(0, 1, 50):
        e.model.configuration(t.sample(fraction * t.duration))
    assert name in t.unchecked_accelerations


def test_positions_crud_binding_and_providers(demo):
    p, m, plan = demo
    chain = plan.chain
    library = PositionManager()
    q = chain.engine.seed()
    joint = library.capture_joint("Home", chain, q)
    library.add(joint)
    copy = library.duplicate(joint.id)
    assert copy.id != joint.id
    copy.name = "Other"
    library.add(copy)
    assert library.get(copy.id).name == "Other"
    cart = library.capture_cartesian("Pick", chain, q)
    library.add(cart)
    assert SavedPoseProvider(library).get_pose(cart.id) == cart.pose
    assert ManualPoseProvider({"target": cart.pose}).get_pose("target") == cart.pose
    for provider in (TFFramePoseProvider(), VisionPoseProvider()):
        with pytest.raises(NotImplementedError):
            provider.get_pose("target")
    foreign = joint.model_copy(update={"model_signature": "another-model"})
    with pytest.raises(ValueError):
        library.resolve_joint(foreign, chain, q)
    task = MotionTask(
        name="Referenced", steps=[TaskStep(action="move_joint", position_id=joint.id)]
    )
    with pytest.raises(ValueError):
        library.remove(joint.id, [task])
    library.remove(copy.id)
    assert len(library.positions) == 2


def test_complete_task_pause_stop_reset_cube(demo):
    p, m, plan = demo
    assert plan.valid and not plan.hardware_ready
    backend = OfflinePreviewBackend()
    backend.connect(m, plan.initial, plan.chain.gripper_joints)
    backend.execute(plan)
    close = next(step for step in plan.steps if step.attach)
    release = [step for step in plan.steps if step.release][-1]
    backend.advance(close.end_time + 0.01)
    attached = backend.read_state()
    assert attached.attached
    before = attached.object_transform.copy()
    backend.pause()
    backend.advance(2)
    np.testing.assert_allclose(backend.read_state().object_transform, before)
    backend.resume()
    backend.advance(release.end_time - backend.elapsed + 0.01)
    released = backend.read_state()
    assert not released.attached
    pick = plan.object_initial[:3, 3]
    assert np.linalg.norm(released.object_transform[:3, 3] - pick) > 0.1
    place = next(pose for pose in p.positions if pose.name.endswith(" · Place"))
    np.testing.assert_allclose(released.object_transform[:3, 3], place.pose.position, atol=0.0002)
    backend.advance(1000)
    assert backend.status == "finished"
    backend.reset()
    assert backend.elapsed == 0 and backend.joints == plan.initial
    backend.execute(plan)
    backend.advance(0.2)
    backend.abort()
    elapsed = backend.elapsed
    backend.advance(2)
    assert backend.elapsed == elapsed


def test_failed_waypoint_blocks_entire_execution(demo):
    p, m, plan = demo
    bad = SavedPosition(
        name="Unreachable",
        kind="cartesian",
        pose=CartesianPose(frame="base_link", end_effector="tool0", position=(99, 0, 0)),
    )
    library = PositionManager(p.positions)
    library.add(bad)
    task = MotionTask(
        name="Failure",
        steps=[
            TaskStep(action="wait", duration=0.1),
            TaskStep(action="move_cartesian", position_id=bad.id),
        ],
    )
    compiled = TaskCompiler(plan.chain, library).compile(task, plan.initial)
    assert not compiled.valid and 1 in compiled.errors
    backend = OfflinePreviewBackend()
    backend.connect(m, plan.initial)
    with pytest.raises(ValueError):
        backend.execute(compiled)
    with pytest.raises(ValueError):
        compiled.sample(0)


def test_relative_frames_home_and_type_mismatch(demo):
    p, m, plan = demo
    library = PositionManager(p.positions)
    task = MotionTask(
        name="Relative",
        steps=[
            TaskStep(action="move_relative", reference_frame="tool0", relative_xyz=(0.01, 0, 0.01)),
            TaskStep(action="home"),
        ],
    )
    compiled = TaskCompiler(plan.chain, library).compile(task, plan.initial)
    assert compiled.valid
    np.testing.assert_allclose(
        plan.chain.forward_kinematics(compiled.steps[0].trajectory.end)[:3, 3]
        - plan.chain.forward_kinematics(plan.initial)[:3, 3],
        [0.01, 0, 0.01],
        atol=0.0001,
    )
    task.steps[0].reference_frame = "camera"
    assert not TaskCompiler(plan.chain, library).compile(task, plan.initial).valid
    task.steps = [TaskStep(action="move_cartesian", position_id=p.positions[0].id)]
    assert not TaskCompiler(plan.chain, library).compile(task, plan.initial).valid


def test_wizard_rejects_bad_offsets_and_unconfirmed_gripper(demo):
    p, m, plan = demo
    library = PositionManager(p.positions)
    request = PickPlaceRequest(
        "Test",
        p.positions[0],
        p.positions[1],
        {"finger_slide": 0.04},
        {"finger_slide": 0},
        "base_link",
        0.01,
        0.02,
        0.01,
    )
    with pytest.raises(ValueError, match="Confirm"):
        PickPlaceGenerator(plan.chain).generate(request, plan.initial, library)
    request.gripper_confirmed = True
    request.lift = 10
    with pytest.raises(ValueError, match="Lift"):
        PickPlaceGenerator(plan.chain).generate(request, plan.initial, library)


def test_portable_roundtrip_and_migrations(demo, tmp_path):
    p, m, plan = demo
    destination = tmp_path / "demo.yaml"
    portable = portable_copy(p, None, destination)
    save_project(portable, destination)
    restored = load_project(destination)
    assert restored.tasks == p.tasks and restored.positions == p.positions
    assert not Path(restored.robot.urdf).is_absolute()
    assert (
        TaskCompiler(plan.chain, PositionManager(restored.positions))
        .compile(restored.tasks[0], restored.robot.joint_positions)
        .valid
    )
    for version in (1, 2):
        text = f"format_version: {version}\nname: Legacy\n"
        destination.write_text(text)
        assert load_project(destination).format_version == 3 and destination.read_text() == text
    destination.write_text("format_version: 99\nname: Future\n")
    with pytest.raises(ValueError):
        load_project(destination)
    with pytest.raises(ValueError):
        Project.model_validate(
            dict(
                name="Bad",
                positions=[
                    dict(
                        kind="cartesian",
                        name="x",
                        pose=dict(frame="base", end_effector="tool", position=[float("nan"), 0, 0]),
                    )
                ],
            )
        )


def test_original_so101_full_demo():
    from ros_robot_studio.robots.model_manager import RobotModelManager

    models = RobotModelManager(Path.home() / ".local/share/ros-robot-studio/robots")
    if models.status("so101_follower").state != "installed":
        pytest.skip("Pinned original model not installed")
    p, m, plan = pick_place_demo(models)
    assert p.name == "SO101_PickAndPlace_Demo" and plan.valid and len(plan.steps) == 13
    assert len(plan.chain.active_joints) == 5
    for step in plan.steps:
        for t in np.linspace(0, step.trajectory.duration, 10):
            m.configuration(step.trajectory.sample(t))


def test_mimic_velocity_limit_controls_driver_duration():
    from ros_robot_studio.robots.urdf import parse_urdf

    model = parse_urdf("""<robot name="coupled"><link name="a"/><link name="b"/><link name="c"/>
    <joint name="driver" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="1" velocity="1"/></joint>
    <joint name="finger" type="revolute"><parent link="b"/><child link="c"/><limit lower="-2" upper="2" velocity=".1" acceleration=".2"/><mimic joint="driver" multiplier="2"/></joint></robot>""")
    e = UniversalKinematicsEngine(model)
    trajectory = TrajectoryGenerator(e).generate(
        {"driver": 0}, {"driver": 0.5}, MotionSettings(speed_scale=1)
    )
    assert trajectory.duration >= 15
    assert trajectory.peak_velocity("driver") * 2 <= 0.1 + 1e-9
    assert trajectory.peak_acceleration("driver") * 2 <= 0.2 + 1e-9


@pytest.mark.parametrize("limit", ['velocity="0"', 'velocity="1" acceleration="0"'])
def test_zero_dynamic_limits_reject_movement(limit):
    from ros_robot_studio.robots.urdf import parse_urdf

    model = parse_urdf(
        f"""<robot><link name="a"/><link name="b"/><joint name="j" type="prismatic"><parent link="a"/><child link="b"/><limit lower="0" upper="1" {limit}/></joint></robot>"""
    )
    with pytest.raises(ValueError, match="zero"):
        TrajectoryGenerator(UniversalKinematicsEngine(model)).generate({"j": 0}, {"j": 0.1})


def test_missing_limits_are_demo_only():
    from ros_robot_studio.robots.urdf import parse_urdf

    model = parse_urdf(
        """<robot><link name="a"/><link name="b"/><joint name="j" type="continuous"><parent link="a"/><child link="b"/></joint></robot>"""
    )
    t = TrajectoryGenerator(UniversalKinematicsEngine(model)).generate({"j": 0}, {"j": 0.2})
    assert t.demo_limits == ("j",) and t.unchecked_accelerations == ("j",)
