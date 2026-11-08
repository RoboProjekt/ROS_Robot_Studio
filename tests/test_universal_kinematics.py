"""Hardware-independent acceptance tests, including exact FK residual validation."""

import math
import threading
from pathlib import Path

import numpy as np
import pytest

from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
from ros_robot_studio.kinematics.solvers import KinematicsSolverManager
from ros_robot_studio.motion.schema import CartesianPose, IKOptions
from ros_robot_studio.projects.store import DATA
from ros_robot_studio.robots.urdf import UnsupportedKinematicsError, parse_urdf


@pytest.fixture
def cartesian():
    engine = UniversalKinematicsEngine.load_robot(DATA / "examples/generic_pick_arm.urdf")
    return engine.create_chain("base_link", "tool0", gripper_joints=["finger_slide"])


@pytest.fixture
def six():
    engine = UniversalKinematicsEngine.load_robot(DATA / "examples/generic_six_axis.urdf")
    return engine.create_chain(engine.model.root, engine.model.links[-1])


def test_chain_and_fk(cartesian):
    assert cartesian.active_joints == ("axis_x", "axis_y", "axis_z")
    assert cartesian.analyze().max_position_rank == 3
    assert not cartesian.analyze().full_pose_available
    q = cartesian.engine.seed({"axis_x": 0.1, "axis_y": -0.05, "axis_z": 0.2})
    t = cartesian.forward_kinematics(q)
    np.testing.assert_allclose(t[:3, 3], [0.1, -0.05, 0.245])
    np.testing.assert_allclose(t, cartesian.model.transforms(q)["tool0"])
    assert set(cartesian.engine.end_effector_candidates) == {"tool0", "finger"}
    assert cartesian.engine.joints["axis_x"].velocity == 0.12
    assert cartesian.engine.joints["axis_x"].effort == 10
    assert cartesian.engine.joints["axis_x"].acceleration == 0.4


@pytest.mark.parametrize("mode", ["position_only", "axis"])
def test_exact_ik_and_deterministic_multistart(cartesian, mode):
    target = cartesian.forward_kinematics(
        cartesian.engine.seed({"axis_x": 0.1, "axis_y": -0.05, "axis_z": 0.2})
    )
    results = [cartesian.inverse_kinematics(target, mode=mode) for _ in range(2)]
    assert all(r.success for r in results)
    assert results[0].joint_positions == results[1].joint_positions
    assert results[0].best.position_error <= 0.0001
    np.testing.assert_allclose(
        cartesian.forward_kinematics(results[0].joint_positions)[:3, 3], target[:3, 3], atol=0.0001
    )
    cartesian.model.configuration(results[0].joint_positions)


def test_unreachable_cannot_be_applied(cartesian):
    t = np.eye(4)
    t[0, 3] = 100
    r = cartesian.inverse_kinematics(t)
    assert not r.success and r.joint_positions is None
    assert r.best.position_error > 99
    assert "Best residual" in r.message


def test_full_six_axis_orientation_and_singularity(six):
    assert six.analyze().full_pose_available
    q = six.engine.seed(
        {name: value for name, value in zip(six.active_joints, [0.3, -0.4, 0.6, 0.2, -0.5, 0.4])}
    )
    target = six.forward_kinematics(q)
    for mode in ("axis", "full_pose"):
        r = six.inverse_kinematics(
            target, six.engine.seed(), mode=mode, options=IKOptions(starts=3)
        )
        assert r.success, r.message
        actual = six.forward_kinematics(r.joint_positions)
        np.testing.assert_allclose(actual[:3, 3], target[:3, 3], atol=0.0001)
        assert r.best.orientation_error <= 0.001
    rank, cond, _ = six.conditioning(six.engine.seed())
    assert rank < 6 and math.isinf(cond)


def test_unsupported_full_and_failed_convergence(cartesian):
    target = cartesian.forward_kinematics(cartesian.engine.seed({"axis_x": 0.1, "axis_z": 0.1}))
    assert cartesian.inverse_kinematics(target, mode="full_pose").status == "unsupported"
    r = cartesian.inverse_kinematics(target, options=IKOptions(starts=1, max_evaluations=1))
    assert not r.success and r.joint_positions is None
    stop = threading.Event()
    stop.set()
    assert cartesian.inverse_kinematics(target, cancel=stop).status == "cancelled"


def test_frames_and_invalid_targets(cartesian):
    q = cartesian.engine.seed()
    pose = CartesianPose(frame="camera", end_effector="tool0", position=(0, 0, 0))
    assert not cartesian.inverse_kinematics(pose).success
    pose.frame = "tool0"
    np.testing.assert_allclose(cartesian.target_matrix(pose, q), cartesian.forward_kinematics(q))
    for value in (np.full((4, 4), np.nan), np.zeros((4, 4)), np.eye(3)):
        assert not cartesian.inverse_kinematics(value).success
    with pytest.raises(ValueError):
        cartesian.engine.create_chain("tool0", "base_link")
    with pytest.raises(ValueError):
        cartesian.engine.create_chain(
            "base_link", "tool0", active_joints=["finger_slide"], gripper_joints=["finger_slide"]
        )


def test_optional_solver_absence(cartesian, monkeypatch):
    import ros_robot_studio.kinematics.solvers as solvers

    monkeypatch.setattr(solvers.importlib.util, "find_spec", lambda name: None)
    assert KinematicsSolverManager().solve(cartesian, np.eye(4)).status == "unavailable"
    assert (
        KinematicsSolverManager().solve(cartesian, np.eye(4), solver="moveit2").status
        == "unsupported"
    )


def test_mimic_limits_and_origins():
    model = parse_urdf("""<robot name="coupled"><link name="base"/><link name="a"/><link name="b"/>
    <joint name="driver" type="revolute"><parent link="base"/><child link="a"/><origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" velocity="1" effort="2"/></joint>
    <joint name="coupled" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0 0 1"/><axis xyz="1 0 0"/><limit lower="-.5" upper=".5" velocity=".1" effort="2"/><mimic joint="driver" multiplier="-2" offset=".1"/></joint></robot>""")
    e = UniversalKinematicsEngine(model)
    chain = e.create_chain("base", "b")
    assert chain.active_joints == ("driver",)
    np.testing.assert_allclose(e.bounds()["driver"], [-0.2, 0.3])
    np.testing.assert_allclose(model.positions({"driver": 0})["b"], [1, 0, 1], atol=1e-9)
    assert model.configuration({"driver": 0.2})["coupled"] == pytest.approx(-0.3)
    with pytest.raises(ValueError):
        model.configuration({"driver": 1})


def test_unsupported_closed_and_floating():
    loop = """<robot name="loop"><link name="a"/><link name="b"/><joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint><joint name="k" type="fixed"><parent link="b"/><child link="a"/></joint></robot>"""
    with pytest.raises(UnsupportedKinematicsError):
        parse_urdf(loop)
    floating = parse_urdf(
        """<robot name="float"><link name="a"/><link name="b"/><joint name="j" type="floating"><parent link="a"/><child link="b"/></joint></robot>"""
    )
    with pytest.raises(UnsupportedKinematicsError):
        UniversalKinematicsEngine(floating).create_chain("a", "b")


def test_installed_original_so101():
    # Deliberately read only the existing reviewed user cache; no download or hardware.
    from ros_robot_studio.robots.model_manager import RobotModelManager

    models = RobotModelManager(Path.home() / ".local/share/ros-robot-studio/robots")
    spec = models.spec("so101_follower")
    if models.status(spec.id).state != "installed":
        pytest.skip("Original pinned SO-101 not installed")
    e = UniversalKinematicsEngine.load_robot(models.resolve(spec.reference))
    c = e.create_chain(e.model.root, spec.default_end_effector, gripper_joints=spec.gripper_joints)
    assert len(c.active_joints) == 5 and "gripper" not in c.active_joints
    assert c.analyze().max_pose_rank == 5
    target = c.forward_kinematics(
        e.seed({"shoulder_lift": -0.4, "elbow_flex": 0.5, "shoulder_pan": 0.3})
    )
    r = c.inverse_kinematics(target)
    assert r.success, r.message
    assert r.best.position_error < 0.0001
    e.model.configuration(r.joint_positions)
    assert c.inverse_kinematics(target, mode="full_pose").status == "unsupported"


def test_default_configuration_honors_offset_mimic_feasibility():
    model = parse_urdf("""<robot><link name="a"/><link name="b"/><link name="c"/>
    <joint name="driver" type="revolute"><parent link="a"/><child link="b"/><limit lower="-2" upper="2"/></joint>
    <joint name="coupled" type="revolute"><parent link="b"/><child link="c"/><limit lower="0" upper=".5"/><mimic joint="driver" multiplier="1" offset="1"/></joint></robot>""")
    assert model.default_configuration()["driver"] == -0.5
    assert model.configuration()["coupled"] == 0.5
    UniversalKinematicsEngine(model).create_chain("a", "c").analyze()
