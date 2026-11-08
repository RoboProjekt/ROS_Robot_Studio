import math

import numpy as np
import pytest

from ros_robot_studio.robots.joint_mapping import UncalibratedJointMapping
from ros_robot_studio.robots.urdf import origin_transform, parse_urdf, pose_rpy


def test_joint_origin_axis_visual_origin_and_scale_are_distinct():
    model = parse_urdf("""<robot><link name="base"/><link name="tool">
      <visual><origin xyz="0 1 0" rpy="0 0 1.5707963267948966"/>
      <geometry><mesh filename="part.stl" scale="2 3 4"/></geometry></visual></link>
      <joint name="roll" type="revolute"><parent link="base"/><child link="tool"/>
      <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/><axis xyz="1 0 0"/>
      <limit lower="-3.14" upper="3.14"/></joint></robot>""")
    transforms = model.transforms({"roll": math.pi / 2})
    transformed = model.visual_transform(model.visuals[0], transforms) @ np.array([1, 0, 0, 1])
    np.testing.assert_allclose(transformed, [1, 0, 3, 1], atol=1e-12)
    np.testing.assert_allclose(transforms["tool"][:3, 3], [1, 0, 0])
    np.testing.assert_allclose(
        transforms["tool"][:3, :3].T @ transforms["tool"][:3, :3], np.eye(3), atol=1e-12
    )


def test_fixed_zero_axis_is_valid_and_prismatic_axis_normalized():
    model = parse_urdf("""<robot><link name="a"/><link name="b"/><link name="c"/>
    <joint name="fixed" type="fixed"><parent link="a"/><child link="b"/><axis xyz="0 0 0"/><origin xyz="0 0 0.2"/></joint>
    <joint name="slide" type="prismatic"><parent link="b"/><child link="c"/><axis xyz="0 0 2"/><limit lower="0" upper="1"/></joint></robot>""")
    np.testing.assert_allclose(model.positions({"slide": 0.5})["c"], [0, 0, 0.7])
    with pytest.raises(ValueError, match="above URDF"):
        model.transforms({"slide": 2})


def test_mimic_joints_resolve_and_cannot_be_commanded_directly():
    model = parse_urdf("""<robot><link name="a"/><link name="b"/><link name="c"/>
    <joint name="main" type="revolute"><parent link="a"/><child link="b"/><limit lower="-2" upper="2"/></joint>
    <joint name="coupled" type="revolute"><parent link="b"/><child link="c"/><limit lower="-2" upper="2"/><mimic joint="main" multiplier="-0.5" offset="0.2"/></joint></robot>""")
    assert model.configuration({"main": 1})["coupled"] == pytest.approx(-0.3)
    with pytest.raises(ValueError, match="coupled"):
        model.configuration({"coupled": 0.2})
    with pytest.raises(ValueError, match="cyclic mimic"):
        parse_urdf(
            """<robot><link name="a"/><link name="b"/><joint name="x" type="continuous"><parent link="a"/><child link="b"/><mimic joint="x"/></joint></robot>"""
        )


@pytest.mark.parametrize(
    "rpy", [(0.2, -0.4, 0.7), (0.5, math.pi / 2, 0.3), (-0.4, -math.pi / 2, 0.2)]
)
def test_rpy_roundtrip_including_gimbal_lock(rpy):
    matrix = origin_transform((0.1, 0.2, 0.3), rpy)
    pose = pose_rpy(matrix)
    reconstructed = origin_transform(pose[:3], pose[3:])
    np.testing.assert_allclose(reconstructed, matrix, atol=1e-9)


def test_missing_nonfinite_and_unknown_joint_values_rejected():
    model = parse_urdf(
        '<robot><link name="a"/><link name="b"/><joint name="x" type="continuous"><parent link="a"/><child link="b"/></joint></robot>'
    )
    for values in [{"x": float("nan")}, {"not_in_urdf": 1}]:
        with pytest.raises(ValueError):
            model.transforms(values)


def test_uncalibrated_mapping_never_invents_a_conversion():
    mapping = UncalibratedJointMapping()
    assert not mapping.calibrated
    for method in (mapping.ros_to_urdf, mapping.hardware_to_urdf, mapping.urdf_to_hardware):
        with pytest.raises(NotImplementedError, match="not calibrated"):
            method("gripper", 50)
    with pytest.raises(NotImplementedError):
        mapping.gripper_to_hardware(50)
