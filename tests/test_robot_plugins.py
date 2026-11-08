import numpy as np
import pytest

from ros_robot_studio.compatibility.checker import check
from ros_robot_studio.plugins.registry import PluginManifest, PluginRegistry
from ros_robot_studio.projects.store import EXAMPLE_URDF, demo_project
from ros_robot_studio.robots.urdf import parse_urdf
from ros_robot_studio.runtime.catalog import catalog


def test_valid_urdf_and_kinematics():
    robot = parse_urdf(EXAMPLE_URDF)
    assert robot.root == "base_link"
    assert len(robot.joints) == 4
    before = robot.positions()["tool_link"]
    after = robot.positions({"shoulder": 0.8})["tool_link"]
    assert not np.allclose(before, after)
    assert len(robot.positions()) == len(robot.links)


@pytest.mark.parametrize(
    "text",
    [
        "<broken",
        "<foo/>",
        "<robot/>",
        '<robot><link name="a"/><link name="a"/></robot>',
        '<robot><link name="a"/><link name="b"/></robot>',
        '<robot><link name="a"/><joint name="x" type="fixed"><parent link="a"/><child link="b"/></joint></robot>',
        '<robot><link name="a"/><link name="b"/><joint name="x" type="revolute"><parent link="a"/><child link="b"/></joint></robot>',
        '<!DOCTYPE robot [<!ENTITY x SYSTEM "file:///etc/passwd">]><robot name="&x;"><link name="a"/></robot>',
        '<robot><link name="a"/><link name="b"/><link name="c"/><joint name="b" type="fixed"><parent link="b"/><child link="c"/></joint><joint name="c" type="fixed"><parent link="c"/><child link="b"/></joint></robot>',
    ],
)
def test_invalid_urdf(text):
    with pytest.raises(Exception):
        parse_urdf(text)


def test_plugins_and_compatibility():
    registry = PluginRegistry()
    assert not registry.errors
    assert registry.plugins["so101"].status == "implemented"
    assert "offline-viewer" in registry.plugins["so101"].capabilities
    assert registry.plugins["so101"].distributions == []
    project = demo_project()
    assert check(project, "jazzy", registry).compatible
    project.robot.profile = "so101"
    report = check(project, "jazzy", registry)
    assert not report.compatible
    assert "unverified" in report.text()
    assert project.default_ros == "humble"


def test_invalid_plugin_does_not_execute_code(tmp_path):
    (tmp_path / "evil.yaml").write_text(
        '!!python/object/apply:os.system ["touch /tmp/should-not-exist"]'
    )
    registry = PluginRegistry(tmp_path)
    assert len(registry.errors) == 1
    assert not registry.plugins
    with pytest.raises(ValueError):
        PluginManifest.model_validate({"id": "bad"})


def test_catalog():
    entries = catalog()
    assert list(entries) == ["humble", "jazzy", "lyrical"]
    assert entries["humble"].ubuntu == "22.04"
    assert entries["lyrical"].base_image == "ros:lyrical-ros-base-resolute"
