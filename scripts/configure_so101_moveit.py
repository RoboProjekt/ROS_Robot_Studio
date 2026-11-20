#!/usr/bin/env python3
"""Generate the isolated SO-101 MoveIt plan-only parameters from the catalog URDF."""

import argparse
from pathlib import Path
from xml.etree import ElementTree as ET

import yaml
from defusedxml import ElementTree as SafeET


def configure(urdf, directory):
    root = SafeET.parse(urdf).getroot()
    if root.get("name") != "so101_new_calib":
        raise ValueError("Use the SO-101 Follower catalog URDF.")
    for mesh in root.findall(".//mesh"):
        relative = Path(mesh.get("filename"))
        path = (urdf.parent / relative).resolve()
        if not path.is_relative_to(urdf.parent) or not path.is_file():
            raise ValueError(f"Cannot resolve catalog mesh {relative}")
        mesh.set("filename", "file:///studio_robot/" + str(path.relative_to(urdf.parent)))

    srdf = ET.Element("robot", {"name": root.get("name")})
    arm = ET.SubElement(srdf, "group", {"name": "arm"})
    ET.SubElement(arm, "chain", {"base_link": "base_link", "tip_link": "gripper_frame_link"})
    gripper = ET.SubElement(srdf, "group", {"name": "gripper"})
    ET.SubElement(gripper, "joint", {"name": "gripper"})
    for joint in root.findall("joint"):
        ET.SubElement(
            srdf,
            "disable_collisions",
            {
                "link1": joint.find("parent").get("link"),
                "link2": joint.find("child").get("link"),
                "reason": "Adjacent",
            },
        )
    # The frame is fixed to gripper_link and carries no collision mesh.
    ET.SubElement(
        srdf,
        "disable_collisions",
        {
            "link1": "wrist_link",
            "link2": "moving_jaw_so101_v1_link",
            "reason": "Adjacent assembly",
        },
    )
    params = {
        "robot_description": ET.tostring(root, encoding="unicode"),
        "robot_description_semantic": ET.tostring(srdf, encoding="unicode"),
        "robot_description_kinematics": {
            "arm": {
                "kinematics_solver": "kdl_kinematics_plugin/KDLKinematicsPlugin",
                "kinematics_solver_timeout": 0.15,
                "kinematics_solver_search_resolution": 0.005,
            },
        },
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "planner_configs": {"RRTConnect": {"type": "geometric::RRTConnect", "range": 0.0}},
            "arm": {"planner_configs": ["RRTConnect"]},
            "gripper": {"planner_configs": ["RRTConnect"]},
        },
        "allow_trajectory_execution": False,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
        "publish_robot_description_semantic": True,
    }
    directory.mkdir(parents=True, exist_ok=True)
    (directory / "moveit.yaml").write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    configure(args.urdf.resolve(), args.directory)
