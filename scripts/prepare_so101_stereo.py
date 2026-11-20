#!/usr/bin/env python3
"""Prepare a stereo scene request for the Isaac Sim 5.1 workstation."""

import argparse
import tempfile
from pathlib import Path

from ros_robot_studio.backends.isaac import IsaacSimSceneManager
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.robots.geometry import load_robot
from ros_robot_studio.vision.stereo import stereo_profiles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path, help="New final USD or USDA file")
    parser.add_argument("--source", type=Path, help="Existing SO-101 USD")
    parser.add_argument("--baseline-mm", type=float, default=80)
    parser.add_argument("--distance-cm", type=float, default=45)
    args = parser.parse_args()
    model = load_robot(args.urdf.resolve(), require_meshes=True)
    if model.name != "so101_new_calib" or model.root != "base_link":
        parser.error("Select the SO-101 Follower catalog URDF.")
    profile = new_profile("isaac_sim").model_copy(update={"distribution": "humble", "domain_id": 1})
    cameras = stereo_profiles(
        profile.id, model.root, args.baseline_mm / 1000, args.distance_cm / 100
    )
    output = args.output.resolve()
    path = IsaacSimSceneManager().prepare(
        profile,
        model,
        Path(tempfile.mkdtemp(prefix="studio-so101-stereo-")),
        existing_scene=args.source,
        stereo=cameras,
        vision_output=output,
    )
    print(path)


if __name__ == "__main__":
    main()
