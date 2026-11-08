"""Maintainer-only source lock generator. Network access is explicit; no model in Git."""

import hashlib
import json
import urllib.request
from pathlib import Path
from xml.etree import ElementTree

REVISION = "a758567c3978dfeefe282ede0500085a48fe8f78"
BASE = f"https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/{REVISION}"
OUTPUT = Path("src/ros_robot_studio/data/robot_models/so101_follower.json")
CACHE = Path("/tmp/so101-source/download")


def download(relative):
    path = CACHE / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(BASE + "/" + relative, timeout=45) as response:
        path.write_bytes(response.read())
    return path.read_bytes()


def main():
    prefix = "Simulation/SO101/"
    urdf_name = "so101_new_calib.urdf"
    urdf = download(prefix + urdf_name)
    xml = ElementTree.fromstring(urdf)
    meshes = sorted({mesh.attrib["filename"] for mesh in xml.iter("mesh")})
    entries = [
        (urdf_name, prefix + urdf_name),
        ("LICENSE", "LICENSE"),
        ("SOURCE_README.md", prefix + "README.md"),
    ]
    entries += [(name, prefix + name) for name in meshes]
    files = []
    for relative, remote in entries:
        data = download(remote)
        files.append(
            dict(
                path=relative,
                source_path=remote,
                size=len(data),
                sha256=hashlib.sha256(data).hexdigest(),
            )
        )
        print(relative, len(data), flush=True)
    manifest = dict(
        schema_version=1,
        id="so101_follower",
        plugin_id="so101",
        name="SO-101 Follower",
        manufacturer="The Robot Studio / Hugging Face",
        model="SO-101",
        variant="follower",
        model_version="1.0.0",
        description="Original follower URDF and STL visuals, new_calib convention. Offline model only; hardware mappings are not calibrated.",
        urdf=urdf_name,
        mesh_directory="assets",
        source_repository="https://github.com/TheRobotStudio/SO-ARM100",
        source_base_url=BASE,
        source_revision=REVISION,
        license="Apache-2.0",
        license_file="LICENSE",
        default_end_effector="gripper_frame_link",
        calibration="new_calib",
        drivers=[],
        verified_ros_distributions=[],
        backends={
            "offline_viewer": "supported",
            "ros2_humble": "not_configured",
            "ros2_jazzy": "not_configured",
            "ros2_lyrical": "not_configured",
            "isaac_sim": "planned",
            "hardware": "planned",
        },
        files=files,
    )
    OUTPUT.write_text(json.dumps(manifest, indent=2) + "\n")
    print("Total bytes", sum(entry["size"] for entry in files))


if __name__ == "__main__":
    main()
