"""Read-only upstream probes. Never pulls an image or installs ROS packages."""

import gzip
import json
import urllib.request
from pathlib import Path

from ros_robot_studio.runtime.catalog import catalog, inspect_registry

report = {}
for identifier, spec in catalog().items():
    result = {"base_image": spec.base_image}
    try:
        result.update(inspect_registry(spec))
    except Exception as exc:
        result["registry_error"] = str(exc)
    codename = {"humble": "jammy", "jazzy": "noble", "lyrical": "resolute"}[identifier]
    url = f"https://packages.ros.org/ros2/ubuntu/dists/{codename}/main/binary-amd64/Packages.gz"
    try:
        with urllib.request.urlopen(url, timeout=30) as response:
            packages = gzip.decompress(response.read()).decode()
        versions = {}
        for package in packages.split("\n\n"):
            fields = dict(
                line.split(": ", 1)
                for line in package.splitlines()
                if ": " in line and not line.startswith(" ")
            )
            if fields.get("Package") in (
                f"ros-{identifier}-rosbridge-suite",
                f"ros-{identifier}-xacro",
            ):
                versions[fields["Package"]] = fields.get("Version")
        result["packages"] = versions
        result["package_index"] = url
    except Exception as exc:
        result["package_error"] = str(exc)
    report[identifier] = result
    print(identifier, json.dumps(result), flush=True)
Path("docs/registry-manifests.json").write_text(json.dumps(report, indent=2) + "\n")
