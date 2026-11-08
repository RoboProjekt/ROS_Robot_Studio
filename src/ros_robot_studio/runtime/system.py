"""Read-only system inspection and explicit, reviewable Docker installer entry point."""

import platform
import shutil
import sys
import urllib.request
from pathlib import Path

from ros_robot_studio.runtime.manager import RuntimeManager


def system_report() -> dict:
    try:
        with urllib.request.urlopen("https://registry-1.docker.io/v2/", timeout=4):
            internet = "Connected"
    except urllib.error.HTTPError as exc:
        internet = "Connected" if exc.code == 401 else str(exc)
    except Exception as exc:
        internet = f"Unavailable: {exc}"
    docker = RuntimeManager().status()
    return {
        "OS": platform.platform(),
        "Architecture": platform.machine(),
        "Python": sys.version.split()[0],
        "Internet": internet,
        "Free disk": f"{shutil.disk_usage(Path.home()).free / 1024**3:.1f} GiB",
        "Docker CLI": shutil.which("docker") or "Not installed",
        "Docker daemon / permissions": docker.message,
        "docker_available": docker.available,
        "Docker process diagnostics": docker.diagnostics,
    }
