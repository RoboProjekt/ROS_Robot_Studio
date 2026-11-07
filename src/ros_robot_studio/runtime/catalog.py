"""Validated catalog plus live Docker Registry v2 manifest inspection."""

import json
import urllib.parse
import urllib.request
from datetime import date
from typing import Literal

from pydantic import Field

from ros_robot_studio.core.models import Distro, Model
from ros_robot_studio.projects.store import DATA


class RuntimeSpec(Model):
    id: Distro
    name: str
    ubuntu: str
    base_image: str = Field(pattern=r"^ros:[a-z0-9-]+$")
    eol: date
    architecture: Literal["amd64"]

    @property
    def image(self) -> str:
        return f"ros-robot-studio/{self.id}:0.1.0"

    @property
    def supported(self) -> bool:
        return date.today() <= self.eol


class Catalog(Model):
    schema_version: Literal[1]
    verified_on: date
    sources: list[str]
    runtimes: list[RuntimeSpec]


def catalog() -> dict[str, RuntimeSpec]:
    data = Catalog.model_validate_json((DATA / "runtimes.json").read_text())
    return {entry.id: entry for entry in data.runtimes}


def request_json(url: str, headers: dict | None = None) -> dict:
    request = urllib.request.Request(url, headers=headers or {})
    with urllib.request.urlopen(request, timeout=15) as response:
        return json.load(response)


def inspect_registry(spec: RuntimeSpec) -> dict:
    """Verify amd64 image and obtain compressed base-layer size, excluding apt expansion."""
    token = request_json(
        "https://auth.docker.io/token?"
        + urllib.parse.urlencode(
            {"service": "registry.docker.io", "scope": "repository:library/ros:pull"}
        )
    )["token"]
    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": ", ".join(
            [
                "application/vnd.oci.image.index.v1+json",
                "application/vnd.docker.distribution.manifest.list.v2+json",
                "application/vnd.docker.distribution.manifest.v2+json",
                "application/vnd.oci.image.manifest.v1+json",
            ]
        ),
    }
    base = "https://registry-1.docker.io/v2/library/ros/manifests/"
    index = request_json(base + spec.base_image.split(":")[1], headers)
    manifest = next(
        (
            m
            for m in index.get("manifests", [])
            if m.get("platform", {}).get("architecture") == spec.architecture
            and m.get("platform", {}).get("os") == "linux"
        ),
        None,
    )
    if manifest is None:
        raise ValueError(f"No verified linux/{spec.architecture} manifest for {spec.base_image}")
    layers = request_json(base + manifest["digest"], headers)
    return {
        "digest": manifest["digest"],
        "base_bytes": sum(layer["size"] for layer in layers["layers"]),
        "pinned_base": spec.base_image.split(":")[0] + "@" + manifest["digest"],
    }
