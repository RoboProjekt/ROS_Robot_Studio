"""Declarative manifests only: discovery never imports third-party Python code."""

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import Field

from ros_robot_studio.core.models import Distro, Model
from ros_robot_studio.projects.store import DATA


class PluginManifest(Model):
    schema_version: Literal[1] = 1
    id: str = Field(pattern=r"^[a-z][a-z0-9-]+$")
    name: str = Field(min_length=1)
    version: str = Field(pattern=r"^\d+\.\d+\.\d+$")
    kind: Literal[
        "robot", "hardware", "controller", "ik", "sensor", "perception", "ros-adapter", "backend"
    ]
    description: str
    distributions: list[Distro]
    packages: dict[str, list[str]] = Field(default_factory=dict)
    dependencies: list[str] = Field(default_factory=list)
    hardware: list[str] = Field(default_factory=list)
    interfaces: list[str] = Field(default_factory=list)
    parameters: dict[str, Any] = Field(default_factory=dict)
    capabilities: list[str] = Field(default_factory=list)
    status: Literal["implemented", "planned"] = "planned"
    alternatives: dict[str, str] = Field(default_factory=dict)


class PluginRegistry:
    def __init__(self, directory: Path | None = None):
        self.plugins: dict[str, PluginManifest] = {}
        self.errors: list[str] = []
        for path in sorted((directory or DATA / "plugins").glob("*.yaml")):
            try:
                if path.stat().st_size > 100_000:
                    raise ValueError("manifest exceeds 100 KB")
                manifest = PluginManifest.model_validate(yaml.safe_load(path.read_text()))
                if manifest.id in self.plugins:
                    raise ValueError(f"duplicate plugin ID: {manifest.id}")
                self.plugins[manifest.id] = manifest
            except (OSError, ValueError, yaml.YAMLError) as exc:
                self.errors.append(f"{path.name}: {exc}")

    def by_kind(self, kind: str) -> list[PluginManifest]:
        return [plugin for plugin in self.plugins.values() if plugin.kind == kind]
