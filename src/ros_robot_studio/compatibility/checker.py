"""Conservative compatibility reports; no speculative source-code migration."""

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from ros_robot_studio.core.models import Project
from ros_robot_studio.plugins.registry import PluginRegistry


class MigrationRule(Protocol):
    """Future rules write into a separate output directory, never into source projects."""

    def applies(self, source: str, target: str) -> bool: ...
    def generate(self, project: Project, destination: Path) -> list[Path]: ...


@dataclass
class CompatibilityReport:
    source: str
    target: str
    issues: list[str]
    packages: list[str]

    @property
    def compatible(self) -> bool:
        return not self.issues

    def text(self) -> str:
        status = "Compatible configuration" if self.compatible else "Review required"
        lines = [
            f"{self.source.title()} → {self.target.title()}: {status}",
            "",
            "Package availability is verified during the runtime build; hardware execution is not certified.",
        ]
        lines += self.issues or ["No known incompatibility in the selected declarative profiles."]
        lines += ["", "Required packages: " + (", ".join(self.packages) or "None")]
        lines += [
            "",
            "Automatic source-code migration is not implemented. Original project defaults are preserved.",
        ]
        return "\n".join(lines)


def check(project: Project, target: str, registry: PluginRegistry) -> CompatibilityReport:
    ids = [
        project.robot.profile,
        project.robot.hardware,
        project.robot.controller,
        project.robot.ik_solver,
    ]
    ids += [sensor.driver for sensor in project.sensors]
    issues, packages = [], set()
    for identifier in dict.fromkeys(ids):
        if not identifier:
            continue
        plugin = registry.plugins.get(identifier)
        if not plugin:
            issues.append(f"{identifier}: plugin manifest not found")
        elif target not in plugin.distributions:
            alternative = plugin.alternatives.get(target)
            issues.append(
                f"{plugin.name}: incompatible or unverified for {target}"
                + (f"; alternative: {alternative}" if alternative else "")
            )
        elif plugin.status == "planned":
            issues.append(f"{plugin.name}: planned; execution not implemented")
        if plugin:
            packages.update(plugin.packages.get(target, []))
    return CompatibilityReport(project.default_ros, target, issues, sorted(packages))
