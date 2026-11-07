"""Docker operations are blocking by contract; GUI callers must use a worker."""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from pathlib import Path

import docker
from docker.errors import NotFound
from ros_robot_studio.core.models import RuntimeConfig
from ros_robot_studio.runtime.catalog import catalog, inspect_registry

OWNER = "io.ros-robot-studio.managed"
VERSION = "0.1.0"


@dataclass
class RuntimeStatus:
    available: bool
    message: str
    installed: list[str] = field(default_factory=list)
    containers: dict[str, str] = field(default_factory=dict)


class RuntimeManager:
    def __init__(self, client_factory=None):
        self.client_factory = client_factory or (lambda: docker.from_env(timeout=15))
        self.specs = catalog()

    def status(self) -> RuntimeStatus:
        client = None
        try:
            client = self.client_factory()
            client.ping()
            installed = []
            for identifier, spec in self.specs.items():
                try:
                    image = client.images.get(spec.image)
                    if image.labels.get(OWNER) == VERSION:
                        installed.append(identifier)
                except NotFound:
                    pass
            containers = {
                c.id: c.status for c in client.containers.list(all=True, filters={"label": OWNER})
            }
            return RuntimeStatus(True, "Docker daemon reachable", installed, containers)
        except Exception as exc:
            return RuntimeStatus(False, f"Docker unavailable: {exc}")
        finally:
            if client:
                client.close()

    def install(self, distro: str, progress=lambda text: None) -> str:
        spec = self.specs[distro]
        if not spec.supported:
            raise ValueError("This distribution is EOL; installation is disabled")
        if self.status().installed.count(distro):
            progress("Managed runtime already installed; using local cache.")
            return spec.image
        client = self.client_factory()
        try:
            info = inspect_registry(spec)
            progress(
                f"Verified amd64 base: {info['base_bytes'] / 1e6:.1f} MB compressed. Additional ROS packages not included."
            )
            for event in client.api.pull(
                info["pinned_base"], stream=True, decode=True, platform="linux/amd64"
            ):
                if "error" in event:
                    raise RuntimeError(event["error"])
                progress(
                    " ".join(str(event.get(k, "")) for k in ("id", "status", "progress")).strip()
                )
            context = Path(__file__).resolve().parents[1] / "data" / "runtime"
            for event in client.api.build(
                path=str(context),
                tag=spec.image,
                rm=True,
                decode=True,
                buildargs={"BASE_IMAGE": info["pinned_base"], "DISTRO": distro},
                labels={OWNER: VERSION},
                platform="linux/amd64",
            ):
                if "error" in event:
                    raise RuntimeError(event["error"])
                if event.get("stream"):
                    progress(event["stream"].strip())
            image = client.images.get(spec.image)
            if image.labels.get(OWNER) != VERSION:
                raise RuntimeError("Built image ownership validation failed")
            progress(f"{spec.image} ready; pinned base digest {info['digest']}")
            return spec.image
        finally:
            client.close()

    def start(self, distro: str, project_id: str, config: RuntimeConfig, integration=False) -> str:
        client = self.client_factory()
        try:
            image = client.images.get(self.specs[distro].image)
            if image.labels.get(OWNER) != VERSION:
                raise ValueError("Only managed Studio images may be launched")
            container = client.containers.run(
                image.id,
                detach=True,
                name=f"ros-studio-{distro}-{uuid.uuid4().hex[:10]}",
                labels={
                    OWNER: VERSION,
                    "studio.project": project_id,
                    "studio.distro": distro,
                    "studio.integration": str(integration).lower(),
                },
                ports={"9090/tcp": ("127.0.0.1", config.port)},
                environment={"ROS_DOMAIN_ID": "42", "RCUTILS_LOGGING_BUFFERED_STREAM": "1"},
                mem_limit=f"{config.memory_mb}m",
                nano_cpus=int(config.cpu_count * 1e9),
                cap_drop=["ALL"],
                security_opt=["no-new-privileges:true"],
                pids_limit=256,
                init=True,
                log_config=docker.types.LogConfig(
                    type="json-file", config={"max-size": "10m", "max-file": "2"}
                ),
            )
            return container.id
        finally:
            client.close()

    def _owned(self, client, identifier: str):
        container = client.containers.get(identifier)
        if container.labels.get(OWNER) != VERSION:
            raise ValueError("Refusing to modify a container not owned by ROS Robot Studio")
        return container

    def stop(self, identifier: str) -> None:
        client = self.client_factory()
        try:
            container = self._owned(client, identifier)
            container.stop(timeout=5)
            container.remove()
        finally:
            client.close()

    def logs(self, identifier: str) -> str:
        client = self.client_factory()
        try:
            return self._owned(client, identifier).logs(tail=200).decode(errors="replace")
        finally:
            client.close()

    def health(self, identifier: str) -> dict:
        client = self.client_factory()
        try:
            container = self._owned(client, identifier)
            container.reload()
            return {
                "status": container.status,
                "health": container.attrs.get("State", {})
                .get("Health", {})
                .get("Status", "unknown"),
            }
        finally:
            client.close()

    def remove_image(self, distro: str, referenced_distros: set[str]) -> None:
        if distro in referenced_distros:
            raise ValueError("Runtime is referenced by the current or a known saved project")
        client = self.client_factory()
        try:
            image = client.images.get(self.specs[distro].image)
            if image.labels.get(OWNER) != VERSION:
                raise ValueError("Image is not owned by Studio")
            # Docker refuses removal while any container references it. Never force.
            client.images.remove(image.id, force=False)
        finally:
            client.close()

    def expand_xacro(self, identifier: str, source: Path) -> str:
        """Expand one selected Xacro inside the owned container, without host mounts."""
        import io
        import tarfile

        if source.stat().st_size > 1_000_000:
            raise ValueError("Xacro exceeds 1 MB limit")
        client = self.client_factory()
        temporary_name = "studio-" + uuid.uuid4().hex + ".xacro"
        try:
            container = self._owned(client, identifier)
            archive = io.BytesIO()
            data = source.read_bytes()
            with tarfile.open(fileobj=archive, mode="w") as tar:
                member = tarfile.TarInfo(temporary_name)
                member.size, member.mode = len(data), 0o600
                tar.addfile(member, io.BytesIO(data))
            container.put_archive("/tmp", archive.getvalue())
            code, output = container.exec_run(
                [
                    "/bin/bash",
                    "-c",
                    'source /opt/ros/$ROS_DISTRO/setup.bash; exec ros2 run xacro xacro "$1"',
                    "studio-xacro",
                    "/tmp/" + temporary_name,
                ],
                demux=True,
            )
            stdout, stderr = output
            if code:
                raise RuntimeError((stderr or stdout or b"Xacro failed").decode(errors="replace"))
            return (stdout or b"").decode()
        finally:
            try:
                container.exec_run(["rm", "-f", "/tmp/" + temporary_name])
            except (UnboundLocalError, Exception):
                pass
            client.close()
