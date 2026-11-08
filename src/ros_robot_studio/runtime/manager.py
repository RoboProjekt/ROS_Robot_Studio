"""Docker operations are blocking by contract; GUI callers must use a worker."""

from __future__ import annotations

import hashlib
import io
import json
import logging
import shutil
import tarfile
import tempfile
import uuid
from dataclasses import dataclass, field
from pathlib import Path

import docker
from docker.errors import NotFound
from ros_robot_studio.core.models import RuntimeConfig
from ros_robot_studio.runtime.catalog import catalog, inspect_registry
from ros_robot_studio.runtime.docker_diagnostics import classify, create_client, snapshot

OWNER = "io.ros-robot-studio.managed"
VERSION = "0.1.0"
RUNTIME_REVISION = "2"
SHARED_FILES = [
    "__init__.py",
    "robots/__init__.py",
    "robots/urdf.py",
    "motion/__init__.py",
    "motion/interpolation.py",
    "runtime/__init__.py",
    "runtime/controller_core.py",
]


def runtime_source_hash():
    root = Path(__file__).resolve().parents[1]
    files = [root / p for p in SHARED_FILES] + [
        p for p in (root / "data/runtime").iterdir() if p.is_file()
    ]
    digest = hashlib.sha256()
    for path in sorted(files):
        digest.update(str(path.relative_to(root)).encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


@dataclass
class RuntimeStatus:
    available: bool
    message: str
    installed: list[str] = field(default_factory=list)
    containers: dict[str, str] = field(default_factory=dict)
    reason: str = "ready"
    image_ids: dict[str, str] = field(default_factory=dict)
    diagnostics: dict = field(default_factory=dict)


class RuntimeManager:
    def __init__(self, client_factory=None):
        self.client_factory = client_factory or create_client
        self.specs = catalog()

    def status(self) -> RuntimeStatus:
        client = None
        diagnostics = snapshot()
        try:
            client = self.client_factory()
            client.ping()
            diagnostics["engine"] = "running (API verified)"
            installed = []
            image_ids = {}
            for identifier, spec in self.specs.items():
                try:
                    image = client.images.get(spec.image)
                    if (
                        image.labels.get(OWNER) == VERSION
                        and image.labels.get("studio.runtime_source") == runtime_source_hash()
                    ):
                        installed.append(identifier)
                        image_ids[identifier] = image.id
                except NotFound:
                    pass
            containers = {
                c.id: c.status for c in client.containers.list(all=True, filters={"label": OWNER})
            }
            return RuntimeStatus(
                True,
                "Docker daemon reachable",
                installed,
                containers,
                image_ids=image_ids,
                diagnostics=diagnostics,
            )
        except Exception as exc:
            reason, hint = classify(diagnostics, exc)
            diagnostics["api_error"] = str(exc)
            return RuntimeStatus(
                False,
                f"Docker · {reason}: {hint}\nAPI: {exc}",
                reason=reason,
                diagnostics=diagnostics,
            )
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
            # Build only a small curated context: shared parser/interpolation, never the venv or host files.
            root = Path(__file__).resolve().parents[1]
            temporary = tempfile.TemporaryDirectory(prefix="studio-runtime-build-")
            context = Path(temporary.name)
            shutil.copytree(root / "data/runtime", context, dirs_exist_ok=True)
            for relative in SHARED_FILES:
                target = context / "shared" / "ros_robot_studio" / relative
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(root / relative, target)
            for event in client.api.build(
                path=str(context),
                tag=spec.image,
                rm=True,
                decode=True,
                buildargs={"BASE_IMAGE": info["pinned_base"], "DISTRO": distro},
                labels={
                    OWNER: VERSION,
                    "studio.runtime_revision": RUNTIME_REVISION,
                    "studio.runtime_source": runtime_source_hash(),
                },
                platform="linux/amd64",
            ):
                if "error" in event:
                    raise RuntimeError(event["error"])
                if event.get("stream"):
                    progress(event["stream"].strip())
            image = client.images.get(spec.image)
            if image.labels.get(OWNER) != VERSION:
                raise RuntimeError("Built image ownership validation failed")
            temporary.cleanup()
            progress(f"{spec.image} ready; pinned base digest {info['digest']}")
            return spec.image
        finally:
            client.close()

    def start(
        self,
        distro: str,
        project_id: str,
        config: RuntimeConfig,
        integration=False,
        robot_config=None,
    ) -> str:
        client = self.client_factory()
        network = container = None
        try:
            image = client.images.get(self.specs[distro].image)
            if image.labels.get(OWNER) != VERSION:
                raise ValueError("Only managed Studio images may be launched")
            if image.labels.get("studio.runtime_source") != runtime_source_hash():
                raise ValueError("Runtime sources changed; install/rebuild the Studio image first")
            from ros_robot_studio.runtime.robot_config import simulation_config

            payload = robot_config or simulation_config()
            if payload.get("profile") not in (
                "managed_simulation",
                "external_simulation",
                "read_only",
            ):
                raise ValueError("Unknown backend runtime profile")
            if payload.get("profile") == "external_simulation" and not payload.get(
                "external_simulation_confirmed"
            ):
                raise ValueError("External simulation requires explicit confirmation")
            if config.network == "host" and (
                not config.external_network_confirmed
                or payload.get("profile") not in ("external_simulation", "read_only")
                or (
                    payload.get("profile") == "external_simulation"
                    and not payload.get("external_simulation_confirmed")
                )
            ):
                raise ValueError(
                    "Host networking requires an explicitly confirmed external simulation profile"
                )
            if config.network == "bridge":
                network = client.networks.create(
                    "ros-studio-" + uuid.uuid4().hex[:12],
                    driver="bridge",
                    labels={OWNER: VERSION, "studio.project": project_id},
                )
            container = client.containers.create(
                image.id,
                name=f"ros-studio-{distro}-{uuid.uuid4().hex[:10]}",
                labels={
                    OWNER: VERSION,
                    "studio.project": project_id,
                    "studio.distro": distro,
                    "studio.integration": str(integration).lower(),
                    "studio.backend_id": payload.get("backend_id", ""),
                    "studio.domain_id": str(config.domain_id),
                    "studio.network": network.id if network else "",
                },
                **(
                    {"network_mode": "host"}
                    if config.network == "host"
                    else {
                        "ports": {"9090/tcp": ("127.0.0.1", config.port)},
                        "network": network.name,
                    }
                ),
                environment={
                    "ROS_DOMAIN_ID": str(config.domain_id),
                    "RMW_IMPLEMENTATION": config.middleware,
                    "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
                    "PYTHONFAULTHANDLER": "1",
                    "STUDIO_BRIDGE_PORT": str(config.port if config.network == "host" else 9090),
                    "STUDIO_BRIDGE_ADDRESS": "127.0.0.1" if config.network == "host" else "0.0.0.0",
                    "ROS_NAMESPACE": config.namespace,
                    # Humble adds SHM even over a UDP-only XML profile when
                    # ROS_LOCALHOST_ONLY=1. Separate container IPC namespaces
                    # then discover peers but cannot receive their samples.
                    # The dedicated XML whitelist enforces loopback instead.
                    "ROS_LOCALHOST_ONLY": "0"
                    if config.fastdds_udp
                    else "1"
                    if config.discovery == "localhost"
                    else "0",
                    "ROS_AUTOMATIC_DISCOVERY_RANGE": "SUBNET"
                    if config.fastdds_udp
                    else "LOCALHOST"
                    if config.discovery == "localhost"
                    else "SUBNET",
                    "FASTRTPS_DEFAULT_PROFILES_FILE": (
                        "/opt/studio/fastdds_localhost.xml"
                        if config.discovery == "localhost"
                        else "/opt/studio/fastdds_udp.xml"
                    )
                    if config.fastdds_udp
                    else "",
                },
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
            data = json.dumps(payload).encode()
            archive = io.BytesIO()
            with tarfile.open(fileobj=archive, mode="w") as tar:
                member = tarfile.TarInfo("studio_robot.json")
                member.size = len(data)
                member.mode = 0o644
                tar.addfile(member, io.BytesIO(data))
            container.put_archive("/tmp", archive.getvalue())
            container.start()
            return container.id
        except Exception:
            if container:
                try:
                    container.stop(timeout=2)
                    container.remove()
                except docker.errors.APIError:
                    logging.getLogger(__name__).exception("Failed start cleanup")
            if network:
                try:
                    network.remove()
                except docker.errors.APIError:
                    logging.getLogger(__name__).exception("Failed network cleanup")
            raise
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
            network_id = container.labels.get("studio.network")
            container.stop(timeout=5)
            container.remove()
            if network_id:
                try:
                    network = client.networks.get(network_id)
                    if network.attrs.get("Labels", {}).get(OWNER) == VERSION:
                        network.remove()
                except docker.errors.APIError:
                    logging.getLogger(__name__).warning(
                        "Container stopped; its network needs cleanup", exc_info=True
                    )
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
            except Exception:
                pass
            client.close()
