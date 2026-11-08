"""Declarative, checksum-locked robot assets with staged offline installations."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import tempfile
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Literal
from urllib.parse import urlparse

from filelock import FileLock, Timeout
from pydantic import Field, model_validator

from ros_robot_studio.core.models import Model
from ros_robot_studio.core.paths import xdg_path
from ros_robot_studio.robots.geometry import load_robot, resolve_mesh_uri, safe_relative_path
from ros_robot_studio.robots.urdf import parse_urdf

CATALOG = Path(__file__).resolve().parents[1] / "data" / "robot_models"


class ModelFile(Model):
    path: str
    source_path: str
    size: int = Field(gt=0, le=80_000_000)
    sha256: str = Field(pattern=r"^[0-9a-f]{64}$")

    @model_validator(mode="after")
    def safe_paths(self):
        safe_relative_path(self.path)
        safe_relative_path(self.source_path)
        return self


class RobotModelManifest(Model):
    schema_version: Literal[1] = 1
    id: str = Field(pattern=r"^[a-z][a-z0-9_-]{1,64}$")
    name: str
    plugin_id: str | None = None
    manufacturer: str
    model: str
    variant: str
    model_version: str = Field(pattern=r"^\d+\.\d+\.\d+$")
    description: str
    urdf: str
    mesh_directory: str
    source_repository: str
    source_base_url: str
    source_revision: str = Field(pattern=r"^[a-f0-9]{40}$")
    license: str
    license_file: str
    default_end_effector: str
    calibration: str
    drivers: list[str] = Field(default_factory=list)
    verified_ros_distributions: list[str] = Field(default_factory=list)
    backends: dict[str, Literal["supported", "not_configured", "planned"]]
    files: list[ModelFile]

    @model_validator(mode="after")
    def coherent_source(self):
        url = urlparse(self.source_base_url)
        if (
            url.scheme != "https"
            or not url.netloc
            or url.username
            or url.password
            or url.query
            or url.fragment
        ):
            raise ValueError("Model source must be an explicit HTTPS base URL")
        if self.source_revision not in url.path.split("/"):
            raise ValueError("Source URL must pin the declared immutable revision")
        for path in (self.urdf, self.mesh_directory, self.license_file):
            safe_relative_path(path)
        paths = [item.path for item in self.files]
        if len(set(paths)) != len(paths) or not {self.urdf, self.license_file} <= set(paths):
            raise ValueError("Model requires unique files, URDF and license")
        if sum(item.size for item in self.files) > 200_000_000:
            raise ValueError("Model exceeds 200 MB download limit")
        return self

    @property
    def download_bytes(self) -> int:
        return sum(item.size for item in self.files)

    @property
    def reference(self) -> str:
        return f"model://{self.id}/{self.source_revision}/{self.urdf}"


@dataclass(frozen=True)
class ModelStatus:
    state: Literal["not_installed", "incomplete", "damaged", "installed"]
    message: str
    directory: Path
    invalid_files: tuple[str, ...] = ()


class ModelUnavailableError(ValueError):
    def __init__(self, identifier: str, revision: str, message: str):
        self.identifier, self.revision = identifier, revision
        super().__init__(message)


def digest_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for data in iter(lambda: stream.read(256 * 1024), b""):
            digest.update(data)
    return digest.hexdigest()


class RobotModelManager:
    """No GUI, ROS, Docker or hardware. All I/O methods must run in a worker."""

    def __init__(
        self,
        root: Path | None = None,
        catalog_directory: Path | None = None,
        opener: Callable | None = None,
    ):
        self.root = root or xdg_path("data") / "robots"
        self.opener = opener or urllib.request.urlopen
        self.catalog: dict[str, RobotModelManifest] = {}
        for path in sorted((catalog_directory or CATALOG).glob("*.json")):
            if path.stat().st_size > 200_000:
                raise ValueError("Model manifest exceeds 200 KB")
            spec = RobotModelManifest.model_validate_json(path.read_text())
            if spec.id in self.catalog:
                raise ValueError(f"Duplicate model ID: {spec.id}")
            self.catalog[spec.id] = spec

    def spec(self, identifier: str, revision: str | None = None) -> RobotModelManifest:
        if identifier not in self.catalog:
            raise ValueError(f"Unknown robot model: {identifier}")
        spec = self.catalog[identifier]
        if revision and revision != spec.source_revision:
            raise ValueError(
                f"Model revision {revision} is not in the reviewed catalog. No newer revision will be substituted automatically."
            )
        return spec

    def directory(self, spec: RobotModelManifest) -> Path:
        candidate = self.root / spec.id / spec.source_revision
        if not candidate.resolve().is_relative_to(self.root.resolve()):
            raise ValueError("Model cache path escapes application data directory")
        return candidate

    @staticmethod
    def valid_file(directory: Path, entry: ModelFile) -> bool:
        path = directory / entry.path
        try:
            return (
                not path.is_symlink()
                and path.resolve().is_relative_to(directory.resolve())
                and path.is_file()
                and path.stat().st_size == entry.size
                and digest_file(path) == entry.sha256
            )
        except OSError:
            return False

    def status(self, identifier: str, revision: str | None = None) -> ModelStatus:
        spec = self.spec(identifier, revision)
        directory = self.directory(spec)
        if not directory.exists():
            return ModelStatus(
                "not_installed", "Not installed · offline model available for download", directory
            )
        bad = tuple(entry.path for entry in spec.files if not self.valid_file(directory, entry))
        missing = any(not (directory / name).is_file() for name in bad)
        receipt = directory / "installation.json"
        try:
            record = json.loads(receipt.read_text())
            registered = (
                record["model_id"] == identifier
                and record["source_revision"] == spec.source_revision
            )
            registered = registered and record.get("model_version") == spec.model_version
            registered = registered and record["files"] == [
                entry.model_dump() for entry in spec.files
            ]
        except (OSError, ValueError, KeyError, TypeError):
            registered = False
        if bad or not registered:
            state = "incomplete" if missing or not registered else "damaged"
            return ModelStatus(
                state, f"{state.title()} installation · repair required", directory, bad
            )
        return ModelStatus(
            "installed", "Installed · original files verified · ready offline", directory
        )

    def resolve(self, reference: str) -> Path:
        url = urlparse(reference)
        if url.scheme != "model" or url.query or url.fragment or url.username or url.port:
            raise ValueError("Invalid model reference")
        parts = url.path.lstrip("/").split("/", 1)
        if len(parts) != 2:
            raise ValueError("Model reference requires revision and URDF")
        revision, urdf = parts
        spec = self.spec(url.netloc, revision)
        if urdf != spec.urdf:
            raise ValueError("Model reference does not match catalog URDF")
        status = self.status(spec.id, revision)
        if status.state != "installed":
            raise ModelUnavailableError(spec.id, revision, f"{spec.name}: {status.message}")
        return status.directory / spec.urdf

    def validate_directory(self, directory: Path, spec: RobotModelManifest):
        invalid = [entry.path for entry in spec.files if not self.valid_file(directory, entry)]
        if invalid:
            raise ValueError("Invalid model checksums: " + ", ".join(invalid))
        model = parse_urdf(directory / spec.urdf)
        listed = {entry.path for entry in spec.files}
        for reference in model.mesh_references:
            asset = resolve_mesh_uri(reference, directory)
            if asset.relative_to(directory.resolve()).as_posix() not in listed:
                raise ValueError(
                    f"URDF references a mesh outside the reviewed manifest: {reference}"
                )
        loaded = load_robot(directory / spec.urdf, require_meshes=True)
        if spec.default_end_effector not in loaded.links:
            raise ValueError("Default end-effector frame missing from URDF")
        return loaded

    def install(
        self,
        identifier: str,
        progress: Callable[[str], None] = lambda message: None,
        revision: str | None = None,
    ) -> Path:
        spec = self.spec(identifier, revision)
        destination = self.directory(spec)
        destination.parent.mkdir(parents=True, exist_ok=True)
        if (destination.parent / ".installation.lock").is_symlink():
            raise ValueError("Unsafe model installation lock symlink")
        try:
            with FileLock(str(destination.parent / ".installation.lock"), timeout=2):
                if self.status(identifier, revision).state == "installed":
                    progress("Ready · using the verified local model; no download required")
                    return destination / spec.urdf
                return self._install(spec, destination, progress)
        except Timeout as exc:
            raise ValueError(
                "Another Studio process is installing this model; retry after it finishes."
            ) from exc

    def _install(self, spec: RobotModelManifest, destination: Path, progress) -> Path:
        completed = 0
        with tempfile.TemporaryDirectory(prefix=".install-", dir=destination.parent) as temporary:
            stage = Path(temporary) / "model"
            stage.mkdir()
            for entry in spec.files:
                target = stage / entry.path
                target.parent.mkdir(parents=True, exist_ok=True)
                if self.valid_file(destination, entry):
                    shutil.copyfile(destination / entry.path, target)
                    completed += entry.size
                    progress(f"{completed / spec.download_bytes:.0%} · reusing {entry.path}")
                else:
                    request = urllib.request.Request(
                        spec.source_base_url + "/" + entry.source_path,
                        headers={"User-Agent": "ROS-Robot-Studio/0.1"},
                    )
                    try:
                        with (
                            self.opener(request, timeout=45) as response,
                            target.open("xb") as stream,
                        ):
                            count = 0
                            while True:
                                data = response.read(128 * 1024)
                                if not data:
                                    break
                                count += len(data)
                                if count > entry.size:
                                    raise ValueError(f"Oversized model download: {entry.path}")
                                stream.write(data)
                                progress(
                                    f"{(completed + count) / spec.download_bytes:.0%} · {entry.path} · {(completed + count) / 1e6:.1f}/{spec.download_bytes / 1e6:.1f} MB"
                                )
                    except (OSError, urllib.error.URLError) as exc:
                        raise ValueError(
                            f"Cannot download {entry.path}. Check internet access, proxy and certificates, then retry. Details: {exc}"
                        ) from exc
                    if not self.valid_file(stage, entry):
                        raise ValueError(
                            f"Checksum or size mismatch: {entry.path}. The installation was not activated."
                        )
                    completed += entry.size
            progress("Validating URDF, all referenced meshes and original checksums…")
            self.validate_directory(stage, spec)
            receipt = dict(
                model_id=spec.id,
                model_version=spec.model_version,
                source_repository=spec.source_repository,
                source_revision=spec.source_revision,
                source_base_url=spec.source_base_url,
                license=spec.license,
                installed_at=datetime.now(timezone.utc).isoformat(),
                files=[entry.model_dump() for entry in spec.files],
            )
            (stage / "installation.json").write_text(json.dumps(receipt, indent=2) + "\n")
            # The previously active tree survives every download/validation failure.
            backup = Path(temporary) / "previous"
            if destination.exists():
                os.replace(destination, backup)
            try:
                os.replace(stage, destination)
            except OSError:
                if backup.exists():
                    os.replace(backup, destination)
                raise
            progress("100% · Ready · original model installed and verified")
        return destination / spec.urdf
