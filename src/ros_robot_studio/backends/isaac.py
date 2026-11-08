"""Local Isaac installation/process/scene management. No Isaac APIs in desktop Python."""

import json
import os
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, Signal

from ros_robot_studio.projects.store import atomic_text

SCENE_SCRIPT = Path(__file__).resolve().parents[1] / "data/isaac/studio_scene.py"
DDS_PROFILE = Path(__file__).resolve().parents[1] / "data/runtime/fastdds_udp.xml"


@dataclass(frozen=True)
class IsaacInstallation:
    path: Path
    version: str
    launcher: Path
    python_launcher: Path
    bridge_distributions: tuple[str, ...]


class IsaacSimInstallationManager:
    def validate(self, directory):
        path = Path(directory).expanduser().resolve()
        launch, python = path / "isaac-sim.sh", path / "python.sh"
        if not path.is_dir() or not launch.is_file() or not python.is_file():
            raise ValueError("Select an Isaac installation with isaac-sim.sh and python.sh")
        if not os.access(launch, os.X_OK) or not os.access(python, os.X_OK):
            raise ValueError("Isaac launchers must be executable")
        version = (path / "VERSION").read_text().strip()
        if not version.startswith("5.1."):
            raise ValueError(
                "This integration is validated for Isaac Sim 5.1; other versions require an adapter"
            )
        bridge = path / "exts/isaacsim.ros2.bridge"
        distributions = tuple(d for d in ("humble", "jazzy") if (bridge / d / "lib").is_dir())
        if not distributions or not (path / "exts/isaacsim.asset.importer.urdf").is_dir():
            raise ValueError("Isaac ROS bridge and official URDF importer are required")
        return IsaacInstallation(path, version, launch, python, distributions)

    def detect(self, configured=()):
        candidates = [Path(p) for p in configured if p]
        if os.environ.get("ISAAC_PATH"):
            candidates.append(Path(os.environ["ISAAC_PATH"]))
        candidates.extend((Path.home() / "isaacsim", Path.home() / "isaac-sim"))
        package_root = Path.home() / ".local/share/ov/pkg"
        if package_root.is_dir():
            candidates.extend(package_root.glob("isaac*"))
        found = {}
        for candidate in candidates:
            try:
                installation = self.validate(candidate)
                found[str(installation.path)] = installation
            except (ValueError, OSError):
                pass
        return list(found.values())


class IsaacSimSceneManager:
    def prepare(self, profile, model, directory, positions=None):
        from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine

        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        if not model.source_path or not model.meshes:
            raise ValueError(
                "Isaac scene generation requires a resolved URDF with its original meshes"
            )
        q = dict(positions or model.default_configuration())
        model.configuration(q)
        signature = UniversalKinematicsEngine(model).signature
        if profile.model_signature and signature != profile.model_signature:
            raise ValueError("Backend model signature differs from selected robot")
        request = dict(
            schema_version=1,
            backend_id=profile.id,
            model_signature=signature,
            urdf=str(model.source_path.resolve()),
            output_directory=str(directory),
            domain_id=profile.domain_id,
            state_topic=profile.state_topic,
            command_topic=profile.command_topic,
            clock_topic=profile.clock_topic,
            initial=q,
            joint_mapping=profile.joint_mapping
            or {j.name: j.name for j in model.joints if j.movable},
            expected_joints={
                j.name: dict(
                    kind=j.kind, lower=j.lower, upper=j.upper, axis=list(j.axis), effort=j.effort
                )
                for j in model.joints
                if j.movable
            },
            expected_link_positions={
                n: matrix[:3, 3].tolist() for n, matrix in model.transforms(q).items()
            },
        )
        path = directory / "scene-request.json"
        atomic_text(path, json.dumps(request, indent=2))
        return path

    def validate_result(self, request_path):
        request = json.loads(Path(request_path).read_text())
        report = json.loads(
            (Path(request["output_directory"]) / "scene-validation.json").read_text()
        )
        if (
            report["model_signature"] != request["model_signature"]
            or report["domain_id"] != request["domain_id"]
        ):
            raise ValueError("Scene provenance/domain mismatch")
        if not report.get("valid") or set(report["joint_names"]) != set(request["expected_joints"]):
            raise ValueError("USD articulation/joint validation failed")
        return report


class IsaacSimProcessManager(QObject):
    changed = Signal(str)
    log = Signal(str, str)
    error = Signal(str, str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.processes = {}
        self.requests = {}
        self.tails = {}

    def start(self, profile, installation, request, headless=False, build_only=False):
        if profile.id in self.processes:
            raise ValueError("This backend already owns an Isaac process")
        if profile.distribution not in installation.bridge_distributions:
            raise ValueError(
                "Requested ROS distribution is unavailable in the selected Isaac bridge"
            )
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.ProcessChannelMode.MergedChannels)
        process.setWorkingDirectory(str(installation.path))
        environment = QProcessEnvironment.systemEnvironment()
        for key in (
            "PYTHONPATH",
            "PYTHONHOME",
            "VIRTUAL_ENV",
            "AMENT_PREFIX_PATH",
            "COLCON_PREFIX_PATH",
            "CMAKE_PREFIX_PATH",
            "CONDA_PREFIX",
        ):
            environment.remove(key)
        environment.insert("ROS_DISTRO", profile.distribution)
        environment.insert("ROS_DOMAIN_ID", str(profile.domain_id))
        environment.insert("RMW_IMPLEMENTATION", profile.middleware)
        environment.insert(
            "LD_LIBRARY_PATH",
            str(installation.path / "exts/isaacsim.ros2.bridge" / profile.distribution / "lib"),
        )
        environment.insert("FASTRTPS_DEFAULT_PROFILES_FILE", str(DDS_PROFILE))
        process.setProcessEnvironment(environment)
        process.setProgram(str(installation.python_launcher))
        args = [str(SCENE_SCRIPT), "--request", str(request)]
        if headless:
            args.append("--headless")
        if build_only:
            args.append("--build-only")
        process.setArguments(args)
        process.readyReadStandardOutput.connect(lambda: self._read(profile.id, process))
        process.finished.connect(lambda code, status: self._finished(profile.id, process, code))
        process.errorOccurred.connect(
            lambda error: self.error.emit(profile.id, process.errorString())
        )
        self.processes[profile.id] = process
        self.requests[profile.id] = str(request)
        self.tails[profile.id] = ""
        process.start()
        self.changed.emit(profile.id)
        return process

    def _read(self, backend_id, process):
        text = bytes(process.readAllStandardOutput()).decode(errors="replace")
        self.tails[backend_id] = (self.tails.get(backend_id, "") + text)[-100000:]
        self.log.emit(backend_id, text[-10000:])

    def _finished(self, backend_id, process, code):
        self._read(backend_id, process)
        if self.processes.get(backend_id) is process:
            self.processes.pop(backend_id, None)
        if code:
            self.error.emit(backend_id, f"Isaac process exited with code {code}; see process log")
        self.changed.emit(backend_id)
        process.deleteLater()

    def stop(self, backend_id):
        # Only processes created and retained here can be stopped; no process-name killing.
        process = self.processes.get(backend_id)
        if process:
            process.terminate()

    def status(self, backend_id):
        process = self.processes.get(backend_id)
        return process.state().name if process else "Not owned / stopped"
