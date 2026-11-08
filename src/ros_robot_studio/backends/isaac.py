"""Local Isaac installation/process/scene management. No Isaac APIs in desktop Python."""

import hashlib
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from uuid import uuid4

from PySide6.QtCore import QObject, QProcess, QProcessEnvironment, QTimer, Signal

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
    def prepare(self, profile, model, directory, positions=None, existing_scene=None):
        from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
        from ros_robot_studio.robots.urdf import origin_transform

        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        if not model.source_path or not model.meshes:
            raise ValueError(
                "Isaac scene generation requires a resolved URDF with its original meshes"
            )
        q = dict(positions or model.default_configuration())
        model.configuration(q)
        if any(name != remote for name, remote in profile.joint_mapping.items()):
            raise ValueError(
                "Generated/imported Isaac scenes retain URDF joint names; use identity mapping or connect an existing external ROS scene"
            )
        signature = UniversalKinematicsEngine(model).signature
        if profile.model_signature and signature != profile.model_signature:
            raise ValueError("Backend model signature differs from selected robot")
        request = dict(
            schema_version=1,
            request_id=uuid4().hex,
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
                    kind=j.kind,
                    lower=j.lower,
                    upper=j.upper,
                    axis=list(j.axis),
                    rpy=list(j.rpy),
                    axis_parent=(origin_transform(j.xyz, j.rpy)[:3, :3] @ j.axis).tolist(),
                    effort=j.effort,
                )
                for j in model.joints
                if j.movable
            },
            expected_link_positions={
                n: matrix[:3, 3].tolist() for n, matrix in model.transforms(q).items()
            },
        )
        from ros_robot_studio.robots.geometry import resolve_mesh_uri

        files = [model.source_path] + [
            resolve_mesh_uri(reference, model.source_path.parent)
            for reference in model.mesh_references
        ]
        request["source_files"] = {
            str(file.resolve()): hashlib.sha256(file.read_bytes()).hexdigest() for file in files
        }
        if existing_scene:
            scene = Path(existing_scene).expanduser().resolve()
            if not scene.is_file() or scene.suffix.lower() not in (".usd", ".usda", ".usdc"):
                raise ValueError("Select an existing USD scene")
            if scene in (directory / "studio-scene.usd", directory / "scene.usd"):
                raise ValueError("Use a separate output directory when opening an existing scene")
            request["existing_scene"] = str(scene)
            request["source_files"][str(scene)] = hashlib.sha256(scene.read_bytes()).hexdigest()
        path = directory / "scene-request.json"
        atomic_text(path, json.dumps(request, indent=2))
        return path

    def validate_result(self, request_path):
        request = json.loads(Path(request_path).read_text())
        report = json.loads(
            (Path(request["output_directory"]) / "scene-validation.json").read_text()
        )
        if (
            report.get("request_id") != request.get("request_id")
            or report["model_signature"] != request["model_signature"]
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
    ready = Signal(str, object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.processes = {}
        self.requests = {}
        self.tails = {}
        self.ready_reports = {}
        self.stopping = set()

    def start(
        self, profile, installation, request, headless=False, build_only=False, auto_play=False
    ):
        data = json.loads(Path(request).read_text())
        if (
            data.get("backend_id") != profile.id
            or data.get("domain_id") != profile.domain_id
            or (profile.model_signature and data.get("model_signature") != profile.model_signature)
        ):
            raise ValueError(
                "Scene request does not match this backend/domain/model; create a new request"
            )
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
        environment.insert("ROS_LOCALHOST_ONLY", "0")
        environment.insert("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")
        environment.insert(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            str(
                DDS_PROFILE.with_name("fastdds_localhost.xml")
                if profile.discovery == "localhost"
                else DDS_PROFILE
            ),
        )
        process.setProcessEnvironment(environment)
        process.setProgram(sys.executable)
        args = [
            str(SCENE_SCRIPT.with_name("process_launcher.py")),
            str(installation.python_launcher),
            str(SCENE_SCRIPT),
            "--request",
            str(request),
        ]
        if auto_play:
            args.append("--auto-play")
        if headless:
            args.append("--headless")
        if build_only:
            args.append("--build-only")
        process.setArguments(args)
        process.readyReadStandardOutput.connect(lambda: self._read(profile.id, process))
        process.finished.connect(lambda code, status: self._finished(profile.id, process, code))
        process.errorOccurred.connect(lambda error: self._process_error(profile.id, process))
        self.processes[profile.id] = process
        self.requests[profile.id] = str(request)
        self.tails[profile.id] = ""
        self.ready_reports.pop(profile.id, None)
        self.stopping.discard(profile.id)
        process.start()
        self.changed.emit(profile.id)
        return process

    def _read(self, backend_id, process):
        text = bytes(process.readAllStandardOutput()).decode(errors="replace")
        self.tails[backend_id] = (self.tails.get(backend_id, "") + text)[-100000:]
        self.log.emit(backend_id, text[-10000:])
        if backend_id not in self.ready_reports and "STUDIO_SCENE_READY " in self.tails[backend_id]:
            try:
                report = IsaacSimSceneManager().validate_result(self.requests[backend_id])
            except (OSError, ValueError, KeyError) as exc:
                self.error.emit(backend_id, "Isaac scene validation failed: " + str(exc))
            else:
                self.ready_reports[backend_id] = report
                self.ready.emit(backend_id, report)
                self.changed.emit(backend_id)

    def _finished(self, backend_id, process, code):
        self._read(backend_id, process)
        if self.processes.get(backend_id) is process:
            self.processes.pop(backend_id, None)
        if backend_id not in self.stopping and (code or backend_id not in self.ready_reports):
            self.error.emit(backend_id, f"Isaac process exited with code {code}; see process log")
        self.stopping.discard(backend_id)
        self.changed.emit(backend_id)
        process.deleteLater()

    def _process_error(self, backend_id, process):
        if self.processes.get(backend_id) is process and backend_id not in self.stopping:
            self.error.emit(backend_id, process.errorString())
            if process.state() == QProcess.ProcessState.NotRunning:
                self.processes.pop(backend_id, None)
                self.changed.emit(backend_id)
                process.deleteLater()

    def stop(self, backend_id):
        # Only processes created and retained here can be stopped; no process-name killing.
        process = self.processes.get(backend_id)
        if process:
            self.stopping.add(backend_id)
            process.terminate()
            # Kit can hang while shutting down a renderer. Only this retained
            # child process may be killed, never another simulator instance.
            QTimer.singleShot(15000, self, lambda: self._stop_timeout(backend_id, process))

    def _stop_timeout(self, backend_id, process):
        if (
            self.processes.get(backend_id) is process
            and process.state() != QProcess.ProcessState.NotRunning
        ):
            self.log.emit(
                backend_id, "Isaac did not stop within 15 seconds; terminating owned child.\n"
            )
            process.kill()

    def status(self, backend_id):
        process = self.processes.get(backend_id)
        return (
            ("Scene ready" if backend_id in self.ready_reports else process.state().name)
            if process
            else "Not owned / stopped"
        )

    def external_instances(self, installation):
        """Read-only discovery. Never acquire or stop processes found by name."""
        owned = {int(p.processId()) for p in self.processes.values() if p.processId()}
        found = []
        records = {}
        for directory in Path("/proc").glob("[0-9]*"):
            try:
                pid = int(directory.name)
                status = (directory / "status").read_text()
                parent = int(
                    next(
                        line.split()[1] for line in status.splitlines() if line.startswith("PPid:")
                    )
                )
                executable = (directory / "exe").resolve()
                command = (
                    (directory / "cmdline")
                    .read_bytes()
                    .replace(b"\0", b" ")
                    .decode(errors="replace")
                )
                records[pid] = (parent, executable, command)
            except (OSError, ValueError, StopIteration):
                pass
        for pid, (_, executable, command) in records.items():
            if not executable.is_relative_to(installation.path):
                continue
            if "python" not in executable.name and "kit" not in executable.name:
                continue
            current, seen = pid, set()
            while current in records and current not in seen and current not in owned:
                seen.add(current)
                current = records[current][0]
            if current not in owned:
                found.append(dict(pid=pid, executable=str(executable), command=command[:250]))
        return found
