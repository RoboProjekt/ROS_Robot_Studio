"""Read-only Docker endpoint and effective-process diagnostics (never changes permissions)."""

from __future__ import annotations

import grp
import json
import os
import pwd
import shutil
import stat
import subprocess
from pathlib import Path

import docker


def endpoint():
    """Follow CLI context precedence; never silently connect to another daemon."""
    context = os.environ.get("DOCKER_CONTEXT", "")
    host = os.environ.get("DOCKER_HOST", "")
    if not context and host:
        return "environment", host, None
    if not context:
        try:
            config = Path(os.environ.get("DOCKER_CONFIG", Path.home() / ".docker"))
            context = json.loads((config / "config.json").read_text()).get(
                "currentContext", "default"
            )
        except (OSError, ValueError):
            context = "default"
    if not isinstance(context, str) or not context:
        raise ValueError("Invalid Docker context in local configuration")
    if context == "default":
        return context, "unix:///var/run/docker.sock", None
    try:
        result = subprocess.run(
            ["docker", "context", "inspect", context],
            capture_output=True,
            text=True,
            timeout=4,
            check=True,
        )
        item = json.loads(result.stdout)[0]
        entry = item["Endpoints"]["docker"]
        tls_path = item.get("Storage", {}).get("TLSPath", "")
        tls = None
        if entry["Host"].startswith("tcp://") and tls_path:
            directory = Path(tls_path) / "docker"
            if (directory / "ca.pem").exists():
                tls = docker.tls.TLSConfig(
                    client_cert=(str(directory / "cert.pem"), str(directory / "key.pem")),
                    ca_cert=str(directory / "ca.pem"),
                    verify=not entry.get("SkipTLSVerify", False),
                )
        return context, entry["Host"], tls
    except (OSError, ValueError, KeyError, IndexError, subprocess.SubprocessError) as exc:
        raise ValueError(f"Docker context {context!r} cannot be resolved: {exc}") from exc


def create_client():
    context, host, tls = endpoint()
    if context == "environment":
        return docker.from_env(timeout=15)
    return docker.DockerClient(base_url=host, tls=tls, timeout=15)


def snapshot():
    user = pwd.getpwuid(os.geteuid()).pw_name
    groups = sorted(set([os.getegid(), *os.getgroups()]))
    account = sorted(os.getgrouplist(user, pwd.getpwnam(user).pw_gid))

    def names(ids):
        return [grp.getgrgid(g).gr_name if _known_group(g) else str(g) for g in ids]

    data = dict(
        pid=os.getpid(),
        user=user,
        uid=os.geteuid(),
        effective_groups=names(groups),
        account_groups=names(account),
        docker_cli=shutil.which("docker") or "missing",
        engine="unknown",
        context="unknown",
        endpoint="unknown",
    )
    try:
        context, host, _ = endpoint()
        data.update(context=context, endpoint=host)
    except ValueError as exc:
        data["context_error"] = str(exc)
        return data
    if host.startswith("unix://"):
        path = Path(host.removeprefix("unix://"))
        data["socket"] = str(path)
        try:
            info = path.stat()
            data.update(
                socket_owner=pwd.getpwuid(info.st_uid).pw_name,
                socket_group=grp.getgrgid(info.st_gid).gr_name,
                socket_mode=f"{stat.S_IMODE(info.st_mode):04o}",
                socket_access=os.access(path, os.R_OK | os.W_OK, effective_ids=True),
            )
        except OSError:
            data["socket_mode"] = "missing"
        if str(path) in ("/var/run/docker.sock", "/run/docker.sock"):
            try:
                result = subprocess.run(
                    ["systemctl", "is-active", "docker"], capture_output=True, text=True, timeout=2
                )
                data["engine"] = result.stdout.strip() or "unknown"
            except (OSError, subprocess.SubprocessError):
                pass
    return data


def _known_group(identifier):
    try:
        grp.getgrgid(identifier)
        return True
    except KeyError:
        return False


def classify(data, error):
    text = str(error)
    if data.get("context_error"):
        return "invalid_context", data["context_error"] + ". Select a valid Docker context."
    denied = (
        isinstance(error, PermissionError)
        or "permission denied" in text.lower()
        or "permissionerror" in text.lower()
    )
    if denied:
        if (
            data.get("socket_group") == "docker"
            and "docker" in data["account_groups"]
            and "docker" not in data["effective_groups"]
        ):
            return "process_groups_inactive", (
                "Your account already belongs to docker, but this Studio process does not. "
                "Sign out of the desktop completely, sign in again, and launch Studio from the new session. "
                "Adding the group again will not fix this running process."
            )
        return "permission_denied", (
            "This process cannot access the selected Docker endpoint. Review its socket ownership, "
            "effective groups and any ACL in the diagnostics. Use the setup assistant to review access options."
        )
    if data.get("docker_cli") == "missing":
        return "not_installed", "Docker CLI is missing; open the setup assistant to install Docker."
    if data.get("engine") in ("inactive", "failed"):
        return "daemon_stopped", "Docker Engine is stopped. Start its service, then recheck."
    if data.get("socket_mode") == "missing":
        return (
            "socket_missing",
            "The selected Docker socket is missing. Check the context and start its Docker Engine.",
        )
    return (
        "api_unreachable",
        "Docker API is unreachable at the selected endpoint. Check Engine, context and transport.",
    )
