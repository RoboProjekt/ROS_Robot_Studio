from unittest.mock import MagicMock

import pytest

from docker.errors import NotFound
from ros_robot_studio.core.models import RuntimeConfig
from ros_robot_studio.runtime.manager import OWNER, RuntimeManager


def test_docker_unavailable():
    def fail():
        raise PermissionError("permission denied")

    status = RuntimeManager(fail).status()
    assert not status.available
    assert "permission denied" in status.message


def test_runtime_status_and_cached_images():
    client = MagicMock()
    client.images.get.side_effect = [
        MagicMock(labels={OWNER: "0.1.0"}),
        NotFound("missing"),
        NotFound("missing"),
    ]
    client.containers.list.return_value = [MagicMock(id="owned", status="running")]
    status = RuntimeManager(lambda: client).status()
    assert status.available
    assert status.installed == ["humble"]
    assert status.containers == {"owned": "running"}
    client.close.assert_called_once()


def test_start_constrained_and_loopback_only():
    client = MagicMock()
    client.images.get.return_value = MagicMock(id="image", labels={OWNER: "0.1.0"})
    client.containers.run.return_value.id = "new-owned-container"
    manager = RuntimeManager(lambda: client)
    assert manager.start("humble", "project", RuntimeConfig()) == "new-owned-container"
    kwargs = client.containers.run.call_args.kwargs
    assert kwargs["ports"] == {"9090/tcp": ("127.0.0.1", 9090)}
    assert kwargs["cap_drop"] == ["ALL"]
    assert kwargs["mem_limit"] == "2048m"
    assert "volumes" not in kwargs and "privileged" not in kwargs


def test_refuse_to_stop_foreign_container():
    client = MagicMock()
    client.containers.get.return_value.labels = {}
    manager = RuntimeManager(lambda: client)
    with pytest.raises(ValueError, match="not owned"):
        manager.stop("foreign")
    client.containers.get.return_value.stop.assert_not_called()


def test_stop_owned_container_and_cleanup():
    client = MagicMock()
    client.containers.get.return_value.labels = {OWNER: "0.1.0"}
    RuntimeManager(lambda: client).stop("owned")
    client.containers.get.return_value.stop.assert_called_once_with(timeout=5)
    client.containers.get.return_value.remove.assert_called_once_with()


def test_remove_referenced_runtime_refuses_before_connecting():
    factory = MagicMock()
    with pytest.raises(ValueError, match="referenced"):
        RuntimeManager(factory).remove_image("humble", {"humble"})
    factory.assert_not_called()


def test_runtime_health():
    client = MagicMock()
    container = client.containers.get.return_value
    container.labels = {OWNER: "0.1.0"}
    container.status = "running"
    container.attrs = {"State": {"Health": {"Status": "healthy"}}}
    assert RuntimeManager(lambda: client).health("owned") == {
        "status": "running",
        "health": "healthy",
    }
