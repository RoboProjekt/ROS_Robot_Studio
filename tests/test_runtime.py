from unittest.mock import MagicMock

import pytest

from docker.errors import NotFound
from ros_robot_studio.core.models import RuntimeConfig
from ros_robot_studio.runtime.manager import OWNER, RuntimeManager, runtime_source_hash


def test_docker_unavailable():
    def fail():
        raise PermissionError("permission denied")

    status = RuntimeManager(fail).status()
    assert not status.available
    assert "permission denied" in status.message


def test_runtime_status_and_cached_images():
    client = MagicMock()
    client.images.get.side_effect = [
        MagicMock(labels={OWNER: "0.1.0", "studio.runtime_source": runtime_source_hash()}),
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
    client.images.get.return_value = MagicMock(
        id="image", labels={OWNER: "0.1.0", "studio.runtime_source": runtime_source_hash()}
    )
    client.containers.create.return_value.id = "new-owned-container"
    manager = RuntimeManager(lambda: client)
    assert manager.start("humble", "project", RuntimeConfig()) == "new-owned-container"
    kwargs = client.containers.create.call_args.kwargs
    assert kwargs["ports"] == {"9090/tcp": ("127.0.0.1", 9090)}
    assert kwargs["cap_drop"] == ["ALL"]
    client.networks.create.assert_called_once()
    assert kwargs["network"] == client.networks.create.return_value.name
    client.containers.create.return_value.start.assert_called_once()
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
    client.containers.get.return_value.labels = {
        OWNER: "0.1.0",
        "studio.runtime_source": runtime_source_hash(),
    }
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
    container.labels = {OWNER: "0.1.0", "studio.runtime_source": runtime_source_hash()}
    container.status = "running"
    container.attrs = {"State": {"Health": {"Status": "healthy"}}}
    assert RuntimeManager(lambda: client).health("owned") == {
        "status": "running",
        "health": "healthy",
    }


def test_host_network_requires_simulation_confirmation():
    client = MagicMock()
    client.images.get.return_value = MagicMock(
        id="image", labels={OWNER: "0.1.0", "studio.runtime_source": runtime_source_hash()}
    )
    manager = RuntimeManager(lambda: client)
    with pytest.raises(ValueError, match="Host networking"):
        manager.start(
            "humble", "project", RuntimeConfig(network="host", external_network_confirmed=True)
        )
    client.containers.create.assert_not_called()


def test_runtime_source_mismatch_requires_rebuild():
    client = MagicMock()
    client.images.get.return_value = MagicMock(
        labels={OWNER: "0.1.0", "studio.runtime_source": "old"}
    )
    with pytest.raises(ValueError, match="rebuild"):
        RuntimeManager(lambda: client).start("humble", "p", RuntimeConfig())
    client.containers.create.assert_not_called()


def test_failed_start_cleans_owned_network():
    client = MagicMock()
    client.images.get.return_value = MagicMock(
        id="image", labels={OWNER: "0.1.0", "studio.runtime_source": runtime_source_hash()}
    )
    client.containers.create.side_effect = RuntimeError("container creation failed")
    with pytest.raises(RuntimeError):
        RuntimeManager(lambda: client).start("humble", "p", RuntimeConfig())
    client.networks.create.return_value.remove.assert_called_once()


def test_stop_removes_only_owned_network():
    client = MagicMock()
    client.containers.get.return_value.labels = {OWNER: "0.1.0", "studio.network": "own-network"}
    network = client.networks.get.return_value
    network.attrs = {"Labels": {OWNER: "0.1.0"}}
    RuntimeManager(lambda: client).stop("own-container")
    network.remove.assert_called_once()
    network.remove.reset_mock()
    network.attrs = {"Labels": {}}
    RuntimeManager(lambda: client).stop("own-container")
    network.remove.assert_not_called()
