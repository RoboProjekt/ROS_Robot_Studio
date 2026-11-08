"""DDS isolation over a common host-loopback network, plus a read-only hardware fixture."""

import io
import json
import tarfile
import time
from pathlib import Path
from uuid import uuid4

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.backends.manager import ROSBackendManager
from ros_robot_studio.backends.profiles import new_profile
from ros_robot_studio.core.robot_state import RobotStateManager
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.ros_backend import ROS2ExecutionBackend
from ros_robot_studio.runtime.manager import OWNER, VERSION, RuntimeManager

FIXTURE = """import json,time,rclpy,os
from rclpy.node import Node
from sensor_msgs.msg import JointState
config=json.load(open('/tmp/fixture.json'));rclpy.init()
print('FIXTURE',os.getpid(),os.getenv('ROS_DOMAIN_ID'),os.getenv('FASTRTPS_DEFAULT_PROFILES_FILE'),flush=True)
node=Node(config['node']);publisher=node.create_publisher(JointState,config['state_topic'],10)
count=0
open('/tmp/command-count','w').write('0')
def command(message):
 global count
 count+=1;open('/tmp/command-count','w').write(str(count))
node.create_subscription(JointState,config['command_topic'],command,10)
def publish():
 message=JointState();message.header.stamp=node.get_clock().now().to_msg()
 message.name=list(config['q']);message.position=list(config['q'].values());publisher.publish(message)
node.create_timer(0.04,publish)
node.create_timer(5,lambda:print('GRAPH',node.get_node_names(),publisher.get_subscription_count(),flush=True))
rclpy.spin(node)
"""


def archive(files):
    stream = io.BytesIO()
    with tarfile.open(fileobj=stream, mode="w") as target:
        for name, text in files.items():
            data = text.encode()
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o644
            target.addfile(info, io.BytesIO(data))
    return stream.getvalue()


def main():
    app = QApplication([])
    runtime = RuntimeManager()
    status = runtime.status()
    assert "humble" in status.installed, status
    p, model, _ = pick_place_demo(prefer_so101=False)
    states = RobotStateManager()
    states.bind(model, p.robot, model.default_configuration())
    manager = ROSBackendManager(runtime, states)
    manager.load([], p.robot)
    errors = []
    manager.failed.connect(errors.append)
    client = runtime.client_factory()
    fixtures = []
    profiles = []
    observed = {}
    token = uuid4().hex[:12]
    state_topic = "/studio/testing_" + token + "/joint_states"
    command_topic = "/studio/testing_" + token + "/joint_command"

    def wait(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while not predicate():
            app.processEvents()
            QTest.qWait(20)
            assert not errors, errors
            assert time.monotonic() < deadline, {
                s.profile.domain_id: (s.status, s.error) for s in manager.sessions.values()
            }

    try:
        for domain, value, kind in (
            (1, 0.03, "generic_ros"),
            (2, 0.06, "generic_ros"),
            (0, 0.09, "real_robot"),
        ):
            config = dict(
                node="studio_fixture_" + token + "_" + str(domain),
                state_topic=state_topic,
                command_topic=command_topic,
                q=dict(model.default_configuration(), axis_x=value),
            )
            container = client.containers.create(
                runtime.specs["humble"].image,
                entrypoint=[
                    "/bin/bash",
                    "-c",
                    "source /opt/ros/humble/setup.bash; exec python3 /tmp/fixture.py",
                ],
                network_mode="host",
                environment={
                    "ROS_DOMAIN_ID": str(domain),
                    "ROS_LOCALHOST_ONLY": "0",
                    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
                    "FASTRTPS_DEFAULT_PROFILES_FILE": "/opt/studio/fastdds_localhost.xml",
                },
                labels={
                    OWNER: VERSION,
                    "studio.integration": "true",
                    "studio.test_role": "domain-fixture",
                },
                cap_drop=["ALL"],
                security_opt=["no-new-privileges:true"],
                mem_limit="512m",
                nano_cpus=1000000000,
            )
            fixtures.append(container.id)
            container.put_archive(
                "/tmp", archive({"fixture.py": FIXTURE, "fixture.json": json.dumps(config)})
            )
            container.start()
            profile = manager.add(
                new_profile(
                    kind,
                    domain_id=domain,
                    network="host",
                    network_confirmed=True,
                    discovery="localhost",
                    state_topic=state_topic,
                    command_topic=command_topic,
                )
            )
            profiles.append(profile)
            observed[domain] = []
            manager.connect_profile(profile.id)
        wait(
            lambda: len(manager.sessions) == 3
            and all(
                "joint_state_feedback" in s.capabilities.available and s.graph.get("nodes")
                for s in manager.sessions.values()
            )
        )
        for _ in range(30):
            app.processEvents()
            QTest.qWait(30)
            for profile in profiles:
                snapshot = states.backend_snapshot(profile.id)
                if snapshot and snapshot.valid:
                    observed[profile.domain_id].append(snapshot.positions["axis_x"])
        for profile, value in zip(profiles, (0.03, 0.06, 0.09)):
            assert len(observed[profile.domain_id]) >= 20
            assert all(abs(q - value) < 1e-8 for q in observed[profile.domain_id])
            graph = manager.sessions[profile.id].graph
            for other in profiles:
                name = "/studio_fixture_" + token + "_" + str(other.domain_id)
                assert (name in graph["nodes"]) == (other.id == profile.id), (
                    profile.domain_id,
                    graph["nodes"],
                )
        hardware = profiles[2]
        manager.activate(hardware.id)
        assert states.snapshot().backend == "ros_read_only" and states.snapshot().ros_domain == 0
        session = manager.sessions[hardware.id]
        backend = ROS2ExecutionBackend(states)
        backend.connect_backend(
            session.transport, model, hardware.execution_config(), session.payload, states.lease()
        )
        try:
            backend.send("goal")
        except ConnectionError:
            pass
        else:
            raise AssertionError("Hardware motion was allowed")
        try:
            session.transport.send({"op": "publish", "topic": command_topic, "msg": {}})
        except PermissionError:
            pass
        else:
            raise AssertionError("Raw read-only publish was allowed")
        backend.disconnect_backend()
        backend.deleteLater()
        counts = []
        for identity in fixtures:
            code, data = runtime._owned(client, identity).exec_run(["cat", "/tmp/command-count"])
            assert code == 0
            counts.append(int(data))
        assert counts == [0, 0, 0]
        manager.deactivate()
        manager.disconnect_all()
        wait(lambda: not manager.sessions and not manager.runner.busy)
        report = dict(
            domains=[1, 2, 0],
            network="shared host loopback, explicit UDP whitelist; no shared IPC",
            same_topic_across_domains=True,
            isolated_jointstates=True,
            separate_ros_graphs=True,
            separate_subscriptions=True,
            observations={str(d): len(v) for d, v in observed.items()},
            hardware_read_only=True,
            hardware_execution_blocked=True,
            raw_publish_blocked=True,
            command_spy_counts=counts,
            hardware_access=False,
        )
        Path("docs/backend-domain-verification.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print(json.dumps(report, indent=2), flush=True)
    except Exception:
        for profile in profiles:
            session = manager.sessions.get(profile.id)
            if session:
                print("DOMAIN_GRAPH", profile.domain_id, session.graph, flush=True)
        for identity in fixtures:
            print(
                "FIXTURE_LOG",
                identity,
                runtime._owned(client, identity).logs(tail=30).decode(errors="replace"),
                flush=True,
            )
        for session in manager.sessions.values():
            print(
                "ADAPTER_LOG",
                runtime._owned(client, session.container_id).logs(tail=15).decode(errors="replace"),
                flush=True,
            )
        raise
    finally:
        errors.clear()
        manager.shutdown()
        wait(lambda: not manager.sessions and not manager.runner.busy)
        for identity in fixtures:
            runtime.stop(identity)
        client.close()
        manager.deleteLater()
        app.processEvents()


if __name__ == "__main__":
    main()
