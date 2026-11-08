"""Test JointState command adapter against a private ROS scene emulator, not Isaac Sim."""

import io
import json
import socket
import tarfile
import time
from pathlib import Path

from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication

from ros_robot_studio.core.models import ExecutionConfig, RuntimeConfig
from ros_robot_studio.core.robot_state import RobotStateManager
from ros_robot_studio.motion.demos import pick_place_demo
from ros_robot_studio.motion.positions import PositionManager
from ros_robot_studio.motion.program import TaskCompiler
from ros_robot_studio.motion.ros_backend import ROS2ExecutionBackend
from ros_robot_studio.motion.schema import MotionTask, TaskStep
from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport
from ros_robot_studio.runtime.manager import RuntimeManager
from ros_robot_studio.runtime.robot_config import simulation_config

SCENE = """import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init()
c=json.loads(Path('/tmp/studio_robot.json').read_text())
class Scene(Node):
 def __init__(self):
  super().__init__('studio_test_scene_emulator');self.q=c['initial'];self.publisher=self.create_publisher(JointState,c['state_topic'],10)
  self.create_subscription(JointState,c['command_topic'],self.command,10);self.create_timer(.02,self.publish)
 def command(self,message):self.q=dict(zip(message.name,message.position))
 def publish(self):
  message=JointState();message.header.stamp=self.get_clock().now().to_msg();message.name=list(self.q);message.position=list(self.q.values());self.publisher.publish(message)
rclpy.spin(Scene())
"""


def main():
    app = QApplication([])
    manager = RuntimeManager()
    project, model, _ = pick_place_demo(prefer_so101=False)
    execution = ExecutionConfig(
        profile="external_simulation",
        external_adapter="joint_state_command",
        external_simulation_confirmed=True,
        state_topic="/studio_test/joint_states",
        command_topic="/studio_test/joint_command",
        action="/studio_test/follow_joint_trajectory",
    )
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        port = probe.getsockname()[1]
    payload = simulation_config(model, model.default_configuration(), execution)
    identifier = None
    client = None
    transport = None
    backend = None
    try:
        identifier = manager.start(
            "humble",
            "external-adapter-acceptance",
            RuntimeConfig(port=port),
            integration=True,
            robot_config=payload,
        )
        client = manager.client_factory()
        container = manager._owned(client, identifier)
        archive = io.BytesIO()
        data = SCENE.encode()
        with tarfile.open(fileobj=archive, mode="w") as tar:
            entry = tarfile.TarInfo("studio_scene_emulator.py")
            entry.size = len(data)
            entry.mode = 0o644
            tar.addfile(entry, io.BytesIO(data))
        container.put_archive("/tmp", archive.getvalue())
        container.exec_run(
            [
                "/bin/bash",
                "-c",
                "source /opt/ros/$ROS_DISTRO/setup.bash; exec python3 /tmp/studio_scene_emulator.py",
            ],
            detach=True,
        )
        states = RobotStateManager()
        states.bind(model, project.robot, model.default_configuration(), "base_link", "tool0")
        lease = states.activate_source("external_simulation", execution.state_topic)
        transport = RosbridgeTransport(f"ws://127.0.0.1:{port}")
        backend = ROS2ExecutionBackend(states)
        errors = []
        backend.error.connect(errors.append)
        transport.message.connect(
            lambda topic, message: states.joint_state(lease, message)
            if topic == execution.state_topic
            else None
        )
        transport.subscribe(execution.state_topic, "sensor_msgs/msg/JointState")
        backend.connect_backend(transport, model, execution, payload, lease)
        transport.connect_transport()

        def wait(predicate):
            deadline = time.monotonic() + 35
            while not predicate():
                app.processEvents()
                QTest.qWait(10)
                if errors:
                    raise AssertionError("; ".join(errors))
                if time.monotonic() > deadline:
                    raise TimeoutError(manager.logs(identifier))

        wait(backend.ready)
        from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine

        chain = UniversalKinematicsEngine(model).create_chain(
            "base_link", "tool0", gripper_joints=["finger_slide"]
        )
        current = backend.read_joint_positions()
        target = dict(current, axis_x=0.03, axis_z=0.02)
        positions = PositionManager()
        position = positions.capture_joint("External target", chain, target)
        positions.add(position)
        plan = TaskCompiler(chain, positions).compile(
            MotionTask(
                name="External command check",
                steps=[TaskStep(action="move_joint", position_id=position.id)],
            ),
            current,
        )
        backend.execute(plan)
        wait(lambda: backend.status == "succeeded")
        for n, value in target.items():
            assert abs(states.snapshot().positions[n] - value) <= execution.goal_tolerance
        report = dict(
            transport="native FollowJointTrajectory → JointState commands → observed JointState feedback",
            scene="private ROS emulator inside owned container",
            adapter="IsaacSimROSAdapter",
            success=True,
            actual_isaac_sim="NOT TESTED",
            hardware_access=False,
        )
        Path("docs/external-adapter-verification.json").write_text(
            json.dumps(report, indent=2) + "\n"
        )
        print(json.dumps(report, indent=2))
    finally:
        if backend:
            backend.disconnect_backend()
        if transport:
            transport.disconnect_transport()
        if client:
            client.close()
        if identifier:
            manager.stop(identifier)
        app.processEvents()


if __name__ == "__main__":
    main()
