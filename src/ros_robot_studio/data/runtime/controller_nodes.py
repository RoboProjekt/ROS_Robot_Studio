"""ROS-only simulator and native FollowJointTrajectory action gateway. No hardware code."""

import json
import os
import threading
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from ros_robot_studio.robots.urdf import parse_urdf
from ros_robot_studio.runtime.controller_core import SimulatedTrajectory


class SimulationController(Node):
    def __init__(self, config):
        super().__init__("studio_simulated_robot_controller", namespace=config.get("namespace", ""))
        self.config = config
        self.model = parse_urdf(config["urdf"])
        self.q = dict(config["initial"])
        self.v = {n: 0.0 for n in self.q}
        self.mapping = {n: config.get("joint_mapping", {}).get(n, n) for n in self.q}
        self.model.configuration(self.q)
        self.lock = threading.RLock()
        self.reserved = False
        self.active = None
        self.validated = {}
        self.group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(JointState, config["state_topic"], 10)
        self.action = ActionServer(
            self,
            FollowJointTrajectory,
            config["action"],
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.group,
        )
        self.create_timer(0.02, self.publish, callback_group=self.group)

    def normalized(self, message):
        data = message_to_ordereddict(message)
        inverse = {v: k for k, v in self.mapping.items()}
        data["joint_names"] = [inverse[n] for n in data["joint_names"]]
        return data

    def goal(self, request):
        with self.lock:
            if self.reserved:
                return GoalResponse.REJECT
            try:
                trajectory = SimulatedTrajectory(
                    self.normalized(request.trajectory),
                    self.model,
                    self.q,
                    self.config.get("start_tolerance", 0.01),
                )
            except (ValueError, KeyError, TypeError) as exc:
                self.get_logger().warning(str(exc))
                return GoalResponse.REJECT
            self.validated[id(request)] = trajectory
            self.reserved = True
            return GoalResponse.ACCEPT

    def publish(self):
        with self.lock:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            resolved = self.model.configuration(self.q)
            message.name = [self.config.get("joint_mapping", {}).get(n, n) for n in resolved]
            message.position = list(resolved.values())
            message.velocity = [
                self.v.get(n, self.model.driver(n)[1] * self.v.get(self.model.driver(n)[0], 0.0))
                for n in resolved
            ]
        self.publisher.publish(message)

    def execute(self, goal):
        result = FollowJointTrajectory.Result()
        self.active = goal
        try:
            trajectory = SimulatedTrajectory(
                self.normalized(goal.request.trajectory),
                self.model,
                self.q,
                self.config.get("start_tolerance", 0.01),
            )
            started = time.monotonic()
            while rclpy.ok():
                if goal.is_cancel_requested:
                    with self.lock:
                        self.v = {n: 0.0 for n in self.q}
                    goal.canceled()
                    result.error_string = "Simulation cancelled"
                    return result
                elapsed = min(time.monotonic() - started, trajectory.duration)
                with self.lock:
                    self.q, self.v, _ = trajectory.sample(elapsed)
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = trajectory.names
                feedback.actual.positions = [self.q[n] for n in trajectory.names]
                feedback.actual.velocities = [self.v[n] for n in trajectory.names]
                feedback.actual.time_from_start.sec = int(elapsed)
                feedback.actual.time_from_start.nanosec = int((elapsed - int(elapsed)) * 1e9)
                goal.publish_feedback(feedback)
                if elapsed >= trajectory.duration:
                    break
                time.sleep(0.02)
            with self.lock:
                self.v = {n: 0.0 for n in self.q}
            goal.succeed()
            result.error_code = 0
            result.error_string = "Simulation trajectory completed"
        except Exception as exc:
            goal.abort()
            result.error_code = -1
            result.error_string = str(exc)
        finally:
            with self.lock:
                self.reserved = False
                self.active = None
                self.validated.clear()
        return result


class ActionGateway(Node):
    def __init__(self, config, controller=None):
        super().__init__("studio_execution_gateway", namespace=config.get("namespace", ""))
        self.config = config
        self.controller = controller
        self.client = ActionClient(self, FollowJointTrajectory, config["action"])
        self.events = self.create_publisher(String, "/studio/execution/events", 10)
        self.create_subscription(String, "/studio/execution/command", self.command, 10)
        self.create_subscription(String, "/studio/execution/heartbeat", self.heartbeat, 10)
        self.goal_handle = None
        self.pending = None
        self.last_heartbeat = time.monotonic()
        self.cancel_pending = False
        self.create_timer(0.25, self.status)

    def emit(self, kind, **data):
        payload = dict(
            kind=kind,
            session=self.config["session"],
            model_signature=self.config["model_signature"],
            simulation=self.config["profile"],
            **data,
        )
        self.events.publish(String(data=json.dumps(payload)))

    def heartbeat(self, message):
        if message.data == self.config["session"]:
            self.last_heartbeat = time.monotonic()

    def status(self):
        if self.pending and time.monotonic() - self.last_heartbeat > 2.0:
            self.cancel_pending = True
            if self.goal_handle:
                self.goal_handle.cancel_goal_async()
        self.emit(
            "controller",
            available=self.client.server_is_ready(),
            busy=self.pending is not None,
            action=self.config["action"],
        )

    def command(self, message):
        try:
            if len(message.data) > 950000:
                raise ValueError("Command too large")
            data = json.loads(message.data)
            if (
                data.get("session") != self.config["session"]
                or data.get("model_signature") != self.config["model_signature"]
            ):
                raise ValueError("Session/model mismatch")
            operation = data.get("operation")
            if operation == "cancel":
                self.cancel_pending = True
                if self.goal_handle:
                    self.goal_handle.cancel_goal_async()
                return
            if operation != "goal":
                raise ValueError("Unknown command")
            if self.pending or not self.client.server_is_ready():
                raise ValueError("Controller unavailable or busy")
            self.pending = data["id"]
            self.cancel_pending = False
            self.last_heartbeat = time.monotonic()
            goal = FollowJointTrajectory.Goal()
            set_message_fields(goal.trajectory, data["trajectory"])
            future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
            future.add_done_callback(self.accepted)
        except Exception as exc:
            self.emit(
                "rejected", id=data.get("id", "") if "data" in locals() else "", message=str(exc)
            )

    def accepted(self, future):
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.emit("rejected", id=self.pending, message="Action server rejected goal")
                self.pending = None
                self.goal_handle = None
                return
            self.emit("accepted", id=self.pending)
            if self.cancel_pending:
                self.goal_handle.cancel_goal_async()
            self.goal_handle.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.emit("failed", id=self.pending, message=str(exc))
            self.pending = None
            self.goal_handle = None

    def feedback(self, message):
        feedback = message.feedback
        t = feedback.actual.time_from_start
        self.emit("feedback", id=self.pending, elapsed=t.sec + t.nanosec * 1e-9)

    def result(self, future):
        try:
            response = future.result()
            kind = (
                "succeeded"
                if response.status == GoalStatus.STATUS_SUCCEEDED
                and response.result.error_code == 0
                else "cancelled"
                if response.status == GoalStatus.STATUS_CANCELED
                else "failed"
            )
            self.emit(
                kind,
                id=self.pending,
                error_code=response.result.error_code,
                message=response.result.error_string,
            )
        except Exception as exc:
            self.emit("failed", id=self.pending, message=str(exc))
        self.pending = None
        self.goal_handle = None


def main():
    config = json.loads(
        Path(os.environ.get("STUDIO_MODEL_FILE", "/tmp/studio_robot.json")).read_text()
    )
    rclpy.init()
    executor = MultiThreadedExecutor(num_threads=4)
    controller = SimulationController(config) if config["profile"] == "managed_simulation" else None
    if (
        config["profile"] == "external_simulation"
        and config.get("external_adapter") == "joint_state_command"
    ):
        from external_adapter import IsaacSimROSAdapter

        controller = IsaacSimROSAdapter(config)
    gateway = ActionGateway(config, controller)
    nodes = [gateway] + ([controller] if controller else [])
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
