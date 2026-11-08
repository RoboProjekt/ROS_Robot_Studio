"""Optional Isaac-compatible JointState command adapter. Observed states determine success."""

import threading
import time

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import JointState

from ros_robot_studio.robots.urdf import parse_urdf
from ros_robot_studio.runtime.controller_core import SimulatedTrajectory


class IsaacSimROSAdapter(Node):
    def __init__(self, config):
        super().__init__(
            "studio_external_simulation_adapter",
            namespace=config.get("namespace", ""),
            cli_args=["--ros-args", "-r", "/clock:=" + config.get("clock_topic", "/clock")],
        )
        if not config.get("external_simulation_confirmed"):
            raise ValueError("Explicit external simulation profile confirmation required")
        if config.get("use_sim_time"):
            self.set_parameters([Parameter("use_sim_time", value=True)])
        self.config = config
        self.model = parse_urdf(config["urdf"])
        self.q = None
        self.received = 0.0
        self.lock = threading.RLock()
        self.busy = False
        self.mapping = {
            n: config.get("joint_mapping", {}).get(n, n) for n in self.model.default_configuration()
        }
        self.commands = self.create_publisher(JointState, config["command_topic"], 10)
        group = ReentrantCallbackGroup()
        self.create_subscription(
            JointState, config["state_topic"], self.state, 10, callback_group=group
        )
        self.action = ActionServer(
            self,
            FollowJointTrajectory,
            config["action"],
            execute_callback=self.execute,
            goal_callback=self.goal,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def state(self, message):
        try:
            incoming = dict(zip(message.name, message.position))
            if len(incoming) != len(message.name):
                return
            q = {n: incoming[remote] for n, remote in self.mapping.items()}
            self.model.configuration(q)
            with self.lock:
                self.q = q
                self.received = time.monotonic()
        except (ValueError, KeyError):
            return

    def normalized(self, trajectory):
        data = message_to_ordereddict(trajectory)
        inverse = {v: k for k, v in self.mapping.items()}
        data["joint_names"] = [inverse[n] for n in data["joint_names"]]
        return data

    def goal(self, request):
        with self.lock:
            if self.busy or self.q is None or time.monotonic() - self.received > 1:
                return GoalResponse.REJECT
            try:
                SimulatedTrajectory(
                    self.normalized(request.trajectory),
                    self.model,
                    self.q,
                    self.config.get("start_tolerance", 0.01),
                )
            except (ValueError, KeyError):
                return GoalResponse.REJECT
            self.busy = True
            return GoalResponse.ACCEPT

    def execute(self, goal):
        result = FollowJointTrajectory.Result()
        try:
            trajectory = SimulatedTrajectory(
                self.normalized(goal.request.trajectory),
                self.model,
                self.q,
                self.config.get("start_tolerance", 0.01),
            )
            started = self.get_clock().now().nanoseconds * 1e-9
            previous = started
            wall_started = time.monotonic()
            while True:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now < previous:
                    raise ValueError("Simulation clock reset; reconnect and replan")
                previous = now
                elapsed = max(0.0, now - started)
                if goal.is_cancel_requested:
                    goal.canceled()
                    result.error_string = "Command stream cancelled"
                    return result
                if time.monotonic() - self.received > 1:
                    raise ValueError("External feedback stale; command stream stopped")
                if time.monotonic() - wall_started > trajectory.duration + 15:
                    raise TimeoutError("External simulation paused or target not reached")
                q, v, _ = trajectory.sample(elapsed)
                if elapsed < trajectory.duration:
                    message = JointState()
                    message.header.stamp = self.get_clock().now().to_msg()
                    message.name = [self.mapping[n] for n in q]
                    message.position = list(q.values())
                    message.velocity = list(v.values())
                    self.commands.publish(message)
                else:
                    target = trajectory.sample(trajectory.duration)[0]
                    if all(
                        abs(self.q[n] - value) <= self.config.get("goal_tolerance", 0.002)
                        for n, value in target.items()
                    ):
                        break
                    message = JointState()
                    message.header.stamp = self.get_clock().now().to_msg()
                    message.name = [self.mapping[n] for n in target]
                    message.position = list(target.values())
                    self.commands.publish(message)
                feedback = FollowJointTrajectory.Feedback()
                feedback.joint_names = list(self.q)
                feedback.actual.positions = list(self.q.values())
                feedback.actual.time_from_start.sec = int(min(elapsed, trajectory.duration))
                goal.publish_feedback(feedback)
                time.sleep(0.02)
            goal.succeed()
            result.error_code = 0
        except Exception as exc:
            goal.abort()
            result.error_code = -1
            result.error_string = str(exc)
        finally:
            self.busy = False
        return result
