"""Container-only ROS launch: bounded recovery for bridge/discovery child failures."""

import os

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, TimerAction
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    restarts = {"rosbridge_websocket": 0, "rosapi": 0}

    def exited(name, event, context):
        if context.is_shutdown:
            return []
        if restarts[name] >= 2:
            reason = f"STUDIO_ROS_CHILD_FAILED {name}: restart budget exhausted"
            return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
        restarts[name] += 1
        message = (
            f"STUDIO_ROS_CHILD_RESTART {name} {restarts[name]}/2 after exit {event.returncode}"
        )
        return [LogInfo(msg=message), TimerAction(period=1.0, actions=[create(name)])]

    def create(name):
        parameters = (
            {
                "port": int(os.environ.get("STUDIO_BRIDGE_PORT", "9090")),
                "address": os.environ.get("STUDIO_BRIDGE_ADDRESS", "0.0.0.0"),
            }
            if name == "rosbridge_websocket"
            else {}
        )
        return Node(
            package="rosbridge_server" if name == "rosbridge_websocket" else "rosapi",
            executable="rosbridge_websocket" if name == "rosbridge_websocket" else "rosapi_node",
            name=name,
            parameters=[parameters] if parameters else [],
            output="screen",
            additional_env={"PYTHONFAULTHANDLER": "1"},
            on_exit=lambda event, context: exited(name, event, context),
        )

    return LaunchDescription([create("rosbridge_websocket"), create("rosapi")])
