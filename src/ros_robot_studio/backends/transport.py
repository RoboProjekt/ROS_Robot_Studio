"""Read-only policy enforced at the outbound protocol boundary, not just buttons."""

from ros_robot_studio.ros_bridge.websocket import RosbridgeTransport


class ReadOnlyRosbridgeTransport(RosbridgeTransport):
    DISCOVERY_SERVICES = frozenset(
        {
            "/rosapi/topics",
            "/rosapi/nodes",
            "/rosapi/publishers",
            "/rosapi/subscribers",
            "/rosapi/topic_type",
            "/rosapi/node_details",
            "/rosapi/services",
            "/rosapi/service_type",
            "/rosapi/get_action_servers",
        }
    )

    def send(self, data):
        operation = data.get("op")
        if operation not in ("subscribe", "unsubscribe") and not (
            operation == "call_service" and data.get("service") in self.DISCOVERY_SERVICES
        ):
            raise PermissionError(
                "Read-only backend blocks publishes, goals and non-discovery services"
            )
        return super().send(data)

    def publish(self, *args, **kwargs):
        raise PermissionError("Read-only backend cannot publish commands")
