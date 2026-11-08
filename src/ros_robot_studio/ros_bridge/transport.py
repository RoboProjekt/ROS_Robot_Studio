"""Swappable Qt signal contract for ROS and explicitly simulated transports."""

from __future__ import annotations

import math
import time
from collections import deque
from typing import Any

from PySide6.QtCore import QObject, QTimer, Signal


class Transport(QObject):
    status = Signal(str)
    graph = Signal(dict)
    message = Signal(str, dict)
    error = Signal(str)
    connection_changed = Signal(bool)

    def connect_transport(self) -> None:
        raise NotImplementedError

    def disconnect_transport(self) -> None:
        raise NotImplementedError

    def subscribe(self, topic: str, message_type: str = "") -> None:
        raise NotImplementedError

    def unsubscribe(self, topic: str) -> None:
        raise NotImplementedError

    def refresh_graph(self) -> None:
        raise NotImplementedError

    def topic_info(self, topic: str) -> None:
        """Optional publisher/subscriber discovery."""


class OfflineTransport(Transport):
    def __init__(self, parent=None, state_provider=None):
        super().__init__(parent)
        self.state_provider = state_provider
        self.subscriptions = set()
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.started = time.monotonic()

    def connect_transport(self):
        self.timer.start()
        self.status.emit("Offline demo · simulated data")
        self.refresh_graph()

    def disconnect_transport(self):
        self.timer.stop()
        self.subscriptions.clear()
        self.status.emit("Disconnected")

    def subscribe(self, topic, message_type=""):
        self.subscriptions.add(topic)

    def unsubscribe(self, topic):
        self.subscriptions.discard(topic)

    def refresh_graph(self):
        self.graph.emit(
            {
                "topics": ["/studio/sine", "/joint_states", "/studio/temperature"],
                "types": [
                    "std_msgs/msg/Float64",
                    "sensor_msgs/msg/JointState",
                    "std_msgs/msg/Float64",
                ],
                "nodes": ["/offline_demo_publisher", "/offline_demo_subscriber"],
                "simulated": True,
            }
        )

    def topic_info(self, topic):
        self.graph.emit(
            {
                "topic_info": topic,
                "publishers": ["/offline_demo_publisher"],
                "subscribers": ["/offline_demo_subscriber"],
                "simulated": True,
            }
        )

    def tick(self):
        t = time.monotonic() - self.started
        samples = {
            "/studio/sine": {"data": math.sin(t)},
            "/studio/temperature": {"data": 34 + 1.5 * math.sin(t * 0.2)},
        }
        state = self.state_provider() if self.state_provider else None
        if state:
            names = list(state.positions)
            samples["/joint_states"] = {
                "name": names,
                "position": [state.positions[n] for n in names],
                "velocity": [state.velocities[n] for n in names]
                if state.velocities and set(names) <= state.velocities.keys()
                else [],
                "effort": [state.efforts[n] for n in names]
                if state.efforts and set(names) <= state.efforts.keys()
                else [],
            }

        for topic in self.subscriptions:
            if topic in samples:
                self.message.emit(topic, samples[topic])


def numeric_fields(message: Any, prefix: str = "", depth=0) -> dict[str, float]:
    """Bound traversal to protect rendering from arbitrarily large arrays/messages."""
    if depth > 5:
        return {}
    if isinstance(message, (int, float)) and not isinstance(message, bool):
        try:
            value = float(message)
            return {prefix or "value": value} if math.isfinite(value) else {}
        except OverflowError:
            return {}
    result = {}
    if isinstance(message, dict):
        for key, value in list(message.items())[:64]:
            result.update(
                numeric_fields(value, f"{prefix}.{key}" if prefix else str(key), depth + 1)
            )
            if len(result) >= 64:
                break
    elif isinstance(message, list):
        for index, value in enumerate(message[:32]):
            result.update(numeric_fields(value, f"{prefix}[{index}]", depth + 1))
    return dict(list(result.items())[:64])


class SampleBuffer:
    def __init__(self, maximum: int = 6000):
        self.samples = deque(maxlen=maximum)

    def append(self, timestamp: float, fields: dict[str, float]) -> None:
        self.samples.append((timestamp, fields))

    def clear(self) -> None:
        self.samples.clear()

    def series(self, field: str, now: float, window: float):
        pairs = [
            (t - now, values[field])
            for t, values in self.samples
            if t >= now - window and field in values
        ]
        return ([p[0] for p in pairs], [p[1] for p in pairs])
