"""Asynchronous rosbridge client with bounded latest-message sampling and reconnect."""

import json
import time

from PySide6.QtCore import QTimer, QUrl
from PySide6.QtNetwork import QAbstractSocket
from PySide6.QtWebSockets import QWebSocket

from ros_robot_studio.ros_bridge.transport import Transport


class RosbridgeTransport(Transport):
    def __init__(self, url: str, parent=None):
        super().__init__(parent)
        self.url = QUrl(url)
        self.socket = QWebSocket(parent=self)
        self.socket.setMaxAllowedIncomingMessageSize(1_000_000)
        self.socket.connected.connect(self._connected)
        self.socket.disconnected.connect(self._disconnected)
        self.socket.errorOccurred.connect(self._error)
        self.socket.textMessageReceived.connect(self._receive)
        self.active = False
        self.attempts = 0
        self.subscriptions = {}
        self.pending = {}
        self.latest = {}
        self.sequence = 0
        self.retry = QTimer(self)
        self.retry.setSingleShot(True)
        self.retry.timeout.connect(self._open)
        self.flush_timer = QTimer(self)
        self.flush_timer.setInterval(50)
        self.flush_timer.timeout.connect(self._flush)
        self.graph_timer = QTimer(self)
        self.graph_timer.setInterval(5000)
        self.graph_timer.timeout.connect(self.refresh_graph)
        self.deadline = QTimer(self)
        self.deadline.setSingleShot(True)
        self.deadline.setInterval(5000)
        self.deadline.timeout.connect(self._connect_timeout)
        self.graph_data = {}
        self.advertised = set()

    @property
    def connected(self):
        return self.socket.state() == QAbstractSocket.SocketState.ConnectedState

    def connect_transport(self):
        self.active = True
        self.attempts = 0
        self.flush_timer.start()
        self._open()

    def _open(self):
        if self.active:
            self.status.emit("Connecting to ROS…")
            self.socket.open(self.url)
            self.deadline.start()

    def _connect_timeout(self):
        if self.active and not self.connected:
            self.error.emit("ROS connection timed out")
            self.socket.abort()
            self._schedule_retry()

    def _connected(self):
        self.deadline.stop()
        self.retry.stop()
        self.attempts = 0
        self.status.emit("ROS connected · container data")
        self.connection_changed.emit(True)
        for topic, message_type in self.subscriptions.items():
            self._subscribe(topic, message_type)
        self.refresh_graph()
        self.graph_timer.start()

    def _schedule_retry(self):
        if self.active and not self.retry.isActive():
            delay = min(1000 * (2 ** min(self.attempts, 4)), 15000)
            self.attempts += 1
            self.status.emit(f"ROS disconnected · retry in {delay // 1000}s")
            self.retry.start(delay)

    def _disconnected(self):
        self.connection_changed.emit(False)
        self.advertised.clear()
        self.deadline.stop()
        self.graph_timer.stop()
        self.pending.clear()
        self.latest.clear()
        self._schedule_retry()

    def _error(self, error):
        if self.active:
            self.error.emit(self.socket.errorString())
            self._schedule_retry()

    def disconnect_transport(self):
        self.active = False
        for topic in list(self.subscriptions):
            self.unsubscribe(topic)
        for timer in (self.retry, self.flush_timer, self.graph_timer, self.deadline):
            timer.stop()
        self.pending.clear()
        self.latest.clear()
        self.socket.abort()
        self.status.emit("Disconnected")

    def send(self, data):
        if self.connected:
            self.socket.sendTextMessage(json.dumps(data))

    def _subscribe(self, topic, message_type):
        data = {
            "op": "subscribe",
            "id": "sub:" + topic,
            "topic": topic,
            "throttle_rate": 50,
            "queue_length": 1,
        }
        if message_type:
            data["type"] = message_type
        if topic.startswith("/studio/") and "/execution/" in topic:
            data["throttle_rate"] = 0
            data["queue_length"] = 100
        self.send(data)

    def subscribe(self, topic, message_type=""):
        if topic not in self.subscriptions:
            self.subscriptions[topic] = message_type
            self._subscribe(topic, message_type)

    def unsubscribe(self, topic):
        self.send({"op": "unsubscribe", "id": "sub:" + topic, "topic": topic})
        self.subscriptions.pop(topic, None)
        self.latest.pop(topic, None)

    def publish(self, topic, message_type, message):
        if not self.connected:
            raise ConnectionError("ROS is disconnected; commands are not queued")
        if len(json.dumps(message)) > 950000:
            raise ValueError("ROS command exceeds size limit")
        if topic not in self.advertised:
            self.send({"op": "advertise", "topic": topic, "type": message_type, "queue_size": 10})
            self.advertised.add(topic)
        self.send({"op": "publish", "topic": topic, "msg": message})

    def service(self, name, args, callback):
        if not self.connected or len(self.pending) >= 32:
            return
        self.sequence += 1
        identifier = f"service:{self.sequence}"
        self.pending[identifier] = (time.monotonic(), callback)
        self.send({"op": "call_service", "id": identifier, "service": name, "args": args})

    def refresh_graph(self):
        def topics(values):
            self.graph_data.update(
                {"topics": values.get("topics", []), "types": values.get("types", [])}
            )
            self.graph.emit(dict(self.graph_data))

        def nodes(values):
            self.graph_data["nodes"] = values.get("nodes", [])
            self.graph.emit(dict(self.graph_data))

        self.service("/rosapi/topics", {}, topics)
        self.service("/rosapi/nodes", {}, nodes)

    def topic_info(self, topic):
        for relation in ("publishers", "subscribers"):

            def receive(values, relation=relation):
                self.graph.emit({"topic_info": topic, relation: values.get(relation, [])})

            self.service("/rosapi/" + relation, {"topic": topic}, receive)

    def _receive(self, raw):
        if not self.active or len(raw) > 1_000_000:
            return
        try:
            data = json.loads(raw)
            if not isinstance(data, dict):
                return
            if data.get("op") == "publish" and data.get("topic") in self.subscriptions:
                if isinstance(data.get("msg"), dict):
                    if data["topic"].startswith("/studio/") and "/execution/" in data["topic"]:
                        self.message.emit(data["topic"], data["msg"])
                    else:
                        self.latest[data["topic"]] = data["msg"]
            elif data.get("op") == "service_response":
                pending = self.pending.pop(data.get("id"), None)
                if pending:
                    if data.get("result", True):
                        pending[1](data.get("values", {}))
                    else:
                        self.error.emit(
                            "ROS service failed: " + str(data.get("service", "unknown"))
                        )
            elif data.get("op") == "status" and data.get("level") == "error":
                self.error.emit(str(data.get("msg", "ROS protocol error")))
        except (ValueError, TypeError, KeyError) as exc:
            self.error.emit(f"Invalid ROS message: {exc}")

    def _flush(self):
        samples, self.latest = self.latest, {}
        for topic, message in samples.items():
            self.message.emit(topic, message)
        now = time.monotonic()
        expired = [key for key, (started, _) in self.pending.items() if now - started > 10]
        for key in expired:
            self.pending.pop(key, None)
        if expired:
            self.error.emit("ROS graph service timed out; check rosapi in the runtime logs")
