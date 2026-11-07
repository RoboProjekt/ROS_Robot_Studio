"""Runs only inside ROS containers. Real ROS publishers and a separate subscriber node."""

import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class DemoPublisher(Node):
    def __init__(self):
        super().__init__("studio_demo_publisher")
        self.signal = self.create_publisher(Float64, "/studio/sine", 10)
        self.joints = self.create_publisher(JointState, "/joint_states", 10)
        self.started = time.monotonic()
        self.create_timer(0.05, self.publish)

    def publish(self):
        t = time.monotonic() - self.started
        self.signal.publish(Float64(data=math.sin(t)))
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["base_rotation", "shoulder", "elbow"]
        message.position = [
            0.4 * math.sin(t * 0.4),
            0.35 * math.sin(t * 0.6),
            0.5 * math.cos(t * 0.5),
        ]
        message.velocity = [
            0.16 * math.cos(t * 0.4),
            0.21 * math.cos(t * 0.6),
            -0.25 * math.sin(t * 0.5),
        ]
        self.joints.publish(message)


class DemoSubscriber(Node):
    def __init__(self):
        super().__init__("studio_demo_subscriber")
        self.count = 0
        self.create_subscription(Float64, "/studio/sine", self.receive, 10)
        self.create_timer(
            5.0, lambda: self.get_logger().info(f"Received {self.count} signal samples")
        )

    def receive(self, message):
        self.count += 1


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = [DemoPublisher(), DemoSubscriber()]
    for node in nodes:
        executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
