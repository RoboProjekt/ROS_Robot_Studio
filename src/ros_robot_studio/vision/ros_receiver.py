"""Executed inside the already-connected ROS container, without desktop dependencies."""

import json
import os
import select
import struct
import sys
import time

import rclpy
from rclpy.clock import ClockType

try:
    from rclpy.event_handler import SubscriptionEventCallbacks
except ImportError:  # Humble
    from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import Buffer, TransformException, TransformListener

MAX_IMAGE = 8 * 1024 * 1024
MAX_METADATA = 64 * 1024


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    config = json.loads(sys.argv[1])
    rclpy.init(args=[])
    node = rclpy.create_node("studio_rgb_" + config["id"][:48])
    transforms = Buffer()
    listener = TransformListener(transforms, node)
    info = {}
    pending = None
    warning = None
    clock = None
    epoch = 0
    dropped = 0
    sequence = 0
    stdout, stdin = sys.stdout.fileno(), sys.stdin.fileno()
    os.set_blocking(stdout, False)

    def camera_info(msg):
        if msg.header.frame_id != config["optical_frame"]:
            return
        stamp = stamp_ns(msg.header.stamp)
        info[stamp] = dict(
            stamp_ns=stamp,
            width=msg.width,
            height=msg.height,
            frame_id=msg.header.frame_id,
            k=list(msg.k),
            d=list(msg.d),
            model=msg.distortion_model,
            binning_x=msg.binning_x,
            binning_y=msg.binning_y,
            roi=bool(
                msg.roi.x_offset
                or msg.roi.y_offset
                or (msg.roi.width and (msg.roi.width != msg.width or msg.roi.height != msg.height))
            ),
        )
        if len(info) > 8:
            info.pop(next(iter(info)))

    def update_clock(msg):
        nonlocal clock, epoch, pending, warning
        value = stamp_ns(msg.clock)
        if clock is not None and value < clock:
            epoch += 1
            pending = None
            info.clear()
            transforms.clear()
            warning = "Simulation time reset; cleared image and calibration buffers"
        clock = value

    def incompatible(_):
        nonlocal warning
        warning = "ROS camera QoS incompatible with publisher"

    def image(msg):
        nonlocal pending, dropped, warning, sequence
        sequence += 1
        if (
            len(msg.data) > MAX_IMAGE
            or not msg.data
            or msg.header.frame_id != config["optical_frame"]
        ):
            warning = "Invalid image size or optical frame"
            return
        if pending is not None:
            dropped += 1
        pending = (msg, time.time_ns(), sequence)

    msg_type = CompressedImage if config["encoding"] == "compressed" else Image
    node.create_subscription(
        msg_type,
        config["rgb_topic"],
        image,
        qos_profile_sensor_data,
        event_callbacks=SubscriptionEventCallbacks(incompatible_qos=incompatible),
    )
    node.create_subscription(CameraInfo, config["info_topic"], camera_info, qos_profile_sensor_data)
    if config["clock_topic"]:
        node.create_subscription(
            Clock, config["clock_topic"], update_clock, qos_profile_sensor_data
        )

    def metadata(msg, receipt, index):
        stamp = stamp_ns(msg.header.stamp)
        calibration = info.get(stamp) or info.get(0)
        if calibration is None and stamp > 0:
            preceding = [key for key in info if 0 < key <= stamp and stamp - key <= 500_000_000]
            if preceding:
                calibration = info[max(preceding)]
        poses, errors = {}, {}
        for target in config["targets"]:
            if not stamp:
                errors[target] = "Image has no ROS acquisition timestamp"
                continue
            try:
                transform = transforms.lookup_transform(
                    target,
                    msg.header.frame_id,
                    Time(nanoseconds=stamp, clock_type=ClockType.ROS_TIME),
                ).transform
                p, q = transform.translation, transform.rotation
                poses[target] = ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            except TransformException as exc:
                errors[target] = str(exc)[:250]
        header = dict(
            kind="image",
            stamp_ns=stamp,
            frame_id=msg.header.frame_id,
            received_wall_ns=receipt,
            sequence=index,
            camera_info=calibration,
            transforms=poses,
            tf_errors=errors,
            epoch=epoch,
            drops=dropped,
        )
        if isinstance(msg, Image):
            header.update(
                encoding=msg.encoding,
                width=msg.width,
                height=msg.height,
                step=msg.step,
                big_endian=bool(msg.is_bigendian),
            )
        else:
            header.update(encoding="compressed", format=msg.format)
        return header

    def packet(header, body=b""):
        data = json.dumps(header, separators=(",", ":"), allow_nan=False).encode()
        if len(data) > MAX_METADATA:
            raise ValueError("Camera metadata exceeds packet limit")
        return struct.pack("!II", len(data), len(body)) + data + body

    output = memoryview(b"")
    last_lease = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - last_lease < 4:
            rclpy.spin_once(node, timeout_sec=0 if output else 0.005)
            readable, _, _ = select.select(
                [stdin], [stdout] if output else [], [], 0.002 if output else 0
            )
            if readable:
                if not os.read(stdin, 256):
                    break
                last_lease = time.monotonic()
            if not output and warning:
                output = memoryview(packet(dict(kind="status", epoch=epoch, message=warning)))
                warning = None
            if not output and pending:
                msg, receipt, index = pending
                pending = None
                try:
                    output = memoryview(packet(metadata(msg, receipt, index), bytes(msg.data)))
                except ValueError as exc:
                    warning = str(exc)
            while output:
                try:
                    output = output[os.write(stdout, output) :]
                except BlockingIOError:
                    break
    except BrokenPipeError:
        pass
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
