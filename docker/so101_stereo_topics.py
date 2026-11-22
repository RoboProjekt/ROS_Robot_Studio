"""Capture the SO-101 camera pair and publish RGB and learned relative depth on ROS 2."""

import argparse
from array import array
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from camera_stream import CameraStream
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

PREFIX = "/so101/usb"
CAMERA_FRAMES = ("so101_usb_camera_1_optical_frame", "so101_usb_camera_2_optical_frame")
PAIR_SLOP_NS = 25_000_000
CONFIDENCE = 0.08


class StereoPolicy:
    def __init__(self, model):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 2
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            model, sess_options=options, providers=["CPUExecutionProvider"]
        )
        inputs = {item.name: item.shape for item in self.session.get_inputs()}
        shape = inputs.get("left_rgb", [])
        if (
            set(inputs) != {"left_rgb", "right_rgb"}
            or shape != inputs.get("right_rgb")
            or len(shape) != 4
            or shape[:2] != [1, 3]
            or any(not isinstance(size, int) or size <= 0 for size in shape[2:])
        ):
            raise ValueError("Expected two fixed-size ONNX inputs with shape [1, 3, height, width]")
        self.input_size = shape[3], shape[2]

    def relative_depth(self, left, right):
        inputs = {}
        for name, image in zip(("left_rgb", "right_rgb"), (left, right), strict=True):
            scaled = cv2.resize(image, self.input_size, interpolation=cv2.INTER_LINEAR)
            inputs[name] = np.ascontiguousarray(scaled.transpose(2, 0, 1)[None], dtype=np.float32)
            inputs[name] /= 255
        disparity, confidence = self.session.run(["left_disparity_px", "confidence"], inputs)
        height, width = left.shape[:2]
        disparity = cv2.resize(disparity[0, 0], (width, height)) * (width / self.input_size[0])
        confidence = cv2.resize(confidence[0, 0], (width, height))
        x = np.arange(width)
        valid = (
            (confidence >= CONFIDENCE) & np.isfinite(disparity) & (disparity > 0) & (x >= disparity)
        )
        depth = np.zeros((height, width), dtype=np.uint8)
        # Map inverse disparity to a fixed display scale. Zero marks invalid pixels.
        depth[valid] = np.rint(1 + 254 * (2 / np.maximum(disparity[valid], 2)))
        return depth


def image_message(pixels, encoding, frame_id, stamp):
    message = Image()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    message.header.frame_id = frame_id
    message.height, message.width = pixels.shape[:2]
    message.encoding = encoding
    message.step = pixels.strides[0]
    message.data = array("B", pixels.tobytes())
    return message


class SingleCameraTopics(Node):
    def __init__(self, device, depth):
        super().__init__("so101_usb_camera")
        self.encoding = "16UC1" if depth else "rgb8"
        topic = f"{PREFIX}/depth/image_raw" if depth else f"{PREFIX}/camera_1/image_raw"
        frame = "so101_usb_native_depth_optical_frame" if depth else CAMERA_FRAMES[0]
        self.publisher = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.camera = CameraStream(
            SimpleNamespace(v4l2_device=device, optical_frame=frame, width=640, height=480, fps=30),
            capture_depth=depth,
        )
        self.create_timer(0.005, self.publish_frame)
        self.camera.start()

    def publish_frame(self):
        frame, error, *_ = self.camera.poll()
        if not self.camera.thread.is_alive():
            raise OSError(error or "USB camera stopped")
        if frame is not None:
            self.publisher.publish(
                image_message(frame.rgb, self.encoding, frame.frame_id, frame.metadata["stamp_ns"])
            )

    def release(self):
        self.camera.stop()
        self.camera.thread.join(timeout=1.0)


class StereoTopics(Node):
    def __init__(self, devices, policy):
        super().__init__("so101_usb_stereo")
        self.policy = policy
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.inference = None
        self.inference_stamp = 0
        self.pending = [None, None]
        self.cameras = tuple(
            CameraStream(
                SimpleNamespace(
                    v4l2_device=device, optical_frame=frame, width=640, height=480, fps=30
                )
            )
            for device, frame in zip(devices, CAMERA_FRAMES, strict=True)
        )
        self.rgb = tuple(
            self.create_publisher(
                Image, f"{PREFIX}/camera_{index}/image_raw", qos_profile_sensor_data
            )
            for index in (1, 2)
        )
        self.stereo = self.create_publisher(
            Image, f"{PREFIX}/stereo/image_raw", qos_profile_sensor_data
        )
        self.depth = self.create_publisher(
            Image, f"{PREFIX}/stereo/depth_relative", qos_profile_sensor_data
        )
        self.create_timer(0.005, self.publish_frames)
        for camera in self.cameras:
            camera.start()

    def publish_frames(self):
        for index, camera in enumerate(self.cameras):
            frame, error, *_ = camera.poll()
            if not camera.thread.is_alive():
                raise OSError(error or f"Camera {index + 1} stopped")
            if frame is not None:
                stamp = frame.metadata["stamp_ns"]
                self.rgb[index].publish(image_message(frame.rgb, "rgb8", frame.frame_id, stamp))
                self.pending[index] = frame

        if self.inference is not None and self.inference.done():
            self.depth.publish(
                image_message(
                    self.inference.result(),
                    "mono8",
                    "so101_usb_depth_optical_frame",
                    self.inference_stamp,
                )
            )
            self.inference = None

        left, right = self.pending
        if left is None or right is None:
            return
        left_stamp, right_stamp = (frame.metadata["stamp_ns"] for frame in (left, right))
        if abs(left_stamp - right_stamp) > PAIR_SLOP_NS:
            older = 0 if left_stamp < right_stamp else 1
            self.pending[older] = None
            return
        self.pending = [None, None]
        if left.rgb.shape != right.rgb.shape:
            raise ValueError("Stereo cameras must supply matching RGB resolutions")
        pair = np.concatenate((left.rgb, right.rgb), axis=1)
        self.stereo.publish(image_message(pair, "rgb8", "so101_usb_stereo_pair", left_stamp))
        if self.inference is None:
            self.inference_stamp = left_stamp
            self.inference = self.worker.submit(self.policy.relative_depth, left.rgb, right.rgb)

    def release(self):
        for camera in self.cameras:
            camera.stop()
        for camera in self.cameras:
            camera.thread.join(timeout=1.0)
        self.worker.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("single", "stereo", "depth"))
    parser.add_argument("camera_1")
    parser.add_argument("camera_2", nargs="?")
    parser.add_argument("model", nargs="?")
    args = parser.parse_args()
    if args.mode == "stereo" and (not args.camera_2 or not args.model):
        parser.error("Stereo requires two camera devices and an ONNX model")
    cv2.setNumThreads(1)
    policy = StereoPolicy(args.model) if args.mode == "stereo" else None
    rclpy.init(args=[])
    node = (
        StereoTopics((args.camera_1, args.camera_2), policy)
        if policy
        else SingleCameraTopics(args.camera_1, args.mode == "depth")
    )
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
