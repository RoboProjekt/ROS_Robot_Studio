"""SO-101 USB operations using a separate LeRobot interpreter."""

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
JOG_COUNTS = 12
JOG_SPEED = 24


def create_bus(port, calibration_path):
    from lerobot.motors import Motor, MotorCalibration, MotorNormMode, SerialMotorsBus

    calibration = {}
    if calibration_path:
        values = json.loads(Path(calibration_path).read_text(encoding="utf-8"))
        if not isinstance(values, dict) or set(values) != set(JOINT_NAMES):
            raise ValueError("Expected a LeRobot calibration for all six SO-101 follower motors.")
        for index, joint in enumerate(JOINT_NAMES, start=1):
            motor = MotorCalibration(**values[joint])
            if motor.id != index or motor.range_min >= motor.range_max:
                raise ValueError(f"Invalid calibration for {joint}.")
            calibration[joint] = motor
    return SerialMotorsBus(
        port=port,
        motors={
            joint: Motor(
                index,
                "sts3215",
                MotorNormMode.RANGE_0_100 if joint == "gripper" else MotorNormMode.DEGREES,
            )
            for index, joint in enumerate(JOINT_NAMES, start=1)
        },
        calibration=calibration,
    )


def sample_bus(bus):
    calibrated = bool(bus.calibration) and bus.is_calibrated
    return {
        "calibrated": calibrated,
        "positions": bus.sync_read("Present_Position", normalize=calibrated, num_retry=1),
        "encoders": bus.sync_read("Present_Position", normalize=False, num_retry=1),
        "ranges": {
            name: (motor.range_min, motor.range_max) for name, motor in bus.calibration.items()
        }
        if calibrated
        else {},
    }


def read_positions(port, calibration_path=None, jog=None, target=None):
    bus = create_bus(port, calibration_path)
    # The bus handshake reads motor identity and firmware. The high-level
    # follower's connect() would additionally configure the motors.
    bus.connect()
    try:
        calibrated = bool(bus.calibration) and bus.is_calibrated
        if jog or target:
            if not calibrated:
                raise ValueError("Manual USB movement requires matching motor calibration.")
            joint, value = jog or target
            if joint not in bus.calibration:
                raise ValueError("Select one calibrated SO-101 joint.")
            jog_joint(bus, joint, value, bus.calibration[joint], absolute=target is not None)
        return sample_bus(bus)
    finally:
        bus.disconnect(disable_torque=False)


class USBControlSession:
    """Own torque for this connection and hold the last acknowledged positions."""

    def __init__(self, bus):
        self.bus = bus
        self.owned = set()
        self.speeds = {}

    @property
    def armed(self):
        return len(self.owned) == len(JOINT_NAMES)

    def arm(self):
        if self.armed:
            return
        bus = self.bus
        if not bus.calibration or not bus.is_calibrated:
            raise ValueError("Matching SO-101 calibration is required before enabling motors.")
        positions = bus.sync_read("Present_Position", normalize=False)
        modes = bus.sync_read("Operating_Mode", normalize=False)
        torque = bus.sync_read("Torque_Enable", normalize=False)
        for name in JOINT_NAMES:
            limits = bus.calibration[name]
            if modes[name] != 0 or torque[name] != 0:
                raise ValueError(
                    f"{name}: position mode and torque off are required before arming."
                )
            if not 0 <= limits.range_min <= positions[name] <= limits.range_max <= 4095:
                raise ValueError(f"{name}: position is outside calibrated travel.")
        try:
            for name in JOINT_NAMES:
                self.speeds[name] = bus.read("Goal_Speed", name, normalize=False)
                bus.write("Goal_Speed", name, JOG_SPEED, normalize=False)
                bus.write("Goal_Position", name, positions[name], normalize=False)
            actual = bus.sync_read("Present_Position", normalize=False)
            if any(abs(actual[n] - positions[n]) > 2 for n in JOINT_NAMES):
                raise ValueError("Arm moved while enabling control. Support it and retry.")
            for name in JOINT_NAMES:
                self.owned.add(name)
                bus.write("Torque_Enable", name, 1, normalize=False)
            if any(v != 1 for v in bus.sync_read("Torque_Enable", normalize=False).values()):
                raise RuntimeError("Motor torque could not be enabled.")
        except Exception:
            self.release()
            raise

    def target(self, name, value):
        if not self.armed:
            raise ValueError("Enable motor control before sending positions.")
        if name not in JOINT_NAMES or isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("Invalid joint position command.")
        limits = self.bus.calibration[name]
        if not max(0, limits.range_min) <= value <= min(4095, limits.range_max):
            raise ValueError(f"{name}: requested position is outside calibrated travel.")
        self.bus.write("Goal_Position", name, value, normalize=False)

    def release(self):
        errors = []
        for name in tuple(self.owned):
            try:
                self.bus.write("Torque_Enable", name, 0, normalize=False, num_retry=2)
                if self.bus.read("Torque_Enable", name, normalize=False) != 0:
                    raise RuntimeError("Torque is still on")
                self.owned.remove(name)
            except Exception as exc:
                errors.append(f"{name}: {exc}")
        if errors:
            raise RuntimeError(
                "Torque release could not be confirmed. Support the arm and cut motor power. "
                + "; ".join(errors)
            )
        for name in tuple(self.speeds):
            self.bus.write("Goal_Speed", name, self.speeds[name], normalize=False)
            del self.speeds[name]


def stream_positions(port, calibration_path):
    commands = queue.Queue(maxsize=256)

    def receive():
        try:
            for line in sys.stdin:
                commands.put(json.loads(line))
        except (ValueError, OSError):
            pass
        finally:
            commands.put({"operation": "close"})

    threading.Thread(target=receive, daemon=True).start()
    bus = create_bus(port, calibration_path)
    control = USBControlSession(bus)
    bus.connect()
    last_contact = time.monotonic()
    try:
        while time.monotonic() - last_contact < 3:
            targets = {}
            for _ in range(256):
                try:
                    command = commands.get_nowait()
                except queue.Empty:
                    break
                last_contact = time.monotonic()
                operation = command.get("operation")
                if operation == "close":
                    return
                if operation == "arm":
                    control.arm()
                elif operation == "disarm":
                    control.release()
                    targets.clear()
                elif operation == "target":
                    targets[command["joint"]] = command["position"]
                elif operation != "ping":
                    raise ValueError("Unknown USB command")
            for name, value in targets.items():
                control.target(name, value)
            sample = sample_bus(bus)
            if control.armed and not sample["calibrated"]:
                raise ValueError("Motor calibration changed while control was enabled.")
            sample["armed"] = control.armed
            print(json.dumps(sample), flush=True)
            time.sleep(0.05)
    finally:
        try:
            control.release()
        finally:
            bus.disconnect(disable_torque=False)


def jog_joint(bus, joint, value, calibration, *, absolute=False):
    """Send one small relative step and return the motor to torque off."""
    if joint not in JOINT_NAMES or (not absolute and value not in (-1, 1)):
        raise ValueError("Select one SO-101 joint and one jog direction.")
    if bus.read("Operating_Mode", joint, normalize=False) != 0:
        raise ValueError(f"{joint} must already be in position mode.")
    if bus.read("Torque_Enable", joint, normalize=False) != 0:
        raise ValueError(
            f"{joint} already has torque enabled. Release it in the controlling application."
        )
    position = bus.read("Present_Position", joint, normalize=False)
    lower, upper = max(0, calibration.range_min), min(4095, calibration.range_max)
    if not lower <= position <= upper:
        raise ValueError(f"{joint} is outside its calibrated travel.")
    requested = value if absolute else position + value * JOG_COUNTS
    goal = max(max(lower, position - JOG_COUNTS), min(min(upper, position + JOG_COUNTS), requested))
    if goal == position:
        if absolute:
            return
        raise ValueError(f"{joint} is at its calibrated limit in that direction.")
    speed = bus.read("Goal_Speed", joint, normalize=False)

    release = False
    try:
        bus.write("Goal_Speed", joint, JOG_SPEED, normalize=False)
        bus.write("Goal_Position", joint, position, normalize=False)
        if abs(bus.read("Present_Position", joint, normalize=False) - position) > 2:
            raise RuntimeError("Joint moved during preparation. Support the arm and read again.")
        release = True
        bus.write("Torque_Enable", joint, 1, normalize=False)
        bus.write("Goal_Position", joint, goal, normalize=False)
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            if abs(bus.read("Present_Position", joint, normalize=False) - goal) <= 2:
                return
            time.sleep(0.04)
        raise RuntimeError("The motor did not reach the target before torque was released.")
    finally:
        if release:
            try:
                bus.write("Torque_Enable", joint, 0, normalize=False, num_retry=2)
                if bus.read("Torque_Enable", joint, normalize=False) != 0:
                    raise RuntimeError("Motor still reports torque on.")
            except (OSError, RuntimeError, ValueError) as exc:
                raise RuntimeError(
                    f"Torque release could not be confirmed for {joint}. Support the arm and cut motor power."
                ) from exc
        bus.write("Goal_Speed", joint, speed, normalize=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port")
    parser.add_argument("calibration", nargs="?")
    parser.add_argument("--joint", choices=JOINT_NAMES)
    parser.add_argument("--stream", action="store_true")
    command = parser.add_mutually_exclusive_group()
    command.add_argument("--direction", type=int, choices=(-1, 1))
    command.add_argument("--target", type=int)
    args = parser.parse_args()
    if (args.joint is None) != (args.direction is None and args.target is None):
        parser.error("--joint requires --direction or --target")
    try:
        jog = (args.joint, args.direction) if args.direction is not None else None
        target = (args.joint, args.target) if args.target is not None else None
        if args.stream:
            stream_positions(args.port, args.calibration)
        else:
            print(json.dumps(read_positions(args.port, args.calibration, jog, target)))
    except (ImportError, OSError, RuntimeError, TypeError, ValueError) as exc:
        sys.exit(str(exc))
