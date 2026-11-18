"""SO-101 USB operations using a separate LeRobot interpreter."""

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

if __package__:
    from .so101_replay import JOINT_NAMES, USBMotion
else:
    from so101_replay import JOINT_NAMES, USBMotion

JOG_COUNTS = 12
JOG_SPEED = 24
USB_SPEED_RANGE = (1, 240)
USB_TORQUE_RANGE = (1, 100)


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
        self.original_limits = {}
        self.applied_limits = {}
        self.limits_to_restore = set()
        self.torque_caps = {}

    @property
    def armed(self):
        return len(self.owned) == len(JOINT_NAMES)

    @staticmethod
    def validate_limits(speed, torque):
        for name, value, (low, high) in (
            ("Speed", speed, USB_SPEED_RANGE),
            ("Maximum torque", torque, USB_TORQUE_RANGE),
        ):
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"{name} must be an integer between {low} and {high}.")

    def arm(self, speed=JOG_SPEED, torque=USB_TORQUE_RANGE[1]):
        self.validate_limits(speed, torque)
        if self.armed:
            return
        bus = self.bus
        if not bus.calibration or not bus.is_calibrated:
            raise ValueError("Matching SO-101 calibration is required before enabling motors.")
        positions = bus.sync_read("Present_Position", normalize=False)
        modes = bus.sync_read("Operating_Mode", normalize=False)
        torque_enabled = bus.sync_read("Torque_Enable", normalize=False)
        for name in JOINT_NAMES:
            limits = bus.calibration[name]
            if modes[name] != 0 or torque_enabled[name] != 0:
                raise ValueError(
                    f"{name}: position mode and torque off are required before arming."
                )
            if not 0 <= limits.range_min <= positions[name] <= limits.range_max <= 4095:
                raise ValueError(f"{name}: position is outside calibrated travel.")
        self.original_limits = {
            register: bus.sync_read(register, normalize=False)
            for register in ("Torque_Limit", "Goal_Speed")
        }
        configured_torque = bus.sync_read("Max_Torque_Limit", normalize=False)
        self.torque_caps = {
            name: min(self.original_limits["Torque_Limit"][name], configured_torque[name])
            for name in JOINT_NAMES
        }
        self.applied_limits = {
            register: values.copy() for register, values in self.original_limits.items()
        }
        try:
            self._apply_limits(speed, torque)
            for name in JOINT_NAMES:
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

    def set_limits(self, speed, torque):
        if not self.armed:
            raise ValueError("Enable motor control before adjusting live limits.")
        self.validate_limits(speed, torque)
        self._apply_limits(speed, torque)

    def _apply_limits(self, speed, torque):
        # The STS3215 volatile torque limit uses 0.1% increments.
        for register, values in (
            (
                "Torque_Limit",
                {name: min(torque * 10, cap) for name, cap in self.torque_caps.items()},
            ),
            ("Goal_Speed", dict.fromkeys(JOINT_NAMES, speed)),
        ):
            applied = self.applied_limits[register]
            changed = {name: value for name, value in values.items() if value != applied[name]}
            if not changed:
                continue
            for name, value in changed.items():
                self.limits_to_restore.add((register, name))
                self.bus.write(register, name, value, normalize=False)
            readback = self.bus.sync_read(register, normalize=False, num_retry=1)
            if any(readback.get(name) != value for name, value in changed.items()):
                raise RuntimeError(f"Could not confirm live {register} on all addressed motors.")
            applied.update(changed)

    def target(self, name, value):
        self.validate_target(name, value)
        self.bus.write("Goal_Position", name, value, normalize=False)

    def target_positions(self, positions):
        for name, value in positions.items():
            self.validate_target(name, value)
        for name, value in positions.items():
            self.bus.write("Goal_Position", name, value, normalize=False)

    def validate_target(self, name, value):
        if not self.armed:
            raise ValueError("Enable motor control before sending positions.")
        if name not in JOINT_NAMES or isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("Invalid joint position command.")
        limits = self.bus.calibration[name]
        if not max(0, limits.range_min) <= value <= min(4095, limits.range_max):
            raise ValueError(f"{name}: requested position is outside calibrated travel.")

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
        for register, name in tuple(self.limits_to_restore):
            try:
                self.bus.write(
                    register, name, self.original_limits[register][name], normalize=False
                )
                self.limits_to_restore.remove((register, name))
            except Exception as exc:
                errors.append(f"{register}/{name}: {exc}")
        if errors:
            raise RuntimeError("Could not restore USB motor settings. " + "; ".join(errors))


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
    motion = USBMotion(control)
    bus.connect()
    last_contact = time.monotonic()
    try:
        while time.monotonic() - last_contact < 3:
            targets = {}
            limits = None
            motion_command = None
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
                    if not motion.active:
                        control.arm(
                            command.get("speed", JOG_SPEED),
                            command.get("torque", USB_TORQUE_RANGE[1]),
                        )
                elif operation == "disarm":
                    control.release()
                    motion.clear("USB motor control disabled.")
                    targets.clear()
                    limits = None
                    motion_command = None
                elif operation in ("record", "replay", "stop_motion"):
                    motion_command = command
                    targets.clear()
                    limits = None
                elif operation == "limits":
                    if not motion.active and motion_command is None:
                        limits = (command["speed"], command["torque"])
                elif operation == "target":
                    if not motion.active and motion_command is None:
                        targets[command["joint"]] = command["position"]
                elif operation != "ping":
                    raise ValueError("Unknown USB command")
            if limits is not None:
                control.set_limits(*limits)
            for name, value in targets.items():
                control.target(name, value)
            sample = sample_bus(bus)
            if control.armed and not sample["calibrated"]:
                raise ValueError("Motor calibration changed while control was enabled.")
            if motion_command:
                motion.requested(motion_command, sample["encoders"])
            motion.sample(sample, time.monotonic())
            sample["armed"] = control.armed
            sample["motion"] = motion.mode
            sample["motion_ack"] = motion.ack
            sample["motion_message"] = motion.message
            print(json.dumps(sample), flush=True)
            time.sleep(0.05)
    finally:
        try:
            control.release()
        finally:
            try:
                motion.clear()
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
