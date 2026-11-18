"""SO-101 USB operations using a separate LeRobot interpreter."""

import argparse
import json
import sys
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


def read_positions(port, calibration_path=None, jog=None, target=None):
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
    bus = SerialMotorsBus(
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
    # The bus handshake reads motor identity and firmware. The high-level
    # follower's connect() would additionally configure the motors.
    bus.connect()
    try:
        calibrated = bool(calibration) and bus.is_calibrated
        if jog or target:
            if not calibrated:
                raise ValueError("Manual USB movement requires matching motor calibration.")
            joint, value = jog or target
            if joint not in calibration:
                raise ValueError("Select one calibrated SO-101 joint.")
            jog_joint(bus, joint, value, calibration[joint], absolute=target is not None)
        positions = bus.sync_read("Present_Position", normalize=calibrated, num_retry=1)
        return {
            "calibrated": calibrated,
            "positions": positions,
            "encoders": bus.sync_read("Present_Position", normalize=False, num_retry=1)
            if calibrated
            else positions,
            "ranges": {
                name: (motor.range_min, motor.range_max) for name, motor in calibration.items()
            }
            if calibrated
            else {},
        }
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
    command = parser.add_mutually_exclusive_group()
    command.add_argument("--direction", type=int, choices=(-1, 1))
    command.add_argument("--target", type=int)
    args = parser.parse_args()
    if (args.joint is None) != (args.direction is None and args.target is None):
        parser.error("--joint requires --direction or --target")
    try:
        jog = (args.joint, args.direction) if args.direction is not None else None
        target = (args.joint, args.target) if args.target is not None else None
        print(json.dumps(read_positions(args.port, args.calibration, jog, target)))
    except (ImportError, OSError, RuntimeError, TypeError, ValueError) as exc:
        sys.exit(str(exc))
