"""Read raw SO-101 follower positions using a separate LeRobot interpreter."""

import json
import sys
from pathlib import Path

JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


def read_positions(port, calibration_path=None):
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
        return {
            "calibrated": calibrated,
            "positions": bus.sync_read("Present_Position", normalize=calibrated, num_retry=1),
        }
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == "__main__":
    try:
        print(json.dumps(read_positions(sys.argv[1], *sys.argv[2:])))
    except (ImportError, OSError, RuntimeError, TypeError, ValueError) as exc:
        sys.exit(str(exc))
