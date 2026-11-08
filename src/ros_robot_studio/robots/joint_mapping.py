"""Future backend mapping contract. No calibration, transport or hardware commands here."""

from typing import Protocol


class JointMappingAdapter(Protocol):
    calibrated: bool

    def ros_to_urdf(self, joint: str, value: float) -> float: ...
    def hardware_to_urdf(self, joint: str, value: float) -> float: ...
    def urdf_to_hardware(self, joint: str, value: float) -> float: ...
    def gripper_to_hardware(self, value: float) -> float: ...


class UncalibratedJointMapping:
    calibrated = False

    @staticmethod
    def _unavailable(*args) -> float:
        raise NotImplementedError(
            "Hardware/ROS mapping is not calibrated. LeRobot gripper 0–100 is not a URDF angle. No command was sent."
        )

    ros_to_urdf = _unavailable
    hardware_to_urdf = _unavailable
    urdf_to_hardware = _unavailable
    gripper_to_hardware = _unavailable
