"""ROS MoveIt plan-only adapter for the SO-101 stereo demo."""

import json
import math
import re
import subprocess
from pathlib import Path

ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")
GRIPPER = "gripper"
BASE = "base_link"
TOOL = "gripper_frame_link"
PHASES = ("Approach", "Grasp", "Lift", "Transport", "Place")
DEFAULT_CONTAINER = "studio-so101-moveit"


def make_request(geometry, snapshot, positions, domain):
    if domain != 1 or snapshot["frame"] != "robot_base":
        raise ValueError("The SO-101 MoveIt demo expects Humble Domain 1 in robot_base.")
    if set(positions) != {*ARM, GRIPPER} or not all(math.isfinite(n) for n in positions.values()):
        raise ValueError("A complete current SO-101 joint state is required.")
    grasp, place = geometry["grasp_center"], geometry["place_center"]
    lift = 0.06
    # Leave jaw clearance over the table when opening the gripper at the drop.
    release = [place[0], place[1], place[2] + 0.012]
    points = [
        [grasp[0], grasp[1], grasp[2] + lift],
        grasp,
        [grasp[0], grasp[1], grasp[2] + lift],
        [place[0], place[1], place[2] + lift],
        release,
    ]
    return {
        "domain": domain,
        "base": BASE,
        "tool": TOOL,
        "arm": ARM,
        "gripper": GRIPPER,
        "positions": positions,
        "table_z": snapshot["table_z"],
        "bowl": next(
            obj["xyz"] for obj in snapshot["objects"] if obj["id"] == geometry["reference_id"]
        ),
        "phases": [{"name": name, "xyz": xyz} for name, xyz in zip(PHASES, points, strict=True)],
    }


def run_plan(container, request):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,80}", container):
        raise ValueError("Enter a valid MoveIt Docker container name.")
    code = (Path(__file__).resolve().parents[1] / "data/runtime/so101_moveit_plan.py").read_text()
    args = [
        "docker",
        "exec",
        "-i",
        "-e",
        f"ROS_DOMAIN_ID={request['domain']}",
        container,
        "bash",
        "-c",
        'source /opt/ros/humble/setup.bash && exec python3 -u -c "$1"',
        "studio-moveit",
        code,
    ]
    try:
        result = subprocess.run(
            args,
            input=json.dumps(request, allow_nan=False),
            capture_output=True,
            text=True,
            timeout=90,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise ValueError("MoveIt plan-only timed out.") from exc
    if result.returncode:
        raise ValueError(result.stderr.strip()[-1200:] or "MoveIt helper failed.")
    output = json.loads(result.stdout)
    if output.get("error"):
        raise ValueError(output["error"])
    return output


def resolve_ik(request, model):
    """Resolve app-owned tool orientation with the existing SO-101 kinematics."""
    from ros_robot_studio.kinematics.engine import UniversalKinematicsEngine
    from ros_robot_studio.kinematics.solvers import KinematicsSolverManager
    from ros_robot_studio.motion.schema import CartesianPose, IKOptions

    if model.root != BASE or model.name != "so101_new_calib":
        raise ValueError("MoveIt requires the pinned SO-101 Follower catalog model.")
    chain = UniversalKinematicsEngine(model).create_chain(BASE, TOOL, ARM, (GRIPPER,))
    options = IKOptions(position_tolerance=0.008, orientation_tolerance=0.18, starts=7)
    solver = KinematicsSolverManager()
    seed = request["positions"]
    stages = [{"name": "Open gripper", "joints": {GRIPPER: 1.2}}]
    for phase in request["phases"]:
        pose = CartesianPose(
            frame=BASE,
            end_effector=TOOL,
            position=phase["xyz"],
            rpy=(math.pi, 0, 0),
            axis="Z",
        )
        result = solver.solve(chain, pose, seed, mode="axis", options=options)
        if not result.success:
            raise ValueError(f"{phase['name']} IK: {result.message}")
        seed = result.joint_positions
        stages.append(dict(phase, joints={name: seed[name] for name in ARM}))
        if phase["name"] == "Grasp":
            stages.append({"name": "Close gripper", "joints": {GRIPPER: 0.1}})
        elif phase["name"] == "Place":
            stages.append({"name": "Release gripper", "joints": {GRIPPER: 1.2}})
    return dict(request, phases=stages)
