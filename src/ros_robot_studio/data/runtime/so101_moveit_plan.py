"""Plan through MoveIt services inside the companion Humble container."""

import fcntl
import json
import sys
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, Constraints, JointConstraint
from moveit_msgs.srv import ApplyPlanningScene, GetMotionPlan, GetPlanningScene, GetStateValidity
from rcl_interfaces.srv import GetParameters
from shape_msgs.msg import SolidPrimitive


def collision(base, identity, xyz, size, operation=CollisionObject.ADD):
    item = CollisionObject()
    item.header.frame_id, item.id, item.operation = base, identity, operation
    if operation == CollisionObject.ADD:
        primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        item.primitives, item.primitive_poses = [primitive], [pose]
    return item


def goal(joints):
    return Constraints(
        joint_constraints=[
            JointConstraint(
                joint_name=name,
                position=value,
                tolerance_above=0.006,
                tolerance_below=0.006,
                weight=1.0,
            )
            for name, value in joints.items()
        ]
    )


def plan(node, data):
    def call(client, request, timeout=5):
        if not client.wait_for_service(timeout_sec=2):
            raise ValueError(f"MoveIt service unavailable: {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise ValueError(f"No MoveIt response from {client.srv_name}")
        return future.result()

    parameters = node.create_client(GetParameters, "/move_group/get_parameters")
    semantic = call(parameters, GetParameters.Request(names=["robot_description_semantic"])).values[
        0
    ]
    root = ET.fromstring(semantic.string_value)
    groups = {group.attrib["name"]: group for group in root.findall("group")}
    if not {"arm", "gripper"} <= groups.keys():
        raise ValueError("Expected arm and gripper groups in the running SO-101 SRDF.")
    chain = groups["arm"].find("chain")
    gripper = {joint.attrib["name"] for joint in groups["gripper"].findall("joint")}
    if (
        chain is None
        or chain.get("base_link") != data["base"]
        or chain.get("tip_link") != data["tool"]
        or gripper != {data["gripper"]}
    ):
        raise ValueError("Live MoveIt group links or gripper differ from the SO-101 demo.")
    scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
    scene_request = GetPlanningScene.Request()
    scene_request.components.components = 1 | 2 | 16
    scene = call(scene_client, scene_request).scene
    if scene.robot_model_name != "so101_new_calib" or scene.robot_model_name != root.get("name"):
        raise ValueError("Planning Scene robot differs from the SO-101 SRDF.")
    apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    planner = node.create_client(GetMotionPlan, "/plan_kinematic_path")
    validity = node.create_client(GetStateValidity, "/check_state_validity")
    z = data["table_z"]
    bowl_x, bowl_y, _ = data["bowl"]
    world = [
        collision(data["base"], "studio_stereo_table", [0.27, 0.0, z - 0.019], [0.38, 0.62, 0.038]),
        collision(
            data["base"], "studio_stereo_bowl", [bowl_x, bowl_y, z + 0.016], [0.092, 0.092, 0.032]
        ),
    ]
    current = data["positions"].copy()
    stages = []
    cube_id = "studio_stereo_carried_cube"

    def attachment(operation):
        change = ApplyPlanningScene.Request()
        change.scene.is_diff = change.scene.robot_state.is_diff = True
        cube = collision(data["tool"], cube_id, [0.0, 0.0, 0.0], [0.055] * 3, operation)
        change.scene.robot_state.attached_collision_objects = [
            AttachedCollisionObject(
                link_name=data["tool"],
                object=cube,
                touch_links=["gripper_link", "moving_jaw_so101_v1_link", data["tool"]],
            )
        ]
        if not call(apply, change).success:
            raise ValueError("MoveIt could not update the planned carried cube.")

    def update_world(items, *, required=True):
        change = ApplyPlanningScene.Request()
        change.scene.is_diff = change.scene.robot_state.is_diff = True
        change.scene.world.collision_objects = items
        if not call(apply, change).success and required:
            raise ValueError("MoveIt rejected the stereo Planning Scene geometry.")

    def remove_world(ids):
        update_world(
            [collision(data["base"], identity, [], [], CollisionObject.REMOVE) for identity in ids],
            required=False,
        )

    try:
        # Detaching a body can put it back into the world. Clear our scoped IDs
        # from an earlier interrupted request before installing this snapshot.
        attachment(CollisionObject.REMOVE)
        remove_world([cube_id])
        update_world(world)
        for phase in data["phases"]:
            if phase["name"] == "Release gripper":
                attachment(CollisionObject.REMOVE)
                remove_world([cube_id])
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            gripper_goal = set(phase["joints"]) == {data["gripper"]}
            motion.group_name = "gripper" if gripper_goal else "arm"
            motion.num_planning_attempts, motion.allowed_planning_time = 2, 3.0
            motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = 0.2
            motion.start_state.joint_state.name = list(current)
            motion.start_state.joint_state.position = list(current.values())
            motion.goal_constraints = [goal(phase["joints"])]
            response = call(planner, request, timeout=7).motion_plan_response
            if response.error_code.val != 1:
                check = GetStateValidity.Request(group_name=motion.group_name)
                check.robot_state.joint_state.name = list(current)
                check.robot_state.joint_state.position = [
                    phase["joints"].get(joint, value) for joint, value in current.items()
                ]
                contacts = call(validity, check).contacts
                detail = ", ".join(
                    sorted(
                        {
                            f"{contact.contact_body_1} / {contact.contact_body_2}"
                            for contact in contacts
                        }
                    )[:4]
                )
                raise ValueError(
                    f"MoveIt {phase['name']}: error {response.error_code.val}. {detail}".strip()
                )
            trajectory = response.trajectory.joint_trajectory
            if set(trajectory.joint_names) != set(phase["joints"]) or not trajectory.points:
                raise ValueError("MoveIt returned an empty or mismatched joint trajectory.")
            for name, value in zip(trajectory.joint_names, trajectory.points[-1].positions):
                current[name] = value
            if phase["name"] == "Close gripper":
                attachment(CollisionObject.ADD)
            stages.append(
                {
                    "name": phase["name"],
                    "xyz": phase.get("xyz"),
                    "joint_names": list(trajectory.joint_names),
                    "points": [
                        {
                            "t": point.time_from_start.sec + point.time_from_start.nanosec / 1e9,
                            "positions": list(point.positions),
                        }
                        for point in trajectory.points
                    ],
                }
            )
    finally:
        attachment(CollisionObject.REMOVE)
        remove_world([cube_id, *(item.id for item in world)])
    return {"groups": sorted(groups), "robot": scene.robot_model_name, "stages": stages}


def main():
    with open("/tmp/studio_so101_moveit_plan.lock", "w") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print(json.dumps({"error": "Another stereo plan is using this MoveIt scene."}))
            return
        serve()


def serve():
    rclpy.init()
    studio = rclpy.create_node("studio_so101_plan_only")
    try:
        print(json.dumps(plan(studio, json.load(sys.stdin)), allow_nan=False))
    except Exception as exc:
        print(json.dumps({"error": str(exc)}))
    finally:
        studio.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
