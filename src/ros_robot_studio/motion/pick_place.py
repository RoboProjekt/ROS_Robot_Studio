"""Generic pick/place generation. Every generated target is compiled and IK-validated."""

from dataclasses import dataclass

import numpy as np

from ros_robot_studio.motion.positions import PositionManager
from ros_robot_studio.motion.program import TaskCompiler
from ros_robot_studio.motion.schema import (
    CartesianPose,
    MotionTask,
    SavedPosition,
    TaskStep,
    VirtualObject,
)
from ros_robot_studio.robots.urdf import pose_rpy


@dataclass
class PickPlaceRequest:
    name: str
    pick: SavedPosition
    place: SavedPosition
    open_values: dict[str, float]
    close_values: dict[str, float]
    offset_frame: str
    approach: float
    lift: float
    retreat: float
    approach_direction: tuple[float, float, float] = (0, 0, 1)
    lift_direction: tuple[float, float, float] = (0, 0, 1)
    retreat_direction: tuple[float, float, float] = (0, 0, 1)
    dwell: float = 0.25
    gripper_confirmed: bool = False


class PickPlaceGenerator:
    def __init__(self, chain, options=None, solver="auto"):
        self.chain, self.options, self.solver = chain, options, solver

    def generate(
        self,
        request: PickPlaceRequest,
        current,
        positions: PositionManager,
        settings=None,
        cancel=None,
        progress=lambda text: None,
    ):
        if not request.gripper_confirmed:
            raise ValueError(
                "Confirm virtual open/close joint values; URDF extrema are not calibrated hardware semantics"
            )
        if not self.chain.gripper_joints:
            raise ValueError(
                "Select and confirm a gripper mapping before generating pick-and-place"
            )
        if set(request.open_values) != set(self.chain.gripper_joints) or set(
            request.close_values
        ) != set(self.chain.gripper_joints):
            raise ValueError("Open/close values must cover all confirmed gripper joints")
        if request.open_values == request.close_values:
            raise ValueError("Open and close configurations must differ")
        for values in (request.open_values, request.close_values):
            self.chain.model.configuration(dict(current, **values))
        for value in (request.approach, request.lift, request.retreat, request.dwell):
            if not np.isfinite(value) or value < 0:
                raise ValueError("Distances and dwell must be finite and nonnegative")
        library = PositionManager(positions.positions)
        compiler = TaskCompiler(self.chain, library, self.options, self.solver)
        pick_q = compiler.resolve_position(request.pick, current, cancel)
        place_q = compiler.resolve_position(request.place, pick_q, cancel)
        pick_t = self.chain.forward_kinematics(pick_q)
        place_t = self.chain.forward_kinematics(place_q)
        if np.linalg.norm(pick_t[:3, 3] - place_t[:3, 3]) < 1e-6:
            raise ValueError("Pick and place locations are identical; choose distinct positions")

        def shifted(matrix, distance, direction, q):
            vector = np.asarray(direction, dtype=float)
            if vector.shape != (3,) or not np.isfinite(vector).all() or np.linalg.norm(vector) == 0:
                raise ValueError("Offset direction must be a finite nonzero 3D vector")
            reference = self.chain.frame_transform(request.offset_frame, q)
            result = matrix.copy()
            result[:3, 3] += reference[:3, :3] @ (vector / np.linalg.norm(vector)) * distance
            return result

        matrices = [
            ("Pre-Pick", shifted(pick_t, request.approach, request.approach_direction, pick_q)),
            ("Pick", pick_t),
            ("Lift", shifted(pick_t, request.lift, request.lift_direction, pick_q)),
            ("Pre-Place", shifted(place_t, request.approach, request.approach_direction, place_q)),
            ("Place", place_t),
            ("Retreat", shifted(place_t, request.retreat, request.retreat_direction, place_q)),
        ]
        names, created = {}, []

        def unique(name):
            existing = {p.name.casefold() for p in library.positions}
            candidate = request.name + " · " + name
            number = 2
            while candidate.casefold() in existing:
                candidate = request.name + " · " + name + f" {number}"
                number += 1
            return candidate

        home = library.capture_joint(unique("Home"), self.chain, current)
        library.add(home)
        created.append(home)
        for name, matrix in matrices:
            source = request.pick if name in ("Pre-Pick", "Pick", "Lift") else request.place
            mode = source.pose.mode if source.kind == "cartesian" else "position_only"
            axis = source.pose.axis if source.kind == "cartesian" else "Z"
            pose = pose_rpy(matrix)
            position = SavedPosition(
                name=unique(name),
                kind="cartesian",
                pose=CartesianPose(
                    frame=self.chain.base_link,
                    end_effector=self.chain.end_effector,
                    position=pose[:3],
                    rpy=pose[3:],
                    mode=mode,
                    axis=axis,
                ),
            )
            library.add(position)
            names[name] = position.id
            created.append(position)
        steps = [
            TaskStep(action="home", name="Home", position_id=home.id),
            TaskStep(
                action="open_gripper",
                name="Open gripper",
                gripper_values=request.open_values,
                release_object=True,
            ),
        ]

        def move(name):
            steps.append(TaskStep(action="move_cartesian", name=name, position_id=names[name]))

        move("Pre-Pick")
        move("Pick")
        steps.append(
            TaskStep(
                action="close_gripper",
                name="Close and attach virtual cube",
                gripper_values=request.close_values,
                attach_object=True,
            )
        )
        if request.dwell:
            steps.append(TaskStep(action="wait", name="Dwell after close", duration=request.dwell))
        for name in ("Lift", "Pre-Place", "Place"):
            move(name)
        steps.append(
            TaskStep(
                action="open_gripper",
                name="Open and release virtual cube",
                gripper_values=request.open_values,
                release_object=True,
            )
        )
        if request.dwell:
            steps.append(
                TaskStep(action="wait", name="Dwell after release", duration=request.dwell)
            )
        move("Retreat")
        steps.append(TaskStep(action="home", name="Return home", position_id=home.id))
        task = MotionTask(
            name=request.name,
            steps=steps,
            home_position_id=home.id,
            virtual_object=VirtualObject(frame=self.chain.base_link, position=tuple(pick_t[:3, 3])),
            extensions={
                "simulation": "simplified visual attachment; no forces, contacts, stability or collision validation"
            },
        )
        if settings:
            task.settings = settings.model_copy(deep=True)
        plan = TaskCompiler(self.chain, library, self.options, self.solver).compile(
            task, current, cancel, progress
        )
        if not plan.valid:
            index, message = next(iter(plan.errors.items()))
            raise ValueError(
                f"Waypoint/step {index + 1} ({task.steps[index].name}) is not valid: {message}. Adjust offsets, frames or taught positions."
            )
        return created, task, plan
