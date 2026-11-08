"""Runs ONLY via Isaac 5.1's official python.sh, never imported by desktop Python."""

import argparse
import hashlib
import json
import os
import signal
import time
import traceback
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=0)
    args, _ = parser.parse_known_args()
    request = json.loads(args.request.read_text())
    source_hashes = request.get(
        "source_files",
        {request["urdf"]: hashlib.sha256(Path(request["urdf"]).read_bytes()).hexdigest()},
    )
    assert all(
        hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest
        for p, digest in source_hashes.items()
    ), "Source files changed since the scene request"
    output = Path(request["output_directory"])
    output.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": args.headless,
            "renderer": "RaytracedLighting",
            "width": 1280,
            "height": 800,
            "extra_args": [
                "--/rtx/directLighting/sampledLighting/enabled=false",
                "--/rtx/directLighting/sampledLighting/autoEnableLightCountThreshold=100000",
            ],
        }
    )

    try:
        run_scene(args, app, request, output, source_hashes)
    except BaseException:
        # Kit disables Python logging during shutdown; report the cause first.
        traceback.print_exc()
        raise
    finally:
        app.close()


def run_scene(args, app, request, output, source_hashes):
    import numpy as np
    import omni.graph.core as og
    import omni.kit.commands
    import omni.usd
    import usdrt
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics

    enable_extension("isaacsim.asset.importer.urdf")
    enable_extension("isaacsim.ros2.bridge")
    app.update()
    if request.get("existing_scene"):
        omni.usd.get_context().open_stage(request["existing_scene"])
        existing = omni.usd.get_context().get_stage()
        assert UsdGeom.GetStageMetersPerUnit(existing) == 1.0, "Existing scene must use meters"
        assert UsdGeom.GetStageUpAxis(existing) == UsdGeom.Tokens.z, "Existing scene must be Z-up"
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
    world.get_physics_context().set_gravity(0.0)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world.scene.add_default_ground_plane(z_position=-0.75)
    # Procedural mounting table; no network asset dependency.
    table = UsdGeom.Cube.Define(stage, "/World/MountingTable")
    table.CreateSizeAttr(1.0)
    UsdGeom.Xformable(table).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.025))
    UsdGeom.Xformable(table).AddScaleOp().Set(Gf.Vec3f(0.8, 0.7, 0.04))
    table.CreateDisplayColorAttr([(0.16, 0.20, 0.26)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(800)
    if request.get("existing_scene"):
        # Open the user-selected USD as the stage; do not overwrite its source.
        stage = omni.usd.get_context().get_stage()
    else:
        status, config = omni.kit.commands.execute("URDFCreateImportConfig")
        assert status
        config.merge_fixed_joints = False
        config.fix_base = True
        config.import_inertia_tensor = True
        config.distance_scale = 1.0
        config.self_collision = False
        config.make_default_prim = True
        config.default_drive_strength = 1000.0
        config.default_position_drive_damping = 50.0
        robot_usd = output / "robot.usd"
        status, root_path = omni.kit.commands.execute(
            "URDFParseAndImportFile",
            urdf_path=request["urdf"],
            import_config=config,
            dest_path=str(robot_usd),
            get_articulation_root=True,
        )
        assert status and root_path, "URDF importer failed"
        # Import to USD above, then reference its default prim into the actual scene.
        robot_prim = stage.DefinePrim("/World/Robot", "Xform")
        # Isaac 5.1 may emit a layer without a defaultPrim. Reference the actual
        # imported top-level prim explicitly instead of an unresolved empty target.
        asset_root = "/" + root_path.strip("/").split("/")[0]
        robot_prim.GetReferences().AddReference(str(robot_usd), asset_root)
    app.update()
    roots = [
        p
        for p in stage.Traverse()
        if str(p.GetPath()).startswith("/World/Robot") and p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    assert len(roots) == 1, "Exactly one robot articulation root required"
    articulation_path = str(roots[0].GetPath())
    solver = PhysxSchema.PhysxArticulationAPI.Apply(roots[0])
    solver.CreateSolverPositionIterationCountAttr(64)
    solver.CreateSolverVelocityIterationCountAttr(8)
    # Use force drives so gains are torques per radian, independent of each
    # link's small inertia. Importer acceleration drives sag under gravity.
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith("/World/Robot") and prim.GetTypeName() in (
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
        ):
            angular = prim.GetTypeName() == "PhysicsRevoluteJoint"
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular" if angular else "linear")
            factor = np.pi / 180 if angular else 1.0
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(5000.0 * factor)
            drive.CreateDampingAttr(50.0 * factor)
            effort = request["expected_joints"][prim.GetName()].get("effort")
            if effort:
                drive.CreateMaxForceAttr(effort)
    robot = world.scene.add(SingleArticulation(prim_path=articulation_path, name="studio_robot"))
    world.reset()
    actual_names = list(robot.dof_names)
    expected = request["expected_joints"]
    assert set(actual_names) == set(expected), f"Joint names mismatch: {actual_names}"
    initial = np.array([request["initial"][n] for n in actual_names])
    robot.set_joint_positions(initial)
    robot.set_joint_velocities(np.zeros(len(actual_names)))
    robot.get_articulation_controller().set_gains(
        kps=np.full(len(actual_names), 5000.0),
        kds=np.full(len(actual_names), 50.0),
        save_to_usd=True,
    )
    robot.get_articulation_controller().apply_action(
        __import__("isaacsim.core.utils.types", fromlist=["ArticulationAction"]).ArticulationAction(
            joint_positions=initial
        )
    )
    properties = robot.dof_properties
    joint_report = {}
    usd_joints = {
        p.GetName(): p
        for p in stage.Traverse()
        if p.GetTypeName() in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint")
    }
    for i, name in enumerate(actual_names):
        lower, upper = float(properties["lower"][i]), float(properties["upper"][i])
        ref = expected[name]
        if ref["kind"] != "continuous":
            assert abs(lower - ref["lower"]) < 1e-4 and abs(upper - ref["upper"]) < 1e-4, (
                f"USD limits changed for {name}: {lower, upper}"
            )
        prim = usd_joints[name]
        expected_type = (
            "PhysicsPrismaticJoint" if ref["kind"] == "prismatic" else "PhysicsRevoluteJoint"
        )
        assert prim.GetTypeName() == expected_type, "USD joint type changed: " + name
        joint = UsdPhysics.Joint(prim)
        axis_name = prim.GetAttribute("physics:axis").Get()
        axis = Gf.Vec3d({"X": (1, 0, 0), "Y": (0, 1, 0), "Z": (0, 0, 1)}[axis_name])
        quat = joint.GetLocalRot0Attr().Get()
        actual_axis = np.array(Gf.Rotation(Gf.Quatd(quat)).TransformDir(axis))
        expected_axis = np.array(ref["axis_parent"])
        axis_error = float(np.linalg.norm(actual_axis - expected_axis))
        assert axis_error < 1e-4, (
            f"USD joint axis differs: {name}: {actual_axis} vs {expected_axis}"
        )
        drive = UsdPhysics.DriveAPI.Get(prim, "linear" if ref["kind"] == "prismatic" else "angular")
        drive_limit = float(drive.GetMaxForceAttr().Get())
        assert drive.GetTypeAttr().Get() == "force", "Expected a force drive: " + name
        if ref.get("effort"):
            assert abs(drive_limit - ref["effort"]) < 1e-4, "URDF effort limit changed: " + name
        joint_report[name] = dict(
            lower=lower,
            upper=upper,
            kind=ref["kind"],
            axis_parent=actual_axis.tolist(),
            axis_error=axis_error,
            drive_type=drive.GetTypeAttr().Get(),
            drive_max_force=drive_limit,
        )
    # Flush articulation positions to USD without gravity disturbing the neutral pose.
    world.step(render=True)
    cache = UsdGeom.XformCache()
    observed_links = {}
    for prim in stage.Traverse():
        if (
            str(prim.GetPath()).startswith("/World/Robot")
            and prim.GetName() in request["expected_link_positions"]
        ):
            observed_links[prim.GetName()] = np.array(
                cache.GetLocalToWorldTransform(prim).ExtractTranslation()
            ).tolist()
    assert set(observed_links) == set(request["expected_link_positions"]), (
        "Imported link frame names differ"
    )
    neutral_error = max(
        float(np.linalg.norm(np.array(observed_links[n]) - q))
        for n, q in request["expected_link_positions"].items()
    )
    print(
        "STUDIO_IMPORT_FRAMES "
        + json.dumps(
            dict(
                error_m=neutral_error,
                positions=robot.get_joint_positions().tolist(),
                links=observed_links,
            )
        ),
        flush=True,
    )
    assert neutral_error < 0.002, f"Imported link poses differ from existing FK: {neutral_error} m"
    world.get_physics_context().set_gravity(-9.81)

    # Explicit per-graph domain context, initialized before any publishers/subscribers.
    keys = og.Controller.Keys
    graph = "/World/ROS2"
    if stage.GetPrimAtPath(graph).IsValid():
        stage.RemovePrim(graph)
    og.Controller.edit(
        {"graph_path": graph, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("States", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("Commands", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("Controller", "isaacsim.core.nodes.IsaacArticulationController"),
                ("Clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", f"{n}.inputs:execIn")
                for n in ("States", "Commands", "Controller", "Clock")
            ]
            + [
                ("Context.outputs:context", f"{n}.inputs:context")
                for n in ("States", "Commands", "Clock")
            ]
            + [
                ("Time.outputs:simulationTime", f"{n}.inputs:timeStamp")
                for n in ("States", "Clock")
            ]
            + [
                ("Commands.outputs:jointNames", "Controller.inputs:jointNames"),
                ("Commands.outputs:positionCommand", "Controller.inputs:positionCommand"),
                ("Commands.outputs:velocityCommand", "Controller.inputs:velocityCommand"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", False),
                ("Context.inputs:domain_id", request["domain_id"]),
                ("States.inputs:topicName", request["state_topic"]),
                ("States.inputs:targetPrim", [usdrt.Sdf.Path(articulation_path)]),
                ("Commands.inputs:topicName", request["command_topic"]),
                ("Controller.inputs:robotPath", articulation_path),
                ("Clock.inputs:topicName", request["clock_topic"]),
            ],
        },
    )
    # Isaac joint names are imported unchanged; arbitrary remapping is validated in Studio.
    assert all(n == remote for n, remote in request["joint_mapping"].items()), (
        "Generated scene currently requires identity mapping; configure custom existing scenes explicitly"
    )
    set_camera_view(eye=np.array([0.75, 0.65, 0.45]), target=np.array([0.10, 0, 0.18]))
    scene_path = output / ("studio-scene.usd" if request.get("existing_scene") else "scene.usd")
    stage.GetRootLayer().Export(str(scene_path))
    report = dict(
        valid=True,
        simulator_process_id=os.getpid(),
        request_id=request.get("request_id"),
        neutral_link_error_m=neutral_error,
        observed_link_positions=observed_links,
        backend_id=request["backend_id"],
        model_signature=request["model_signature"],
        domain_id=request["domain_id"],
        stage_units=UsdGeom.GetStageMetersPerUnit(stage),
        articulation_root=articulation_path,
        joint_names=actual_names,
        joints=joint_report,
        initial_positions=dict(zip(actual_names, map(float, robot.get_joint_positions()))),
        scene=str(scene_path),
        urdf_unmodified=all(
            hashlib.sha256(Path(p).read_bytes()).hexdigest() == digest
            for p, digest in source_hashes.items()
        ),
        physical_grasp_validated=False,
        drive_stiffness=5000.0,
        drive_damping=50.0,
    )
    (output / "scene-validation.json").write_text(json.dumps(report, indent=2))
    print("STUDIO_SCENE_READY " + json.dumps(report), flush=True)
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    start = time.monotonic()
    frames = 0
    image_requested = False
    try:
        if not args.build_only:
            world.play()
            while (
                running
                and app.is_running()
                and (not args.max_seconds or time.monotonic() - start < args.max_seconds)
            ):
                before = time.monotonic()
                world.step(render=True)
                frames += 1
                if request.get("capture_image") and frames >= 60 and not image_requested:
                    from omni.kit.viewport.utility import (
                        capture_viewport_to_file,
                        get_active_viewport,
                    )

                    capture_viewport_to_file(get_active_viewport(), request["capture_image"])
                    image_requested = True
                time.sleep(max(0, 1 / 60 - (time.monotonic() - before)))
    finally:
        world.stop()


if __name__ == "__main__":
    main()
