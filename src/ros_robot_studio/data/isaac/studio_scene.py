"""Runs ONLY via Isaac 5.1's official python.sh, never imported by desktop Python."""

import argparse
import json
import signal
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--build-only", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=0)
    args, _ = parser.parse_known_args()
    request = json.loads(args.request.read_text())
    output = Path(request["output_directory"])
    output.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {"headless": args.headless, "renderer": "RaytracedLighting", "width": 1280, "height": 800}
    )

    import numpy as np
    import omni.graph.core as og
    import omni.kit.commands
    import omni.usd
    import usdrt
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.viewports import set_camera_view
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    enable_extension("isaacsim.asset.importer.urdf")
    enable_extension("isaacsim.ros2.bridge")
    app.update()
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 60)
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    world.scene.add_default_ground_plane(z_position=-0.75)
    # Procedural mounting table; no network asset dependency.
    table = UsdGeom.Cube.Define(stage, "/World/MountingTable")
    table.CreateSizeAttr(1.0)
    UsdGeom.Xformable(table).AddScaleOp().Set(Gf.Vec3f(0.8, 0.7, 0.04))
    UsdGeom.Xformable(table).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.025))
    table.CreateDisplayColorAttr([(0.16, 0.20, 0.26)])
    UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    light = UsdLux.DomeLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(800)
    status, config = omni.kit.commands.execute("URDFCreateImportConfig")
    assert status
    config.merge_fixed_joints = False
    config.fix_base = True
    config.import_inertia_tensor = True
    config.distance_scale = 1.0
    config.self_collision = False
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
    robot_prim.GetReferences().AddReference(str(robot_usd))
    app.update()
    roots = [
        p
        for p in stage.Traverse()
        if str(p.GetPath()).startswith("/World/Robot") and p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    assert len(roots) == 1, "Exactly one robot articulation root required"
    articulation_path = str(roots[0].GetPath())
    robot = world.scene.add(SingleArticulation(prim_path=articulation_path, name="studio_robot"))
    world.reset()
    actual_names = list(robot.dof_names)
    expected = request["expected_joints"]
    assert set(actual_names) == set(expected), f"Joint names mismatch: {actual_names}"
    initial = np.array([request["initial"][n] for n in actual_names])
    robot.set_joint_positions(initial)
    robot.set_joint_velocities(np.zeros(len(actual_names)))
    robot.get_articulation_controller().set_gains(
        kps=np.full(len(actual_names), 1000.0), kds=np.full(len(actual_names), 50.0)
    )
    robot.get_articulation_controller().apply_action(
        __import__("isaacsim.core.utils.types", fromlist=["ArticulationAction"]).ArticulationAction(
            joint_positions=initial
        )
    )
    properties = robot.dof_properties
    joint_report = {}
    for i, name in enumerate(actual_names):
        lower, upper = float(properties["lower"][i]), float(properties["upper"][i])
        ref = expected[name]
        if ref["kind"] != "continuous":
            assert abs(lower - ref["lower"]) < 1e-4 and abs(upper - ref["upper"]) < 1e-4, (
                f"USD limits changed for {name}: {lower, upper}"
            )
        joint_report[name] = dict(lower=lower, upper=upper, kind=ref["kind"])
    # Explicit per-graph domain context, initialized before any publishers/subscribers.
    keys = og.Controller.Keys
    graph = "/World/ROS2"
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
    scene_path = output / "scene.usd"
    stage.GetRootLayer().Export(str(scene_path))
    report = dict(
        valid=True,
        backend_id=request["backend_id"],
        model_signature=request["model_signature"],
        domain_id=request["domain_id"],
        stage_units=UsdGeom.GetStageMetersPerUnit(stage),
        articulation_root=articulation_path,
        joint_names=actual_names,
        joints=joint_report,
        initial_positions=dict(zip(actual_names, map(float, robot.get_joint_positions()))),
        scene=str(scene_path),
        urdf_unmodified=True,
        physical_grasp_validated=False,
        drive_stiffness=1000.0,
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
                time.sleep(max(0, 1 / 60 - (time.monotonic() - before)))
    finally:
        world.stop()
        app.close()


if __name__ == "__main__":
    main()
