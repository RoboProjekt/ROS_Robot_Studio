"""Isaac-only USD authoring and official RGB/CameraInfo/TF OmniGraph setup."""

import json
import math
import os
import tempfile

import numpy as np
import omni.graph.core as og
import usdrt
from pxr import Gf, Usd, UsdGeom

METADATA = "studioVisionRGB"
GRAPH = "/World/StudioVisionROS"


def read_cameras(stage):
    raw = stage.GetRootLayer().customLayerData.get(METADATA, "[]")
    cameras = json.loads(raw)
    if not isinstance(cameras, list) or len(cameras) > 8:
        raise ValueError("Invalid Studio RGB camera metadata")
    return cameras


def vector(value):
    return [float(item) for item in value]


def tf_pose(matrix):
    quaternion = matrix.ExtractRotationQuat().GetNormalized()
    return vector(matrix.ExtractTranslation()), [
        *vector(quaternion.GetImaginary()),
        quaternion.GetReal(),
    ]


def author_camera(stage, profile):
    mount = profile["mount"]
    path = "/World/StudioCameras/" + profile["id"]
    camera = UsdGeom.Camera.Define(stage, path)
    xform = UsdGeom.XformCommonAPI(camera)
    if not xform.SetTranslate(Gf.Vec3d(*mount["position"])) or not xform.SetRotate(
        Gf.Vec3f(*mount["usd_rpy_deg"]), UsdGeom.XformCommonAPI.RotationOrderXYZ
    ):
        raise ValueError("Cannot author the camera pose")
    aperture = 20.0
    camera.GetHorizontalApertureAttr().Set(aperture)
    camera.GetVerticalApertureAttr().Set(aperture * profile["height"] / profile["width"])
    camera.GetFocalLengthAttr().Set(
        aperture / (2 * math.tan(math.radians(mount["horizontal_fov"]) / 2))
    )
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 100))
    item = {
        key: profile[key]
        for key in (
            "id",
            "rgb_topic",
            "info_topic",
            "width",
            "height",
            "fps",
            "camera_frame",
            "optical_frame",
            "world_frame",
            "base_frame",
        )
    }
    item["prim"] = path
    table = UsdGeom.Cube.Get(stage, "/World/MountingTable")
    if table:
        matrix = UsdGeom.Xformable(table).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        top = table.GetSizeAttr().Get() / 2
        point = matrix.Transform(Gf.Vec3d(0, 0, top))
        tangent_x = matrix.TransformDir(Gf.Vec3d(1, 0, 0))
        tangent_y = matrix.TransformDir(Gf.Vec3d(0, 1, 0))
        normal = Gf.Cross(tangent_x, tangent_y).GetNormalized()
        item["plane"] = dict(
            name="Mounting table surface",
            frame=profile["world_frame"],
            point=vector(point),
            normal=vector(normal),
        )
        if mount["demo_cube"]:
            contact = matrix.Transform(Gf.Vec3d(0.26, -0.29, top))
            cube = UsdGeom.Cube.Define(stage, "/World/StudioRGBTestCube")
            cube.CreateSizeAttr(0.055)
            cube.CreateDisplayColorAttr([(0.85, 0.012, 0.012)])
            center = contact + normal * 0.0275
            UsdGeom.XformCommonAPI(cube).SetTranslate(center)
            item["evaluation_contact"] = vector(contact)
    elif mount["demo_cube"]:
        raise ValueError("The RGB demo cube needs the Studio mounting table")
    return item


def configure(stage, request, cameras):
    if request.get("vision_camera"):
        new = author_camera(stage, request["vision_camera"])
        cameras = [c for c in cameras if c["id"] != new["id"]] + [new]
    if not cameras:
        return []
    for fields in (("rgb_topic", "info_topic"), ("camera_frame", "optical_frame")):
        values = [camera[field] for camera in cameras for field in fields]
        if len(values) != len(set(values)):
            raise ValueError("RGB cameras in one scene require unique topics and camera frames")
    metadata = dict(stage.GetRootLayer().customLayerData)
    metadata[METADATA] = json.dumps(cameras)
    stage.GetRootLayer().customLayerData = metadata
    nodes = [
        ("Tick", "omni.graph.action.OnPlaybackTick"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("Time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ]
    connections = []
    values = [
        ("Context.inputs:domain_id", request["domain_id"]),
        ("Context.inputs:useDomainIDEnvVar", False),
        ("Time.inputs:resetOnStop", True),
    ]

    def static_transform(name, parent, child, position, orientation):
        nodes.append((name, "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"))
        connections.extend(
            [
                ("Tick.outputs:tick", f"{name}.inputs:execIn"),
                ("Context.outputs:context", f"{name}.inputs:context"),
                ("Time.outputs:simulationTime", f"{name}.inputs:timeStamp"),
            ]
        )
        values.extend(
            (f"{name}.inputs:{key}", value)
            for key, value in (
                ("parentFrameId", parent),
                ("childFrameId", child),
                ("staticPublisher", True),
                ("topicName", "/tf_static"),
                ("translation", position),
                ("rotation", orientation),
            )
        )

    bases = set()
    for index, camera in enumerate(cameras):
        width, height, fps = camera["width"], camera["height"], camera["fps"]
        if not (16 <= width <= 1920 and 16 <= height <= 1080 and 1 <= fps <= 60):
            raise ValueError("Invalid persisted RGB camera dimensions or rate")
        prim = UsdGeom.Camera.Get(stage, camera["prim"])
        if not prim:
            raise ValueError("Persisted RGB camera prim is missing")
        render, image, info = (f"{prefix}{index}" for prefix in ("Render", "RGB", "Info"))
        nodes.extend(
            [
                (render, "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                (image, "isaacsim.ros2.bridge.ROS2CameraHelper"),
                (info, "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ]
        )
        connections.append(("Tick.outputs:tick", f"{render}.inputs:execIn"))
        values.extend(
            [
                (f"{render}.inputs:width", width),
                (f"{render}.inputs:height", height),
                (f"{render}.inputs:cameraPrim", [usdrt.Sdf.Path(camera["prim"])]),
                (f"{image}.inputs:type", "rgb"),
            ]
        )
        for name, topic in ((image, camera["rgb_topic"]), (info, camera["info_topic"])):
            connections.extend(
                [
                    (f"{render}.outputs:execOut", f"{name}.inputs:execIn"),
                    (f"{render}.outputs:renderProductPath", f"{name}.inputs:renderProductPath"),
                    ("Context.outputs:context", f"{name}.inputs:context"),
                ]
            )
            values.extend(
                [
                    (f"{name}.inputs:topicName", topic),
                    (f"{name}.inputs:frameId", camera["optical_frame"]),
                    (f"{name}.inputs:frameSkipCount", max(0, round(60 / fps) - 1)),
                    (f"{name}.inputs:resetSimulationTimeOnStop", True),
                ]
            )
        world_from_usd = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )
        # Gf uses row-vector matrices. Apply each local basis change before the USD pose.
        optical_from_usd = Gf.Matrix4d(1).SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 180))
        world_from_optical = optical_from_usd * world_from_usd
        link_from_optical = Gf.Matrix4d(
            0,
            -1,
            0,
            0,
            0,
            0,
            -1,
            0,
            1,
            0,
            0,
            0,
            0,
            0,
            0,
            1,
        )
        world_from_link = link_from_optical.GetInverse() * world_from_optical
        world, base = camera["world_frame"], camera["base_frame"]
        static_transform(
            f"CameraTF{index}", world, camera["camera_frame"], *tf_pose(world_from_link)
        )
        static_transform(
            f"OpticalTF{index}",
            camera["camera_frame"],
            camera["optical_frame"],
            *tf_pose(link_from_optical),
        )
        if (world, base) not in bases:
            robot_base = [
                p
                for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Robot"))
                if p.GetName() == base and p.IsA(UsdGeom.Xformable)
            ]
            if len(robot_base) != 1:
                raise ValueError(f"Expected one robot base prim for {base!r}")
            base_pose = UsdGeom.Xformable(robot_base[0]).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()
            )
            static_transform(f"BaseTF{index}", world, base, *tf_pose(base_pose))
            bases.add((world, base))
        camera["world_from_optical"] = np.asarray(world_from_optical).T.tolist()
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: nodes,
            keys.CONNECT: connections,
            keys.SET_VALUES: values,
        },
    )
    return cameras


def export_derived(stage, destination):
    # Flatten the composed source so the exported scene can be moved independently.
    parent, suffix = os.path.dirname(destination), os.path.splitext(destination)[1]
    fd, temporary = tempfile.mkstemp(prefix=".studio-rgb-", suffix=suffix, dir=parent)
    os.close(fd)
    try:
        if not stage.Flatten().Export(temporary):
            raise ValueError("Could not export the derived RGB scene")
        os.link(temporary, destination)
    finally:
        os.unlink(temporary)
