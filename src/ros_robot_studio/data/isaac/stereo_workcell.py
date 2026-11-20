"""USD fixtures used by the Isaac stereo scene builder."""

import math
import random

from pxr import Gf, Usd, UsdGeom, UsdPhysics


def author_fixtures(stage):
    table = UsdGeom.Cube.Get(stage, "/World/MountingTable")
    if not table:
        table = UsdGeom.Cube.Define(stage, "/World/MountingTable")
        table.CreateSizeAttr(1.0)
        pose = UsdGeom.XformCommonAPI(table)
        pose.SetTranslate(Gf.Vec3d(0, 0, -0.025))
        pose.SetScale(Gf.Vec3f(0.8, 0.7, 0.04))
        table.CreateDisplayColorAttr([(0.17, 0.21, 0.24)])
        UsdPhysics.CollisionAPI.Apply(table.GetPrim())
    matrix = UsdGeom.Xformable(table).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    surface = matrix.Transform(Gf.Vec3d(0, 0, table.GetSizeAttr().Get() / 2))[2]
    fixtures = "/World/StudioStereoFixtures"
    UsdGeom.Xform.Define(stage, fixtures)
    cube = UsdGeom.Cube.Define(stage, fixtures + "/RedCube")
    cube.CreateSizeAttr(0.055)
    cube.CreateDisplayColorAttr([(0.7, 0.015, 0.015)])
    UsdGeom.XformCommonAPI(cube).SetTranslate(Gf.Vec3d(0.21, -0.075, surface + 0.0275))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    rng = random.Random(101)
    for row in range(7):
        for col in range(7):
            detail = UsdGeom.Cube.Define(stage, f"{fixtures}/RedCube/Tile_{row}_{col}")
            detail.CreateSizeAttr(1)
            pose = UsdGeom.XformCommonAPI(detail)
            pose.SetScale(Gf.Vec3f(0.0062, 0.0062, 0.0002))
            pose.SetTranslate(Gf.Vec3d((col - 3) * 0.0071, (row - 3) * 0.0071, 0.0276))
            detail.CreateDisplayColorAttr([(rng.uniform(0.22, 0.86), 0.012, 0.012)])

    bowl = UsdGeom.Mesh.Define(stage, fixtures + "/BlueBowl")
    rings = (
        (0.001, 0.003),
        (0.022, 0.005),
        (0.033, 0.014),
        (0.043, 0.031),
        (0.046, 0.031),
        (0.036, 0.010),
        (0.022, 0.001),
        (0.001, 0.001),
    )
    sides = 36
    points = [
        (r * math.cos(2 * math.pi * j / sides), r * math.sin(2 * math.pi * j / sides), z)
        for r, z in rings
        for j in range(sides)
    ]
    faces = [
        (
            i * sides + j,
            i * sides + (j + 1) % sides,
            (i + 1) * sides + (j + 1) % sides,
            (i + 1) * sides + j,
        )
        for i in range(len(rings) - 1)
        for j in range(sides)
    ]
    bowl.CreatePointsAttr(points)
    bowl.CreateFaceVertexCountsAttr([4] * len(faces))
    bowl.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    bowl.CreateSubdivisionSchemeAttr("none")
    color = bowl.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform)
    color.Set([(0.01, 0.025, rng.uniform(0.32, 0.9)) for _ in faces])
    UsdGeom.XformCommonAPI(bowl).SetTranslate(Gf.Vec3d(0.18, 0.055, surface))
