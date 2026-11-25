#!/usr/bin/env python3
"""params.json + Fusion meshes → USD (Isaac Lab / Newton / PhysX). Same physics as build_mjcf.py.

    python model/build_usd.py            → writes model/sbr.usda   (needs `pxr`: usd-core, comes with Isaac Lab)

Frame (same as the MJCF): x forward, y left, z up, origin at the axle center; wheels spin about y.
Physics (mass, CoM, inertia, wheel size, track) comes ONLY from params.json (measured); the CAD meshes are visual.
Colliders: wheel cylinders + a box for the body + a box for the battery (on top, the first thing to hit the floor).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from build_mjcf import load_params  # noqa: E402

MESHES = os.path.join(HERE, "meshes")
AXLE_CAD_MM = np.array([-50.05, -0.26, 0.0])     # axle center in the Fusion assembly frame (meshes/info.json)


def cad_to_robot(v):
    """Fusion frame (mm, X = axle with +X left, −Y front, Z up) → robot frame (m, x front, y left, z up)."""
    v = (v - AXLE_CAD_MM) * 0.001
    return np.stack([-v[..., 1], v[..., 0], v[..., 2]], -1)


def read_stl(name):
    b = open(os.path.join(MESHES, name + ".stl"), "rb").read()
    n = int(np.frombuffer(b[80:84], "<u4")[0])
    tri = np.frombuffer(b[84:84 + n * 50], np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))["v"]
    return cad_to_robot(tri.astype(np.float64))      # (n, 3, 3) in metres, robot frame


def build(out=os.path.join(HERE, "sbr.usda"), p=None):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, Vt

    p = p or load_params()
    b, w, m = p["body"], p["wheel"], p["motor"]
    d, mass = b["com_height"], b["mass"]
    iyy = b["pendulum_length_L"] * mass * d - mass * d * d       # same formulas as build_mjcf.py
    sx, sy, sz = b["size"]
    ixx = min(mass * (sy ** 2 + sz ** 2) / 12, iyy + mass * (sx ** 2 + sy ** 2) / 12)
    izz = mass * (sx ** 2 + sy ** 2) / 12
    r, width, ty = w["radius"], w["width"], w["track"] / 2

    stage = Usd.Stage.CreateNew(out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/sbr")
    stage.SetDefaultPrim(root.GetPrim())
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

    def rigid(path, mass_, com, inertia):
        x = UsdGeom.Xform.Define(stage, path)
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        ma = UsdPhysics.MassAPI.Apply(x.GetPrim())
        ma.CreateMassAttr(mass_)
        ma.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        ma.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        return x

    def mesh(path, tri, offset, color):
        pts, idx = np.unique(np.round(tri.reshape(-1, 3) - offset, 6), axis=0, return_inverse=True)
        g = UsdGeom.Mesh.Define(stage, path)
        g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
        g.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(tri)))
        g.CreateFaceVertexIndicesAttr(Vt.IntArray(idx.reshape(-1).astype(int).tolist()))
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        g.CreateDoubleSidedAttr(True)

    def box(path, lo, hi):
        c = UsdGeom.Cube.Define(stage, path)
        c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*((lo + hi) / 2)))
        c.AddScaleOp().Set(Gf.Vec3f(*(hi - lo)))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim())
        c.MakeInvisible()

    # body: measured mass/inertia; colliders from the mesh extents; CAD meshes as visuals
    body = rigid("/sbr/body", mass, (0, 0, d), (ixx, iyy, izz))
    tb, tbat = read_stl("body"), read_stl("battery")
    mesh("/sbr/body/visual", tb, np.zeros(3), (0.55, 0.6, 0.67))
    mesh("/sbr/body/battery_visual", tbat, np.zeros(3), (0.16, 0.5, 0.75))
    box("/sbr/body/collision", tb.reshape(-1, 3).min(0), tb.reshape(-1, 3).max(0))
    box("/sbr/body/battery_collision", tbat.reshape(-1, 3).min(0), tbat.reshape(-1, 3).max(0))

    iw_spin = w["mass"] * r * r / 2
    iw_other = w["mass"] * (3 * r * r + width ** 2) / 12
    for name, y in (("left", ty), ("right", -ty)):
        wheel = rigid(f"/sbr/wheel_{name}", w["mass"], (0, 0, 0), (iw_other, iw_spin, iw_other))
        wheel.AddTranslateOp().Set(Gf.Vec3d(0, y, 0))
        tw = read_stl(f"wheel_{name}")
        mesh(f"/sbr/wheel_{name}/visual", tw, np.array([0, y, 0]), (0.15, 0.15, 0.15))
        cyl = UsdGeom.Cylinder.Define(stage, f"/sbr/wheel_{name}/collision")
        cyl.CreateAxisAttr("Y")
        cyl.CreateRadiusAttr(r)
        cyl.CreateHeightAttr(width)
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        cyl.MakeInvisible()
        j = UsdPhysics.RevoluteJoint.Define(stage, f"/sbr/body/{name}")
        j.CreateBody0Rel().SetTargets([body.GetPath()])
        j.CreateBody1Rel().SetTargets([wheel.GetPath()])
        j.CreateAxisAttr("Y")
        j.CreateLocalPos0Attr(Gf.Vec3f(0, y, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        # stepper = velocity servo (USD angular drives are per degree): Isaac Lab's actuator cfg sets the same values
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(m["servo_gain"] * np.pi / 180)
        drive.CreateMaxForceAttr(m["torque_limit"])

    mat = UsdPhysics.MaterialAPI.Apply(stage.DefinePrim("/sbr/material", "Material"))
    mat.CreateStaticFrictionAttr(p["floor_friction"])
    mat.CreateDynamicFrictionAttr(p["floor_friction"])
    stage.GetRootLayer().Save()
    return out


def _selftest():
    tb = read_stl("body").reshape(-1, 3)
    tw = read_stl("wheel_left").reshape(-1, 3)
    assert abs(tw[:, 2].min() + 0.0375) < 1e-3, "wheel bottom must be at the floor (z = −r)"
    assert 0.06 < tw[:, 1].mean() < 0.09, "left wheel must be at +y"
    assert tb[:, 2].max() > 0.14, "body top ~0.15 m above the axle"


if __name__ == "__main__":
    _selftest()
    print("wrote", build())
