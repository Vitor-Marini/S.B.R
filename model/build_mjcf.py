#!/usr/bin/env python3
"""params.json → MuJoCo model (MJCF). One source of truth for the robot's physics.

    python3 model/build_mjcf.py            → writes model/sbr.xml (to open in `python -m mujoco.viewer`)
    from build_mjcf import build; xml = build()

Frame: x forward, y left, z up. Pitch = rotation about y (positive = leaning forward). Wheels spin about y.
"""
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))


def load_params(path=os.path.join(HERE, "params.json")):
    return json.load(open(path))


def build(p=None):
    p = p or load_params()
    b, w, m = p["body"], p["wheel"], p["motor"]
    d, mass = b["com_height"], b["mass"]
    iyy = b["pendulum_length_L"] * mass * d - mass * d * d     # pitch inertia about the CoM, from the measured L
    sx, sy, sz = b["size"]
    ixx = mass * (sy ** 2 + sz ** 2) / 12                       # roll / yaw: box estimate (barely matter here)
    izz = mass * (sx ** 2 + sy ** 2) / 12
    ixx = min(ixx, iyy + izz)                                   # a real body needs A + B >= C (small L broke it)
    r, hw, ty = w["radius"], w["width"] / 2, w["track"] / 2
    iw_spin = w["mass"] * r * r / 2
    iw_other = w["mass"] * (3 * r * r + w["width"] ** 2) / 12
    tq = m["torque_limit"]
    wheels = ""
    for name, y in (("left", ty), ("right", -ty)):
        wheels += f"""
      <body name="wheel_{name}" pos="0 {y} 0">
        <joint name="{name}" type="hinge" axis="0 1 0" damping="0.0005" armature="{m['rotor_inertia']}"/>
        <inertial pos="0 0 0" mass="{w['mass']}" diaginertia="{iw_other} {iw_spin} {iw_other}"/>
        <geom type="cylinder" size="{r} {hw}" euler="90 0 0" rgba=".92 .92 .9 1" friction="{p['floor_friction']} 0.01 0.001"/>
        <geom type="box" size="{r * 0.9} {hw * 1.05} 0.004" rgba=".2 .2 .2 1" contype="0" conaffinity="0" mass="0"/>
      </body>"""
    return f"""<mujoco model="sbr">
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/></visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" rgb1=".75 .75 .72" rgb2=".68 .68 .65" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance=".1"/>
  </asset>
  <worldbody>
    <light pos="0 -1 2" dir="0 .5 -1"/>
    <geom name="floor" type="plane" size="5 5 .1" material="grid" friction="{p['floor_friction']} 0.01 0.001"/>
    <body name="robot" pos="0 0 {r}">
      <freejoint name="root"/>
      <inertial pos="0 0 {d}" mass="{mass}" diaginertia="{ixx} {iyy} {izz}"/>
      <geom type="box" pos="0 0 {sz / 2 - 0.02}" size="{sx / 2} {sy / 2 - 0.01} {sz / 2}" rgba=".12 .12 .13 1" contype="0" conaffinity="0" mass="0"/>
      <geom type="box" pos="0 0 {sz - 0.005}" size="{sx / 2 * 0.9} {sy / 2 * 0.8} 0.02" rgba=".1 .45 .55 1" contype="0" conaffinity="0" mass="0"/>
      <geom type="box" pos="{sx / 2} 0 {sz / 2}" size="0.005 0.02 0.01" rgba="1 .5 0 1" contype="0" conaffinity="0" mass="0"/>
      <site name="imu" pos="0 0 {p['imu_height']}"/>{wheels}
    </body>
  </worldbody>
  <actuator>
    <velocity name="left" joint="left" kv="{m['servo_gain']}" forcerange="-{tq} {tq}" forcelimited="true"/>
    <velocity name="right" joint="right" kv="{m['servo_gain']}" forcerange="-{tq} {tq}" forcelimited="true"/>
  </actuator>
  <sensor>
    <gyro name="gyro" site="imu"/>
    <accelerometer name="accel" site="imu"/>
  </sensor>
</mujoco>
"""


if __name__ == "__main__":
    out = os.path.join(HERE, "sbr.xml")
    open(out, "w").write(build())
    print("wrote", out)
