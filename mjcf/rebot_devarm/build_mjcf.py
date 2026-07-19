"""Build the menagerie-style rebot_devarm.xml from the urdf-to-mjcf output.

Base: `urdf-to-mjcf <urdf> -ct convex_hull` (discoverse-dev/urdf-to-mjcf) —
gives per-link bodies with the exact URDF inertials (gripper_end kept as a
separate welded body, so no fixed-joint inertial-merge bug) plus coacd convex
collision meshes. This script layers the menagerie conventions on top:

  - fixed base (drop the freejoint), base_link inertial from the URDF
  - <option> integrator, <default> joint armature/damping + actuator classes
  - position actuators with the validated PD gains (rs06 / rs00 / gripper)
  - home + raised keyframes
  - meshdir=assets, flat mesh paths

Gravity parity verified: MuJoCo qfrc_bias vs Pinocchio g(q) < 1e-5 N.m.
Run: python build_mjcf.py   (writes rebot_devarm.xml next to it)
"""
import re
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
SRC = "/tmp/u2m/rebot.xml"

tree = ET.parse(SRC)
root = tree.getroot()
root.set("model", "rebot_devarm")

wb = root.find("worldbody")
base = wb.find("body[@name='base_link']")

# 1. fixed base: drop the freejoint
fj = base.find("freejoint")
if fj is not None:
    base.remove(fj)

# 2. base_link inertial from URDF (welded, but keep the model mass correct)
ine = ET.Element("inertial", {
    "pos": "2.87642e-06 -0.000122302 0.0243376", "mass": "1.1774",
    "fullinertia": "0.0013304 0.00213119 0.00275877 1e-08 0 0"})
base.insert(0, ine)

# 3. drop inline floor / light (go to scene.xml)
for g in wb.findall("geom"):
    wb.remove(g)
for l in wb.findall("light"):
    wb.remove(l)

# 4. flatten mesh file paths to meshdir=assets
for mesh in root.iter("mesh"):
    f = mesh.get("file")
    mesh.set("file", f.split("/")[-1])
comp = root.find("compiler")
comp.set("meshdir", "assets")
comp.set("autolimits", "true")

# 5. option
opt = ET.Element("option", {"integrator": "implicitfast", "cone": "elliptic", "impratio": "10"})
root.insert(list(root).index(comp) + 1, opt)

# 6. defaults: add joint dynamics + actuator classes under class "robot"
CLASS = {  # joint -> (actuator class, forcerange, kp, kv, joint damping)
    "joint1": ("rs06", 36, 900, 60, 5), "joint2": ("rs06", 36, 900, 60, 5),
    "joint3": ("rs06", 36, 900, 60, 5), "joint4": ("rs00", 14, 120, 10, 2),
    "joint5": ("rs00", 14, 120, 10, 2), "joint6": ("rs00", 14, 120, 10, 2),
    "joint_left": ("gripper", 500, 100, 4, 1), "joint_right": ("gripper", 500, 100, 4, 1),
}
robot_def = root.find("default/default[@class='robot']")
robot_def.set("class", "robot")
# joint base defaults
jd = ET.SubElement(robot_def, "joint"); jd.set("armature", "0.01"); jd.set("frictionloss", "0.2")
for cls, (fr, kp, kv, dmp) in {
    "rs06": (36, 900, 60, 5), "rs00": (14, 120, 10, 2), "gripper": (500, 100, 4, 1)}.items():
    dc = ET.SubElement(robot_def, "default"); dc.set("class", cls)
    j = ET.SubElement(dc, "joint"); j.set("damping", str(dmp))
    p = ET.SubElement(dc, "position"); p.set("kp", str(kp)); p.set("kv", str(kv))
    p.set("forcerange", f"-{fr} {fr}")

# 7. assign joint + actuator classes
for joint in root.iter("joint"):
    n = joint.get("name")
    if n in CLASS:
        joint.set("class", CLASS[n][0])
act = root.find("actuator")
for m in list(act):
    act.remove(m)
for n, (cls, *_ ) in CLASS.items():
    p = ET.SubElement(act, "position"); p.set("class", cls); p.set("name", n); p.set("joint", n)

# 8. keyframe
kf = ET.SubElement(root, "keyframe")
ET.SubElement(kf, "key", {"name": "home", "qpos": "0 " * 7 + "0", "ctrl": "0 " * 7 + "0"})
ET.SubElement(kf, "key", {"name": "raised",
    "qpos": "0 -0.7 -1.1 0 0 0 0 0", "ctrl": "0 -0.7 -1.1 0 0 0 0 0"})

# drop the stray second <visual> the tool appends
vis = root.findall("visual")
for v in vis[1:]:
    root.remove(v)

# strip scene-only assets (they belong in scene.xml, not the model)
asset = root.find("asset")
for el in list(asset):
    name = el.get("name", "")
    if el.tag == "texture" and el.get("type") == "skybox":
        asset.remove(el)
    elif name == "groundplane":
        asset.remove(el)

ET.indent(tree, space="  ")
out = HERE / "rebot_devarm.xml"
tree.write(out, encoding="unicode", xml_declaration=False)
print("WROTE", out)
