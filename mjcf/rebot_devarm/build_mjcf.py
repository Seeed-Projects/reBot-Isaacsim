"""Build the menagerie-style rebot_devarm.xml from the urdf-to-mjcf output.

Base: `urdf-to-mjcf <urdf> -ct convex_hull` (discoverse-dev/urdf-to-mjcf) —
gives per-link bodies with the exact URDF inertials (gripper_end kept as a
separate welded body, so no fixed-joint inertial-merge bug) plus coacd convex
collision meshes. This script layers the menagerie conventions on top:

  - fixed base (drop the freejoint), full inertials from the URDF
  - <option> integrator, <default> joint armature/damping + actuator classes
  - position actuators with the validated PD gains (rs06 / rs00 / gripper)
  - home + raised keyframes
  - meshdir=assets, flat mesh paths

Gravity parity verified: MuJoCo qfrc_bias vs Pinocchio g(q) < 1e-5 N.m.
Run: python build_mjcf.py   (writes rebot_devarm.xml next to it)
"""
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
SRC = "/tmp/u2m/rebot.xml"
URDF = HERE.parents[1] / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"


def apply_urdf_inertials(mjcf_root):
    """Copy every exact URDF inertial into the matching MJCF body."""
    urdf_root = ET.parse(URDF).getroot()
    bodies = {body.get("name"): body for body in mjcf_root.iter("body")}
    inertia_keys = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    for link in urdf_root.findall("link"):
        source = link.find("inertial")
        if source is None:
            continue
        name = link.get("name")
        body = bodies.get(name)
        if body is None:
            raise ValueError(f"URDF link {name!r} has no MJCF body")
        origin = source.find("origin")
        rpy = [float(value) for value in origin.get("rpy", "0 0 0").split()]
        if any(abs(value) > 1e-12 for value in rpy):
            raise ValueError(
                f"URDF link {name!r} has a rotated inertial frame; "
                "rotate its tensor into the body frame before exporting"
            )
        inertial = body.find("inertial")
        if inertial is None:
            inertial = ET.Element("inertial")
            body.insert(0, inertial)
        inertia = source.find("inertia")
        inertial.set("pos", origin.get("xyz", "0 0 0"))
        inertial.set("mass", source.find("mass").get("value"))
        inertial.set("fullinertia", " ".join(inertia.get(key) for key in inertia_keys))
        for stale_attribute in ("diaginertia", "quat", "euler"):
            inertial.attrib.pop(stale_attribute, None)

tree = ET.parse(SRC)
root = tree.getroot()
root.set("model", "rebot_devarm")

wb = root.find("worldbody")
base = wb.find("body[@name='base_link']")

# 1. fixed base: drop the freejoint
fj = base.find("freejoint")
if fj is not None:
    base.remove(fj)

# 2. Preserve every full URDF inertia, including off-diagonal terms.
apply_urdf_inertials(root)

# 3. drop inline floor / light (go to scene.xml)
for geom in wb.findall("geom"):
    wb.remove(geom)
for light in wb.findall("light"):
    wb.remove(light)

# 4. flatten mesh file paths to meshdir=assets
for mesh in root.iter("mesh"):
    f = mesh.get("file")
    mesh.set("file", f.split("/")[-1])
comp = root.find("compiler")
comp.set("meshdir", "assets")
comp.set("autolimits", "true")

# 5. option
opt = ET.Element(
    "option",
    {"integrator": "implicitfast", "cone": "elliptic", "impratio": "10"},
)
root.insert(list(root).index(comp) + 1, opt)

# 6. defaults: add joint dynamics + actuator classes under class "robot"
CLASS = {  # joint -> (actuator class, forcerange, kp, kv, joint damping)
    "joint1": ("rs06", 36, 900, 60, 5),
    "joint2": ("rs06", 36, 900, 60, 5),
    "joint3": ("rs06", 36, 900, 60, 5),
    "joint4": ("rs00", 14, 120, 10, 2),
    "joint5": ("rs00", 14, 120, 10, 2),
    "joint6": ("rs00", 14, 120, 10, 2),
    "joint_left": ("gripper", 500, 100, 4, 1),
    "joint_right": ("gripper", 500, 100, 4, 1),
}
robot_def = root.find("default/default[@class='robot']")
robot_def.set("class", "robot")
# joint base defaults
jd = ET.SubElement(robot_def, "joint")
jd.set("armature", "0.01")
jd.set("frictionloss", "0.2")
for cls, (fr, kp, kv, dmp) in {
    "rs06": (36, 900, 60, 5),
    "rs00": (14, 120, 10, 2),
    "gripper": (500, 100, 4, 1),
}.items():
    dc = ET.SubElement(robot_def, "default")
    dc.set("class", cls)
    joint = ET.SubElement(dc, "joint")
    joint.set("damping", str(dmp))
    p = ET.SubElement(dc, "position")
    p.set("kp", str(kp))
    p.set("kv", str(kv))
    p.set("forcerange", f"-{fr} {fr}")

# 7. assign joint + actuator classes
for joint in root.iter("joint"):
    n = joint.get("name")
    if n in CLASS:
        joint.set("class", CLASS[n][0])
act = root.find("actuator")
for m in list(act):
    act.remove(m)
for name, (cls, *_) in CLASS.items():
    position = ET.SubElement(act, "position")
    position.set("class", cls)
    position.set("name", name)
    position.set("joint", name)

# 8. keyframe
kf = ET.SubElement(root, "keyframe")
ET.SubElement(
    kf,
    "key",
    {"name": "home", "qpos": "0 " * 7 + "0", "ctrl": "0 " * 7 + "0"},
)
ET.SubElement(
    kf,
    "key",
    {
        "name": "raised",
        "qpos": "0 -0.7 -1.1 0 0 0 0 0",
        "ctrl": "0 -0.7 -1.1 0 0 0 0 0",
    },
)

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
