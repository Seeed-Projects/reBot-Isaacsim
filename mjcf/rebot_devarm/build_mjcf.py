"""Build the menagerie-style rebot_devarm.xml directly from the repo URDF.

Self-contained and deterministic: parses
`urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf` and emits the MJCF with

  - fixed base, every URDF value byte-verbatim (inertial pos/mass/full
    tensor including off-diagonals, joint origins/axes/ranges) — only the
    joint-origin rotations are converted (rpy -> body quat)
  - <default> joint armature/damping + actuator classes, position actuators
    with the validated PD gains (rs06 / rs00 / gripper) and
    ctrlrange = joint range
  - home + raised keyframes, meshdir=assets, flat mesh paths

Meshes are the committed files in `assets/` (visual: the URDF link STL, or
the pre-merged `<link>_merged_*.obj` where the URDF splits a link into
several visual parts; collision: the `<link>_convex.stl` convex hulls from
the original discoverse-dev/urdf-to-mjcf import). The script only
references them; it does not regenerate geometry.

Gravity parity vs Pinocchio g(q) < 1e-5 N.m: `parity_mujoco_vs_pinocchio.py`.
Run: python build_mjcf.py   (writes rebot_devarm.xml next to it)
Reproducibility gate: `python build_mjcf.py && git diff --exit-code -- rebot_devarm.xml`
"""
import math
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
URDF = HERE.parents[1] / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
ASSETS = HERE / "assets"
OUT = HERE / "rebot_devarm.xml"

INERTIA_KEYS = ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
JOINT_TYPES = {"revolute": "hinge", "prismatic": "slide"}
# class -> (joint damping, kp, kv, forcerange): hardware-validated PD gains
DYNAMICS = {
    "rs06": ("5", "900", "60", "-36 36"),
    "rs00": ("2", "120", "10", "-14 14"),
    "gripper": ("1", "100", "4", "-500 500"),
}
JOINT_CLASS = {
    "joint1": "rs06",
    "joint2": "rs06",
    "joint3": "rs06",
    "joint4": "rs00",
    "joint5": "rs00",
    "joint6": "rs00",
    "joint_left": "gripper",
    "joint_right": "gripper",
}
# name -> qpos (== ctrl): home = URDF zero, raised = elbow-up L pose
KEYFRAMES = (
    ("home", "0 0 0 0 0 0 0 0"),
    ("raised", "0 -0.7 -1.1 0 0 0 0 0"),
)


def fmt4(value):
    """Format a quat component the way the committed model does (4 dp)."""
    rounded = round(value, 4)
    if rounded == 0:
        return "0"
    return f"{rounded:.4f}".rstrip("0").rstrip(".")


def rpy_to_quat(roll, pitch, yaw):
    """URDF fixed-axis rpy (R = Rz(y) Ry(p) Rx(r)) -> wxyz quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def origin_xyz_rpy(element):
    """xyz string (verbatim) + parsed rpy of an <origin>, defaulting to zero."""
    origin = None if element is None else element.find("origin")
    if origin is None:
        return "0 0 0", (0.0, 0.0, 0.0)
    rpy = tuple(float(v) for v in origin.get("rpy", "0 0 0").split())
    return origin.get("xyz", "0 0 0"), rpy


def unsign_zero(value):
    """Normalize URDF '-0' to '0' (numeric identity, matches committed XML)."""
    return value[1:] if value.startswith("-") and float(value) == 0.0 else value


class Builder:
    def __init__(self):
        self.urdf = ET.parse(URDF).getroot()
        self.links = {link.get("name"): link for link in self.urdf.findall("link")}
        self.child_joints = {}
        child_names = set()
        for joint in self.urdf.findall("joint"):
            parent = joint.find("parent").get("link")
            self.child_joints.setdefault(parent, []).append(joint)
            child_names.add(joint.find("child").get("link"))
        roots = [name for name in self.links if name not in child_names]
        if len(roots) != 1:
            raise ValueError(f"URDF must have exactly one root link, got {roots}")
        self.root_link = roots[0]
        self.meshes = []  # (name, file) in traversal order
        self.actuated = []  # (joint name, "lower upper") in traversal order

    def add_mesh(self, name, file):
        if not (ASSETS / file).is_file():
            raise FileNotFoundError(f"missing committed mesh {ASSETS / file}")
        if (name, file) not in self.meshes:
            self.meshes.append((name, file))

    def add_inertial(self, body, link_name, link):
        source = link.find("inertial")
        if source is None:
            raise ValueError(f"URDF link {link_name!r} has no inertial")
        xyz, rpy = origin_xyz_rpy(source)
        if any(abs(v) > 1e-12 for v in rpy):
            raise ValueError(
                f"URDF link {link_name!r} has a rotated inertial frame; "
                "rotate its tensor into the body frame before exporting"
            )
        inertia = source.find("inertia")
        full = " ".join(unsign_zero(inertia.get(key)) for key in INERTIA_KEYS)
        attrs = {
            "pos": xyz,
            "mass": source.find("mass").get("value"),
            "fullinertia": full,
        }
        ET.SubElement(body, "inertial", attrs)

    def add_joint(self, body, joint):
        name = joint.get("name")
        limit = joint.find("limit")
        range_ = f'{limit.get("lower")} {limit.get("upper")}'
        attrs = {
            "name": name,
            "type": JOINT_TYPES[joint.get("type")],
            "range": range_,
            "axis": joint.find("axis").get("xyz"),
            "class": JOINT_CLASS[name],
        }
        ET.SubElement(body, "joint", attrs)
        self.actuated.append((name, range_))

    def add_geoms(self, body, link_name, link):
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                _, rpy = origin_xyz_rpy(element)
                if any(abs(v) > 1e-12 for v in rpy):
                    raise ValueError(f"{link_name} {tag} has a rotated origin")
        visuals = link.findall("visual")
        if len(visuals) > 1:
            # multi-part visuals were pre-merged into one OBJ at import time
            merged = sorted(ASSETS.glob(f"{link_name}_merged_*.obj"))
            if len(merged) != 1:
                raise FileNotFoundError(
                    f"expected one {link_name}_merged_*.obj in assets/, "
                    f"found {len(merged)}"
                )
            mesh_name = merged[0].stem
            self.add_mesh(mesh_name, merged[0].name)
            attrs = {
                "name": f"{link_name}_visual_0",
                "type": "mesh",
                "mesh": mesh_name,
                "material": "default_material",
                "class": "visual",
                "pos": "0 0 0",
            }
        else:
            file = Path(visuals[0].find("geometry/mesh").get("filename")).name
            mesh_name = f"{link_name}_{Path(file).stem}"
            self.add_mesh(mesh_name, file)
            attrs = {
                "name": f"{link_name}_visual",
                "type": "mesh",
                "mesh": mesh_name,
                "material": "default_material",
                "class": "visual",
            }
        ET.SubElement(body, "geom", attrs)
        hull = f"{link_name}_convex"
        self.add_mesh(hull, f"{hull}.stl")
        collision = {
            "type": "mesh",
            "class": "collision",
            "name": f"{link_name}_collision_{hull}",
            "mesh": hull,
        }
        ET.SubElement(body, "geom", collision)

    def add_body(self, parent_element, link_name, joint):
        link = self.links[link_name]
        attrs = {"name": link_name}
        if joint is None:
            attrs["childclass"] = "robot"
            attrs["pos"] = "0 0 0"
        else:
            xyz, rpy = origin_xyz_rpy(joint)
            attrs["pos"] = xyz
            if any(abs(v) > 1e-12 for v in rpy):
                quat = rpy_to_quat(*rpy)
                attrs["quat"] = " ".join(fmt4(c) for c in quat)
        body = ET.SubElement(parent_element, "body", attrs)
        self.add_inertial(body, link_name, link)
        if joint is not None and joint.get("type") != "fixed":
            self.add_joint(body, joint)
        self.add_geoms(body, link_name, link)
        for child_joint in self.child_joints.get(link_name, []):
            self.add_body(body, child_joint.find("child").get("link"), child_joint)
        return body

    def build(self):
        root = ET.Element("mujoco", {"model": "rebot_devarm"})

        default = ET.SubElement(root, "default")
        robot = ET.SubElement(default, "default", {"class": "robot"})
        visual_class = ET.SubElement(robot, "default", {"class": "visual"})
        ET.SubElement(
            visual_class, "geom", {"contype": "0", "conaffinity": "0", "group": "2"}
        )
        collision_class = ET.SubElement(robot, "default", {"class": "collision"})
        ET.SubElement(
            collision_class, "geom", {"contype": "0", "conaffinity": "1", "group": "3"}
        )
        ET.SubElement(robot, "joint", {"armature": "0.01", "frictionloss": "0.2"})
        for name, (damping, kp, kv, forcerange) in DYNAMICS.items():
            actuator_class = ET.SubElement(robot, "default", {"class": name})
            ET.SubElement(actuator_class, "joint", {"damping": damping})
            ET.SubElement(
                actuator_class,
                "position",
                {"kp": kp, "kv": kv, "forcerange": forcerange},
            )

        compiler = {
            "angle": "radian",
            "meshdir": "assets",
            "balanceinertia": "true",
            "autolimits": "true",
        }
        ET.SubElement(root, "compiler", compiler)
        option = {"integrator": "implicitfast", "cone": "elliptic", "impratio": "10"}
        ET.SubElement(root, "option", option)
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", {"offwidth": "3840", "offheight": "2160"})

        worldbody = ET.SubElement(root, "worldbody")
        self.add_body(worldbody, self.root_link, None)

        asset = ET.SubElement(root, "asset")
        ET.SubElement(
            asset, "material", {"name": "default_material", "rgba": "0.7 0.7 0.7 1"}
        )
        for name, file in self.meshes:
            ET.SubElement(asset, "mesh", {"name": name, "file": file})

        if sorted(name for name, _ in self.actuated) != sorted(JOINT_CLASS):
            raise ValueError("actuated joints do not match JOINT_CLASS")
        actuator = ET.SubElement(root, "actuator")
        for name, range_ in self.actuated:
            attrs = {
                "class": JOINT_CLASS[name],
                "name": name,
                "joint": name,
                "ctrlrange": range_,
            }
            ET.SubElement(actuator, "position", attrs)

        keyframe = ET.SubElement(root, "keyframe")
        for name, qpos in KEYFRAMES:
            ET.SubElement(keyframe, "key", {"name": name, "qpos": qpos, "ctrl": qpos})
        return root


def main():
    root = Builder().build()
    ET.indent(root, space="  ")
    OUT.write_text(ET.tostring(root, encoding="unicode") + "\n")
    print("WROTE", OUT)


if __name__ == "__main__":
    main()
