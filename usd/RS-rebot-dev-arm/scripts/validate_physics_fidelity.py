#!/usr/bin/env python3
"""Validate reBot URDF, composed USD, and MJCF physics fidelity.

Reference values are derived from the URDF with this script's own tensor math
(no helpers shared with the exporters), the MJCF is checked both at the XML
level and — when mujoco is importable — against the compiled mjModel, and the
USD asset is checked under every physics-bearing Physics variant.

Run with a Python environment that has pxr (usd-core); installing mujoco
additionally enables the compiled-model check:

    python validate_physics_fidelity.py
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from pxr import Sdf, Usd, UsdPhysics

try:
    import mujoco
except ImportError:
    mujoco = None

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
REPO_ROOT = PACKAGE_DIR.parents[1]
URDF_PATH = REPO_ROOT / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
USD_PATH = PACKAGE_DIR / "00-arm-rs_asm-v3.usda"
MJCF_PATH = REPO_ROOT / "mjcf/rebot_devarm/rebot_devarm.xml"

MASS_ATOL = 1e-7
COM_ATOL = 1e-8
INERTIA_ATOL = 5e-10
# MuJoCo's compiler round-trips fullinertia through its iterative 3x3
# eigensolver, which reproduces the tensor to ~2e-9 kg*m^2 on this asset.
MODEL_INERTIA_ATOL = 1e-8
LIMIT_ATOL = 2e-4
FRAME_POS_ATOL = 1e-7
# URDF rpy values are truncated (1.5708 vs pi/2), so rotation-matrix entries
# only match to ~1e-5.
FRAME_ROT_ATOL = 1e-4


def repo_label(path: Path) -> str:
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def vector(text: str | None, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    return np.asarray([float(value) for value in text.split()], dtype=float)


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def quaternion_matrix(wxyz) -> np.ndarray:
    w, x, y, z = [float(value) for value in wxyz]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        return np.eye(3)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def six_value_tensor(values) -> np.ndarray:
    ixx, iyy, izz, ixy, ixz, iyz = [float(value) for value in values]
    return np.asarray(
        [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
    )


def parse_urdf(path: Path, failures: list[str]):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        components = (
            None
            if inertia is None
            else [inertia.get(key) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        )
        if mass is None or mass.get("value") is None or components is None or None in components:
            failures.append(f"{name}: URDF inertial is missing mass or inertia values")
            continue
        origin = inertial.find("origin")
        rotation = rpy_matrix(
            vector(origin.get("rpy") if origin is not None else None)
        )
        tensor = six_value_tensor(components)
        links[name] = {
            "mass": float(mass.get("value")),
            "com": vector(origin.get("xyz") if origin is not None else None),
            # Tensor about the COM, expressed in the link frame.
            "tensor": rotation @ tensor @ rotation.T,
        }

    joints = {}
    frames = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        origin = joint.find("origin")
        child = joint.find("child")
        if child is not None:
            frames[child.get("link")] = {
                "pos": vector(origin.get("xyz") if origin is not None else None),
                "rotation": rpy_matrix(
                    vector(origin.get("rpy") if origin is not None else None)
                ),
            }
        limit = joint.find("limit")
        if limit is None:
            continue
        values = [limit.get(key) for key in ("effort", "velocity", "lower", "upper")]
        if None in values:
            failures.append(f"{name}: URDF limit is missing attributes")
            continue
        joints[name] = {
            "type": joint.get("type"),
            "effort": float(values[0]),
            "velocity": float(values[1]),
            "lower": float(values[2]),
            "upper": float(values[3]),
        }
    return links, joints, frames


def parse_mjcf(path: Path, failures: list[str]):
    root = ET.parse(path).getroot()
    bodies = {}
    for body in root.iter("body"):
        name = body.get("name")
        if name is None:
            continue
        entry = {
            "pos": vector(body.get("pos")),
            "rotation": quaternion_matrix(vector(body.get("quat"), (1, 0, 0, 0))),
            "inertial": None,
        }
        inertial = body.find("inertial")
        if inertial is not None:
            oriented = inertial.get("quat") or inertial.get("euler")
            if inertial.get("fullinertia"):
                # fullinertia is authored directly in the body frame; MuJoCo
                # rejects it when combined with an inertial orientation.
                if oriented:
                    failures.append(
                        f"{name}: MJCF fullinertia combined with an inertial "
                        "orientation (invalid in MuJoCo)"
                    )
                tensor = six_value_tensor(vector(inertial.get("fullinertia")))
            elif inertial.get("diaginertia"):
                rotation = np.eye(3)
                if inertial.get("quat"):
                    rotation = quaternion_matrix(vector(inertial.get("quat")))
                elif inertial.get("euler"):
                    rotation = rpy_matrix(vector(inertial.get("euler")))
                tensor = rotation @ np.diag(vector(inertial.get("diaginertia"))) @ rotation.T
            else:
                failures.append(
                    f"{name}: MJCF inertial has neither fullinertia nor diaginertia"
                )
                tensor = None
            if inertial.get("mass") is None:
                failures.append(f"{name}: MJCF inertial is missing mass")
            entry["inertial"] = {
                "mass": float(inertial.get("mass", "nan")),
                "com": vector(inertial.get("pos")),
                "tensor": tensor,
            }
        bodies[name] = entry
    return bodies


def check_mjcf_source(urdf_links, urdf_frames, mjcf_bodies, failures, metrics):
    """Compare XML-level MJCF inertials against the URDF in the body frame.

    Each MJCF body frame must coincide with the URDF link frame (pos/quat of
    the parent joint origin); the inertials then compare in that common frame,
    so a mis-rotated inertial (gripper_end class of bug) shows up directly.
    """
    for name, reference in urdf_links.items():
        body = mjcf_bodies.get(name)
        if body is None:
            failures.append(f"{name}: missing MJCF body")
            continue
        frame = urdf_frames.get(name, {"pos": np.zeros(3), "rotation": np.eye(3)})
        if np.max(np.abs(body["pos"] - frame["pos"])) > FRAME_POS_ATOL:
            failures.append(
                f"{name}: MJCF body position differs from URDF joint origin"
            )
        if np.max(np.abs(body["rotation"] - frame["rotation"])) > FRAME_ROT_ATOL:
            failures.append(
                f"{name}: MJCF body frame is rotated against the URDF link frame; "
                "inertial comparison would not be in a common frame"
            )
        inertial = body["inertial"]
        if inertial is None:
            failures.append(f"{name}: missing MJCF body inertial")
            continue
        if abs(inertial["mass"] - reference["mass"]) > MASS_ATOL:
            failures.append(f"{name}: MJCF mass mismatch")
        if np.max(np.abs(inertial["com"] - reference["com"])) > COM_ATOL:
            failures.append(f"{name}: MJCF center-of-mass mismatch")
        if inertial["tensor"] is None:
            continue
        error = float(np.max(np.abs(inertial["tensor"] - reference["tensor"])))
        metrics["max_mjcf_source_inertia_error_kgm2"] = max(
            metrics["max_mjcf_source_inertia_error_kgm2"], error
        )
        if error > INERTIA_ATOL:
            failures.append(f"{name}: MJCF full inertia mismatch {error:.3e}")


def check_mjcf_model(urdf_links, urdf_joints, failures, metrics):
    """Compile the MJCF and compare mjModel values against the URDF.

    This is independent of build_mjcf.py: MuJoCo eigendecomposes the authored
    fullinertia into body_inertia/body_iquat, and the reference principal
    moments come from this script's own eigendecomposition of the URDF tensor.
    """
    if mujoco is None:
        print(
            "WARNING: mujoco is not importable; skipping the compiled-model "
            "MJCF check",
            file=sys.stderr,
        )
        return {"ran": False, "reason": "mujoco not importable"}
    try:
        model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    except Exception as error:  # mujoco raises plain Exceptions for XML errors
        failures.append(f"MJCF failed to compile: {error}")
        return {"ran": False, "reason": "compile error"}

    for name, reference in urdf_links.items():
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            failures.append(f"{name}: missing body in compiled mjModel")
            continue
        mass_error = abs(float(model.body_mass[body_id]) - reference["mass"])
        com_error = float(np.max(np.abs(model.body_ipos[body_id] - reference["com"])))
        rotation = quaternion_matrix(model.body_iquat[body_id])
        full_tensor = rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T
        inertia_error = float(np.max(np.abs(full_tensor - reference["tensor"])))
        principal = np.sort(np.linalg.eigvalsh(reference["tensor"]))
        principal_error = float(
            np.max(np.abs(np.sort(model.body_inertia[body_id].copy()) - principal))
        )
        metrics["max_mjcf_model_mass_error_kg"] = max(
            metrics["max_mjcf_model_mass_error_kg"], mass_error
        )
        metrics["max_mjcf_model_com_error_m"] = max(
            metrics["max_mjcf_model_com_error_m"], com_error
        )
        metrics["max_mjcf_model_inertia_error_kgm2"] = max(
            metrics["max_mjcf_model_inertia_error_kgm2"], inertia_error
        )
        metrics["max_mjcf_model_principal_inertia_error_kgm2"] = max(
            metrics["max_mjcf_model_principal_inertia_error_kgm2"], principal_error
        )
        if mass_error > MASS_ATOL:
            failures.append(f"{name}: mjModel mass mismatch {mass_error:.3e}")
        if com_error > COM_ATOL:
            failures.append(f"{name}: mjModel center-of-mass mismatch {com_error:.3e}")
        if inertia_error > MODEL_INERTIA_ATOL:
            failures.append(f"{name}: mjModel full inertia mismatch {inertia_error:.3e}")
        if principal_error > MODEL_INERTIA_ATOL:
            failures.append(
                f"{name}: mjModel principal inertia mismatch {principal_error:.3e}"
            )

    for name, reference in urdf_joints.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            failures.append(f"{name}: missing joint in compiled mjModel")
            continue
        joint_range = model.jnt_range[joint_id]
        if (
            abs(float(joint_range[0]) - reference["lower"]) > LIMIT_ATOL
            or abs(float(joint_range[1]) - reference["upper"]) > LIMIT_ATOL
        ):
            failures.append(
                f"{name}: mjModel range {joint_range.tolist()} != URDF "
                f"[{reference['lower']}, {reference['upper']}]"
            )
    return {"ran": True, "mujoco_version": mujoco.__version__}


def open_variant(path: Path, selection: str) -> Usd.Stage:
    root_layer = Sdf.Layer.FindOrOpen(str(path))
    session_layer = Sdf.Layer.CreateAnonymous("fidelity-validation-session.usda")
    stage = Usd.Stage.Open(root_layer, session_layer)
    stage.SetEditTarget(session_layer)
    variant_set = stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics")
    # SetVariantSelection succeeds even for names with no variant spec.
    if selection not in variant_set.GetVariantNames():
        raise RuntimeError(f"Physics variant '{selection}' does not exist")
    if not variant_set.SetVariantSelection(selection):
        raise RuntimeError(f"failed to select Physics={selection}")
    return stage


def attr_value(prim: Usd.Prim, name: str, failures: list[str], context: str):
    attribute = prim.GetAttribute(name)
    if not attribute or not attribute.HasAuthoredValue():
        failures.append(f"{context}: {name} is not authored")
        return None
    return attribute.Get()


def usd_tensor(prim: Usd.Prim, failures: list[str], context: str) -> np.ndarray | None:
    diagonal = attr_value(prim, "physics:diagonalInertia", failures, context)
    quaternion = attr_value(prim, "physics:principalAxes", failures, context)
    if diagonal is None or quaternion is None:
        return None
    wxyz = [quaternion.GetReal(), *quaternion.GetImaginary()]
    rotation = quaternion_matrix(wxyz)
    return rotation @ np.diag(np.asarray(diagonal, dtype=float)) @ rotation.T


def authored_schemas(prim: Usd.Prim) -> set[str]:
    """Return registered and authored schema tokens.

    Standalone OpenUSD filters unregistered PhysX/Newton schemas from
    GetAppliedSchemas(), but their authored list-op tokens remain available.
    """
    schemas = {str(schema) for schema in prim.GetAppliedSchemas()}
    list_op = prim.GetMetadata("apiSchemas")
    if list_op:
        schemas.update(str(schema) for schema in list_op.GetAddedOrExplicitItems())
    return schemas


def check_usd_variant(selection, urdf_links, urdf_joints, failures, metrics):
    try:
        stage = open_variant(USD_PATH, selection)
    except RuntimeError as error:
        failures.append(f"Physics={selection}: {error}")
        return
    context = f"Physics={selection}"

    usd_links = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if UsdPhysics.MassAPI(prim)
        and UsdPhysics.MassAPI(prim).GetMassAttr().HasAuthoredValue()
    }
    if set(usd_links) != set(urdf_links):
        failures.append(
            f"{context}: USD link set differs: expected {sorted(urdf_links)}, "
            f"got {sorted(usd_links)}"
        )

    for name, reference in urdf_links.items():
        prim = usd_links.get(name)
        if prim is None:
            failures.append(f"{context}: {name}: missing USD link inertial")
            continue
        link_context = f"{context}: {name}"
        mass = attr_value(prim, "physics:mass", failures, link_context)
        com = attr_value(prim, "physics:centerOfMass", failures, link_context)
        tensor = usd_tensor(prim, failures, link_context)
        newton_inertia = attr_value(prim, "newton:inertia", failures, link_context)
        if mass is None or com is None or tensor is None or newton_inertia is None:
            continue
        mass_error = abs(float(mass) - reference["mass"])
        com_error = float(np.max(np.abs(np.asarray(com) - reference["com"])))
        inertia_error = float(np.max(np.abs(tensor - reference["tensor"])))
        newton_error = float(
            np.max(np.abs(six_value_tensor(newton_inertia) - reference["tensor"]))
        )
        metrics["max_usd_mass_error_kg"] = max(
            metrics["max_usd_mass_error_kg"], mass_error
        )
        metrics["max_usd_com_error_m"] = max(
            metrics["max_usd_com_error_m"], com_error
        )
        metrics["max_usd_inertia_error_kgm2"] = max(
            metrics["max_usd_inertia_error_kgm2"], inertia_error
        )
        metrics["max_newton_inertia_error_kgm2"] = max(
            metrics["max_newton_inertia_error_kgm2"], newton_error
        )
        if mass_error > MASS_ATOL or com_error > COM_ATOL or inertia_error > INERTIA_ATOL:
            failures.append(
                f"{link_context}: USD inertial mismatch "
                f"mass={mass_error:.3e}, com={com_error:.3e}, inertia={inertia_error:.3e}"
            )
        if newton_error > INERTIA_ATOL:
            failures.append(
                f"{link_context}: Newton full inertia mismatch {newton_error:.3e}"
            )

    usd_joints = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if UsdPhysics.Joint(prim) and prim.GetName() in urdf_joints
    }
    for name, reference in urdf_joints.items():
        prim = usd_joints.get(name)
        if prim is None:
            failures.append(f"{context}: {name}: missing USD joint")
            continue
        joint_context = f"{context}: {name}"
        drive_kind = "linear" if reference["type"] == "prismatic" else "angular"
        max_force = attr_value(
            prim, f"drive:{drive_kind}:physics:maxForce", failures, joint_context
        )
        if max_force is not None and abs(float(max_force) - reference["effort"]) > LIMIT_ATOL:
            failures.append(
                f"{joint_context}: drive maxForce {max_force} "
                f"!= URDF effort {reference['effort']}"
            )

        if "PhysxJointAPI" not in authored_schemas(prim):
            failures.append(f"{joint_context}: PhysxJointAPI is not applied")
        expected_velocity = (
            math.degrees(reference["velocity"])
            if reference["type"] in {"revolute", "continuous"}
            else reference["velocity"]
        )
        physx_velocity = attr_value(
            prim, "physxJoint:maxJointVelocity", failures, joint_context
        )
        if (
            physx_velocity is not None
            and abs(float(physx_velocity) - expected_velocity) > LIMIT_ATOL
        ):
            failures.append(
                f"{joint_context}: PhysX maxJointVelocity {physx_velocity} "
                f"!= expected {expected_velocity}"
            )

    scenes = [
        prim for prim in stage.Traverse() if prim.GetTypeName() == "PhysicsScene"
    ]
    if len(scenes) != 1:
        failures.append(
            f"{context}: expected one composed PhysicsScene, got {len(scenes)}"
        )
        return
    scene = scenes[0]
    gravity_direction = attr_value(
        scene, "physics:gravityDirection", failures, context
    )
    gravity_magnitude = attr_value(
        scene, "physics:gravityMagnitude", failures, context
    )
    if gravity_direction is not None and not np.allclose(
        np.asarray(gravity_direction, dtype=float), [0, 0, -1], atol=1e-7
    ):
        failures.append(f"{context}: unexpected gravity direction {gravity_direction}")
    if gravity_magnitude is not None and abs(float(gravity_magnitude) - 9.81) > 1e-6:
        failures.append(f"{context}: unexpected gravity magnitude {gravity_magnitude}")


def validate() -> dict:
    failures: list[str] = []
    metrics = {
        "max_usd_mass_error_kg": 0.0,
        "max_usd_com_error_m": 0.0,
        "max_usd_inertia_error_kgm2": 0.0,
        "max_newton_inertia_error_kgm2": 0.0,
        "max_mjcf_source_inertia_error_kgm2": 0.0,
        "max_mjcf_model_mass_error_kg": 0.0,
        "max_mjcf_model_com_error_m": 0.0,
        "max_mjcf_model_inertia_error_kgm2": 0.0,
        "max_mjcf_model_principal_inertia_error_kgm2": 0.0,
    }
    result = {
        "passed": False,
        "urdf": repo_label(URDF_PATH),
        "usd": repo_label(USD_PATH),
        "mjcf": repo_label(MJCF_PATH),
        "physics_variants_checked": [],
        "mjcf_compiled_check": {"ran": False, "reason": "not reached"},
        "links_checked": 0,
        "joints_checked": 0,
        "metrics": metrics,
        "failures": failures,
    }

    missing = [path for path in (URDF_PATH, USD_PATH, MJCF_PATH) if not path.exists()]
    if missing:
        failures.extend(f"missing input file: {path}" for path in missing)
        return result

    urdf_links, urdf_joints, urdf_frames = parse_urdf(URDF_PATH, failures)
    mjcf_bodies = parse_mjcf(MJCF_PATH, failures)
    result["links_checked"] = len(urdf_links)
    result["joints_checked"] = len(urdf_joints)

    check_mjcf_source(urdf_links, urdf_frames, mjcf_bodies, failures, metrics)
    result["mjcf_compiled_check"] = check_mjcf_model(
        urdf_links, urdf_joints, failures, metrics
    )

    stage = Usd.Stage.Open(str(USD_PATH))
    default_prim = stage.GetDefaultPrim()
    if not default_prim:
        failures.append("USD asset has no default prim")
        return result
    variant_set = default_prim.GetVariantSets().GetVariantSet("Physics")
    variants = [name for name in variant_set.GetVariantNames() if name != "none"]
    if "physics" not in variants:
        failures.append(
            f"Physics variant set has no 'physics' variant (found {variants})"
        )
    result["physics_variants_checked"] = variants
    for selection in variants:
        check_usd_variant(selection, urdf_links, urdf_joints, failures, metrics)

    result["passed"] = not failures
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", type=Path, help="Write the complete result as JSON")
    args = parser.parse_args()
    result = validate()
    text = json.dumps(result, indent=2)
    print(text)
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(text + "\n")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
