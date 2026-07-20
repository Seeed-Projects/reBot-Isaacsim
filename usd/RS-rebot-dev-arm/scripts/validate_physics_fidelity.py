#!/usr/bin/env python3
"""Validate reBot URDF, composed USD, and MJCF physics fidelity.

Run with an Isaac Sim/OpenUSD Python environment:

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

SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_DIR = SCRIPT_DIR.parent
REPO_ROOT = PACKAGE_DIR.parents[1]
URDF_PATH = REPO_ROOT / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"
USD_PATH = PACKAGE_DIR / "00-arm-rs_asm-v3.usda"
MJCF_PATH = REPO_ROOT / "mjcf/rebot_devarm/rebot_devarm.xml"

MASS_ATOL = 1e-7
COM_ATOL = 1e-8
INERTIA_ATOL = 5e-10
LIMIT_ATOL = 2e-4


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


def parse_urdf(path: Path):
    root = ET.parse(path).getroot()
    links = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        rotation = rpy_matrix(
            vector(origin.get("rpy") if origin is not None else None)
        )
        inertia = inertial.find("inertia")
        tensor = six_value_tensor(
            [
                inertia.get("ixx"),
                inertia.get("iyy"),
                inertia.get("izz"),
                inertia.get("ixy"),
                inertia.get("ixz"),
                inertia.get("iyz"),
            ]
        )
        links[link.get("name")] = {
            "mass": float(inertial.find("mass").get("value")),
            "com": vector(origin.get("xyz") if origin is not None else None),
            "tensor": rotation @ tensor @ rotation.T,
        }

    joints = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "effort": float(limit.get("effort")),
            "velocity": float(limit.get("velocity")),
        }
    return links, joints


def open_variant(path: Path, selection: str) -> Usd.Stage:
    root_layer = Sdf.Layer.FindOrOpen(str(path))
    session_layer = Sdf.Layer.CreateAnonymous("fidelity-validation-session.usda")
    stage = Usd.Stage.Open(root_layer, session_layer)
    stage.SetEditTarget(session_layer)
    variant_set = stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics")
    if not variant_set.SetVariantSelection(selection):
        raise RuntimeError(f"failed to select Physics={selection}")
    return stage


def usd_tensor(prim: Usd.Prim) -> np.ndarray:
    diagonal = np.asarray(prim.GetAttribute("physics:diagonalInertia").Get(), dtype=float)
    quaternion = prim.GetAttribute("physics:principalAxes").Get()
    wxyz = [quaternion.GetReal(), *quaternion.GetImaginary()]
    rotation = quaternion_matrix(wxyz)
    return rotation @ np.diag(diagonal) @ rotation.T


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


def parse_mjcf(path: Path):
    root = ET.parse(path).getroot()
    bodies = {}
    for body in root.iter("body"):
        inertial = body.find("inertial")
        if inertial is None:
            continue
        if inertial.get("fullinertia"):
            tensor = six_value_tensor(vector(inertial.get("fullinertia")))
        else:
            tensor = np.diag(vector(inertial.get("diaginertia")))
        if inertial.get("quat"):
            rotation = quaternion_matrix(vector(inertial.get("quat")))
            tensor = rotation @ tensor @ rotation.T
        elif inertial.get("euler"):
            rotation = rpy_matrix(vector(inertial.get("euler")))
            tensor = rotation @ tensor @ rotation.T
        bodies[body.get("name")] = {
            "mass": float(inertial.get("mass")),
            "com": vector(inertial.get("pos")),
            "tensor": tensor,
        }
    return bodies


def validate() -> dict:
    urdf_links, urdf_joints = parse_urdf(URDF_PATH)
    mjcf_bodies = parse_mjcf(MJCF_PATH)
    stage = open_variant(USD_PATH, "physics")
    failures = []
    metrics = {
        "max_usd_mass_error_kg": 0.0,
        "max_usd_com_error_m": 0.0,
        "max_usd_inertia_error_kgm2": 0.0,
        "max_newton_inertia_error_kgm2": 0.0,
        "max_mjcf_inertia_error_kgm2": 0.0,
    }

    usd_links = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if UsdPhysics.MassAPI(prim)
        and UsdPhysics.MassAPI(prim).GetMassAttr().HasAuthoredValue()
    }
    if set(usd_links) != set(urdf_links):
        failures.append(
            f"USD link set differs: expected {sorted(urdf_links)}, got {sorted(usd_links)}"
        )

    for name, reference in urdf_links.items():
        prim = usd_links.get(name)
        if prim is None:
            failures.append(f"{name}: missing USD link inertial")
            continue
        mass_error = abs(float(prim.GetAttribute("physics:mass").Get()) - reference["mass"])
        com_error = float(
            np.max(
                np.abs(
                    np.asarray(prim.GetAttribute("physics:centerOfMass").Get())
                    - reference["com"]
                )
            )
        )
        inertia_error = float(np.max(np.abs(usd_tensor(prim) - reference["tensor"])))
        newton_error = float(
            np.max(
                np.abs(
                    six_value_tensor(prim.GetAttribute("newton:inertia").Get())
                    - reference["tensor"]
                )
            )
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
                f"{name}: USD inertial mismatch "
                f"mass={mass_error:.3e}, com={com_error:.3e}, inertia={inertia_error:.3e}"
            )
        if newton_error > INERTIA_ATOL:
            failures.append(f"{name}: Newton full inertia mismatch {newton_error:.3e}")

        mjcf = mjcf_bodies.get(name)
        if mjcf is None:
            failures.append(f"{name}: missing MJCF body inertial")
            continue
        mjcf_error = float(np.max(np.abs(mjcf["tensor"] - reference["tensor"])))
        metrics["max_mjcf_inertia_error_kgm2"] = max(
            metrics["max_mjcf_inertia_error_kgm2"], mjcf_error
        )
        if abs(mjcf["mass"] - reference["mass"]) > MASS_ATOL:
            failures.append(f"{name}: MJCF mass mismatch")
        if np.max(np.abs(mjcf["com"] - reference["com"])) > COM_ATOL:
            failures.append(f"{name}: MJCF center-of-mass mismatch")
        if mjcf_error > INERTIA_ATOL:
            failures.append(f"{name}: MJCF full inertia mismatch {mjcf_error:.3e}")

    usd_joints = {
        prim.GetName(): prim
        for prim in stage.Traverse()
        if UsdPhysics.Joint(prim) and prim.GetName() in urdf_joints
    }
    for name, reference in urdf_joints.items():
        prim = usd_joints.get(name)
        if prim is None:
            failures.append(f"{name}: missing USD joint")
            continue
        drive_kind = "linear" if reference["type"] == "prismatic" else "angular"
        max_force = prim.GetAttribute(f"drive:{drive_kind}:physics:maxForce")
        if not max_force or not max_force.HasAuthoredValue():
            failures.append(f"{name}: drive maxForce is not authored")
        elif abs(float(max_force.Get()) - reference["effort"]) > LIMIT_ATOL:
            failures.append(
                f"{name}: drive maxForce {max_force.Get()} != URDF effort {reference['effort']}"
            )

        if "PhysxJointAPI" not in authored_schemas(prim):
            failures.append(f"{name}: PhysxJointAPI is not applied")
        physx_velocity = prim.GetAttribute("physxJoint:maxJointVelocity")
        expected_velocity = (
            math.degrees(reference["velocity"])
            if reference["type"] in {"revolute", "continuous"}
            else reference["velocity"]
        )
        if not physx_velocity or not physx_velocity.HasAuthoredValue():
            failures.append(f"{name}: PhysX maxJointVelocity is not authored")
        elif abs(float(physx_velocity.Get()) - expected_velocity) > LIMIT_ATOL:
            failures.append(
                f"{name}: PhysX maxJointVelocity {physx_velocity.Get()} "
                f"!= expected {expected_velocity}"
            )

    for selection in ("physics", "mujoco"):
        variant_stage = open_variant(USD_PATH, selection)
        scenes = [
            prim
            for prim in variant_stage.Traverse()
            if prim.GetTypeName() == "PhysicsScene"
        ]
        if len(scenes) != 1:
            failures.append(
                f"Physics={selection}: expected one composed PhysicsScene, got {len(scenes)}"
            )
            continue
        scene = scenes[0]
        gravity_direction = np.asarray(
            scene.GetAttribute("physics:gravityDirection").Get(), dtype=float
        )
        gravity_magnitude = float(scene.GetAttribute("physics:gravityMagnitude").Get())
        if not np.allclose(gravity_direction, [0, 0, -1], atol=1e-7):
            failures.append(
                f"Physics={selection}: unexpected gravity direction {gravity_direction}"
            )
        if abs(gravity_magnitude - 9.81) > 1e-6:
            failures.append(
                f"Physics={selection}: unexpected gravity magnitude {gravity_magnitude}"
            )

    return {
        "passed": not failures,
        "urdf": str(URDF_PATH.relative_to(REPO_ROOT)),
        "usd": str(USD_PATH.relative_to(REPO_ROOT)),
        "mjcf": str(MJCF_PATH.relative_to(REPO_ROOT)),
        "links_checked": len(urdf_links),
        "joints_checked": len(urdf_joints),
        "metrics": metrics,
        "failures": failures,
    }


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
