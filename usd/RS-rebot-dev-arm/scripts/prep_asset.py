"""Post-process the raw urdf-usd-converter 0.3.0 export for Isaac Sim gain tuning.

The converter authors kinematic-only joints (no drives), uniform convexHull
colliders, and no Isaac robot schema. This script re-applies the validated
July-2026 setup from the uploaded RS-rebot-dev-arm package:

  1. PhysicsDriveAPI on the 8 actuated joints with validated gains and effort limits.
  2. Hybrid colliders: convexDecomposition on gripper_end/left/right only.
  3. newton:selfCollisionEnabled=0 + MuJoCo-Warp solver caps on base_link.
  4. PhysX velocity limits and a composed PhysicsScene with explicit gravity.
  5. Isaac robot schema (IsaacRobotAPI/IsaacLinkAPI/IsaacJointAPI + rels)
     so the Gain Tuner GUI robot dropdown finds the asset.

Run with any python that has pxr (usd-core):
  .demo/bin/python prep_asset.py
"""

import math
from pathlib import Path

from pxr import Sdf, Usd, UsdPhysics

ASSET_DIR = Path(__file__).resolve().parent.parent
TOP = ASSET_DIR / "00-arm-rs_asm-v3.usda"
PHYSICS = ASSET_DIR / "payloads" / "Physics" / "physics.usda"
INSTANCES = ASSET_DIR / "payloads" / "instances.usda"

ROOT = "/tn__00armrs_asmv3_hJ6D"

# joint -> (drive kind, stiffness, damping). Validated 2026-07-07 (8/8 pass,
# errors ~1e-5 rad, Newton/PhysX parity; gt_analysis_2026-07-07/joint_drives.csv).
GAINS = {
    "joint1": ("angular", 500.0, 60.0),
    "joint2": ("angular", 1500.0, 96.0),
    "joint3": ("angular", 1000.0, 76.0),
    "joint4": ("angular", 150.0, 18.0),
    "joint5": ("angular", 80.0, 10.0),
    "joint6": ("angular", 50.0, 7.0),
    "joint_left": ("linear", 100.0, 4.0),
    "joint_right": ("linear", 100.0, 4.0),
}

# joint -> (URDF effort, velocity in USD units). Revolute velocity is deg/s;
# prismatic velocity remains m/s.
LIMITS = {
    "joint1": (36.0, 2864.789),
    "joint2": (36.0, 2864.789),
    "joint3": (36.0, 2864.789),
    "joint4": (14.0, 2291.8313),
    "joint5": (14.0, 2291.8313),
    "joint6": (14.0, 2291.8313),
    "joint_left": (500.0, 10.0),
    "joint_right": (500.0, 10.0),
}

# Concave parts that must make real contact; everything else stays convexHull
# (hybrid collider outcome validated: gripper blocked->pass on BOTH engines).
DECOMP_LINKS = ("gripper_end", "gripper_left", "gripper_right")

LINK_ORDER = [
    "base_link", "link1", "link2", "link3", "link4", "link5", "link6",
    "gripper_end", "gripper_left", "gripper_right",
]
JOINT_ORDER = [
    "root_joint", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
    "j_gripper_end", "joint_left", "joint_right",
]


def link_path(name: str) -> str:
    chain = ["Geometry"]
    for link in LINK_ORDER:
        chain.append(link)
        if link == name:
            break
    # gripper_left/right hang off gripper_end, not off each other
    if name in ("gripper_left", "gripper_right"):
        chain = ["Geometry", *LINK_ORDER[:8], name]
    return ROOT + "/" + "/".join(chain)


def author_physics_layer() -> None:
    stage = Usd.Stage.Open(str(PHYSICS))
    drives = 0
    for prim in stage.TraverseAll():
        tname = prim.GetTypeName()
        name = prim.GetName()
        if name in GAINS and tname in ("PhysicsRevoluteJoint", "PhysicsPrismaticJoint"):
            kind, k, d = GAINS[name]
            drive = UsdPhysics.DriveAPI.Apply(prim, kind)
            drive.CreateTypeAttr().Set("force")
            drive.CreateStiffnessAttr().Set(k)
            drive.CreateDampingAttr().Set(d)
            drive.CreateTargetPositionAttr().Set(0.0)
            effort, velocity = LIMITS[name]
            drive.CreateMaxForceAttr().Set(effort)
            prim.AddAppliedSchema("PhysxJointAPI")
            prim.CreateAttribute(
                "physxJoint:maxJointVelocity", Sdf.ValueTypeNames.Float
            ).Set(velocity)
            drives += 1
    assert drives == 8, f"expected 8 drives, authored {drives}"
    stage.GetRootLayer().Save()
    print(f"[physics.usda] drives={drives}")


def author_collision_layer() -> None:
    stage = Usd.Stage.Open(str(INSTANCES))
    decomp = 0
    for prim in stage.TraverseAll():
        attr = prim.GetAttribute("physics:approximation")
        if not attr or attr.Get() not in ("convexHull", "convexDecomposition"):
            continue
        approximation = (
            "convexDecomposition"
            if any(link in str(prim.GetPath()) for link in DECOMP_LINKS)
            else "convexHull"
        )
        attr.Set(approximation)
        if approximation == "convexDecomposition":
            decomp += 1
    assert decomp == 3, f"expected 3 decomposition colliders, got {decomp}"
    stage.GetRootLayer().Save()
    print(f"[instances.usda] convexDecomposition={decomp}")


def author_top_layer() -> None:
    stage = Usd.Stage.Open(str(TOP))
    assert stage.GetEditTarget().GetLayer() == stage.GetRootLayer()

    base = stage.GetPrimAtPath(link_path("base_link"))
    assert base, "base_link not found"
    base.CreateAttribute("newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)
    a = base.CreateAttribute("newton:solver:nconmax", Sdf.ValueTypeNames.Int, custom=True)
    a.Set(8192)
    a = base.CreateAttribute("newton:solver:njmax", Sdf.ValueTypeNames.Int, custom=True)
    a.Set(32768)

    root = stage.GetPrimAtPath(ROOT)
    root.AddAppliedSchema("IsaacRobotAPI")
    rj = root.CreateRelationship("isaac:physics:robotJoints")
    for j in JOINT_ORDER:
        rj.AddTarget(f"{ROOT}/Physics/{j}")
    rl = root.CreateRelationship("isaac:physics:robotLinks")
    for link in LINK_ORDER:
        rl.AddTarget(link_path(link))

    for link in LINK_ORDER:
        prim = stage.GetPrimAtPath(link_path(link))
        assert prim, f"link {link} not found at {link_path(link)}"
        prim.AddAppliedSchema("IsaacLinkAPI")
    for j in JOINT_ORDER:
        prim = stage.GetPrimAtPath(f"{ROOT}/Physics/{j}")
        assert prim, f"joint {j} not found"
        prim.AddAppliedSchema("IsaacJointAPI")

    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    scene_prim = scene.GetPrim()
    scene_prim.AddAppliedSchema("NewtonSceneAPI")
    scene_prim.AddAppliedSchema("MjcSceneAPI")

    stage.GetRootLayer().Save()
    print("[top layer] physics scene + newton attrs + robot schema authored")


def verify() -> None:
    stage = Usd.Stage.Open(str(TOP))
    drives = {}
    approx = {}
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        for kind in ("angular", "linear"):
            d = UsdPhysics.DriveAPI(prim, kind)
            if d and d.GetStiffnessAttr().HasAuthoredValue():
                drives[prim.GetName()] = (
                    kind,
                    d.GetStiffnessAttr().Get(),
                    d.GetDampingAttr().Get(),
                    d.GetTypeAttr().Get(),
                    d.GetMaxForceAttr().Get(),
                    prim.GetAttribute("physxJoint:maxJointVelocity").Get(),
                )
        a = prim.GetAttribute("physics:approximation")
        if a and a.Get():
            approx[str(prim.GetPath()).rsplit("/", 2)[-2]] = a.Get()
    print("\ncomposed drives:")
    for n in GAINS:
        print(f"  {n:12s} {drives.get(n)}")
    assert set(drives) == set(GAINS), f"drive mismatch: {set(GAINS) ^ set(drives)}"
    for name, (_, _, _, _, effort, velocity) in drives.items():
        expected_effort, expected_velocity = LIMITS[name]
        assert math.isclose(effort, expected_effort, rel_tol=0.0, abs_tol=1e-6)
        assert math.isclose(velocity, expected_velocity, rel_tol=0.0, abs_tol=2e-4)
    print("composed approximations:", approx)
    assert len(approx) == 10
    assert sum(value == "convexDecomposition" for value in approx.values()) == 3
    base = stage.GetPrimAtPath(link_path("base_link"))
    print(
        "newton attrs:",
        base.GetAttribute("newton:selfCollisionEnabled").Get(),
        base.GetAttribute("newton:solver:nconmax").Get(),
        base.GetAttribute("newton:solver:njmax").Get(),
    )
    # The Isaac robot schema is not registered in this bare usd-core install,
    # so GetAppliedSchemas() filters the tokens out — read authored metadata.
    def authored_schemas(prim):
        lo = prim.GetMetadata("apiSchemas")
        if not lo:
            return []
        return list(lo.GetAddedOrExplicitItems())

    root = stage.GetPrimAtPath(ROOT)
    print("root schemas (authored):", authored_schemas(root))
    assert "IsaacRobotAPI" in authored_schemas(root)
    n_links = sum(
        1 for p in stage.Traverse() if "IsaacLinkAPI" in authored_schemas(p)
    )
    n_joints = sum(
        1 for p in stage.Traverse() if "IsaacJointAPI" in authored_schemas(p)
    )
    print(f"IsaacLinkAPI x{n_links}, IsaacJointAPI x{n_joints}")
    assert n_links == 10 and n_joints == 10
    rj = root.GetRelationship("isaac:physics:robotJoints")
    rl = root.GetRelationship("isaac:physics:robotLinks")
    print("robotJoints targets:", len(rj.GetTargets()), "robotLinks targets:", len(rl.GetTargets()))
    assert len(rj.GetTargets()) == 10 and len(rl.GetTargets()) == 10


if __name__ == "__main__":
    author_physics_layer()
    author_collision_layer()
    author_top_layer()
    verify()
    print("\nOK — asset prepped for gain tuning")
