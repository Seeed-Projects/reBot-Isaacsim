"""Post-process the raw urdf-usd-converter 0.3.0 export for Isaac Sim gain tuning.

The converter authors kinematic-only joints (no drives), uniform convexHull
colliders, and no Isaac robot schema. This script turns that raw export into
the validated July-2026 RS-rebot-dev-arm setup, and it is a fixed point of the
committed tree: running it on a pristine checkout changes nothing, and running
it twice in a row is byte-idempotent.

  1. PhysicsDriveAPI on the 8 actuated joints with validated gains and effort
     limits; each drive targetPosition mirrors the joint's authored
     PhysicsJointStateAPI position, so the committed home pose survives
     regeneration without a hardcoded pose table.
  2. Newton soft-limit gains on every driven joint (Newton enforces
     UsdPhysics limits as penalty springs; the 100 N*m/rad defaults are
     decorative against gravity-scale torques).
  3. Hybrid colliders: convexDecomposition on gripper_end/left/right only,
     and the 10 collision-instance prims live in the Physics payload so
     Physics=none composes zero colliders.
  4. Articulation flags + MuJoCo-Warp solver caps on base_link, with the
     PhysX self-collision rationale authored as attribute comment metadata.
  5. Isaac robot schema (IsaacRobotAPI/IsaacLinkAPI/IsaacJointAPI + rels) in
     payloads/robot.usda so the Gain Tuner GUI robot dropdown finds the asset.
  6. Exactly one PhysicsScene, in the asset root layer (with the explicit
     PhysX-gravity rationale); dead converter scene specs are removed.
     payloads/Physics/mujoco.usda is regenerated as a documented alias of
     physics.usda.

Run with any python that has pxr (usd-core):
  .demo/bin/python prep_asset.py
"""

import math
from pathlib import Path

from pxr import Sdf, Usd, UsdPhysics

ASSET_DIR = Path(__file__).resolve().parent.parent
TOP = ASSET_DIR / "00-arm-rs_asm-v3.usda"
BASE = ASSET_DIR / "payloads" / "base.usda"
ROBOT = ASSET_DIR / "payloads" / "robot.usda"
INSTANCES = ASSET_DIR / "payloads" / "instances.usda"
PHYSICS = ASSET_DIR / "payloads" / "Physics" / "physics.usda"
MUJOCO = ASSET_DIR / "payloads" / "Physics" / "mujoco.usda"

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

# Newton enforces UsdPhysics joint limits as SOFT penalty springs and defaults
# joint_limit_ke=100 N*m/rad / kd=1, which gravity blows straight through
# (joint3 rested ~0.3 rad past its limit in the drop test). Newton's importer
# reads these per-joint attributes; for revolute joints the value is
# per-degree and is divided by pi/180 on import, so 174.533 -> force-space
# ke=1e4 N*m/rad and 1.74533 -> kd=100 (verified: overshoot +1.52 -> +0.106
# rad transient, settling +0.0007 rad). Prismatic gains are force-space
# directly (N/m, N*s/m). PhysX ignores these and keeps its hard limits.
LIMIT_GAINS = {
    "angular": (174.533, 1.74533),
    "linear": (10000.0, 100.0),
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

# rigid-body link -> name of its collision-instance child (references the
# purpose=guide collider prototype in instances.usda). These defs belong in
# physics.usda so that Physics=none composes zero colliders.
COLLISION_INSTANCES = {
    "base_link": "base_link_1",
    "link1": "link1_1",
    "link2": "link2",
    "link3": "link3",
    "link4": "link4",
    "link5": "link5",
    "link6": "link6_1",
    "gripper_end": "gripper_end",
    "gripper_left": "gripper_left",
    "gripper_right": "gripper_right",
}

SELF_COLLISION_COMMENT = (
    "PhysX needs PhysxArticulationAPI applied for the self-collision flag to"
    " take effect. Newton reads newton:selfCollisionEnabled, but under PhysX"
    " a bare physxArticulation:* attribute is ignored unless the schema is"
    " present -> self-collision defaults ON, the arm self-intersects in its"
    " home pose and diverges to NaN."
)

GRAVITY_COMMENT = (
    "Explicit gravity so non-Newton engines (PhysX) get a valid field."
    " Without these, PhysX reads an undefined/degenerate gravity (observed as"
    " magnitude = -inf with direction (0,0,0)), which sends every rigid body"
    " tunnelling through the ground on the first step. Newton derives gravity"
    " from NewtonSceneAPI defaults, but PhysX needs the standard USD Physics"
    " attributes set here. -Z, 9.81 m/s^2 matches the MuJoCo/Newton parity."
)

MUJOCO_VARIANT_COMMENT = (
    "Composes identically to Physics=physics; see the layer doc in"
    " payloads/Physics/mujoco.usda."
)

MUJOCO_DOC = """Physics=mujoco alias of Physics=physics.

This layer sublayers physics.usda and currently adds no opinions of its own.
A payload arc maps only the defaultPrim subtree, so scene-level MuJoCo
opinions (MjcSceneAPI on /PhysicsScene) cannot vary per variant; MjcSceneAPI
is applied unconditionally on the scene in the asset root layer instead. The
variant is kept as a stable selection target for MuJoCo-schema consumers:
author future mjc:* joint/actuator opinions in this layer, under the
defaultPrim subtree, where they will compose only for Physics=mujoco.
"""


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


def authored_api_schemas(prim) -> list:
    listop = prim.GetMetadata("apiSchemas")
    return list(listop.GetAddedOrExplicitItems()) if listop else []


def ensure_api_schema(prim, token: str) -> None:
    if token not in authored_api_schemas(prim):
        assert prim.AddAppliedSchema(token), f"cannot apply {token} on {prim.GetPath()}"


def ensure_attr(prim, name, type_name, value, custom=False, uniform=False):
    attr = prim.GetAttribute(name)
    if not attr:
        variability = Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying
        attr = prim.CreateAttribute(name, type_name, custom, variability)
    if attr.Get() != value:
        attr.Set(value)
    return attr


def ensure_comment(obj, text: str) -> None:
    if obj.GetMetadata("comment") != text:
        obj.SetMetadata("comment", text)


def move_collision_instances() -> None:
    """Collision-instance defs belong to the physics payload, not base.usda.

    The raw export defines them in base.usda, where they compose in every
    variant (Physics=none silently keeps 10 invisible static colliders).
    Move the specs verbatim into physics.usda and re-anchor their relative
    references (physics.usda lives one directory deeper than base.usda).
    """
    base = Sdf.Layer.FindOrOpen(str(BASE))
    physics = Sdf.Layer.FindOrOpen(str(PHYSICS))
    moved = 0
    for link, child in COLLISION_INSTANCES.items():
        path = Sdf.Path(f"{link_path(link)}/{child}")
        src = base.GetPrimAtPath(path)
        dst = physics.GetPrimAtPath(path)
        if src:
            if not dst:
                assert Sdf.CopySpec(base, path, physics, path), f"copy {path}"
                dst = physics.GetPrimAtPath(path)
                refs = [
                    Sdf.Reference("../" + r.assetPath.lstrip("./"), r.primPath)
                    for r in dst.referenceList.prependedItems
                ]
                dst.referenceList.prependedItems.clear()
                for ref in refs:
                    dst.referenceList.prependedItems.append(ref)
            del base.GetPrimAtPath(path.GetParentPath()).nameChildren[child]
            moved += 1
        assert physics.GetPrimAtPath(path), f"collision instance missing: {path}"
        for ref in physics.GetPrimAtPath(path).referenceList.prependedItems:
            assert ref.assetPath == "../instances.usda", f"bad anchor on {path}"
    base.Save()
    physics.Save()
    print(f"[base.usda -> physics.usda] collision instances moved={moved}")


def author_physics_layer() -> dict:
    stage = Usd.Stage.Open(str(PHYSICS))
    layer = stage.GetRootLayer()

    # Snapshot the authored joint states up front: the script mirrors them
    # into the drive targets and must never modify them (the home pose is
    # data, not something this script derives).
    states = {}
    for prim in stage.TraverseAll():
        name = prim.GetName()
        if name in GAINS:
            kind = GAINS[name][0]
            attr = prim.GetAttribute(f"state:{kind}:physics:position")
            if attr and attr.HasAuthoredValue():
                states[name] = attr.Get()

    # The converter emits a root-level PhysicsScene sibling of the defaultPrim.
    # A payload arc maps only the defaultPrim subtree, so it never composes;
    # the one live scene is authored in the asset root layer (author_top_layer).
    if layer.GetPrimAtPath("/PhysicsScene"):
        del layer.rootPrims["PhysicsScene"]

    base = stage.GetPrimAtPath(link_path("base_link"))
    assert base, "base_link not found"
    ensure_api_schema(base, "PhysxArticulationAPI")
    flag = ensure_attr(
        base, "physxArticulation:enabledSelfCollisions", Sdf.ValueTypeNames.Bool, False
    )
    ensure_comment(flag, SELF_COLLISION_COMMENT)
    ensure_attr(base, "newton:selfCollisionEnabled", Sdf.ValueTypeNames.Bool, False)
    ensure_attr(base, "newton:solver:nconmax", Sdf.ValueTypeNames.Int, 8192, custom=True)
    ensure_attr(base, "newton:solver:njmax", Sdf.ValueTypeNames.Int, 32768, custom=True)

    drives = 0
    for prim in stage.TraverseAll():
        name = prim.GetName()
        if name not in GAINS or prim.GetTypeName() not in (
            "PhysicsRevoluteJoint",
            "PhysicsPrismaticJoint",
        ):
            continue
        kind, stiffness, damping = GAINS[name]
        ensure_api_schema(prim, f"PhysicsDriveAPI:{kind}")
        ensure_api_schema(prim, f"PhysicsJointStateAPI:{kind}")
        ensure_api_schema(prim, "PhysxJointAPI")
        float_type = Sdf.ValueTypeNames.Float
        ensure_attr(
            prim, f"drive:{kind}:physics:type", Sdf.ValueTypeNames.Token,
            "force", uniform=True,
        )
        ensure_attr(prim, f"drive:{kind}:physics:stiffness", float_type, stiffness)
        ensure_attr(prim, f"drive:{kind}:physics:damping", float_type, damping)
        # Mirror the authored joint state into the drive target so the home
        # pose survives regeneration. Both attributes use the same units
        # (degrees for angular, meters for linear), so no conversion. A raw
        # export has no authored state; author the schema default (0) once
        # and never touch an existing value.
        if name not in states:
            ensure_attr(prim, f"state:{kind}:physics:position", float_type, 0.0)
            states[name] = 0.0
        ensure_attr(
            prim, f"drive:{kind}:physics:targetPosition", float_type, states[name]
        )
        effort, velocity = LIMITS[name]
        ensure_attr(prim, f"drive:{kind}:physics:maxForce", float_type, effort)
        ensure_attr(prim, "urdf:limit:effort", float_type, effort, custom=True)
        ensure_attr(prim, "physxJoint:maxJointVelocity", float_type, velocity)
        ensure_attr(prim, "newton:velocityLimit", float_type, velocity)
        limit_ke, limit_kd = LIMIT_GAINS[kind]
        ensure_attr(prim, f"newton:{kind}:limitStiffness", float_type, limit_ke)
        ensure_attr(prim, f"newton:{kind}:limitDamping", float_type, limit_kd)
        drives += 1
    assert drives == 8, f"expected 8 drives, authored {drives}"
    layer.Save()
    print(f"[physics.usda] drives={drives}, limit gains authored, dead scene removed")
    return states


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
        if attr.Get() != approximation:
            attr.Set(approximation)
        if approximation == "convexDecomposition":
            decomp += 1
    assert decomp == 3, f"expected 3 decomposition colliders, got {decomp}"
    stage.GetRootLayer().Save()
    print(f"[instances.usda] convexDecomposition={decomp}")


def author_robot_layer() -> None:
    """Regenerate the Isaac robot schema overs in payloads/robot.usda.

    robot.usda is the sole owner of the robot-schema opinions (it sublayers
    into base.usda); the layer content is fully derived, so rebuild it from
    scratch for deterministic bytes.
    """
    stage = Usd.Stage.Open(str(ROBOT))
    if stage.GetPrimAtPath(ROOT):
        stage.RemovePrim(ROOT)

    root = stage.OverridePrim(ROOT)
    ensure_api_schema(root, "IsaacRobotAPI")
    joints_rel = root.CreateRelationship("isaac:physics:robotJoints", custom=False)
    for joint in JOINT_ORDER:
        joints_rel.AddTarget(f"{ROOT}/Physics/{joint}")
    links_rel = root.CreateRelationship("isaac:physics:robotLinks", custom=False)
    for link in LINK_ORDER:
        links_rel.AddTarget(link_path(link))

    for link in LINK_ORDER:
        ensure_api_schema(stage.OverridePrim(link_path(link)), "IsaacLinkAPI")
    for joint in JOINT_ORDER:
        prim = stage.OverridePrim(f"{ROOT}/Physics/{joint}")
        ensure_api_schema(prim, "IsaacJointAPI")

    stage.GetRootLayer().Save()
    print("[robot.usda] robot schema regenerated (10 links, 10 joints)")


def author_mujoco_layer() -> None:
    """Regenerate payloads/Physics/mujoco.usda as a documented physics alias.

    The raw export's only unique opinion here (MjcSceneAPI on a root-level
    /PhysicsScene over) is dead: payload arcs map only the defaultPrim
    subtree. Keep the variant as a stable selection target, drop the dead
    spec plus the converter's empty scaffold overs, and say so in the doc.
    """
    layer = Sdf.Layer.FindOrOpen(str(MUJOCO))
    for name in [prim.name for prim in layer.rootPrims]:
        del layer.rootPrims[name]
    if list(layer.subLayerPaths) != ["./physics.usda"]:
        layer.subLayerPaths.clear()
        layer.subLayerPaths.append("./physics.usda")
    if layer.documentation != MUJOCO_DOC:
        layer.documentation = MUJOCO_DOC
    Sdf.CreatePrimInLayer(layer, ROOT)
    layer.Save()
    print("[mujoco.usda] regenerated as documented alias of physics.usda")


def author_top_layer() -> None:
    stage = Usd.Stage.Open(str(TOP))
    layer = stage.GetRootLayer()
    assert stage.GetEditTarget().GetLayer() == layer

    scene = UsdPhysics.Scene.Define(stage, "/PhysicsScene")
    scene.CreateGravityDirectionAttr().Set((0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)
    scene_prim = scene.GetPrim()
    ensure_api_schema(scene_prim, "NewtonSceneAPI")
    ensure_api_schema(scene_prim, "MjcSceneAPI")
    ensure_comment(scene_prim, GRAVITY_COMMENT)

    variant = layer.GetPrimAtPath(ROOT).variantSets["Physics"].variants["mujoco"]
    if variant.primSpec.comment != MUJOCO_VARIANT_COMMENT:
        variant.primSpec.comment = MUJOCO_VARIANT_COMMENT

    layer.Save()
    print("[top layer] physics scene + variant docs authored")


def _colliders(stage) -> dict:
    counts = {}
    proxies = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    for prim in Usd.PrimRange.Stage(stage, proxies):
        if "PhysicsCollisionAPI" not in authored_api_schemas(prim):
            continue
        approximation = prim.GetAttribute("physics:approximation").Get()
        counts[approximation] = counts.get(approximation, 0) + 1
    return counts


def verify(states: dict) -> None:
    stage = Usd.Stage.Open(str(TOP))
    stage.SetEditTarget(stage.GetSessionLayer())

    scenes = [p for p in stage.TraverseAll() if p.GetTypeName() == "PhysicsScene"]
    assert len(scenes) == 1, f"expected exactly 1 PhysicsScene, got {scenes}"
    gravity = scenes[0].GetAttribute("physics:gravityMagnitude").Get()
    assert math.isclose(gravity, 9.81, rel_tol=1e-6)
    assert scenes[0].GetMetadata("comment") == GRAVITY_COMMENT

    scratch = Usd.Stage.CreateInMemory()
    holder = scratch.DefinePrim("/robot")
    holder.GetReferences().AddReference(str(TOP))
    pulled = [p for p in scratch.TraverseAll() if p.GetTypeName() == "PhysicsScene"]
    assert not pulled, f"referencing the defaultPrim pulled scenes: {pulled}"

    variant_set = stage.GetPrimAtPath(ROOT).GetVariantSets().GetVariantSet("Physics")
    assert variant_set.GetVariantSelection() == "physics"
    mujoco_layer = Sdf.Layer.FindOrOpen(str(MUJOCO))
    assert list(mujoco_layer.subLayerPaths) == ["./physics.usda"]

    for selection in ("physics", "mujoco"):
        assert variant_set.SetVariantSelection(selection)
        max_forces = []
        for name, (kind, stiffness, damping) in GAINS.items():
            prim = stage.GetPrimAtPath(f"{ROOT}/Physics/{name}")
            assert prim, f"[{selection}] joint {name} not composed"

            def get(attr, prim=prim):
                return prim.GetAttribute(attr).Get()
            assert get(f"drive:{kind}:physics:stiffness") == stiffness
            assert get(f"drive:{kind}:physics:damping") == damping
            target = get(f"drive:{kind}:physics:targetPosition")
            state = get(f"state:{kind}:physics:position")
            assert target == state, f"{name}: target {target} != state {state}"
            assert state == states[name], (
                f"{name}: authored state changed {states[name]} -> {state}"
            )
            effort, velocity = LIMITS[name]
            max_forces.append(get(f"drive:{kind}:physics:maxForce"))
            assert max_forces[-1] == effort
            assert math.isclose(
                get("physxJoint:maxJointVelocity"), velocity, rel_tol=1e-6
            )
            assert math.isclose(get("newton:velocityLimit"), velocity, rel_tol=1e-6)
            limit_ke, limit_kd = LIMIT_GAINS[kind]
            assert math.isclose(
                get(f"newton:{kind}:limitStiffness"), limit_ke, rel_tol=1e-6
            ), f"{name}: limitStiffness"
            assert math.isclose(
                get(f"newton:{kind}:limitDamping"), limit_kd, rel_tol=1e-6
            ), f"{name}: limitDamping"
        assert sorted(max_forces) == [14.0] * 3 + [36.0] * 3 + [500.0] * 2
        counts = _colliders(stage)
        assert counts == {"convexHull": 7, "convexDecomposition": 3}, (
            f"[{selection}] colliders {counts}"
        )
        print(f"[verify] Physics={selection}: 8 drives, targets==states, "
              f"limit gains OK, colliders {counts}")

    assert variant_set.SetVariantSelection("none")
    assert not _colliders(stage), "Physics=none must compose zero colliders"
    assert not [
        p for p in stage.Traverse()
        if "PhysicsRigidBodyAPI" in authored_api_schemas(p)
    ], "Physics=none must compose zero rigid bodies"
    print("[verify] Physics=none: no colliders, no rigid bodies")
    assert variant_set.SetVariantSelection("physics")

    base = stage.GetPrimAtPath(link_path("base_link"))
    flag = base.GetAttribute("physxArticulation:enabledSelfCollisions")
    assert flag.Get() is False and flag.GetMetadata("comment") == SELF_COLLISION_COMMENT
    assert base.GetAttribute("newton:selfCollisionEnabled").Get() is False
    assert base.GetAttribute("newton:solver:nconmax").Get() == 8192
    assert base.GetAttribute("newton:solver:njmax").Get() == 32768

    root = stage.GetPrimAtPath(ROOT)
    # The Isaac robot schema is not registered in a bare usd-core install, so
    # GetAppliedSchemas() filters the tokens out — read authored metadata.
    assert "IsaacRobotAPI" in authored_api_schemas(root)
    links = sum(
        1 for p in stage.Traverse() if "IsaacLinkAPI" in authored_api_schemas(p)
    )
    joints = sum(
        1 for p in stage.Traverse() if "IsaacJointAPI" in authored_api_schemas(p)
    )
    assert links == 10 and joints == 10, f"IsaacLinkAPI x{links}, IsaacJointAPI x{joints}"
    joints_rel = root.GetRelationship("isaac:physics:robotJoints")
    links_rel = root.GetRelationship("isaac:physics:robotLinks")
    assert len(joints_rel.GetTargets()) == 10 and len(links_rel.GetTargets()) == 10
    print(f"[verify] robot schema: IsaacLinkAPI x{links}, IsaacJointAPI x{joints}, "
          f"rels 10/10")


if __name__ == "__main__":
    move_collision_instances()
    states = author_physics_layer()
    author_collision_layer()
    author_robot_layer()
    author_mujoco_layer()
    author_top_layer()
    verify(states)
    print("\nOK — asset prepped; re-running is a byte-level no-op")
