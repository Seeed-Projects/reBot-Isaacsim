---
name: validating-urdf-usd-multi-engine
description: >
  Validate a robot asset (URDF or legacy USD) for physics parity across Newton and
  PhysX in Isaac Sim 6 — headless, scripted, evidence-first. Covers: converting URDF
  with newton-physics/urdf-usd-converter, authoring drives, porting MDL materials,
  running the REAL Gain Tuner SnapToLimitsTest per-joint (the only correct methodology),
  isolating collision artifacts (convexHull inflation) from genuine self-contact,
  dynamic tracking benchmarks (sinusoid/step/hold), and the MuJoCo-Warp nconmax
  silent-overflow pitfall. Use when: a robot behaves differently under Newton vs PhysX,
  a Gain Tuner test fails/oscillates/blocks, deciding which of several assets is "the
  good one", or packaging a validated USD for both engines. Triggers: Newton vs PhysX,
  gain tuner fail, urdf-usd-converter, asset parity, snap to limits, robot oscillates
  under Newton.
---

# Validating URDF/USD robot assets across Newton and PhysX

Proven end-to-end on the reBot DevArm (8-DOF, 6 revolute + 2 prismatic gripper),
Isaac Sim 6.0 develop (Kit 110), dt=0.002, July 2026. Outcome: legacy multi-backend
asset was the root cause of "Newton instability" (isaac-sim/IsaacSim#681); the
converter-generated asset showed full Newton↔PhysX parity with unchanged PhysX gains.

## Core principles (learned the hard way)

1. **Suspect the asset before the engine.** A SolidWorks/Blender-era USD with
   physx/physics/mujoco variant payloads can carry transforms and conventions that
   destabilize Newton. Convert the source URDF with the official
   `urdf-usd-converter` (>= v0.2.0 round-trips exact URDF inertia) and A/B it.
2. **Never retune gains to fix an unstable-looking Newton run until you have
   isolated collisions.** Soft "critically damped" gains can pass a static settle
   test while resonating in dynamic tracking (we measured amp ratio 1.56 @ 0.5 Hz)
   and under-holding gravity. Retuning treated the symptom.
3. **Per-joint sequences only.** Commanding all joints to limits simultaneously
   folds the arm into itself and produces cascading fail/NaN — that is harness
   error, not engine or asset error. The Gain Tuner GUI does one joint per sequence.
4. **blocked vs fail matters.** Gain Tuner classification: `fail` = still moving
   (gains/dynamics issue); `blocked` = stalled at zero velocity (physical
   obstruction — often convexHull inflation, sometimes real self-contact).
5. **Engine agreement is the gold metric.** If Newton and PhysX produce the same
   per-joint status pattern, the remaining issues belong to the asset/URDF, not
   to either engine.

## Step 1 — Convert the URDF

```bash
uv venv conv-venv --python 3.12 && . conv-venv/bin/activate
uv pip install --prerelease=allow urdf-usd-converter   # prerelease flag needed (tinyobjloader rc dep)
# meshes must resolve relative to the URDF location — copy the .urdf next to meshes/ if needed
urdf_usd_converter robot.urdf out_dir
```

Output: Atomic Component (`robot.usda` + `Payload/{Physics,Geometry,Materials,...}`).
The converter does NOT author drives (URDF has no gains) and only carries flat URDF
colors — both must be added (steps 2–3).

## Step 2 — Author drives (keep PhysX-proven gains)

```python
from pxr import Usd, UsdPhysics
stage = Usd.Stage.Open('out_dir/Payload/Physics.usda')
for prim in stage.Traverse():
    if prim.GetName() in gains and prim.GetTypeName() in ('PhysicsRevoluteJoint','PhysicsPrismaticJoint'):
        kind, k, d = gains[prim.GetName()]      # 'angular'|'linear', stiffness, damping
        drive = UsdPhysics.DriveAPI.Apply(prim, kind)
        drive.CreateTypeAttr().Set('force'); drive.CreateStiffnessAttr().Set(k)
        drive.CreateDampingAttr().Set(d); drive.CreateTargetPositionAttr().Set(0.0)
stage.GetRootLayer().Save()
```

## Step 3 — Port MDL materials from a legacy asset (optional but asked for)

Copy the legacy `materials.usda` into `Payload/`, reference it under the asset root,
then bind per-mesh with `UsdShade.MaterialBindingAPI` using the part→material map
extracted from the legacy `instances.usda`. Verify zero unbound meshes:

```python
unbound = [p.GetName() for p in stage.Traverse() if p.GetTypeName()=='Mesh'
           and not UsdShade.MaterialBindingAPI(p).GetDirectBinding().GetMaterial()]
```

## Step 4 — Headless harness essentials

```python
from isaacsim import SimulationApp
# Newton REQUIRES the newton kit experience; enable_extension() after boot does NOT
# create the tensor backend ("Failed to create simulation view with backend 'newton'")
exp = f"{ISAACSIM_PATH}/apps/isaacsim.exp.full.newton.kit" if engine=="newton" else ""
app = SimulationApp({"headless": True}, experience=exp)
...
SimulationManager.switch_physics_engine(engine)
stage_utils.open_stage(usd)
while stage_utils.is_stage_loading(): app.update()
for _ in range(30): app.update()   # payload composition can LAG is_stage_loading()
SimulationManager.setup_simulation(dt=0.002, device="cpu")
# find root: match "ArticulationRootAPI" in str(schema) — schemas are TfTokens, not str
# ALWAYS assert the active engine — auto_switch can override your request:
assert str(SimulationManager.get_active_physics_engine()).lower() == engine
```

Pass ABSOLUTE usd paths to scripts launched via `python.sh` from another cwd.

## Step 5 — Run the REAL Gain Tuner per-joint

```python
from isaacsim.robot_setup.gain_tuner.gains_tuner import JointMode
from isaacsim.robot_setup.gain_tuner.snap_to_limits import SnapToLimitsTest
app_utils.enable_extension("isaacsim.robot_setup.gain_tuner")
for j in range(num_dofs):                    # ONE joint per sequence
    test = SnapToLimitsTest()
    test.setup(art, [j], {j: int(JointMode.POSITION)}, {"hold_duration":1.0,"tolerance":0.01})
    gen = test.run()
    try:
        while True:
            test._step = DT; next(gen); app.update()
    except StopIteration as si:
        result = si.value                     # TestResult; .joint_metrics[j]['status']
```

Run the matrix: {asset A, asset B} × {newton, physx} × {collision on, collision off}.

## Step 6 — Isolate collision artifacts

Disable collisions by REPLACING the schema list, never emptying it (an empty
`prepend apiSchemas = []` can break prim composition and lose the articulation root):

```bash
sed -i 's/prepend apiSchemas = \["PhysicsCollisionAPI", "NewtonCollisionAPI", "PhysicsMeshCollisionAPI", "NewtonMeshCollisionAPI"\]/prepend apiSchemas = ["PhysicsMassAPI"]/' Payload/Physics.usda
```

Interpretation:
- blocked WITH collision + pass WITHOUT + both engines agree → convexHull inflation
  (hulls fill concavities: clevis links, gripper finger gaps) or true self-contact
  at limit extremes. Either way the asset/gains are fine.
- fail (oscillating) only under Newton → suspect the asset transforms; A/B against
  a fresh conversion before touching gains.

## Step 7 — Dynamic tracking (a settle test is NOT enough)

Protocol that exposed the bad retuned gains: ramp to mid-range (2 s) → hold (1 s,
drift metric) → all-joint sinusoid A=0.3 rad f=0.5 Hz (10 s; report RMS, amp ratio,
phase lag via sin/cos correlation, skip first period) → 0.5 rad step on joint1
(overshoot %, 2 % settle time). Parity target: Δrms < 0.005 rad between engines.

For the full statistical report + standard 5-figure set (tracking/error/diff/step/
Bland-Altman) and parity acceptance thresholds, see
[`references/engine-ab-statistics.md`](references/engine-ab-statistics.md).

## Newton-specific findings (2026-07-07 session)

- **Gain Tuner dropdown requires the Robot schema.** "No robot selected/found" with a
  valid articulation means the asset lacks `IsaacRobotAPI`. Apply RobotAPI on the robot
  root + LinkAPI/JointAPI on every rigid body/joint + `isaac:physics:robotLinks`/
  `robotJoints` relationships (recent URDF importers author these; older assets do not).
  Reopen the Gain Tuner window afterwards. This asset ships with the schema applied.
- **The Gain Tuner "Disable Self-Collisions" checkbox is a NO-OP on Newton.** It writes
  `physxArticulation:enabledSelfCollisions` (PhysX attr, absent on converter assets).
  Newton's parser resolves self-collision from `newton:selfCollisionEnabled`
  (NewtonArticulationRootAPI) on the articulation root. Author that attr and stop/play
  (cook-time) instead. This asset ships `newton:selfCollisionEnabled = 0`.
- **Measure contacts at HOME pose before trusting self-collision.** With self-collision
  ON this decomposition produces 6915 permanent self-contacts at rest (75% of them
  gripper_left↔gripper_right) — the convex pieces interpenetrate by construction.
  Symptom: snap tests report `blocked` on the heavy-load joints (here joint2 upper /
  joint4 lower) with errors ~0.1–0.6 rad. Probe with
  `ns.contacts.rigid_contact_shape0/1` + `model.shape_body` to map contacts to body
  pairs. Self-collision OFF is the only sane default for this collision geometry:
  8/8 joints pass with errors ~1e-5 rad and unchanged gains.
- **Limit trimming CANNOT fix collision-blocked joints.** The stall angle is
  pose-dependent (base_link↔gripper contact depends on the whole arm configuration):
  trims to 173°/168°/166° (joint2) and −55°/−40° (joint4) all still blocked. A 1-D
  joint limit cannot encode an N-D constraint — keep manufacturer limits and let
  collision (or a planner) handle it.
- **Surgical alternative — the "plus" variant** (`00-arm-rs_asm-v3-plus`): keep
  self-collision ON but author `physics:filteredPairs` for the 19 garbage pairs
  (>20 contacts at home). Home contacts drop 6915 → 34 while REAL fold collision
  stays active. Newton's parser only honors shape-level targets, so the patched
  `newton_stage._apply_usd_filtered_pairs()` (2026-07-07, not in develop stock)
  expands body-level relationships to all shape pairs (~8000) at parse time.
  On stock develop the plus asset degrades to unfiltered self-collision ON.

## Pitfalls

- **MuJoCo-Warp contact cap overflows SILENTLY.** Stock `nconmax=200`, `njmax=1200`
  (`MuJoCoSolverConfig`). convexDecomposition colliders exceed it → contacts dropped,
  garbage sim, log spam only (`Number of Newton contacts (N) exceeded MJWarp limit`).
  Raise before physics init:
  ```python
  from isaacsim.physics.newton.impl import extension as _next
  _next._newton_stage.cfg.solver_cfg.nconmax = 8192
  _next._newton_stage.cfg.solver_cfg.njmax = 32768
  ```
  Newton uses `max(user value, geometry estimate)`, so raising is always safe on
  modern builds; verify the real allocation with `solver.mjw_data.naconmax` (note
  the `na` prefix). This asset PERSISTS the caps in USD as custom attrs on the
  articulation root — `newton:solver:nconmax = 8192`, `newton:solver:njmax = 32768` —
  honored by the patched `newton_stage._apply_usd_solver_overrides()` (2026-07-07,
  not in develop stock; stock ignores the attrs harmlessly).
- **convexDecomposition is not a free upgrade.** It fixed gripper blocked on PhysX but
  broke engine agreement (spurious grazing contacts differ per engine: PhysX arm rms
  0.011→0.166 rad). Ship convexHull unless a measured need says otherwise.
  **Best-of-both: HYBRID colliders** — convexDecomposition ONLY on concave parts that
  must make real contact (gripper palm + fingers), convexHull everywhere else. Edit the
  per-prim `physics:approximation` attrs in Payload/Physics.usda, then re-validate the
  switched joints AND regression-check the joints decomposition previously broke.
  Validated outcome on the reBot gripper: blocked→pass on BOTH engines, fingers close
  to the exact limit (0.0500→0.0000 m), arm joints unaffected.
  **SDF colliders are NOT an option for parity work**: PhysxSDFMeshCollisionAPI is
  PhysX-only, Newton doesn't consume it — it silently breaks the engine A/B.
- **Never edit the asset between the two engine runs of an A/B.** A collider edit made
  mid-comparison poisoned the gripper rows (corr −0.85 vs ≥0.9999 elsewhere) because
  Newton loaded old colliders and PhysX new ones. Check asset file mtimes vs run
  timestamps before trusting a comparison; declare the caveat in stats output if violated.
- **`SimulationApp({"renderer": None})` crashes** (`.lower()` on None). Omit the key.
- **`Articulation.get_dof_types()` does not exist** — classify joints by name/limits.
- **`enable_extension("isaacsim.physics.newton")` after boot is NOT enough** for the
  Newton tensor backend; use the newton kit experience file.
- The old-vintage extscache may spew importer/extension startup errors
  (asset_importer, libxml2) — harmless for physics work, filter them out.
- Legacy assets may flip joint conventions vs URDF (e.g. limits [-180,0] vs [0,180])
  — compare per-joint before assuming metric differences are engine differences.
- **Background batch scripts: never gate on `pgrep -f <pattern>` where the pattern
  matches the gate's own command line.** A `while pgrep -f track_bench; do sleep; done`
  wait-loop inside a script whose own cmdline contains "track_bench" deadlocks forever
  (self-match). Two queued batches sat blocked ~45 min this way. Use PID files, or
  simply run batches strictly sequentially in one script with no gates.
- **GUI Script Editor: create `Articulation` AFTER timeline.play()**, else the view has
  0 DOFs and `set_dof_position_targets` fails with
  `could not broadcast ... (1,0) and requested shape (1,N)`. Recreate the view after
  every Stop→Play cycle; assert `art.num_dofs == N` before commanding.

## Recording evidence videos

To capture the robot moving (e.g. gripper close/open) as a real-frames MP4, see
[`references/headless-recording.md`](references/headless-recording.md) — camera aiming
from stage data, RTX warmup, and the blank-gray-frames pitfall (camera inside geometry;
always luminance-check sample frames before accepting a recording).

## Verification checklist for the final package

From a fresh unzip, with pxr: articulation root present; expected collider count;
all meshes material-bound; all drives present with intended gains (read back
stiffness/damping); include VALIDATION.md (methodology + results table) and
evidence/*.json from the actual runs.
