# RS-rebot-dev-arm re-export validation — 2026-07-17

Asset: `urdf-usd-converter 0.3.0 @554f3dc` on Seeed main `b094da6` (PR#3 mass update).
Runtime: Isaac Sim 6.0.1 aarch64 (GB10), dt=0.002, device=cpu, headless.
Gain tuner extension: `isaacsim.robot_setup.gain_tuner-3.6.1` (develop 2026-07-17, loaded via isolated --ext-folder override).
Methodology: per-joint `SnapToLimitsTest` (hold 1.0 s, tolerance 0.01), self-collision OFF,
hybrid colliders (convexHull arm / convexDecomposition gripper), validated July gains unchanged.
July-07 baseline columns read the committed snapshots in `evidence/baselines/` (provenance in its README).

## Per-joint snap-to-limits (max of lower/upper hold error)

| joint | gains K/D | Newton 3.6.1 (new) | PhysX 3.6.1 (new) | Newton 3.5.3 Jul-07 baseline | PhysX 3.5.3 Jul-07 baseline |
|---|---|---|---|---|---|
| joint1 | 500/60 | **pass** 7.2e-06 | **pass** 1.7e-04 | **pass** 5.2e-06 | **pass** 1.7e-04 |
| joint2 | 1500/96 | **pass** 1.8e-05 | **pass** 4.8e-04 | **blocked** 1.1e-01 | **blocked** 7.5e-01 |
| joint3 | 1000/76 | **pass** 1.2e-04 | **pass** 2.4e-06 | **pass** 1.2e-04 | **pass** 2.6e-06 |
| joint4 | 150/18 | **pass** 6.9e-05 | **pass** 3.4e-05 | **blocked** 5.7e-01 | **blocked** 4.2e-01 |
| joint5 | 80/10 | **pass** 3.7e-06 | **pass** 4.3e-05 | **pass** 2.4e-07 | **pass** 5.7e-05 |
| joint6 | 50/7 | **pass** 8.8e-06 | **pass** 2.8e-04 | **pass** 7.4e-06 | **pass** 4.6e-04 |
| joint_left | 100/4 | **pass** 3.8e-08 | **pass** 2.9e-07 | **pass** 4.4e-04 (decomp run) | **pass** 6.1e-04 (decomp run) |
| joint_right | 100/4 | **pass** 9.7e-08 | **pass** 5.2e-08 | **pass** 2.2e-04 (decomp run) | **pass** 3.0e-04 (decomp run) |

July baseline notes: full-matrix runs (`baselines/gt_pj_newasset_*.json`) predate the hybrid-collider fix
(joint2/joint4/grippers blocked by convexHull inflation with collision on); the shipped asset's
gripper baseline is `baselines/gt_grip_decomp_*.json` and the final GUI matrix (self-collision OFF) was 8/8 pass.

## Gravity-compensation impact of PR#3 masses (current gains, worst in-limit pose)

| joint | tau_g old [N·m] | tau_g new [N·m] | droop old [deg] | droop new [deg] | f_n old/new [Hz] | zeta old/new |
|---|---|---|---|---|---|---|
| joint1 | 0.000 | 0.000 | 0.00e+00 | 0.00e+00 | 53.4/53.3 | 20.14/20.08 |
| joint2 | 15.194 | 15.018 | 1.01e-02 | 1.00e-02 | 53.7/53.7 | 10.79/10.79 |
| joint3 | -6.657 | -6.710 | 6.66e-03 | 6.71e-03 | 77.1/77.0 | 18.40/18.39 |
| joint4 | 1.944 | -1.965 | 1.30e-02 | 1.31e-02 | 80.5/80.0 | 30.35/30.16 |
| joint5 | -0.778 | -0.800 | 9.72e-03 | 1.00e-02 | 114.1/113.2 | 44.82/44.45 |
| joint6 | -0.001 | -0.001 | 1.64e-05 | 1.64e-05 | 418.1/408.4 | 183.90/179.63 |

Conclusion: the mass redistribution changes worst-case gravity torque by <2% and static droop stays
≤0.013 deg on every joint — the validated gains remain correct for simulation. Real-arm gravity-feel
issues are a firmware feedforward (mass/CoM) concern, not a USD drive-gain concern.

## Hardware spec deviations

The asset follows the vendor URDF verbatim (faithful-to-vendor policy: deviations from the
published hardware spec are documented here, never silently corrected in the model):

| item | modeled (URDF/USD/MJCF) | hardware spec | note |
|---|---|---|---|
| J1 range | ±160.4° (±2.8 rad) | ±150° | model exceeds the spec sheet |
| J2/J3 range | 180° span ([-180°, 0]) | 220° span | model narrower than spec |
| J4 range | [-102.6°, +96.8°] | ±90° (vendor URDF and spec agree) | repo URDF is the outlier — recommend reading the firmware limit registers to settle it |
| wrist velocity (J4–J6) | 40 rad/s | RS-00 no-load 33 rad/s | limit above achievable no-load speed |
| gripper | 2 independent 500 N prismatic fingers, strokes 0.05 / 0.0715 m | 1 RS-00 driving both fingers via a rack | downstream consumers command both fingers; asymmetric strokes come from CAD |
| total mass | 6.009 kg | 6.5–6.7 kg | modeled mass ≈8% light |

## Known deltas vs the uploaded `usd/RS-rebot-dev-arm`

- Masses: PR#3 values baked (link2 1.552, link3 1.252, link4 0.46, link5 0.2012, link6 0.1 kg; total 6.01 kg).
  Inertia tensors were rescaled with the mass update and preserve the current URDF within float32 USD precision;
  `newton:inertia` and the MJCF full tensors preserve all six URDF components exactly.
- Joint limits follow the repo URDF: j2/j3 ∈ [-180°, 0], j4 ∈ [-102.6°, +96.8°] — the uploaded asset
  (converted from a different local URDF) uses j2/j3 ∈ [0, +180°], j4 ±90°. Mirror convention: check
  sim2real sign mapping and home poses before swapping assets.
- `newton:velocityLimit` and `physxJoint:maxJointVelocity` preserve URDF velocity limits on both backends.
- Drive `maxForce` preserves URDF effort limits (36 N·m RS-06, 14 N·m RS-00, 500 N gripper).
- No MDL materials in 0.3.0 output (UsdPreviewSurface only); no legacy `payloads/` transformer package.
- Post-export edits re-applied by `scripts/prep_asset.py`: drives/limits (July gains, targets from the
  authored joint-state home pose), Newton joint-limit gains (`newton:*:limitStiffness/limitDamping` — Newton
  enforces UsdPhysics limits as penalty springs and its defaults are too weak against gravity), explicit
  physics scene, gripper convexDecomposition, `newton:selfCollisionEnabled=0`, solver caps
  nconmax=8192/njmax=32768, and Isaac robot schema.

Evidence: `evidence/gt_pj_new_newton.json`, `evidence/gt_pj_new_physx.json`, `evidence/gravity_droop.json`,
`evidence/physics_fidelity_validation.json`, `evidence/physics_fidelity_dynamic_newton.json`,
`evidence/physics_fidelity_dynamic_physx.json`, and logs alongside. Harnesses:
`scripts/gaintuner_perjoint_361.py`, `scripts/run_full_matrix.sh`,
`scripts/validate_physics_fidelity.py`, and `scripts/validate_dynamic_physics.py`.
