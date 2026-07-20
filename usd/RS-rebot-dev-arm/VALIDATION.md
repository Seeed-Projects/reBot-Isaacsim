# RS-rebot-dev-arm re-export validation — 2026-07-17

Asset: `urdf-usd-converter 0.3.0 @554f3dc` on Seeed main `b094da6` (PR#3 mass update).
Runtime: Isaac Sim 6.0.1 aarch64 (GB10), dt=0.002, device=cpu, headless.
Gain tuner extension: `isaacsim.robot_setup.gain_tuner-3.6.1` (develop 2026-07-17, loaded via isolated --ext-folder override).
Methodology: per-joint `SnapToLimitsTest` (hold 1.0 s, tolerance 0.01), self-collision OFF,
hybrid colliders (convexHull arm / convexDecomposition gripper), validated July gains unchanged.

## Per-joint snap-to-limits (max of lower/upper hold error)

| joint | gains K/D | Newton 3.6.1 | PhysX 3.6.1 |
|---|---|---|---|
| joint1 | 500/60 | **pass** 7.2e-06 | **pass** 1.7e-04 |
| joint2 | 1500/96 | **pass** 1.8e-05 | **pass** 4.8e-04 |
| joint3 | 1000/76 | **pass** 1.2e-04 | **pass** 2.4e-06 |
| joint4 | 150/18 | **pass** 6.9e-05 | **pass** 3.4e-05 |
| joint5 | 80/10 | **pass** 3.7e-06 | **pass** 4.3e-05 |
| joint6 | 50/7 | **pass** 8.8e-06 | **pass** 2.8e-04 |
| joint_left | 100/4 | **pass** 3.8e-08 | **pass** 2.9e-07 |
| joint_right | 100/4 | **pass** 9.7e-08 | **pass** 5.2e-08 |

## Gravity-compensation impact of PR#3 masses (current gains, worst in-limit pose)

| joint | tau_g old [N·m] | tau_g new [N·m] | droop old [deg] | droop new [deg] | f_n old/new [Hz] | zeta old/new |
|---|---|---|---|---|---|---|
| joint1 | 0.000 | 0.000 | 0.00e+00 | 0.00e+00 | 53.4/53.3 | 20.14/20.08 |
| joint2 | 15.194 | 15.018 | 1.01e-02 | 1.00e-02 | 53.7/53.7 | 10.79/10.79 |
| joint3 | -6.657 | -6.710 | 6.66e-03 | 6.71e-03 | 77.1/77.0 | 18.40/18.39 |
| joint4 | 1.944 | -1.965 | 1.30e-02 | 1.31e-02 | 80.5/80.0 | 30.35/30.16 |
| joint5 | -0.778 | -0.800 | 9.72e-03 | 1.00e-02 | 114.1/113.2 | 44.82/44.45 |
| joint6 | -0.001 | -0.001 | 1.64e-05 | 1.64e-05 | 418.1/408.4 | 183.90/179.63 |

Conclusion: the mass redistribution changes worst-case gravity-torque magnitude by at most 2.9% and modeled static droop stays at or below 0.0131 deg.
This simulation study does not identify the mass or center-of-mass parameters of the physical hardware.

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
- Post-export edits re-applied by `scripts/prep_asset.py`: drives/limits and matching startup target/state,
  explicit physics scene, gripper convexDecomposition, Newton/PhysX self-collision disabled,
  solver caps nconmax=8192/njmax=32768, and Isaac robot schema.

## Physics-fidelity smoke — 2026-07-20

The static validator checks all 10 URDF/USD/MJCF inertials, all 8 drive effort limits, both
Newton and PhysX velocity attributes, startup target/state agreement, articulation schemas,
self-collision overrides, and standalone PhysicsScene composition.

- Static fidelity: **PASS**, 10 links / 8 joints.
- Newton dynamic: **PASS**, max hold error 2.006e-04 rad / 7.898e-09 m; max measured-window excursion 4.983e-05 rad / 0.000e+00 m; 2733 discrete physics steps.
- PhysX dynamic: **PASS**, max hold error 6.420e-04 rad / 1.550e-07 m; max measured-window excursion 1.466e-05 rad / 1.192e-07 m; 2782 discrete physics steps.

The dynamic smoke runs at physics dt=0.002 on `cuda:0`. During measured phases it advances no
application frames: each sample follows one `SimulationManager.step(steps=1)` call, a verified +1
physics-step counter increment, and backend Fabric synchronization (explicit for Newton). It verifies
runtime ingestion/readback of effort and velocity limits, one composed scene, convergence before
measurement, bounded error/excursion over the complete hold window, a short passive response, and
limits at every discrete physics step in the measured phases. It does **not** observe solver-internal
substeps or claim torque/velocity saturation enforcement, hard-stop enforcement, or quantitative
Newton/PhysX trajectory parity.
Evidence generation records and checks the exact validator, shared contract, and USD-package SHA-256
values.

From the repository root, with `ISAACSIM_PATH` set to the Isaac Sim release directory:

```bash
$ISAACSIM_PATH/python.sh usd/RS-rebot-dev-arm/scripts/validate_physics_fidelity.py \
  --json usd/RS-rebot-dev-arm/evidence/physics_fidelity_validation.json
$ISAACSIM_PATH/python.sh usd/RS-rebot-dev-arm/scripts/validate_dynamic_physics.py \
  usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda newton \
  usd/RS-rebot-dev-arm/evidence/physics_fidelity_dynamic_newton.json
$ISAACSIM_PATH/python.sh usd/RS-rebot-dev-arm/scripts/validate_dynamic_physics.py \
  usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda physx \
  usd/RS-rebot-dev-arm/evidence/physics_fidelity_dynamic_physx.json
```

Evidence: `evidence/gt_pj_new_newton.json`, `evidence/gt_pj_new_physx.json`, `evidence/gravity_droop.json`,
`evidence/physics_fidelity_validation.json`, `evidence/physics_fidelity_dynamic_newton.json`,
and `evidence/physics_fidelity_dynamic_physx.json`. Harnesses:
`scripts/gaintuner_perjoint_361.py`, `scripts/run_full_matrix.sh`,
`scripts/validate_physics_fidelity.py`, `scripts/validate_dynamic_physics.py`,
and `scripts/dynamic_evidence_contract.py`.
