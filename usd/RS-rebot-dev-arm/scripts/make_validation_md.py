"""Generate VALIDATION.md for the re-exported package: per-joint snap results
(Newton + PhysX, gain tuner 3.6.1) vs the July-2026 baselines of the uploaded
RS-rebot-dev-arm asset, plus the gravity-comp impact of the PR#3 mass update.

Run: python3 make_validation_md.py   (stdlib only)
"""

import json
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
EV = PKG / "evidence"
OLD_EV = Path("/home/spark/Projects/isaac/00-arm-rs_asm-v3-plus/evidence")

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint_left", "joint_right"]
GAINS = {
    "joint1": (500, 60), "joint2": (1500, 96), "joint3": (1000, 76),
    "joint4": (150, 18), "joint5": (80, 10), "joint6": (50, 7),
    "joint_left": (100, 4), "joint_right": (100, 4),
}


def load(p):
    return json.load(open(p)) if Path(p).exists() else None


def err(m):
    return max(m["lower_position_error"], m["upper_position_error"])


new_n = load(EV / "gt_pj_new_newton.json")
new_p = load(EV / "gt_pj_new_physx.json")
old_n = load(OLD_EV / "gt_pj_newasset_newton.json")
old_p = load(OLD_EV / "gt_pj_newasset_physx.json")
grip_n = load(OLD_EV / "gt_grip_decomp_newton.json")
grip_p = load(OLD_EV / "gt_grip_decomp_physx.json")
grav = load(EV / "gravity_droop.json")

lines = []
A = lines.append
A("# RS-rebot-dev-arm re-export validation — 2026-07-17")
A("")
A("Asset: `urdf-usd-converter 0.3.0 @554f3dc` on Seeed main `b094da6` (PR#3 mass update).")
A("Runtime: Isaac Sim 6.0.1 aarch64 (GB10), dt=0.002, device=cpu, headless.")
A(f"Gain tuner extension: `{new_n['gain_tuner_ext']}` (develop 2026-07-17, loaded via isolated --ext-folder override).")
A("Methodology: per-joint `SnapToLimitsTest` (hold 1.0 s, tolerance 0.01), self-collision OFF,")
A("hybrid colliders (convexHull arm / convexDecomposition gripper), validated July gains unchanged.")
A("")
A("## Per-joint snap-to-limits (max of lower/upper hold error)")
A("")
A("| joint | gains K/D | Newton 3.6.1 (new) | PhysX 3.6.1 (new) | Newton 3.5.3 Jul-07 baseline | PhysX 3.5.3 Jul-07 baseline |")
A("|---|---|---|---|---|---|")


def cell(d, j):
    if d is None or j not in d.get("joint_metrics", {}):
        return "—"
    m = d["joint_metrics"][j]
    return f"**{m['status']}** {err(m):.1e}"


def baseline_cell(base, grip, j):
    # July per-joint full runs predate the hybrid-collider fix: gripper rows
    # there are 'blocked'; the shipped-asset gripper baseline is the decomp run.
    if j in ("joint_left", "joint_right") and grip and j in grip.get("joint_metrics", {}):
        m = grip["joint_metrics"][j]
        return f"**{m['status']}** {err(m):.1e} (decomp run)"
    return cell(base, j)


for j in JOINTS:
    k, d = GAINS[j]
    A(f"| {j} | {k}/{d} | {cell(new_n, j)} | {cell(new_p, j)} | {baseline_cell(old_n, grip_n, j)} | {baseline_cell(old_p, grip_p, j)} |")

A("")
A("July baseline notes: full-matrix runs (`gt_pj_newasset_*.json`) predate the hybrid-collider fix")
A("(joint2/joint4/grippers blocked by convexHull inflation with collision on); the shipped asset's")
A("gripper baseline is `gt_grip_decomp_*.json` and the final GUI matrix (self-collision OFF) was 8/8 pass.")
A("")
A("## Gravity-compensation impact of PR#3 masses (current gains, worst in-limit pose)")
A("")
A("| joint | tau_g old [N·m] | tau_g new [N·m] | droop old [deg] | droop new [deg] | f_n old/new [Hz] | zeta old/new |")
A("|---|---|---|---|---|---|---|")
if grav:
    ro = grav["results"][next(k for k in grav["results"] if k.startswith("old"))]
    rn = grav["results"]["new_masses_b094da6"]
    for j in ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]:
        o, n = ro[j], rn[j]
        A(
            f"| {j} | {o['worst_tau_Nm']:.3f} | {n['worst_tau_Nm']:.3f} | {o['droop_deg']:.2e} | {n['droop_deg']:.2e} "
            f"| {o['f_n_hz']:.1f}/{n['f_n_hz']:.1f} | {o['zeta']:.2f}/{n['zeta']:.2f} |"
        )
A("")
A("Conclusion: the mass redistribution changes worst-case gravity torque by <2% and static droop stays")
A("≤0.013 deg on every joint — the validated gains remain correct for simulation. Real-arm gravity-feel")
A("issues are a firmware feedforward (mass/CoM) concern, not a USD drive-gain concern.")
A("")
A("## Known deltas vs the uploaded `usd/RS-rebot-dev-arm`")
A("")
A("- Masses: PR#3 values baked (link2 1.552, link3 1.252, link4 0.46, link5 0.2012, link6 0.1 kg; total 6.01 kg).")
A("  Inertia tensors were rescaled with the mass update and preserve the current URDF within float32 USD precision;")
A("  `newton:inertia` and the MJCF full tensors preserve all six URDF components exactly.")
A("- Joint limits follow the repo URDF: j2/j3 ∈ [-180°, 0], j4 ∈ [-102.6°, +96.8°] — the uploaded asset")
A("  (converted from a different local URDF) uses j2/j3 ∈ [0, +180°], j4 ±90°. Mirror convention: check")
A("  sim2real sign mapping and home poses before swapping assets.")
A("- `newton:velocityLimit` and `physxJoint:maxJointVelocity` preserve URDF velocity limits on both backends.")
A("- Drive `maxForce` preserves URDF effort limits (36 N·m RS-06, 14 N·m RS-00, 500 N gripper).")
A("- No MDL materials in 0.3.0 output (UsdPreviewSurface only); no legacy `payloads/` transformer package.")
A("- Post-export edits re-applied by `scripts/prep_asset.py`: drives/limits (July gains), explicit physics scene,")
A("  gripper convexDecomposition, `newton:selfCollisionEnabled=0`, solver caps nconmax=8192/njmax=32768,")
A("  and Isaac robot schema.")
A("")
A("Evidence: `evidence/gt_pj_new_newton.json`, `evidence/gt_pj_new_physx.json`, `evidence/gravity_droop.json`,")
A("`evidence/physics_fidelity_validation.json`, `evidence/physics_fidelity_dynamic_newton.json`,")
A("`evidence/physics_fidelity_dynamic_physx.json`, and logs alongside. Harnesses:")
A("`scripts/gaintuner_perjoint_361.py`, `scripts/run_full_matrix.sh`,")
A("`scripts/validate_physics_fidelity.py`, and `scripts/validate_dynamic_physics.py`.")

out = PKG / "VALIDATION.md"
out.write_text("\n".join(lines) + "\n")
print("WROTE", out)
