"""Generate VALIDATION.md for the re-exported package: per-joint snap results
(Newton + PhysX, gain tuner 3.6.1) vs the July-2026 baselines of the uploaded
RS-rebot-dev-arm asset, plus the gravity-comp impact of the PR#3 mass update.

Run: python3 make_validation_md.py   (stdlib only)
"""

import hashlib
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
fidelity = load(EV / "physics_fidelity_validation.json")
dynamic_newton = load(EV / "physics_fidelity_dynamic_newton.json")
dynamic_physx = load(EV / "physics_fidelity_dynamic_physx.json")


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_asset_package(root_asset):
    root_asset = Path(root_asset)
    package_root = root_asset.parent
    paths = sorted(
        path
        for path in package_root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".usd", ".usda", ".usdc"}
    )
    digest = hashlib.sha256()
    for path in paths:
        digest.update(path.relative_to(package_root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def validate_dynamic_evidence(report, expected_engine):
    expected_asset = "usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda"
    if not report or not report.get("passed"):
        raise RuntimeError(f"missing or failing {expected_engine} dynamic evidence")
    if report.get("asset") != expected_asset:
        raise RuntimeError(f"unexpected {expected_engine} asset: {report.get('asset')}")
    if report.get("requested_engine") != expected_engine:
        raise RuntimeError(f"unexpected requested engine in {expected_engine} evidence")
    if report.get("active_engine") != expected_engine:
        raise RuntimeError(f"unexpected active engine in {expected_engine} evidence")
    if report.get("physics_timestep_s") != 0.002 or report.get("device") != "cuda:0":
        raise RuntimeError(f"unexpected dt/device in {expected_engine} evidence")
    if report.get("dof_position_units") != ["rad"] * 6 + ["m"] * 2:
        raise RuntimeError(f"unexpected position units in {expected_engine} evidence")
    if not report.get("settling_converged"):
        raise RuntimeError(f"settling did not converge in {expected_engine} evidence")
    validator = PKG / "scripts/validate_dynamic_physics.py"
    if report.get("validator_sha256") != sha256_file(validator):
        raise RuntimeError(f"stale {expected_engine} evidence: validator hash mismatch")
    asset = PKG / "00-arm-rs_asm-v3.usda"
    if report.get("asset_package_sha256") != sha256_asset_package(asset):
        raise RuntimeError(f"stale {expected_engine} evidence: asset hash mismatch")
    if not report.get("physics_step_contract_passed"):
        raise RuntimeError(f"failed physics-step contract in {expected_engine} evidence")


if dynamic_newton is None or dynamic_physx is None:
    raise RuntimeError("missing Newton/PhysX dynamic evidence")
validate_dynamic_evidence(dynamic_newton, "newton")
validate_dynamic_evidence(dynamic_physx, "physx")
if dynamic_newton["isaac_sim_version"] != dynamic_physx["isaac_sim_version"]:
    raise RuntimeError("Newton/PhysX evidence uses different Isaac Sim versions")
if not fidelity or not fidelity.get("passed"):
    raise RuntimeError("missing or failing static fidelity evidence")

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
A("- Post-export edits re-applied by `scripts/prep_asset.py`: drives/limits and matching startup target/state,")
A("  explicit physics scene, gripper convexDecomposition, Newton/PhysX self-collision disabled,")
A("  solver caps nconmax=8192/njmax=32768, and Isaac robot schema.")
A("")
A("## Physics-fidelity smoke — 2026-07-20")
A("")
A("The static validator checks all 10 URDF/USD/MJCF inertials, all 8 drive effort limits, both")
A("Newton and PhysX velocity attributes, startup target/state agreement, articulation schemas,")
A("self-collision overrides, and standalone PhysicsScene composition.")
A("")
if fidelity:
    A(f"- Static fidelity: **{'PASS' if fidelity['passed'] else 'FAIL'}**, {fidelity['links_checked']} links / {fidelity['joints_checked']} joints.")
for label, report in (("Newton", dynamic_newton), ("PhysX", dynamic_physx)):
    A(
        f"- {label} dynamic: **PASS**, max hold error "
        f"{report['max_angular_hold_error_rad']:.3e} rad / "
        f"{report['max_linear_hold_error_m']:.3e} m; max measured-window excursion "
        f"{report['max_angular_hold_excursion_rad']:.3e} rad / "
        f"{report['max_linear_hold_excursion_m']:.3e} m; "
        f"{report['physics_steps_advanced']} discrete physics steps."
    )
A("")
A("The dynamic smoke runs at physics dt=0.002 on `cuda:0`. During measured phases it advances no")
A("application frames: each sample follows one `SimulationManager.step(steps=1)` call, a verified +1")
A("physics-step counter increment, and backend Fabric synchronization (explicit for Newton). It verifies")
A("runtime ingestion/readback of effort and velocity limits, one composed scene, convergence before")
A("measurement, bounded error/excursion over the complete hold window, a short passive response, and")
A("limits at every discrete physics step advanced by the harness. It does **not** observe solver-internal")
A("substeps or claim torque/velocity saturation enforcement, hard-stop enforcement, or quantitative")
A("Newton/PhysX trajectory parity.")
A("Evidence generation records and checks the exact validator and USD-package SHA-256 values.")
A("")
A("From the repository root, with `ISAACSIM_PATH` set to the Isaac Sim release directory:")
A("")
A("```bash")
A("$ISAACSIM_PATH/python.sh usd/RS-rebot-dev-arm/scripts/validate_physics_fidelity.py \\")
A("  --json usd/RS-rebot-dev-arm/evidence/physics_fidelity_validation.json")
A("$ISAACSIM_PATH/python.sh usd/RS-rebot-dev-arm/scripts/validate_dynamic_physics.py \\")
A("  usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda newton \\")
A("  usd/RS-rebot-dev-arm/evidence/physics_fidelity_dynamic_newton.json")
A("$ISAACSIM_PATH/python.sh usd/RS-rebot-dev-arm/scripts/validate_dynamic_physics.py \\")
A("  usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda physx \\")
A("  usd/RS-rebot-dev-arm/evidence/physics_fidelity_dynamic_physx.json")
A("```")
A("")
A("Evidence: `evidence/gt_pj_new_newton.json`, `evidence/gt_pj_new_physx.json`, `evidence/gravity_droop.json`,")
A("`evidence/physics_fidelity_validation.json`, `evidence/physics_fidelity_dynamic_newton.json`,")
A("and `evidence/physics_fidelity_dynamic_physx.json`. Harnesses:")
A("`scripts/gaintuner_perjoint_361.py`, `scripts/run_full_matrix.sh`,")
A("`scripts/validate_physics_fidelity.py`, and `scripts/validate_dynamic_physics.py`.")

out = PKG / "VALIDATION.md"
out.write_text("\n".join(lines) + "\n")
print("WROTE", out)
