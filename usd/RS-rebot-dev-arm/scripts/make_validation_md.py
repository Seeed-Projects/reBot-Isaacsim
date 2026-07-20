"""Generate VALIDATION.md from versioned, semantically validated evidence.

Run: python3 make_validation_md.py   (stdlib only)
"""

import hashlib
import json
import math
from pathlib import Path

PKG = Path(__file__).resolve().parent.parent
EV = PKG / "evidence"
OUT = PKG / "VALIDATION.md"
OUT.unlink(missing_ok=True)

from dynamic_evidence_contract import atomic_write_text, dynamic_evidence_problems  # noqa: E402

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint_left", "joint_right"]
GAINS = {
    "joint1": (500, 60), "joint2": (1500, 96), "joint3": (1000, 76),
    "joint4": (150, 18), "joint5": (80, 10), "joint6": (50, 7),
    "joint_left": (100, 4), "joint_right": (100, 4),
}


def load(p):
    path = Path(p)
    if not path.is_file():
        raise FileNotFoundError(f"required evidence is missing: {path}")
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def err(m):
    return max(m["lower_position_error"], m["upper_position_error"])


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def validate_gain_report(report, expected_engine):
    if report.get("engine") != expected_engine:
        raise RuntimeError(f"unexpected gain-report engine: {expected_engine}")
    expected_usd_label = "usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda"
    if report.get("usd") != expected_usd_label:
        raise RuntimeError(f"unexpected gain-report USD label: {expected_engine}")
    if report.get("dt") != 0.002:
        raise RuntimeError(f"unexpected gain-report dt: {expected_engine}")
    steps = report.get("sim_steps")
    if not isinstance(steps, int) or steps <= 0:
        raise RuntimeError(f"invalid gain-report step count: {expected_engine}")
    sim_time = report.get("sim_time_s")
    if not finite_number(sim_time) or not math.isclose(
        sim_time, steps * 0.002, rel_tol=0.0, abs_tol=1e-12
    ):
        raise RuntimeError(f"inconsistent gain-report simulation time: {expected_engine}")
    if report.get("gain_tuner_ext") != "isaacsim.robot_setup.gain_tuner-3.6.1":
        raise RuntimeError(f"unexpected gain-tuner version: {expected_engine}")
    metrics = report.get("joint_metrics")
    if not isinstance(metrics, dict) or set(metrics) != set(JOINTS):
        raise RuntimeError(f"gain report must contain every joint exactly once: {expected_engine}")
    for joint in JOINTS:
        metric = metrics[joint]
        if metric.get("status") not in {"pass", "blocked", "fail"}:
            raise RuntimeError(f"invalid status for {expected_engine}/{joint}")
        for key in ("lower_position_error", "upper_position_error"):
            if not finite_number(metric.get(key)) or metric[key] < 0:
                raise RuntimeError(f"invalid {key} for {expected_engine}/{joint}")


def validate_gravity_report(report):
    arm_joints = JOINTS[:6]
    if report.get("g") != -9.81 or report.get("grid") != "5 points/joint over limits":
        raise RuntimeError("unexpected gravity-study configuration")
    gains = report.get("gains_stored_per_deg")
    if not isinstance(gains, dict) or set(gains) != set(arm_joints):
        raise RuntimeError("gravity report must contain gains for all six arm joints")
    for joint in arm_joints:
        if tuple(gains[joint]) != GAINS[joint]:
            raise RuntimeError(f"gravity-report gains changed for {joint}")
    results = report.get("results")
    expected_sets = {"old_masses_c2eba19", "new_masses_b094da6"}
    if not isinstance(results, dict) or set(results) != expected_sets:
        raise RuntimeError("gravity report must contain both identified mass sets")
    scalar_fields = (
        "worst_tau_Nm",
        "droop_rad",
        "droop_deg",
        "m_eq_kgm2",
        "f_n_hz",
        "zeta",
    )
    for mass_set in expected_sets:
        metrics = results[mass_set]
        if not isinstance(metrics, dict) or set(metrics) != set(arm_joints):
            raise RuntimeError(f"gravity report is missing joints for {mass_set}")
        for joint in arm_joints:
            metric = metrics[joint]
            if any(not finite_number(metric.get(key)) for key in scalar_fields):
                raise RuntimeError(f"gravity metric is incomplete for {mass_set}/{joint}")
            pose = metric.get("worst_pose")
            if not isinstance(pose, list) or len(pose) != 6 or not all(finite_number(value) for value in pose):
                raise RuntimeError(f"gravity pose is incomplete for {mass_set}/{joint}")


def validate_static_report(report):
    if report.get("passed") is not True or report.get("failures") != []:
        raise RuntimeError("static fidelity evidence is failing")
    if report.get("links_checked") != 10 or report.get("joints_checked") != 8:
        raise RuntimeError("static fidelity counts are incomplete")
    expected_paths = {
        "urdf": "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf",
        "usd": "usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda",
        "mjcf": "mjcf/rebot_devarm/rebot_devarm.xml",
    }
    for key, value in expected_paths.items():
        if report.get(key) != value:
            raise RuntimeError(f"unexpected static-fidelity {key} path")
    metrics = report.get("metrics")
    expected_metrics = {
        "max_usd_mass_error_kg",
        "max_usd_com_error_m",
        "max_usd_inertia_error_kgm2",
        "max_newton_inertia_error_kgm2",
        "max_mjcf_inertia_error_kgm2",
    }
    if not isinstance(metrics, dict) or set(metrics) != expected_metrics:
        raise RuntimeError("static fidelity metrics are incomplete")
    if any(not finite_number(value) or value < 0 for value in metrics.values()):
        raise RuntimeError("static fidelity metrics contain invalid values")


new_n = load(EV / "gt_pj_new_newton.json")
new_p = load(EV / "gt_pj_new_physx.json")
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
    problems = dynamic_evidence_problems(report, expected_engine)
    if report.get("validation_errors") != problems:
        raise RuntimeError(
            f"{expected_engine} stored validation errors do not match recomputed errors: {problems}"
        )
    if report.get("passed") is not (not problems):
        raise RuntimeError(f"{expected_engine} passed flag contradicts recomputed evidence")
    if problems:
        raise RuntimeError(f"failing {expected_engine} dynamic evidence: {problems}")
    validator = PKG / "scripts/validate_dynamic_physics.py"
    if report.get("validator_sha256") != sha256_file(validator):
        raise RuntimeError(f"stale {expected_engine} evidence: validator hash mismatch")
    contract = PKG / "scripts/dynamic_evidence_contract.py"
    if report.get("contract_sha256") != sha256_file(contract):
        raise RuntimeError(f"stale {expected_engine} evidence: contract hash mismatch")
    asset = PKG / "00-arm-rs_asm-v3.usda"
    if report.get("asset_package_sha256") != sha256_asset_package(asset):
        raise RuntimeError(f"stale {expected_engine} evidence: asset hash mismatch")
    if not report.get("physics_step_contract_passed"):
        raise RuntimeError(f"failed physics-step contract in {expected_engine} evidence")


validate_gain_report(new_n, "newton")
validate_gain_report(new_p, "physx")
validate_gravity_report(grav)
validate_static_report(fidelity)
validate_dynamic_evidence(dynamic_newton, "newton")
validate_dynamic_evidence(dynamic_physx, "physx")
if dynamic_newton["isaac_sim_version"] != dynamic_physx["isaac_sim_version"]:
    raise RuntimeError("Newton/PhysX evidence uses different Isaac Sim versions")

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
A("| joint | gains K/D | Newton 3.6.1 | PhysX 3.6.1 |")
A("|---|---|---|---|")


def cell(d, j):
    m = d["joint_metrics"][j]
    return f"**{m['status']}** {err(m):.1e}"


for j in JOINTS:
    k, d = GAINS[j]
    A(f"| {j} | {k}/{d} | {cell(new_n, j)} | {cell(new_p, j)} |")
A("")
A("## Gravity-compensation impact of PR#3 masses (current gains, worst in-limit pose)")
A("")
A("| joint | tau_g old [N·m] | tau_g new [N·m] | droop old [deg] | droop new [deg] | f_n old/new [Hz] | zeta old/new |")
A("|---|---|---|---|---|---|---|")
ro = grav["results"]["old_masses_c2eba19"]
rn = grav["results"]["new_masses_b094da6"]
arm_joints = JOINTS[:6]
for j in arm_joints:
    o, n = ro[j], rn[j]
    A(
        f"| {j} | {o['worst_tau_Nm']:.3f} | {n['worst_tau_Nm']:.3f} | {o['droop_deg']:.2e} | {n['droop_deg']:.2e} "
        f"| {o['f_n_hz']:.1f}/{n['f_n_hz']:.1f} | {o['zeta']:.2f}/{n['zeta']:.2f} |"
    )
torque_magnitude_changes = [
    abs(abs(rn[j]["worst_tau_Nm"]) - abs(ro[j]["worst_tau_Nm"]))
    / abs(ro[j]["worst_tau_Nm"])
    for j in arm_joints
    if ro[j]["worst_tau_Nm"] != 0
]
max_torque_magnitude_change_pct = 100.0 * max(torque_magnitude_changes)
max_droop_deg = max(abs(rn[j]["droop_deg"]) for j in arm_joints)
A("")
A(
    f"Conclusion: the mass redistribution changes worst-case gravity-torque magnitude by at most "
    f"{max_torque_magnitude_change_pct:.1f}% and modeled static droop stays at or below "
    f"{max_droop_deg:.4f} deg."
)
A("This simulation study does not identify the mass or center-of-mass parameters of the physical hardware.")
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
A(f"- Static fidelity: **PASS**, {fidelity['links_checked']} links / {fidelity['joints_checked']} joints.")
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
A("limits at every discrete physics step in the measured phases. It does **not** observe solver-internal")
A("substeps or claim torque/velocity saturation enforcement, hard-stop enforcement, or quantitative")
A("Newton/PhysX trajectory parity.")
A("Evidence generation records and checks the exact validator, shared contract, and USD-package SHA-256")
A("values.")
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
A("`scripts/validate_physics_fidelity.py`, `scripts/validate_dynamic_physics.py`,")
A("and `scripts/dynamic_evidence_contract.py`.")

atomic_write_text(OUT, "\n".join(lines) + "\n")
print("WROTE", OUT)
