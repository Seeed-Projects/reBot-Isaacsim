#!/usr/bin/env python3
"""Run a short PhysX or Newton smoke test for the reBot USD asset.

Run from the repo root with the Isaac Sim Python launcher:

    export ISAACSIM_PATH=...   # Isaac Sim install (contains python.sh + apps/)
    "$ISAACSIM_PATH"/python.sh usd/RS-rebot-dev-arm/scripts/validate_dynamic_physics.py \\
        usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda newton \\
        usd/RS-rebot-dev-arm/evidence/physics_fidelity_dynamic_newton.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import traceback
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[2]

# Limit tolerance applied to every sampled DOF through ramp/hold/drop: Newton
# enforces UsdPhysics limits as penalty springs, so a small transient
# overshoot is expected even with the authored newton limit gains; PhysX
# clamps hard. Values past these tolerances mean limits are not enforced.
REVOLUTE_LIMIT_TOL = 0.15  # rad
PRISMATIC_LIMIT_TOL = 0.005  # m

parser = argparse.ArgumentParser(
    description="Ramp/hold/gravity-drop smoke test for the reBot USD asset."
)
parser.add_argument("asset", type=Path, help="USD asset to simulate")
parser.add_argument("engine", choices=("newton", "physx"))
parser.add_argument("output", type=Path, help="evidence JSON to write")
parser.add_argument(
    "--device", default="cuda:0", help="physics device (default: %(default)s)"
)
args = parser.parse_args()

asset_path = args.asset.resolve()
engine = args.engine
output_path = args.output

isaacsim_path = os.environ.get("ISAACSIM_PATH", "")
if not isaacsim_path or not (Path(isaacsim_path) / "apps").is_dir():
    raise SystemExit(
        "ISAACSIM_PATH is not set or does not point at an Isaac Sim install "
        "(no apps/ directory found). Set it and run this script with the "
        "bundled Python launcher, e.g.\n"
        "  export ISAACSIM_PATH=~/isaacsim\n"
        '  "$ISAACSIM_PATH"/python.sh '
        "usd/RS-rebot-dev-arm/scripts/validate_dynamic_physics.py "
        "ASSET.usda newton OUTPUT.json"
    )
if not asset_path.exists():
    raise SystemExit(f"asset not found: {asset_path}")

try:
    asset_label = asset_path.relative_to(REPO_ROOT).as_posix()
except ValueError:
    asset_label = str(asset_path)

from isaacsim import SimulationApp  # noqa: E402

experience = (
    Path(isaacsim_path) / "apps/isaacsim.exp.full.newton.kit"
    if engine == "newton"
    else Path(isaacsim_path) / "apps/isaacsim.exp.base.python.kit"
)
app = SimulationApp(
    {"headless": True, "width": 640, "height": 480},
    experience=str(experience),
)
app_utils = None
output: dict = {}


def write_output() -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2)
        stream.write("\n")
    print(json.dumps(output, indent=2), flush=True)


try:
    import numpy as np
    import omni.usd
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager

    SimulationManager.switch_physics_engine(engine)
    stage_utils.open_stage(str(asset_path))
    while stage_utils.is_stage_loading():
        app.update()
    for _ in range(10):
        app.update()

    stage = omni.usd.get_context().get_stage()

    def physics_scenes():
        return [
            str(prim.GetPath())
            for prim in stage.Traverse()
            if prim.GetTypeName() == "PhysicsScene"
        ]

    scenes_before = physics_scenes()
    SimulationManager.setup_simulation(dt=0.002, device=args.device)
    scenes_after = physics_scenes()

    articulation_root = next(
        str(prim.GetPath())
        for prim in stage.Traverse()
        if any("ArticulationRootAPI" in str(schema) for schema in prim.GetAppliedSchemas())
    )
    articulation = Articulation(articulation_root)
    app_utils.play(commit=True)
    for _ in range(10):
        app.update()

    names = list(articulation.dof_names)
    expected_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint_left",
        "joint_right",
    ]
    if names != expected_names:
        raise RuntimeError(f"unexpected DOF order: {names}")

    max_efforts = articulation.get_dof_max_efforts().numpy()[0].copy()
    max_velocities = articulation.get_dof_max_velocities().numpy()[0].copy()
    expected_max_efforts = np.asarray(
        [36.0, 36.0, 36.0, 14.0, 14.0, 14.0, 500.0, 500.0]
    )
    expected_max_velocities = np.asarray(
        [50.0, 50.0, 50.0, 40.0, 40.0, 40.0, 10.0, 10.0]
    )

    limits = articulation.get_dof_limits()
    lower_limits = limits[0].numpy()[0].copy()
    upper_limits = limits[1].numpy()[0].copy()
    # 6 revolute DOFs, then the 2 prismatic fingers (order asserted above).
    limit_tolerance = np.asarray(
        [REVOLUTE_LIMIT_TOL] * 6 + [PRISMATIC_LIMIT_TOL] * 2
    )

    def positions():
        return articulation.get_dof_positions().numpy()[0].copy()

    def sim_time() -> float:
        return float(SimulationManager.get_simulation_time())

    limit_excess: dict[str, np.ndarray] = {}

    def track_limits(phase: str, current: np.ndarray) -> None:
        excess = np.maximum(lower_limits - current, current - upper_limits)
        worst = limit_excess.setdefault(phase, np.full(len(names), -np.inf))
        np.maximum(worst, excess, out=worst)

    start = positions()
    target = np.asarray([0.0, -0.7, -1.1, 0.0, 0.0, 0.0, 0.02, 0.02])

    def step(position):
        articulation.set_dof_position_targets(
            np.asarray(position, dtype=np.float32).reshape(1, -1)
        )
        app.update()

    ramp_steps = 250
    ramp_start_time = sim_time()
    for index in range(ramp_steps):
        alpha = (index + 1) / ramp_steps
        step(start * (1.0 - alpha) + target * alpha)
        if index % 10 == 9:
            track_limits("ramp", positions())
    track_limits("ramp", positions())
    ramp_seconds = sim_time() - ramp_start_time

    hold_samples = []
    hold_start_time = sim_time()
    for index in range(500):
        step(target)
        if index % 50 == 0:
            sample = positions()
            hold_samples.append(sample)
            track_limits("hold", sample)
    hold_end = positions()
    track_limits("hold", hold_end)
    hold_seconds = sim_time() - hold_start_time
    hold_error = np.abs(hold_end - target)
    hold_drift = np.abs(hold_samples[-1] - hold_samples[0])

    kp, kd = articulation.get_dof_gains()
    kp_array = kp.numpy().copy()
    kd_array = kd.numpy().copy()
    joint3 = names.index("joint3")
    kp_array[0, joint3] = 0.0
    kd_array[0, joint3] = 0.0
    articulation.set_dof_gains(kp_array, kd_array)
    before_drop = positions()
    drop_start_time = sim_time()
    for index in range(50):
        step(target)
        if index % 5 == 4:
            track_limits("drop", positions())
    after_drop = positions()
    track_limits("drop", after_drop)
    drop_seconds = sim_time() - drop_start_time
    gravity_drop = float(after_drop[joint3] - before_drop[joint3])

    joint3_within_limits = bool(
        lower_limits[joint3] - limit_tolerance[joint3]
        <= after_drop[joint3]
        <= upper_limits[joint3] + limit_tolerance[joint3]
    )
    limits_respected = bool(
        all(
            np.all(limit_excess[phase] <= limit_tolerance)
            for phase in ("ramp", "hold", "drop")
        )
    )

    other_engine = "physx" if engine == "newton" else "newton"
    cross_engine = None
    sibling_candidates = [
        output_path.parent / f"physics_fidelity_dynamic_{other_engine}.json"
    ]
    if engine in output_path.name:
        sibling_candidates.insert(
            0, output_path.with_name(output_path.name.replace(engine, other_engine))
        )
    for sibling in sibling_candidates:
        if not sibling.is_file():
            continue
        try:
            sibling_data = json.loads(sibling.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        other_drop = sibling_data.get(
            "gravity_drop_joint3_rad", sibling_data.get("gravity_drop_joint3_0p1s")
        )
        if other_drop is None:
            continue
        cross_engine = {
            "engine": other_engine,
            "evidence": sibling.name,
            "gravity_drop_joint3_rad": other_drop,
            "abs_difference_rad": abs(gravity_drop - float(other_drop)),
        }
        break

    criteria = {
        "active_engine_matches": bool(
            str(SimulationManager.get_active_physics_engine()).lower() == engine
        ),
        "single_physics_scene": bool(
            scenes_before == ["/PhysicsScene"] and scenes_after == ["/PhysicsScene"]
        ),
        "max_efforts_match": bool(
            np.allclose(max_efforts, expected_max_efforts, rtol=0.0, atol=1e-4)
        ),
        "max_velocities_match": bool(
            np.allclose(max_velocities, expected_max_velocities, rtol=0.0, atol=1e-4)
        ),
        "hold_error_ok": bool(hold_error.max() < 0.01),
        "gravity_drop_moved": bool(abs(gravity_drop) > 1e-5),
        # A freed joint3 cannot plausibly travel farther than its pi-rad span.
        "gravity_drop_plausible": bool(abs(gravity_drop) <= math.pi),
        "joint3_final_within_limits": joint3_within_limits,
        "limits_respected_all_phases": limits_respected,
    }

    output = {
        "asset": asset_label,
        "requested_engine": engine,
        "active_engine": str(SimulationManager.get_active_physics_engine()).lower(),
        "device": args.device,
        "articulation_root": articulation_root,
        "dof_names": names,
        "max_efforts": max_efforts.tolist(),
        "expected_max_efforts": expected_max_efforts.tolist(),
        "max_velocities": max_velocities.tolist(),
        "expected_max_velocities": expected_max_velocities.tolist(),
        "dof_lower_limits": lower_limits.tolist(),
        "dof_upper_limits": upper_limits.tolist(),
        "limit_tolerance": limit_tolerance.tolist(),
        "limit_excess_max": {
            phase: values.tolist() for phase, values in limit_excess.items()
        },
        "physics_scenes_before_setup": scenes_before,
        "physics_scenes_after_setup": scenes_after,
        "start_positions": start.tolist(),
        "target_positions": target.tolist(),
        "ramp_sim_seconds": ramp_seconds,
        "hold_sim_seconds": hold_seconds,
        "hold_end_positions": hold_end.tolist(),
        "hold_abs_error": hold_error.tolist(),
        "hold_max_abs_error": float(hold_error.max()),
        "hold_drift": hold_drift.tolist(),
        "gravity_drop_sim_seconds": drop_seconds,
        "gravity_drop_joint3_rad": gravity_drop,
        "joint3_before_drop": float(before_drop[joint3]),
        "joint3_after_drop": float(after_drop[joint3]),
        "cross_engine_gravity_drop": cross_engine,
        "criteria": criteria,
        "passed": all(criteria.values()),
    }
except SystemExit:
    raise
except Exception:
    traceback.print_exc()
    output = {
        "asset": asset_label,
        "requested_engine": engine,
        "device": args.device,
        "error": traceback.format_exc(),
        "passed": False,
    }
finally:
    if output:
        write_output()
    try:
        if app_utils is not None:
            app_utils.stop()
    except Exception:
        pass
    app.close()

sys.exit(0 if output.get("passed") else 1)
