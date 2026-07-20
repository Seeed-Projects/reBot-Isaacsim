#!/usr/bin/env python3
"""Run a short PhysX or Newton smoke test for the reBot USD asset.

Use the Isaac Sim Python launcher and point the script at the release directory:

    ISAACSIM_PATH=/path/to/isaac-sim /path/to/isaac-sim/python.sh \
        validate_dynamic_physics.py ASSET.usda newton OUTPUT.json
    ISAACSIM_PATH=/path/to/isaac-sim /path/to/isaac-sim/python.sh \
        validate_dynamic_physics.py ASSET.usda physx OUTPUT.json
"""

from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import importlib
import json
import os
from pathlib import Path
import sys
import traceback

from dynamic_evidence_contract import (
    DOF_NAMES,
    HOLD_ERROR_TOLERANCES,
    HOLD_EXCURSION_TOLERANCES,
    HOLD_MEASUREMENT_PHYSICS_STEPS,
    LOWER_POSITION_LIMITS,
    PASSIVE_PROBE_PHYSICS_STEPS,
    PHYSICS_TIMESTEP_S,
    POSITION_LIMIT_TOLERANCES,
    RAMP_PHYSICS_STEPS,
    REST_VELOCITY_TOLERANCES,
    SETTLE_CONSECUTIVE_STEPS_REQUIRED,
    SETTLE_ERROR_TOLERANCES,
    SETTLE_MAX_PHYSICS_STEPS,
    TARGET_POSITIONS,
    UPPER_POSITION_LIMITS,
    EXPECTED_MAX_EFFORTS,
    EXPECTED_MAX_VELOCITIES,
    atomic_write_json,
    dynamic_evidence_problems,
)

if len(sys.argv) != 4:
    raise SystemExit(
        "usage: validate_dynamic_physics.py ASSET.usda ENGINE OUTPUT.json"
    )

asset_path = Path(sys.argv[1]).resolve()
engine = sys.argv[2]
output_path = Path(sys.argv[3]).resolve()
if output_path == asset_path:
    raise SystemExit("OUTPUT.json must not overwrite the asset")
output_path.parent.mkdir(parents=True, exist_ok=True)
output_path.unlink(missing_ok=True)
if engine not in {"newton", "physx"}:
    raise SystemExit("ENGINE must be newton or physx")
if not asset_path.is_file():
    raise SystemExit(f"asset does not exist: {asset_path}")

from isaacsim import SimulationApp  # noqa: E402

isaac_sim_path = os.environ.get("ISAACSIM_PATH") or os.environ.get("ISAAC_PATH")
if not isaac_sim_path:
    raise SystemExit("set ISAACSIM_PATH to the Isaac Sim release directory")
experience = (
    Path(isaac_sim_path) / "apps/isaacsim.exp.full.newton.kit"
    if engine == "newton"
    else Path(isaac_sim_path) / "apps/isaacsim.exp.base.python.kit"
)
app = SimulationApp(
    {"headless": True, "width": 640, "height": 480},
    experience=str(experience),
)
app_utils = None
# Fail closed: only a fully written, passing report changes this to zero.
exit_code = 1


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_asset_package(root_asset: Path) -> str:
    package_root = root_asset.parent
    asset_files = sorted(
        path
        for path in package_root.rglob("*")
        if path.is_file() and path.suffix.lower() in {".usd", ".usda", ".usdc"}
    )
    digest = hashlib.sha256()
    for path in asset_files:
        relative = path.relative_to(package_root).as_posix().encode("utf-8")
        digest.update(relative)
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


try:
    import numpy as np
    import omni.usd
    import isaacsim.core.experimental.utils.app as app_utils
    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.version import get_version

    timestep = PHYSICS_TIMESTEP_S
    device = "cuda:0"
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
    SimulationManager.setup_simulation(dt=timestep, device=device)
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
    newton_stage = None
    fabric_sync_mode = "PhysX update_fabric=True"
    if engine == "newton":
        newton_module = importlib.import_module("isaacsim.physics.newton")
        newton_stage = newton_module.acquire_stage()
        if newton_stage is None:
            raise RuntimeError("Newton stage is unavailable")
        fabric_sync_mode = "explicit Newton stage update_fabric()"

    names = list(articulation.dof_names)
    expected_names = list(DOF_NAMES)
    if names != expected_names:
        raise RuntimeError(f"unexpected DOF order: {names}")

    max_efforts = articulation.get_dof_max_efforts().numpy()[0].copy()
    max_velocities = articulation.get_dof_max_velocities().numpy()[0].copy()
    expected_max_efforts = np.asarray(EXPECTED_MAX_EFFORTS)
    expected_max_velocities = np.asarray(EXPECTED_MAX_VELOCITIES)
    lower_limits = np.asarray(LOWER_POSITION_LIMITS)
    upper_limits = np.asarray(UPPER_POSITION_LIMITS)
    position_tolerances = np.asarray(POSITION_LIMIT_TOLERANCES)
    hold_error_tolerances = np.asarray(HOLD_ERROR_TOLERANCES)
    hold_excursion_tolerances = np.asarray(HOLD_EXCURSION_TOLERANCES)
    settle_error_tolerances = np.asarray(SETTLE_ERROR_TOLERANCES)
    rest_velocity_tolerances = np.asarray(REST_VELOCITY_TOLERANCES)

    start = articulation.get_dof_positions().numpy()[0].copy()
    target = np.asarray(TARGET_POSITIONS)
    observed_positions = [start.copy()]

    def advance_one_physics_step(position):
        """Advance and sample exactly one discrete physics step.

        No application frame is advanced during measured phases, so the playing
        timeline cannot auto-step physics. SimulationManager.step is the public
        manual-stepping API. It updates PhysX Fabric; Newton requires an explicit
        stage Fabric sync after the manual step.
        """

        articulation.set_dof_position_targets(
            np.asarray(position, dtype=np.float32).reshape(1, -1)
        )
        before = SimulationManager.get_num_physics_steps()
        SimulationManager.step(steps=1, update_fabric=True)
        if newton_stage is not None:
            newton_stage.update_fabric()
        after = SimulationManager.get_num_physics_steps()
        if after - before != 1:
            raise RuntimeError(
                "manual step did not advance exactly one physics step: "
                f"before={before}, after={after}"
            )
        current = articulation.get_dof_positions().numpy()[0].copy()
        observed_positions.append(current)
        return current

    physics_steps_start = SimulationManager.get_num_physics_steps()
    simulation_time_start = SimulationManager.get_simulation_time()

    ramp_steps = RAMP_PHYSICS_STEPS
    for index in range(ramp_steps):
        alpha = (index + 1) / ramp_steps
        advance_one_physics_step(start * (1.0 - alpha) + target * alpha)

    settle_max_steps = SETTLE_MAX_PHYSICS_STEPS
    settle_consecutive_steps_required = SETTLE_CONSECUTIVE_STEPS_REQUIRED
    settle_consecutive_steps = 0
    hold_settle_steps = 0
    settle_tail_positions = []
    settle_tail_velocities = []
    for hold_settle_steps in range(1, settle_max_steps + 1):
        current = advance_one_physics_step(target)
        velocity = articulation.get_dof_velocities().numpy()[0].copy()
        settle_tail_positions.append(current.copy())
        settle_tail_velocities.append(velocity.copy())
        if len(settle_tail_positions) > settle_consecutive_steps_required:
            settle_tail_positions.pop(0)
            settle_tail_velocities.pop(0)
        settled = bool(
            np.all(np.abs(current - target) < settle_error_tolerances)
            and np.all(np.abs(velocity) < rest_velocity_tolerances)
        )
        settle_consecutive_steps = settle_consecutive_steps + 1 if settled else 0
        if settle_consecutive_steps >= settle_consecutive_steps_required:
            break
    settling_converged = settle_consecutive_steps >= settle_consecutive_steps_required
    hold_start = articulation.get_dof_positions().numpy()[0].copy()

    hold_measurement_steps = HOLD_MEASUREMENT_PHYSICS_STEPS
    hold_samples = [hold_start.copy()]
    for _ in range(hold_measurement_steps):
        hold_samples.append(advance_one_physics_step(target))
    hold_end = hold_samples[-1].copy()
    hold_array = np.asarray(hold_samples)
    hold_error_by_dof = np.max(np.abs(hold_array - target), axis=0)
    hold_excursion_by_dof = np.max(np.abs(hold_array - hold_start), axis=0)
    hold_min = hold_array.min(axis=0)
    hold_max = hold_array.max(axis=0)

    kp, kd = articulation.get_dof_gains()
    kp_array = kp.numpy().copy()
    kd_array = kd.numpy().copy()
    joint3 = names.index("joint3")
    kp_array[0, joint3] = 0.0
    kd_array[0, joint3] = 0.0
    articulation.set_dof_gains(kp_array, kd_array)
    passive_start = articulation.get_dof_positions().numpy()[0].copy()
    passive_start_velocity = articulation.get_dof_velocities().numpy()[0].copy()
    passive_probe_steps = PASSIVE_PROBE_PHYSICS_STEPS
    passive_probe_step_start = SimulationManager.get_num_physics_steps()
    passive_probe_time_start = SimulationManager.get_simulation_time()
    passive_trace = [passive_start.copy()]
    for _ in range(passive_probe_steps):
        passive_trace.append(advance_one_physics_step(target))
    passive_end = passive_trace[-1].copy()
    passive_probe_step_end = SimulationManager.get_num_physics_steps()
    passive_probe_time_end = SimulationManager.get_simulation_time()
    passive_motion = float(passive_end[joint3]) - float(passive_start[joint3])

    physics_steps_end = SimulationManager.get_num_physics_steps()
    simulation_time_end = SimulationManager.get_simulation_time()
    physics_steps_advanced = physics_steps_end - physics_steps_start
    expected_physics_steps = ramp_steps + hold_settle_steps + hold_measurement_steps + passive_probe_steps
    simulation_time_advanced = simulation_time_end - simulation_time_start
    expected_simulation_time = expected_physics_steps * timestep
    passive_probe_physics_steps = passive_probe_step_end - passive_probe_step_start
    passive_probe_simulation_time = passive_probe_time_end - passive_probe_time_start

    observed = np.asarray(observed_positions)
    observed_min = observed.min(axis=0)
    observed_max = observed.max(axis=0)
    positions_within_limits = bool(
        np.all(observed_min >= lower_limits - position_tolerances)
        and np.all(observed_max <= upper_limits + position_tolerances)
    )

    angular = slice(0, 6)
    linear = slice(6, 8)
    max_angular_hold_error = float(hold_error_by_dof[angular].max())
    max_linear_hold_error = float(hold_error_by_dof[linear].max())
    max_angular_hold_excursion = float(hold_excursion_by_dof[angular].max())
    max_linear_hold_excursion = float(hold_excursion_by_dof[linear].max())

    repo_root = Path(__file__).resolve().parents[3]
    try:
        asset_label = str(asset_path.relative_to(repo_root))
    except ValueError:
        asset_label = str(asset_path)

    active_engine = str(SimulationManager.get_active_physics_engine()).lower()
    time_tolerance = timestep * 1e-3
    physics_step_contract_passed = bool(
        physics_steps_advanced == expected_physics_steps
        and abs(simulation_time_advanced - expected_simulation_time) <= time_tolerance
        and passive_probe_physics_steps == passive_probe_steps
        and abs(passive_probe_simulation_time - passive_probe_steps * timestep)
        <= time_tolerance
    )
    output = {
        "asset": asset_label,
        "asset_package_sha256": sha256_asset_package(asset_path),
        "validator_sha256": sha256_file(Path(__file__).resolve()),
        "contract_sha256": sha256_file(Path(__file__).with_name("dynamic_evidence_contract.py")),
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "isaac_sim_version": get_version()[0],
        "physics_timestep_s": timestep,
        "device": device,
        "requested_engine": engine,
        "active_engine": active_engine,
        "articulation_root": articulation_root,
        "dof_names": names,
        "dof_position_units": ["rad"] * 6 + ["m"] * 2,
        "dof_velocity_units": ["rad/s"] * 6 + ["m/s"] * 2,
        "actuator_limit_validation": "runtime readback; saturation enforcement is not tested",
        "max_efforts": max_efforts.tolist(),
        "expected_max_efforts": expected_max_efforts.tolist(),
        "max_velocities": max_velocities.tolist(),
        "expected_max_velocities": expected_max_velocities.tolist(),
        "physics_scenes_before_setup": scenes_before,
        "physics_scenes_after_setup": scenes_after,
        "physics_step_validation": (
            "readback after every discrete physics step in the measured phases; "
            "solver-internal substeps and hard-stop enforcement are not tested"
        ),
        "physics_steps_advanced": physics_steps_advanced,
        "expected_physics_steps": expected_physics_steps,
        "simulation_time_advanced_s": simulation_time_advanced,
        "expected_simulation_time_s": expected_simulation_time,
        "stepping_mode": (
            "playing timeline without application updates; "
            "SimulationManager.step(steps=1, update_fabric=True)"
        ),
        "fabric_sync_mode": fabric_sync_mode,
        "physics_step_contract_passed": physics_step_contract_passed,
        "position_limit_validation": (
            "all discrete physics-step readbacks advanced by this smoke remain in range"
        ),
        "lower_position_limits": lower_limits.tolist(),
        "upper_position_limits": upper_limits.tolist(),
        "position_limit_tolerances": position_tolerances.tolist(),
        "observed_min_positions": observed_min.tolist(),
        "observed_max_positions": observed_max.tolist(),
        "positions_within_limits": positions_within_limits,
        "start_positions": start.tolist(),
        "target_positions": target.tolist(),
        "ramp_physics_steps": ramp_steps,
        "hold_settle_physics_steps": hold_settle_steps,
        "settle_max_physics_steps": settle_max_steps,
        "settle_consecutive_steps_required": settle_consecutive_steps_required,
        "settling_converged": settling_converged,
        "settle_error_tolerances": settle_error_tolerances.tolist(),
        "settle_tail_positions": [position.tolist() for position in settle_tail_positions],
        "settle_tail_velocities": [velocity.tolist() for velocity in settle_tail_velocities],
        "hold_measurement_physics_steps": hold_measurement_steps,
        "hold_start_positions": hold_start.tolist(),
        "hold_end_positions": hold_end.tolist(),
        "hold_min_positions": hold_min.tolist(),
        "hold_max_positions": hold_max.tolist(),
        "hold_max_abs_error_by_dof": hold_error_by_dof.tolist(),
        "hold_max_excursion_by_dof": hold_excursion_by_dof.tolist(),
        "hold_error_tolerances": hold_error_tolerances.tolist(),
        "hold_excursion_tolerances": hold_excursion_tolerances.tolist(),
        "max_angular_hold_error_rad": max_angular_hold_error,
        "max_linear_hold_error_m": max_linear_hold_error,
        "max_angular_hold_excursion_rad": max_angular_hold_excursion,
        "max_linear_hold_excursion_m": max_linear_hold_excursion,
        "passive_probe_joint": "joint3",
        "passive_probe_physics_steps": passive_probe_physics_steps,
        "passive_probe_simulation_time_s": passive_probe_simulation_time,
        "passive_start_positions": passive_start.tolist(),
        "passive_end_positions": passive_end.tolist(),
        "passive_start_velocity": passive_start_velocity.tolist(),
        "rest_velocity_tolerances": rest_velocity_tolerances.tolist(),
        "passive_probe_positions_joint3": [
            float(position[joint3]) for position in passive_trace
        ],
        "passive_motion_joint3_rad": passive_motion,
    }
    validation_errors = dynamic_evidence_problems(output, engine)
    output["validation_errors"] = validation_errors
    output["passed"] = not validation_errors
    atomic_write_json(output_path, output)
    print(json.dumps(output, indent=2), flush=True)
    exit_code = 0 if output["passed"] else 1
except BaseException:
    traceback.print_exc()
    raise
finally:
    try:
        if app_utils is not None:
            app_utils.stop()
    except Exception:
        pass
    app.close(exit_code=exit_code)

raise SystemExit(exit_code)
