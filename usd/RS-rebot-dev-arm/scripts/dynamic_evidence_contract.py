"""Pure, stdlib-only contract for reBot dynamic validation evidence."""

from datetime import datetime
import json
import math
import os
from pathlib import Path
import re

DOF_NAMES = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
    "joint6",
    "joint_left",
    "joint_right",
)
POSITION_UNITS = ("rad",) * 6 + ("m",) * 2
VELOCITY_UNITS = ("rad/s",) * 6 + ("m/s",) * 2
EXPECTED_MAX_EFFORTS = (36.0, 36.0, 36.0, 14.0, 14.0, 14.0, 500.0, 500.0)
EXPECTED_MAX_VELOCITIES = (50.0, 50.0, 50.0, 40.0, 40.0, 40.0, 10.0, 10.0)
LOWER_POSITION_LIMITS = (-2.8, -3.14, -3.14, -1.79, -1.57, -3.14, 0.0, 0.0)
UPPER_POSITION_LIMITS = (2.8, 0.0, 0.0, 1.69, 1.57, 3.14, 0.05, 0.0715)
POSITION_LIMIT_TOLERANCES = (1e-4,) * 6 + (1e-5,) * 2
TARGET_POSITIONS = (0.0, -0.7, -1.1, 0.0, 0.0, 0.0, 0.02, 0.02)
HOLD_ERROR_TOLERANCES = (0.01,) * 6 + (5e-4,) * 2
HOLD_EXCURSION_TOLERANCES = (0.01,) * 6 + (5e-4,) * 2
SETTLE_ERROR_TOLERANCES = (5e-3,) * 6 + (2.5e-4,) * 2
REST_VELOCITY_TOLERANCES = (0.01,) * 6 + (1e-3,) * 2
PHYSICS_TIMESTEP_S = 0.002
RAMP_PHYSICS_STEPS = 2000
SETTLE_MAX_PHYSICS_STEPS = 10000
SETTLE_CONSECUTIVE_STEPS_REQUIRED = 100
HOLD_MEASUREMENT_PHYSICS_STEPS = 500
PASSIVE_PROBE_PHYSICS_STEPS = 10
PASSIVE_MOTION_MIN_RAD = 1e-5
EXPECTED_ASSET = "usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda"
EXPECTED_ARTICULATION_ROOT = "/tn__00armrs_asmv3_hJ6D/Geometry/base_link"
EXPECTED_SCENES = ("/PhysicsScene",)
EXPECTED_DEVICE = "cuda:0"
_HASH_RE = re.compile(r"^[0-9a-f]{64}$")


def atomic_write_text(output_path, text):
    """Atomically install text and remove the temporary file on any failure."""

    output_path = Path(output_path)
    temporary_path = output_path.with_name(f".{output_path.name}.{os.getpid()}.tmp")
    temporary_path.unlink(missing_ok=True)
    try:
        with temporary_path.open("w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, output_path)
    finally:
        temporary_path.unlink(missing_ok=True)


def atomic_write_json(output_path, payload):
    """Serialize JSON and atomically install it at output_path."""

    atomic_write_text(output_path, json.dumps(payload, indent=2) + "\n")


def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _close(left, right, tolerance=1e-12):
    return _is_number(left) and _is_number(right) and math.isclose(
        float(left), float(right), rel_tol=0.0, abs_tol=tolerance
    )


def _vectors_close(left, right, tolerance=1e-12):
    return len(left) == len(right) and all(
        _close(actual, expected, tolerance) for actual, expected in zip(left, right)
    )


def dynamic_evidence_problems(report, expected_engine=None):
    """Return every internal-contract violation in a dynamic evidence mapping."""

    if not isinstance(report, dict):
        return ["evidence is not a JSON object"]

    problems = []

    def require(condition, message):
        if not condition:
            problems.append(message)

    def vector(key, length=8):
        value = report.get(key)
        if not isinstance(value, list) or len(value) != length:
            problems.append(f"{key} must contain {length} values")
            return None
        if not all(_is_number(item) and math.isfinite(float(item)) for item in value):
            problems.append(f"{key} contains non-finite or non-numeric values")
            return None
        return [float(item) for item in value]

    def matrix(key, rows, columns=8):
        value = report.get(key)
        if not isinstance(value, list) or len(value) != rows:
            problems.append(f"{key} must contain {rows} rows")
            return None
        converted = []
        for row in value:
            if not isinstance(row, list) or len(row) != columns:
                problems.append(f"{key} rows must contain {columns} values")
                return None
            if not all(_is_number(item) and math.isfinite(float(item)) for item in row):
                problems.append(f"{key} contains non-finite or non-numeric values")
                return None
            converted.append([float(item) for item in row])
        return converted

    requested_engine = report.get("requested_engine")
    active_engine = report.get("active_engine")
    require(requested_engine in {"newton", "physx"}, "invalid requested_engine")
    require(active_engine == requested_engine, "active_engine does not match requested_engine")
    if expected_engine is not None:
        require(requested_engine == expected_engine, "unexpected requested engine")
    require(report.get("asset") == EXPECTED_ASSET, "unexpected asset path")
    require(report.get("articulation_root") == EXPECTED_ARTICULATION_ROOT, "unexpected articulation root")
    require(report.get("device") == EXPECTED_DEVICE, "unexpected device")
    require(_close(report.get("physics_timestep_s"), PHYSICS_TIMESTEP_S), "unexpected physics timestep")
    require(tuple(report.get("dof_names", ())) == DOF_NAMES, "unexpected DOF order")
    require(tuple(report.get("dof_position_units", ())) == POSITION_UNITS, "unexpected position units")
    require(tuple(report.get("dof_velocity_units", ())) == VELOCITY_UNITS, "unexpected velocity units")
    require(tuple(report.get("physics_scenes_before_setup", ())) == EXPECTED_SCENES, "unexpected pre-setup scene set")
    require(tuple(report.get("physics_scenes_after_setup", ())) == EXPECTED_SCENES, "unexpected post-setup scene set")
    require(isinstance(report.get("isaac_sim_version"), str) and bool(report.get("isaac_sim_version")), "missing Isaac Sim version")
    try:
        datetime.fromisoformat(report.get("generated_at_utc", ""))
    except (TypeError, ValueError):
        problems.append("invalid generated_at_utc")
    for key in ("validator_sha256", "contract_sha256", "asset_package_sha256"):
        require(isinstance(report.get(key), str) and bool(_HASH_RE.fullmatch(report[key])), f"invalid {key}")

    expected_fabric_mode = {
        "newton": "explicit Newton stage update_fabric()",
        "physx": "PhysX update_fabric=True",
    }.get(requested_engine if isinstance(requested_engine, str) else "")
    require(report.get("fabric_sync_mode") == expected_fabric_mode, "unexpected Fabric synchronization mode")
    require(
        report.get("stepping_mode")
        == "playing timeline without application updates; SimulationManager.step(steps=1, update_fabric=True)",
        "unexpected stepping mode",
    )

    max_efforts = vector("max_efforts")
    expected_efforts = vector("expected_max_efforts")
    max_velocities = vector("max_velocities")
    expected_velocities = vector("expected_max_velocities")
    if expected_efforts is not None:
        require(_vectors_close(expected_efforts, EXPECTED_MAX_EFFORTS), "stored expected effort limits changed")
    if max_efforts is not None:
        require(_vectors_close(max_efforts, EXPECTED_MAX_EFFORTS, 1e-4), "runtime effort limits do not match")
    if expected_velocities is not None:
        require(_vectors_close(expected_velocities, EXPECTED_MAX_VELOCITIES), "stored expected velocity limits changed")
    if max_velocities is not None:
        require(_vectors_close(max_velocities, EXPECTED_MAX_VELOCITIES, 1e-4), "runtime velocity limits do not match")

    lower = vector("lower_position_limits")
    upper = vector("upper_position_limits")
    limit_tolerances = vector("position_limit_tolerances")
    observed_min = vector("observed_min_positions")
    observed_max = vector("observed_max_positions")
    start_positions = vector("start_positions")
    if lower is not None:
        require(_vectors_close(lower, LOWER_POSITION_LIMITS), "lower position limits changed")
    if upper is not None:
        require(_vectors_close(upper, UPPER_POSITION_LIMITS), "upper position limits changed")
    if limit_tolerances is not None:
        require(_vectors_close(limit_tolerances, POSITION_LIMIT_TOLERANCES), "position-limit tolerances changed")
    recomputed_positions_within_limits = False
    if (
        lower is not None
        and upper is not None
        and limit_tolerances is not None
        and observed_min is not None
        and observed_max is not None
    ):
        recomputed_positions_within_limits = all(
            minimum >= low - tolerance and maximum <= high + tolerance
            for minimum, maximum, low, high, tolerance in zip(
                observed_min, observed_max, lower, upper, limit_tolerances
            )
        )
    require(
        report.get("positions_within_limits") is recomputed_positions_within_limits,
        "positions_within_limits is inconsistent with observed extrema",
    )
    require(recomputed_positions_within_limits, "observed positions exceed authored limits")
    if start_positions is not None and observed_min is not None and observed_max is not None:
        require(
            all(
                minimum <= position <= maximum
                for position, minimum, maximum in zip(
                    start_positions, observed_min, observed_max
                )
            ),
            "start positions are outside observed extrema",
        )

    target = vector("target_positions")
    hold_start = vector("hold_start_positions")
    hold_end = vector("hold_end_positions")
    hold_min = vector("hold_min_positions")
    hold_max = vector("hold_max_positions")
    hold_errors = vector("hold_max_abs_error_by_dof")
    hold_excursions = vector("hold_max_excursion_by_dof")
    hold_error_tolerances = vector("hold_error_tolerances")
    hold_excursion_tolerances = vector("hold_excursion_tolerances")
    settle_error_tolerances = vector("settle_error_tolerances")
    rest_velocity_tolerances = vector("rest_velocity_tolerances")
    passive_start_velocity = vector("passive_start_velocity")
    passive_start_positions = vector("passive_start_positions")
    passive_end_positions = vector("passive_end_positions")
    settle_tail_positions = matrix(
        "settle_tail_positions", SETTLE_CONSECUTIVE_STEPS_REQUIRED
    )
    settle_tail_velocities = matrix(
        "settle_tail_velocities", SETTLE_CONSECUTIVE_STEPS_REQUIRED
    )

    if target is not None:
        require(_vectors_close(target, TARGET_POSITIONS), "target positions changed")
    if hold_error_tolerances is not None:
        require(_vectors_close(hold_error_tolerances, HOLD_ERROR_TOLERANCES), "hold-error tolerances changed")
    if hold_excursion_tolerances is not None:
        require(_vectors_close(hold_excursion_tolerances, HOLD_EXCURSION_TOLERANCES), "hold-excursion tolerances changed")
    if settle_error_tolerances is not None:
        require(_vectors_close(settle_error_tolerances, SETTLE_ERROR_TOLERANCES), "settling tolerances changed")
    if rest_velocity_tolerances is not None:
        require(_vectors_close(rest_velocity_tolerances, REST_VELOCITY_TOLERANCES), "rest-velocity tolerances changed")

    recomputed_errors = None
    recomputed_excursions = None
    if (
        target is not None
        and hold_start is not None
        and hold_end is not None
        and hold_min is not None
        and hold_max is not None
    ):
        require(
            all(low <= start <= high and low <= end <= high for low, high, start, end in zip(hold_min, hold_max, hold_start, hold_end)),
            "hold endpoints are outside stored hold extrema",
        )
        recomputed_errors = [
            max(abs(low - desired), abs(high - desired))
            for low, high, desired in zip(hold_min, hold_max, target)
        ]
        recomputed_excursions = [
            max(abs(low - start), abs(high - start))
            for low, high, start in zip(hold_min, hold_max, hold_start)
        ]
    if recomputed_errors is not None and hold_errors is not None:
        require(_vectors_close(hold_errors, recomputed_errors), "hold error vector is inconsistent with extrema")
    if recomputed_excursions is not None and hold_excursions is not None:
        require(_vectors_close(hold_excursions, recomputed_excursions), "hold excursion vector is inconsistent with extrema")
    if hold_errors is not None and hold_error_tolerances is not None:
        require(all(value < tolerance for value, tolerance in zip(hold_errors, hold_error_tolerances)), "hold error exceeds tolerance")
    if hold_excursions is not None and hold_excursion_tolerances is not None:
        require(all(value < tolerance for value, tolerance in zip(hold_excursions, hold_excursion_tolerances)), "hold excursion exceeds tolerance")

    if hold_errors is not None:
        require(_close(report.get("max_angular_hold_error_rad"), max(hold_errors[:6])), "published angular hold error is inconsistent")
        require(_close(report.get("max_linear_hold_error_m"), max(hold_errors[6:])), "published linear hold error is inconsistent")
    if hold_excursions is not None:
        require(_close(report.get("max_angular_hold_excursion_rad"), max(hold_excursions[:6])), "published angular hold excursion is inconsistent")
        require(_close(report.get("max_linear_hold_excursion_m"), max(hold_excursions[6:])), "published linear hold excursion is inconsistent")

    recomputed_settling_converged = False
    if (
        settle_tail_positions is not None
        and settle_tail_velocities is not None
        and target is not None
        and settle_error_tolerances is not None
        and rest_velocity_tolerances is not None
    ):
        recomputed_settling_converged = all(
            all(
                abs(position - desired) < position_tolerance
                and abs(velocity) < velocity_tolerance
                for position, desired, position_tolerance, velocity, velocity_tolerance in zip(
                    positions,
                    target,
                    settle_error_tolerances,
                    velocities,
                    rest_velocity_tolerances,
                )
            )
            for positions, velocities in zip(
                settle_tail_positions, settle_tail_velocities
            )
        )
    require(
        report.get("settling_converged") is recomputed_settling_converged,
        "settling_converged is inconsistent with the settling tail",
    )
    require(recomputed_settling_converged, "settling did not converge")
    if settle_tail_positions is not None and hold_start is not None:
        require(
            _vectors_close(settle_tail_positions[-1], hold_start),
            "hold start is discontinuous with the settling tail",
        )
    require(report.get("settle_max_physics_steps") == SETTLE_MAX_PHYSICS_STEPS, "unexpected settling step cap")
    require(
        report.get("settle_consecutive_steps_required") == SETTLE_CONSECUTIVE_STEPS_REQUIRED,
        "unexpected consecutive settling requirement",
    )
    settle_steps = report.get("hold_settle_physics_steps")
    require(
        isinstance(settle_steps, int)
        and SETTLE_CONSECUTIVE_STEPS_REQUIRED <= settle_steps <= SETTLE_MAX_PHYSICS_STEPS,
        "invalid settling step count",
    )
    if passive_start_velocity is not None and rest_velocity_tolerances is not None:
        require(
            all(abs(value) < tolerance for value, tolerance in zip(passive_start_velocity, rest_velocity_tolerances)),
            "passive probe did not start from rest",
        )
    if hold_end is not None and passive_start_positions is not None:
        require(
            _vectors_close(hold_end, passive_start_positions),
            "passive probe start is discontinuous with hold end",
        )
    if (
        passive_start_positions is not None
        and passive_end_positions is not None
        and observed_min is not None
        and observed_max is not None
    ):
        require(
            all(
                minimum <= start <= maximum and minimum <= end <= maximum
                for start, end, minimum, maximum in zip(
                    passive_start_positions,
                    passive_end_positions,
                    observed_min,
                    observed_max,
                )
            ),
            "passive endpoint positions are outside observed extrema",
        )

    require(report.get("ramp_physics_steps") == RAMP_PHYSICS_STEPS, "unexpected ramp step count")
    require(
        report.get("hold_measurement_physics_steps") == HOLD_MEASUREMENT_PHYSICS_STEPS,
        "unexpected hold measurement step count",
    )
    require(report.get("passive_probe_joint") == "joint3", "unexpected passive probe joint")
    passive_steps = report.get("passive_probe_physics_steps")
    require(passive_steps == PASSIVE_PROBE_PHYSICS_STEPS, "unexpected passive probe step count")

    expected_steps = None
    if isinstance(settle_steps, int):
        expected_steps = RAMP_PHYSICS_STEPS + settle_steps + HOLD_MEASUREMENT_PHYSICS_STEPS + PASSIVE_PROBE_PHYSICS_STEPS
        require(report.get("expected_physics_steps") == expected_steps, "stored expected physics-step count is inconsistent")
        require(report.get("physics_steps_advanced") == expected_steps, "physics-step count is inconsistent")
    expected_time = expected_steps * PHYSICS_TIMESTEP_S if expected_steps is not None else None
    if expected_time is not None:
        require(_close(report.get("expected_simulation_time_s"), expected_time), "stored expected simulation time is inconsistent")
        require(_close(report.get("simulation_time_advanced_s"), expected_time, PHYSICS_TIMESTEP_S * 1e-3), "simulation time is inconsistent")
    require(
        _close(
            report.get("passive_probe_simulation_time_s"),
            PASSIVE_PROBE_PHYSICS_STEPS * PHYSICS_TIMESTEP_S,
            PHYSICS_TIMESTEP_S * 1e-3,
        ),
        "passive probe simulation time is inconsistent",
    )
    recomputed_step_contract = (
        expected_steps is not None
        and report.get("physics_steps_advanced") == expected_steps
        and expected_time is not None
        and _close(report.get("simulation_time_advanced_s"), expected_time, PHYSICS_TIMESTEP_S * 1e-3)
        and passive_steps == PASSIVE_PROBE_PHYSICS_STEPS
        and _close(
            report.get("passive_probe_simulation_time_s"),
            PASSIVE_PROBE_PHYSICS_STEPS * PHYSICS_TIMESTEP_S,
            PHYSICS_TIMESTEP_S * 1e-3,
        )
    )
    require(
        report.get("physics_step_contract_passed") is recomputed_step_contract,
        "physics_step_contract_passed is inconsistent",
    )
    require(recomputed_step_contract, "physics-step contract failed")

    passive_trace = vector("passive_probe_positions_joint3", PASSIVE_PROBE_PHYSICS_STEPS + 1)
    if passive_trace is not None:
        if passive_start_positions is not None:
            require(
                _close(passive_trace[0], passive_start_positions[2]),
                "passive trace start is inconsistent with passive start positions",
            )
        if passive_end_positions is not None:
            require(
                _close(passive_trace[-1], passive_end_positions[2]),
                "passive trace end is inconsistent with passive end positions",
            )
        recomputed_motion = passive_trace[-1] - passive_trace[0]
        require(_close(report.get("passive_motion_joint3_rad"), recomputed_motion), "passive motion is inconsistent with trace")
        require(recomputed_motion > PASSIVE_MOTION_MIN_RAD, "passive motion is too small")
        if observed_min is not None and observed_max is not None:
            require(
                min(passive_trace) >= observed_min[2] and max(passive_trace) <= observed_max[2],
                "passive trace is outside observed joint3 extrema",
            )

    return problems
