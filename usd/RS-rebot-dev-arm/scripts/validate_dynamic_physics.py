#!/usr/bin/env python3
"""Run a short PhysX or Newton smoke test for the reBot USD asset.

Use the Isaac Sim Python launcher, for example:

    python.sh validate_dynamic_physics.py ASSET.usda newton OUTPUT.json
    python.sh validate_dynamic_physics.py ASSET.usda physx OUTPUT.json
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

if len(sys.argv) != 4:
    raise SystemExit(
        "usage: validate_dynamic_physics.py ASSET.usda ENGINE OUTPUT.json"
    )

asset_path = Path(sys.argv[1]).resolve()
engine = sys.argv[2]
output_path = Path(sys.argv[3])
if engine not in {"newton", "physx"}:
    raise SystemExit("ENGINE must be newton or physx")

from isaacsim import SimulationApp  # noqa: E402

experience = (
    Path(os.environ["ISAACSIM_PATH"]) / "apps/isaacsim.exp.full.newton.kit"
    if engine == "newton"
    else Path(os.environ["ISAACSIM_PATH"]) / "apps/isaacsim.exp.base.python.kit"
)
app = SimulationApp(
    {"headless": True, "width": 640, "height": 480},
    experience=str(experience),
)
app_utils = None
exit_code = 0

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
    SimulationManager.setup_simulation(dt=0.002, device="cuda:0")
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

    start = articulation.get_dof_positions().numpy()[0].copy()
    target = np.asarray([0.0, -0.7, -1.1, 0.0, 0.0, 0.0, 0.02, 0.02])

    def step(position):
        articulation.set_dof_position_targets(
            np.asarray(position, dtype=np.float32).reshape(1, -1)
        )
        app.update()

    ramp_steps = 250
    for index in range(ramp_steps):
        alpha = (index + 1) / ramp_steps
        step(start * (1.0 - alpha) + target * alpha)

    hold_samples = []
    for index in range(500):
        step(target)
        if index % 50 == 0:
            hold_samples.append(articulation.get_dof_positions().numpy()[0].copy())
    hold_end = articulation.get_dof_positions().numpy()[0].copy()
    hold_error = np.abs(hold_end - target)
    hold_drift = np.abs(hold_samples[-1] - hold_samples[0])

    kp, kd = articulation.get_dof_gains()
    kp_array = kp.numpy().copy()
    kd_array = kd.numpy().copy()
    joint3 = names.index("joint3")
    kp_array[0, joint3] = 0.0
    kd_array[0, joint3] = 0.0
    articulation.set_dof_gains(kp_array, kd_array)
    before_drop = articulation.get_dof_positions().numpy()[0].copy()
    for _ in range(50):
        step(target)
    after_drop = articulation.get_dof_positions().numpy()[0].copy()
    gravity_drop = float(after_drop[joint3] - before_drop[joint3])

    try:
        asset_label = str(asset_path.relative_to(Path.cwd().resolve()))
    except ValueError:
        asset_label = str(asset_path)

    output = {
        "asset": asset_label,
        "requested_engine": engine,
        "active_engine": str(SimulationManager.get_active_physics_engine()).lower(),
        "articulation_root": articulation_root,
        "dof_names": names,
        "max_efforts": max_efforts.tolist(),
        "expected_max_efforts": expected_max_efforts.tolist(),
        "max_velocities": max_velocities.tolist(),
        "expected_max_velocities": expected_max_velocities.tolist(),
        "physics_scenes_before_setup": scenes_before,
        "physics_scenes_after_setup": scenes_after,
        "start_positions": start.tolist(),
        "target_positions": target.tolist(),
        "hold_end_positions": hold_end.tolist(),
        "hold_abs_error": hold_error.tolist(),
        "hold_max_abs_error": float(hold_error.max()),
        "hold_drift": hold_drift.tolist(),
        "gravity_drop_joint3_0p1s": gravity_drop,
        "passed": bool(
            str(SimulationManager.get_active_physics_engine()).lower() == engine
            and scenes_before == ["/PhysicsScene"]
            and scenes_after == ["/PhysicsScene"]
            and np.allclose(max_efforts, expected_max_efforts, rtol=0.0, atol=1e-4)
            and np.allclose(
                max_velocities, expected_max_velocities, rtol=0.0, atol=1e-4
            )
            and hold_error.max() < 0.01
            and abs(gravity_drop) > 1e-5
        ),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        json.dump(output, stream, indent=2)
        stream.write("\n")
    print(json.dumps(output, indent=2), flush=True)
    exit_code = 0 if output["passed"] else 1
finally:
    try:
        if app_utils is not None:
            app_utils.stop()
    except Exception:
        pass
    app.close()

raise SystemExit(exit_code)
