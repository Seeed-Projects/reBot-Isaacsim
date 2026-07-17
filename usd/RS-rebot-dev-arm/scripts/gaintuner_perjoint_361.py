"""Run the REAL Gain Tuner SnapToLimitsTest headless, forcing the LATEST
gain tuner extension (3.6.1, from the omni_isaac_sim develop checkout) onto
the Isaac Sim 6.0.1 runtime.

Changes vs the stock headless snap harness: --ext-folder override + version
assert, package-root imports (only valid in >=3.6.0, doubles as a version
check).

Usage: python.sh gaintuner_perjoint_361.py <usd> <engine: newton|physx> <out_json>
Env:   ONLY_JOINTS=joint1,joint2  (optional name filter)
       GT_EXT_DIR  REQUIRED: an isolated folder containing ONLY a symlink to
                   the gain tuner source extension you want (3.6.1+), e.g.
                     mkdir -p /tmp/gt_ext && ln -s \
                       <omni_isaac_sim>/source/extensions/isaacsim.robot_setup.gain_tuner \
                       /tmp/gt_ext/
                   Do NOT point it at the whole source/extensions tree: that
                   shadows dozens of built extensions with unbuilt develop
                   copies and segfaults Kit at startup.
"""
import json
import os
import sys

USD, ENGINE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
DT = 0.002
GT_EXT_DIR = os.environ.get("GT_EXT_DIR")
assert GT_EXT_DIR and os.path.isdir(GT_EXT_DIR), (
    "set GT_EXT_DIR to an isolated folder holding only a symlink to the "
    "gain tuner extension (see module docstring)"
)

from isaacsim import SimulationApp

_release = os.environ["ISAACSIM_PATH"]
_exp = (
    os.path.join(_release, "apps", "isaacsim.exp.full.newton.kit")
    if ENGINE == "newton"
    else ""
)
app = SimulationApp(
    {"headless": True, "extra_args": ["--ext-folder", GT_EXT_DIR]},
    experience=_exp,
)

import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

app_utils.enable_extension("isaacsim.robot_setup.gain_tuner")
app.update()

import omni.kit.app

_ext_mgr = omni.kit.app.get_app().get_extension_manager()
_gt_id = _ext_mgr.get_enabled_extension_id("isaacsim.robot_setup.gain_tuner")
print(f"[EXT] gain_tuner resolved: {_gt_id}", flush=True)
assert "3.6.1" in str(_gt_id), f"expected gain_tuner 3.6.1, got {_gt_id}"

if ENGINE == "newton":
    # convexDecomposition colliders generate >200 contacts; raise MuJoCo-Warp caps
    from isaacsim.physics.newton.impl import extension as _next
    _ns = getattr(_next, "_newton_stage", None)
    if _ns is not None:
        _ns.cfg.solver_cfg.nconmax = 8192
        _ns.cfg.solver_cfg.njmax = 32768
        print(f"[CFG] nconmax={_ns.cfg.solver_cfg.nconmax} njmax={_ns.cfg.solver_cfg.njmax}", flush=True)

SimulationManager.switch_physics_engine(ENGINE)
stage_utils.open_stage(USD)
while stage_utils.is_stage_loading():
    app.update()
for _ in range(30):
    app.update()

SimulationManager.setup_simulation(dt=DT, device="cpu")

import omni.usd
from pxr import UsdPhysics

stage = omni.usd.get_context().get_stage()
root = None
for p in stage.Traverse():
    if any("ArticulationRootAPI" in str(s) for s in p.GetAppliedSchemas()):
        root = str(p.GetPath())
        break
assert root, "no articulation root"

from isaacsim.core.experimental.prims import Articulation

art = Articulation(root)
app_utils.play(commit=True)
for _ in range(10):
    app.update()
assert str(SimulationManager.get_active_physics_engine()).lower() == ENGINE

# Package-root imports: only exposed since 3.6.0 (__init__ of 3.5.x exported
# UIBuilder only) — this line is itself a version gate.
from isaacsim.robot_setup.gain_tuner import JointMode, SnapToLimitsTest

num = art.num_dofs
names = list(art.dof_names)
indices = list(range(num))
modes = {i: int(JointMode.POSITION) for i in indices}

# PER-JOINT sequences: one joint per test run (official GUI methodology).
# Other joints hold their home pose via their own drives.
all_metrics = {}
total_steps = 0
MAX_STEPS = int(120.0 / DT)  # per-joint cap
_only = os.environ.get("ONLY_JOINTS")
if _only:
    _sel = [i for i, n in enumerate(names) if n in _only.split(",")]
    indices = _sel
    print("[ONLY]", [names[i] for i in indices], flush=True)
for j in indices:
    test = SnapToLimitsTest()
    test.setup(art, [j], {j: modes[j]}, {"hold_duration": 1.0, "tolerance": 0.01})
    gen = test.run()
    result = None
    steps = 0
    try:
        while steps < MAX_STEPS:
            test._step = DT
            next(gen)
            app.update()
            steps += 1
            if steps % 2000 == 0:
                import time as _t
                print(f"PROGRESS joint={names[j]} steps={steps} sim_t={steps*DT:.1f}s wall={_t.time():.0f}", flush=True)
    except StopIteration as si:
        result = si.value
    assert result is not None, f"joint {names[j]} did not finish within {MAX_STEPS} steps"
    for idx, m in (result.joint_metrics or {}).items():
        all_metrics[int(idx)] = m
    total_steps += steps
    print(f"JOINT_DONE {names[j]} status={result.joint_metrics.get(j, {}).get('status')}", flush=True)

steps = total_steps

metrics = {}
for idx, m in all_metrics.items():
    clean = {}
    for k, v in m.items():
        if isinstance(v, (np.floating, np.integer)):
            v = v.item()
        elif isinstance(v, np.ndarray):
            v = v.tolist()
        clean[k] = v
    metrics[names[int(idx)]] = clean

out = {
    "engine": ENGINE,
    "usd": USD,
    "dt": DT,
    "sim_steps": steps,
    "sim_time_s": steps * DT,
    "gain_tuner_ext": str(_gt_id),
    "joint_metrics": metrics,
}
json.dump(out, open(OUT, "w"), indent=1, default=str)
print("WROTE", OUT)
app_utils.stop()
app.close()
