"""Sim2real hold test: raise the arm in Isaac Sim and check it holds gravity.

Mirrors the 2026-07-17 real-arm session: ramp to the elbow-up L pose
(hardware motor frame j2=+0.7, j3=+1.1 -> this USD's mirrored frame
j2=-0.7, j3=-1.1), hold, measure droop and drift. Then optionally zero the
elbow drive gains to confirm gravity acts (the joint must fall), re-catch,
and return to rest.

Usage: python.sh sim_hold_test.py <usd> <engine: newton|physx> <out_json>
"""
import json
import os
import sys
import time

USD, ENGINE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
DT = 0.002
RAISED = [0.0, -0.7, -1.1, 0.0, 0.0, 0.0, 0.0, 0.0]
RAMP_S, HOLD_S = 4.0, 10.0

from isaacsim import SimulationApp

_release = os.environ["ISAACSIM_PATH"]
_exp = (
    os.path.join(_release, "apps", "isaacsim.exp.full.newton.kit")
    if ENGINE == "newton"
    else ""
)
app = SimulationApp({"headless": True}, experience=_exp)

import numpy as np
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

if ENGINE == "newton":
    from isaacsim.physics.newton.impl import extension as _next
    _ns = getattr(_next, "_newton_stage", None)
    if _ns is not None:
        _ns.cfg.solver_cfg.nconmax = 8192
        _ns.cfg.solver_cfg.njmax = 32768

SimulationManager.switch_physics_engine(ENGINE)
stage_utils.open_stage(USD)
while stage_utils.is_stage_loading():
    app.update()
for _ in range(30):
    app.update()

SimulationManager.setup_simulation(dt=DT, device="cpu")

import omni.usd

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

names = list(art.dof_names)
num = art.num_dofs
print("dofs:", names, flush=True)
assert num == 8
j2, j3 = names.index("joint2"), names.index("joint3")

def q_now():
    return art.get_dof_positions().numpy()[0].copy()

def step(target):
    art.set_dof_position_targets(np.asarray(target, dtype=np.float32).reshape(1, -1))
    app.update()

q0 = q_now()
print("start pose:", np.round(q0, 4), flush=True)
out = {"engine": ENGINE, "usd": USD, "dt": DT, "raised": RAISED}

# ---- ramp to raised pose
steps = int(RAMP_S / DT)
for i in range(steps):
    a = (i + 1) / steps
    step(q0 * (1 - a) + np.array(RAISED) * a)
q_reach = q_now()
print("after ramp:", np.round(q_reach, 4), flush=True)

# ---- hold
hold_log = []
t0 = time.time()
for i in range(int(HOLD_S / DT)):
    step(RAISED)
    if i % 250 == 0:
        hold_log.append(q_now().tolist())
q_end = q_now()
err = np.abs(q_end - np.array(RAISED))
drift = np.abs(q_end - hold_log[0]) if hold_log else np.zeros(num)
print(f"HOLD {HOLD_S}s: max |target-pos| = {err.max():.2e} rad (joint{int(err.argmax())+1 if err.argmax()<6 else 'grip'})", flush=True)
print("per-dof err:", np.round(err, 6).tolist(), flush=True)
print("drift during hold:", np.round(np.abs(np.array(hold_log[-1]) - np.array(hold_log[0])), 6).tolist(), flush=True)
out["hold_err"] = err.tolist()
out["hold_drift"] = (np.array(hold_log[-1]) - np.array(hold_log[0])).tolist()
out["held"] = bool(err.max() < 0.01)

# ---- gravity check: zero the elbow gains, watch it fall, re-catch
dropped = None
try:
    kps, kds = art.get_dof_gains()
    kps = kps.numpy().copy(); kds = kds.numpy().copy()
    kps0, kds0 = kps.copy(), kds.copy()
    kps[0, j3] = 0.0
    kds[0, j3] = 0.0
    art.set_dof_gains(kps, kds)
    q_before = q_now()
    for _ in range(int(1.0 / DT)):   # 1 s free fall on the elbow
        step(RAISED)
    q_after = q_now()
    dropped = float(q_after[j3] - q_before[j3])
    print(f"GRAVITY CHECK: elbow gains zeroed 1 s -> joint3 moved {dropped:+.3f} rad", flush=True)
    art.set_dof_gains(kps0, kds0)
    for _ in range(int(2.0 / DT)):   # re-catch at raised target
        step(RAISED)
    q_recatch = q_now()
    print(f"re-catch: joint3 back to {q_recatch[j3]:+.4f} (target {RAISED[j3]})", flush=True)
    out["gravity_drop_1s"] = dropped
    out["recatch_err"] = float(abs(q_recatch[j3] - RAISED[j3]))
except AttributeError as e:
    print(f"gain API not available ({e}) — skipping drop test", flush=True)

# ---- return to rest
for i in range(int(RAMP_S / DT)):
    a = (i + 1) / (RAMP_S / DT)
    step(np.array(RAISED) * (1 - a) + q0 * a)
print("returned to rest:", np.round(q_now(), 4), flush=True)

json.dump(out, open(OUT, "w"), indent=1)
print("WROTE", OUT, flush=True)
app_utils.stop()
app.close()
