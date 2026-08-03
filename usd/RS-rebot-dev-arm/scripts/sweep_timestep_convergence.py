"""The residual is a timestep floor: quantify it and recommend a rate.

Established, all measured:
  * frictionloss makes it worse (0.1466 -> 0.1832 mm), so the measured 13.6 N
    of hardware friction must NOT be written into the asset
  * the equality constraint is already saturated: solref 0.004 -> 0.0002 gives
    an identical 0.1466 mm, and loosening solimp only reduces it by giving up
    coupling force
  * the 1:1 coupling holds exactly (L-R = 0.00000 mm) across the whole sweep

That leaves integration rate as the only real lever. Quantify the convergence
so the asset can carry a documented, defensible recommendation instead of a
vague "use a smaller dt".

Also verify the coupling still holds at every rate: a faster solver that breaks
the mechanism would be a false win.
"""

import json
import math

import mujoco
import numpy as np

from pathlib import Path

MJCF = str(Path(__file__).resolve().parents[3] / "mjcf" / "rebot_devarm" / "rebot_devarm.xml")
DEG = math.pi / 180.0
HOME = [0.0, -90 * DEG, -1 * DEG, 0.0, 0.0, 0.0]
GRIP = 0.02
AMP1, AMP2 = 0.4, 0.2
HW_NOISE_MM = 0.00197
STROKE_MM = 50.0


def run(timestep, freq, solver_iterations=None):
    spec = mujoco.MjSpec.from_file(MJCF)
    spec.option.timestep = timestep
    if solver_iterations is not None:
        spec.option.iterations = solver_iterations
    model = spec.compile()
    data = mujoco.MjData(model)

    aid = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
           for i in range(model.nu)}
    jid = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i): i
           for i in range(model.njnt)}
    qadr = {n: model.jnt_qposadr[i] for n, i in jid.items()}

    for i, n in enumerate([f"joint{k}" for k in range(1, 7)]):
        data.qpos[qadr[n]] = HOME[i]
        data.ctrl[aid[n]] = HOME[i]
    for n in ("joint_left", "joint_right"):
        data.qpos[qadr[n]] = GRIP
        data.ctrl[aid[n]] = GRIP
    mujoco.mj_forward(model, data)
    for _ in range(int(0.5 / timestep)):
        mujoco.mj_step(model, data)

    dev = asym = 0.0
    steps = int(round(3.0 / (freq * timestep)))
    for i in range(steps):
        t = i * timestep
        data.ctrl[aid["joint1"]] = AMP1 * math.sin(2 * math.pi * freq * t)
        data.ctrl[aid["joint2"]] = HOME[1] + AMP2 * math.sin(2 * math.pi * freq * t)
        mujoco.mj_step(model, data)
        vals = [data.qpos[qadr[n]] for n in ("joint_left", "joint_right")]
        if not np.isfinite(vals).all():
            return float("nan"), float("nan"), float("nan")
        dev = max(dev, max(abs(v - GRIP) for v in vals))
        asym = max(asym, abs(vals[0] - vals[1]))

    # coupling check at this rate
    for n in ("joint_left", "joint_right"):
        data.qpos[qadr[n]] = 0.005
        data.ctrl[aid[n]] = 0.040
    mujoco.mj_forward(model, data)
    for _ in range(int(1.5 / timestep)):
        mujoco.mj_step(model, data)
    track = abs(data.qpos[qadr["joint_left"]] - data.qpos[qadr["joint_right"]]) * 1e3
    return dev * 1e3, asym * 1e3, track


print("=" * 92)
print("Residual vs integration rate   (shipped asset, nothing else changed)")
print("=" * 92)
header = (f"{'rate':>10s} {'dt':>10s} " +
          " ".join(f"{f:>8}Hz" for f in (0.5, 2.0, 8.0)) +
          f" {'coupling':>10s} {'% stroke @2Hz':>15s}")
print(header)
print("-" * len(header))

rows = []
# The MJCF declares timestep = 0.002 s = 500 Hz. Sweeping below that DEGRADES
# the asset rather than describing it, so start at the native rate and go up.
for rate in (500, 1000, 2000, 4000):
    dt = 1.0 / rate
    devs = []
    track = None
    for freq in (0.5, 2.0, 8.0):
        d, a, tr = run(dt, freq)
        devs.append(d)
        if freq == 2.0:
            track = tr
    pct = 100.0 * devs[1] / STROKE_MM
    rows.append({"rate_hz": rate, "dt": dt, "dev_mm": devs,
                 "coupling_mm": track, "pct_stroke_2hz": pct})
    tag = "  <-- asset native" if rate == 500 else ""
    print(f"{rate:8d}Hz {dt:10.6f} " + " ".join(f"{d:10.4f}" for d in devs) +
          f" {track:10.5f} {pct:14.3f}%" + tag)

print()
print(f"real gripper encoder noise floor = {HW_NOISE_MM} mm "
      f"({100*HW_NOISE_MM/STROKE_MM:.4f}% of stroke)")
print("'coupling' is |left - right| after opening both fingers: must stay ~0")

print()
print("=" * 92)
print("Convergence")
print("=" * 92)
base = rows[0]["dev_mm"][1]
for r in rows:
    d = r["dev_mm"][1]
    print(f"  {r['rate_hz']:5d} Hz -> {d:8.4f} mm   {base/d:5.2f}x better than native 500 Hz")

json.dump({"rows": rows, "hw_noise_mm": HW_NOISE_MM, "stroke_mm": STROKE_MM},
          open("timestep_convergence.json", "w"),
          indent=1)
print("\nsaved -> /home/spark/rebot_gripper_diag/timestep_convergence.json")
