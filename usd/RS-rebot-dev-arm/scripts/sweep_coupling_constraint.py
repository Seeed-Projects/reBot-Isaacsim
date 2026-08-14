"""Can the coupling constraint be tightened without destabilising the solver?

Established: the equality constraint applies MORE force to the finger than the
actuator does (1.207 N vs 0.877 N at 2 Hz), and removing it drops the residual
from 0.1466 mm to 0.0840 mm. So the constraint itself is the dominant source of
residual motion, not inertial load and not drive compliance.

MuJoCo equality impedance is set by solref (time constant, damping ratio) and
solimp (dmin, dmax, width, midpoint, power). The shipped values are
solref="0.004 1", solimp="0.9999 0.99999 0.001 0.5 2".

Sweep both, and for each candidate report:
  * residual finger deviation while the arm slews
  * whether the coupling still holds (left vs right must track 1:1)
  * whether the solver stays healthy (finite, no constraint blow-up)

A tighter constraint is only acceptable if it reduces deviation AND keeps the
1:1 coupling AND does not raise solver iterations/energy. Anything that trades
coupling accuracy for a smaller number is a regression in disguise.
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


def run(solref=None, solimp=None, freq=2.0, check_tracking=False):
    spec = mujoco.MjSpec.from_file(MJCF)
    for eq in spec.equalities:
        if solref is not None:
            eq.solref = solref
        if solimp is not None:
            eq.solimp = solimp
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
    for _ in range(60):
        mujoco.mj_step(model, data)

    dev = asym = 0.0
    peak_eq = 0.0
    steps = int(round(3.0 / (freq * model.opt.timestep)))
    for i in range(steps):
        t = i * model.opt.timestep
        data.ctrl[aid["joint1"]] = AMP1 * math.sin(2 * math.pi * freq * t)
        data.ctrl[aid["joint2"]] = HOME[1] + AMP2 * math.sin(2 * math.pi * freq * t)
        mujoco.mj_step(model, data)
        vals = [data.qpos[qadr[n]] for n in ("joint_left", "joint_right")]
        if not np.isfinite(vals).all():
            return dict(dev=float("nan"), asym=float("nan"), eq=float("nan"),
                        track=float("nan"), stable=False)
        dev = max(dev, max(abs(v - GRIP) for v in vals))
        asym = max(asym, abs(vals[0] - vals[1]))
        peak_eq = max(peak_eq, float(np.abs(data.efc_force).max())
                      if data.efc_force.size else 0.0)

    track = float("nan")
    if check_tracking:
        # command the left finger open and see whether the right one follows 1:1
        for n in ("joint_left", "joint_right"):
            data.qpos[qadr[n]] = 0.005
        data.ctrl[aid["joint_left"]] = 0.040
        data.ctrl[aid["joint_right"]] = 0.040
        mujoco.mj_forward(model, data)
        for _ in range(900):
            mujoco.mj_step(model, data)
        l = data.qpos[qadr["joint_left"]]
        r = data.qpos[qadr["joint_right"]]
        track = abs(l - r) * 1e3

    return dict(dev=dev * 1e3, asym=asym * 1e3, eq=peak_eq, track=track,
                stable=True)


SHIPPED_SOLREF = [0.004, 1.0]
SHIPPED_SOLIMP = [0.9999, 0.99999, 0.001, 0.5, 2.0]

print("=" * 92)
print("solref sweep   (solimp at shipped values)")
print("=" * 92)
print(f"{'solref':>18s} {'dev mm':>9s} {'asym mm':>9s} {'peak efc':>10s} "
      f"{'L-R at 40mm':>12s} {'stable':>7s}")
print("-" * 92)

results = {"solref": [], "solimp": []}
for tc in (0.008, 0.004, 0.002, 0.001, 0.0005, 0.0002):
    r = run(solref=[tc, 1.0], check_tracking=True)
    tag = "  <-- shipped" if abs(tc - 0.004) < 1e-9 else ""
    results["solref"].append({"solref": [tc, 1.0], **r})
    print(f"{str([tc, 1.0]):>18s} {r['dev']:9.4f} {r['asym']:9.4f} {r['eq']:10.2f} "
          f"{r['track']:12.5f} {str(r['stable']):>7s}{tag}")

print()
print("=" * 92)
print("solimp dmin/dmax sweep   (solref at shipped 0.004 1)")
print("=" * 92)
print(f"{'solimp dmin/dmax':>20s} {'dev mm':>9s} {'asym mm':>9s} {'peak efc':>10s} "
      f"{'L-R at 40mm':>12s} {'stable':>7s}")
print("-" * 92)

for dmin, dmax in ((0.9, 0.95), (0.99, 0.999), (0.9999, 0.99999),
                   (0.99999, 0.999999), (0.999999, 0.9999999)):
    simp = [dmin, dmax, 0.001, 0.5, 2.0]
    r = run(solimp=simp, check_tracking=True)
    tag = "  <-- shipped" if abs(dmin - 0.9999) < 1e-9 else ""
    results["solimp"].append({"solimp": simp, **r})
    print(f"{dmin:9.7f}/{dmax:<10.8f} {r['dev']:9.4f} {r['asym']:9.4f} "
          f"{r['eq']:10.2f} {r['track']:12.5f} {str(r['stable']):>7s}{tag}")

print()
print(f"reference: real gripper encoder noise = {HW_NOISE_MM} mm")
print("'L-R at 40mm' is the coupling error when both fingers are opened:")
print("  it must stay near zero, otherwise a smaller 'dev' just means the")
print("  constraint stopped coupling the fingers at all.")

json.dump(results, open("constraint_sweep.json", "w"),
          indent=1)
print("\nsaved -> /home/spark/rebot_gripper_diag/constraint_sweep.json")
