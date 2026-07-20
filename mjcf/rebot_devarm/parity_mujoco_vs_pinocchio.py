"""Gravity parity check: MuJoCo qfrc_bias (at rest) vs Pinocchio g(q).

Both are built from the same URDF, so the generalized gravity torque must
agree. Pinocchio is the cross-checked reference (it matches Isaac Sim PhysX +
Newton drive droop and the real-arm PD-sweep measurements to 3 digits), so a
small MuJoCo-vs-Pinocchio residual means the MJCF is gravity-consistent with
the URDF, the USD asset, and hardware.

Run: python parity_mujoco_vs_pinocchio.py
Writes parity_mujoco_vs_pinocchio.json.
"""
import json
from pathlib import Path

import mujoco
import numpy as np
import pinocchio as pin

HERE = Path(__file__).resolve().parent
URDF = HERE.parents[1] / "urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf"

m = mujoco.MjModel.from_xml_path(str(HERE / "rebot_devarm.xml"))
d = mujoco.MjData(m)
model = pin.buildModelFromUrdf(str(URDF))
data = model.createData()

POSES = {
    "home":  [0, 0, 0, 0, 0, 0],
    "L":     [0, -0.7, -1.1, 0, 0, 0],
    "reach": [0.3, -1.2, -0.8, 0.4, 0.5, 0],
    "wrist": [0, -0.9, -1.4, 0.6, 0.9, 0],
    "twist": [1.0, -0.5, -2.0, -0.8, -1.2, 1.5],
}

res = {"engines": {"mujoco": mujoco.__version__, "pinocchio": pin.__version__},
       "unit": "N.m", "poses": {}}
for name, q6 in POSES.items():
    d.qpos[:6] = q6
    d.qpos[6:] = 0
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    q = np.zeros(model.nq)
    q[:6] = q6
    g = pin.computeGeneralizedGravity(model, data, q)[:6]
    diff = np.abs(d.qfrc_bias[:6] - g)
    res["poses"][name] = {
        "mujoco_bias": [round(x, 4) for x in d.qfrc_bias[:6]],
        "pinocchio_g": [round(x, 4) for x in g],
        "max_abs_diff": round(float(diff.max()), 8),
    }
    print(f"{name:6s} max|MuJoCo-Pinocchio| = {diff.max():.2e} N.m")

res["max_diff_all"] = round(max(p["max_abs_diff"] for p in res["poses"].values()), 8)
print(f"\n>>> MAX over all poses = {res['max_diff_all']:.3e} N.m")
out = HERE / "parity_mujoco_vs_pinocchio.json"
out.write_text(json.dumps(res, indent=1) + "\n")
