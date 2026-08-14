"""Does adding the measured friction reproduce the real gripper's behaviour?

Measured on the physical arm (MIT mode, motor 7):

    breakaway torque  0.100 N*m  ->  13.6 N reflected through r = 7.353 mm/rad

The pinion sits between two opposed racks, so torque balance is
tau = r*(F_left + F_right); with the fingers travelling 1:1 that is
6.8 N of friction per finger joint.

Compare that against the inertial load the fingers actually see when the arm
slews: 0.10 N at 0.5 Hz, 0.94 N at 4 Hz. Friction exceeds it by 14-130x, so on
the real robot the jaws physically cannot be shifted by wrist acceleration --
they are locked by friction, not held by drive stiffness.

The asset currently declares frictionloss = 0.2 N (a generic default applied to
every joint), i.e. 34x less than measured. This sweeps candidate values and
reports the residual finger motion, so the fix is chosen from data rather than
assumed.

Target: residual below the 0.00197 mm encoder noise floor of the real gripper.
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

MEASURED_TOTAL_N = 0.100 / 7.353e-3      # 13.6 N reflected to the mechanism
PER_FINGER_N = MEASURED_TOTAL_N / 2.0    # 6.8 N, two racks share the pinion


def run(frictionloss, freq, armature=None):
    spec = mujoco.MjSpec.from_file(MJCF)
    for joint in spec.joints:
        if joint.name in ("joint_left", "joint_right"):
            joint.frictionloss = frictionloss
            if armature is not None:
                joint.armature = armature
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
    steps = int(round(3.0 / (freq * model.opt.timestep)))
    for i in range(steps):
        t = i * model.opt.timestep
        data.ctrl[aid["joint1"]] = AMP1 * math.sin(2 * math.pi * freq * t)
        data.ctrl[aid["joint2"]] = HOME[1] + AMP2 * math.sin(2 * math.pi * freq * t)
        mujoco.mj_step(model, data)
        vals = [data.qpos[qadr[n]] for n in ("joint_left", "joint_right")]
        if not np.isfinite(vals).all():
            return float("nan"), float("nan")
        dev = max(dev, max(abs(v - GRIP) for v in vals))
        asym = max(asym, abs(vals[0] - vals[1]))
    return dev * 1e3, asym * 1e3


def can_still_open(frictionloss):
    """The drive must still be able to open the jaws against that friction."""
    spec = mujoco.MjSpec.from_file(MJCF)
    for joint in spec.joints:
        if joint.name in ("joint_left", "joint_right"):
            joint.frictionloss = frictionloss
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
        data.qpos[qadr[n]] = 0.005
        data.ctrl[aid[n]] = 0.045          # command wide open
    mujoco.mj_forward(model, data)
    for _ in range(1200):
        mujoco.mj_step(model, data)
    return data.qpos[qadr["joint_left"]] * 1e3


print("=" * 88)
print("Measured friction, reflected to the fingers")
print("=" * 88)
print(f"  breakaway torque      0.100 N*m")
print(f"  reflected total       {MEASURED_TOTAL_N:.2f} N")
print(f"  per finger joint      {PER_FINGER_N:.2f} N   (tau = r*(F_left+F_right))")
print(f"  asset currently ships 0.20 N  ->  {PER_FINGER_N/0.2:.0f}x too low")

print()
print("=" * 88)
print("Residual finger motion vs frictionloss")
print("=" * 88)
header = f"{'frictionloss':>13s} " + " ".join(f"{f:>9}Hz" for f in (0.5, 1.0, 2.0, 4.0, 8.0))
print(header)
print("-" * len(header))

rows = []
for fl in (0.2, 1.0, 3.0, PER_FINGER_N, 13.6):
    devs = []
    for freq in (0.5, 1.0, 2.0, 4.0, 8.0):
        d, _ = run(fl, freq)
        devs.append(d)
    mark = "  <-- measured" if abs(fl - PER_FINGER_N) < 1e-6 else ""
    rows.append({"frictionloss_N": fl, "dev_mm": devs})
    print(f"{fl:13.2f} " + " ".join(f"{d:11.4f}" for d in devs) + mark)

print()
print(f"real gripper encoder noise floor = {HW_NOISE_MM} mm")

print()
print("=" * 88)
print("Sanity: can the drive still open the jaws against that friction?")
print("=" * 88)
for fl in (0.2, PER_FINGER_N, 13.6):
    reached = can_still_open(fl)
    ok = "OK" if reached > 40.0 else "TOO STIFF -- jaws cannot open"
    print(f"  frictionloss {fl:5.2f} N -> commanded 45 mm, reached {reached:6.2f} mm   {ok}")

json.dump({"per_finger_N": PER_FINGER_N, "total_N": MEASURED_TOTAL_N,
           "rows": rows, "hw_noise_mm": HW_NOISE_MM},
          open("friction_sweep.json", "w"), indent=1)
print("\nsaved -> /home/spark/rebot_gripper_diag/friction_sweep.json")
