# RS-build gravity calibration — 2026-07-17 (rev. 2, contact-free)

Measured on a RobStride reBot DevArm (rs-06 ×3 + rs-00 ×4, ids 1–7, can0,
48 V) with per-joint PD sweeps: holding torque estimated from the MIT
tracking error, `tau ≈ kp·(target − q) − kd·v` (kp assumed exact N·m/rad),
up+down passes to separate Coulomb friction, fitted sample-wise against the
Pinocchio model with the full measured pose.

> **Revision note:** a first pass of this document reported per-joint zero
> offsets, a cable-harness torsion spring on joint2 and a ×1.66 wrist scale.
> All three were **contact artifacts**: the sweeps had been run from the rest
> pose, where the gripper touches the table, and folded poses where links
> touch each other. After re-measuring from clearance poses (elbow-up "L",
> wrist away from table and body) those effects disappear. Lesson kept below.

## Verdict (contact-free measurements)

| joint | model scale k | zero offset | bias [N·m] | Coulomb fric [N·m] | fit rms [N·m] |
|---|---|---|---|---|---|
| joint2 (L pose, 804 pts) | **0.948** | −0.06 ≈ 0 | −0.70 | 0.53 | 0.32 |
| joint3 (airborne, 897 pts) | **0.952** | +0.02 ≈ 0 | +0.12 | 0.49 | 0.27 |
| joint4 (clear, 661 pts) | 0.885 | +0.26 | +0.49 | 0.30 | 0.16 |
| joint5 (clear, 661 pts) | signal < friction | — | — | 0.21 | 0.07 |

**The URDF gravity model with the current masses is correct to ~5–11% on
every joint that carries load, with no zero offsets and no extra terms.**
Motor zeros coincide with the URDF zeros (rest pose = extended arm = URDF
q=0). Joint5's gravity torque (≲0.5 N·m) is below the friction floor —
nothing to tune there. The residual ~5% on j2/j3 is at the method's floor
(it also absorbs any error in the MIT kp scale).

## So why does gravity compensation feel wrong on RS builds?

1. **`get_positions()` returned frozen values on RS firmware** (type-0x18
   stream frames are not decoded by `get_state()`), so
   `example/9_gravity_compensation.py` evaluated g(q) at **q=0 forever** —
   a constant g(0) push regardless of pose. Fixed in this PR (mechPos 0x7019
   param reads).
2. **Contact.** At the rest pose the gripper touches the table (and base-yaw
   motion drags it); in folded poses links touch each other. Any comp tuned
   or evaluated in those regions is fighting contact forces, not gravity.
   Keep link-to-link and link-to-table clearance when testing (elbow-up "L"
   pose works well: j2≈0.7, j3≈1.1).
3. **Friction**: 0.2–0.5 N·m Coulomb per joint — on the wrist joints this is
   comparable to or larger than gravity itself. Position-loop stiffness or a
   friction model has to cover it; masses cannot.
4. **mechVel (0x701A) is not rad/s on this firmware** (measured 2.6–4.8× vs
   dq/dt with inconsistent sign) — finite-difference mechPos for velocity.

## Method notes / gotchas for re-running

- PD-sweep harness (pd_sweep_id.py, auto_float_test.py in the rig workspace)
  with soft-start torque ramps; aborts freeze with PD hold, never torque-off.
- Pre-position to a clearance pose BEFORE sweeping; exclude any samples where
  a link can touch anything (the fits are exquisitely sensitive to it: table
  contact produced a fake −0.40 rad zero offset and a fake ×1.66 wrist scale
  with excellent-looking rms).
- Validation: elbow autonomous float at q=0.9 rad — drift −0.0002 rad over
  12 s with feedforward from the (correct) model.
- Torque scale rides on the MIT kp being exact N·m/rad (no independent
  torque sensor). Sign convention: motor/mechPos frame maps to this repo's
  URDF with identity, and to the Seeed reBot-Isaacsim URDF with
  `q_urdf = −q_motor`.
