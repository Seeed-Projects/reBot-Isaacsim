# Why the gripper still moves when the arm slews

Measured 2026-08-01/02 on the physical arm (RobStride motor 7 over `can0`) and
in MuJoCo 3.10 (`mjcf/rebot_devarm/rebot_devarm.xml`, native `timestep = 0.002 s`)
and Isaac Sim 6.1 / PhysX. Gripper holding 20 mm while the arm sweeps `joint1`
±0.4 rad and `joint2` ±0.2 rad. Raw data in `evidence/`.

The gripper is driven by a single motor through one pinion and two opposed
racks, so a reasonable expectation is that the jaws should not move at all while
that motor holds position. On the real robot they effectively don't. In
simulation they deviate 0.0064 mm at 0.25 Hz and 0.25 mm at 8 Hz (PhysX). This
documents what that residual is, what it is *not*, and which knob actually moves
it — every claim below is a measurement, including the ones that refuted the
obvious explanations.

## Hardware reference values

Measured on the real gripper in MIT mode, after the official MotorBridge Studio
parameter template was written and verified (`evidence/mit_calibration.json`,
`evidence/breakaway.json`):

| quantity | measured | at the finger |
|---|---|---|
| breakaway torque | 0.100 N·m | **13.6 N** |
| closed-loop stiffness at `kp = 3` N·m/rad | — | **66 759 N/m** |
| encoder noise, motor unpowered | 268.5 µrad | 0.00197 mm |
| backlash (both directions free) | ±0.05 rad | ±0.37 mm |

The stiffness figure is the median of 12 plateaus spanning 64 800–69 900 N/m
(4 % spread), including the return sweep at reversed sign. Theory predicts
`kp/r² = 55 487 N/m`; the measurement is 1.20× that, the excess being series
structural compliance and friction assisting the hold.

**Reading the drive gains correctly:** `kp = 50` N·m/rad in the vendor library's
`MIT` block is a torque-per-radian stiffness, so reflecting it through the
transmission (`kp/r²` with r = 7.353 mm/rad) gives **924 785 N/m** and is
dimensionally valid — *for MIT mode only*. The factory template's `loc_kp = 10`
is a `pos_vel` position-loop gain feeding a velocity setpoint; it is **not** a
stiffness and cannot be reflected the same way.

## It is not drive compliance

Raising the gripper drive stiffness 16× barely changes it (2 Hz slew):

| drive stiffness | 1 250 | 2 500 | **5 000** *(shipped)* | 10 000 | 20 000 N/m |
|---|---|---|---|---|---|
| deviation | 0.1516 | 0.1481 | **0.1466** | 0.1460 | 0.1459 mm |

The actuator is also nowhere near saturation: peak finger force is **0.88 N of
the 1904 N** available (0.05 % of range).

## It is not modellable as joint friction

The real mechanism has substantial static friction — 13.6 N reflected at the
finger, i.e. 6.8 N per finger joint since the pinion drives two racks. Compared
against the inertial load the jaws actually see while the arm slews, friction
dominates by **14–130×**:

| slew | wrist accel | inertial load per jaw | friction / load |
|---|---|---|---|
| 0.5 Hz | 1.37 m/s² | 0.103 N | 132× |
| 2 Hz | 6.09 m/s² | 0.458 N | 30× |
| 4 Hz | 12.53 m/s² | 0.942 N | 14× |

So on the real robot the jaws physically cannot be shifted by wrist
acceleration. The obvious conclusion — write the measured friction into the
asset — was tested and is **wrong**:

| `frictionloss` | 0.5 Hz | 2 Hz | 4 Hz |
|---|---|---|---|
| **0.2 N** *(shipped)* | 0.0333 | **0.1466** | 0.3485 mm |
| 6.8 N *(measured)* | 0.0381 | **0.1832** | 0.3486 mm |
| 13.6 N | 0.0381 | 0.1832 | 0.3678 mm |

Modelling the real friction makes the simulation **worse**. The reason is the
next section: in sim the joint is driven by a constraint, and friction cannot
oppose a constraint — it only forces the solver to push harder to satisfy it.

## It is the coupling constraint

Measuring the internal forces at 2 Hz shows the equality constraint applies
*more* force to the finger than the actuator does:

| case | deviation | actuator force | constraint force |
|---|---|---|---|
| baseline (shipped) | 0.1466 mm | 0.877 N | **1.207 N** |
| coupling disabled | **0.0840 mm** | 0.455 N | 0.200 N |
| coupling disabled + friction 6.8 N | 1.2207 mm | 5.763 N | 6.174 N |

Disabling the coupling removes 43 % of the residual. That makes the constraint
the dominant contributor — ahead of inertial load, which acts through
`dx = m·a/K` and is real but secondary:

| slew | wrist accel | measured dx | predicted `m·a/K` |
|---|---|---|---|
| 0.5 Hz | 1.37 m/s² | 0.0333 mm | 0.0207 mm |
| 2 Hz | 6.09 m/s² | 0.1466 mm | 0.0916 mm |
| 4 Hz | 12.53 m/s² | 0.3485 mm | 0.1884 mm |

The ~1.6–1.8× gap between predicted and measured is the constraint's share.

**The constraint is already as tight as it can usefully be.** Sweeping its
impedance changes nothing:

| `solref` | 0.008 1 | **0.004 1** *(shipped)* | 0.002 1 | 0.001 1 | 0.0005 1 | 0.0002 1 |
|---|---|---|---|---|---|---|
| deviation | 0.5817 | **0.1466** | 0.1466 | 0.1466 | 0.1466 | 0.1466 mm |

It saturates at the shipped `0.004`. Loosening `solimp` to `0.9 0.95` does give
0.1269 mm, but only because a softer constraint transmits less force — that is
giving up coupling fidelity, not fixing anything. Throughout every sweep the
1:1 coupling held exactly: opening both fingers to 40 mm left `|left − right|`
at **0.00000 mm**.

## The only real lever is the integration rate

With friction ruled out and the constraint saturated, the residual is a solver
floor. It converges cleanly as `1/dt` — doubling the rate halves the deviation:

| rate | dt | 0.5 Hz | 2 Hz | 8 Hz | coupling error | % of stroke @2Hz |
|---|---|---|---|---|---|---|
| **500 Hz** *(asset native)* | 0.002 s | 0.0332 | **0.1466** | 0.5032 | 0.00000 mm | 0.293 % |
| 1000 Hz | 0.001 s | 0.0163 | 0.0737 | 0.3355 | 0.00000 mm | 0.147 % |
| 2000 Hz | 0.0005 s | 0.0082 | 0.0375 | 0.1757 | 0.00000 mm | 0.075 % |
| 4000 Hz | 0.00025 s | 0.0043 | 0.0195 | 0.0922 | 0.00000 mm | 0.039 % |

The coupling stays exact at every rate, so this is a genuine accuracy gain
rather than a trade.

## Guidance

- **Normal teleop (≤ 2 Hz wrist motion):** the asset at its native 500 Hz keeps
  the jaws within 0.15 mm, **0.29 % of the 50 mm stroke**. Nothing to change.
- **Sub-0.05 mm fidelity:** raise the physics rate. 2000 Hz gives 0.0375 mm at
  4× the cost; the relationship is linear in `dt`, so budget accordingly.
- **Do not** write the measured hardware friction into the asset — it is
  physically real but makes the simulation less accurate, for the reason above.
- **Do not** stiffen the gripper drive to compensate: 16× buys 0.4 %.
- **Do not** loosen `solimp` to make the number look better; it works by
  weakening the very coupling the asset exists to model.

The residual is not a defect in the mechanism model. The jaws are coupled
exactly, the drive is not the limit, and the real robot's friction is
mechanically irrelevant to what simulation is doing here.

## Reproducing the hardware measurements

Three properties of `third_party/reBotArm_control_py` cost several failed runs
and are worth knowing before repeating any of this on the physical arm.

**`send_mit()` is silently ignored unless the motor is switched to MIT first.**
There is no error: the frames are accepted, telemetry keeps flowing, and the
measured torque simply never tracks the command — a ramp from 0.05 to 2.0 N·m
produced a flat −0.13 N·m at every step. Check with `run_mode` (`0x7005`, read
as **i8**; `robstride_get_param_f32` raises "parameter type mismatch"): `0` is
MIT, `1` is POS_VEL. Call `motor.ensure_mode(Mode.MIT, 1000)` first, which —
unlike `mode_pos_vel()` — writes no gains, and restore `Mode.POS_VEL` in a
`finally`.

**`send_pos_vel(pos, vlim)` persists `vlim` into the motor.** It is not carried
in the command frame; it is written to parameter `0x7017` (`limit_spd`), as
`example/0x01rs06_test.py` shows. Likewise `mode_pos_vel()` calls
`_write_pv_params()`, which writes `0x7017`, `0x701E` (`loc_kp`), `0x701F`
(`spd_kp`) and `0x7020` (`spd_ki`) from the library YAML, overwriting the
vendor's calibrated template without warning. Detect it by reading parameters
from all seven joints and comparing — joints your scripts never touched still
hold the factory values. Repair through MotorBridge Studio (Read Parameters →
Apply Default Template → Write Parameters), which verifies by read-back; note
the template is **per-joint, not uniform**, so do not "restore" one joint by
copying another's. `limit_spd` is not part of the template and must be restored
separately.

**Position-mode stall detection is not viable at this feedback rate.** The
type-`0x18` broadcast arrives at 40 Hz (25.01 ms, stdev 0.27 ms) while the drive
goes from free to hard stop in less than one sample interval: a 1.0 N·m stall
threshold was never observed, with consecutive samples reading ≈0 then
−3.76 N·m. Torque mode avoids the problem entirely, since the commanded torque
bounds the effort by construction rather than by reaction time.

Guards that follow from the above, for any contact-seeking move on this arm:
probe the direction with a small step before ramping (a calibration note is a
hypothesis, not a fact), size that step above the ±0.37 mm backlash or "both
directions free" will mislead you, cap the torque well below the motor's 14 N·m,
treat a constant-zero torque after motion as a latched fault rather than
stillness, and get a camera on the robot when the operator cannot see it.

Raw data: `evidence/mit_calibration.json` (stiffness, real hardware),
`evidence/breakaway.json` (friction, real hardware),
`evidence/friction_sweep.json`, `evidence/constraint_sweep.json`,
`evidence/timestep_convergence.json`, `evidence/compliance_analysis.json`,
`evidence/residual_cause.json`, `evidence/mechanism.json`,
`evidence/slew_sweep.csv`.

Scripts that produce it, all runnable from a clone:

| script | what it measures | needs the arm |
|---|---|---|
| `scripts/measure_gripper_stiffness.py` | closed-loop stiffness, MIT mode | yes |
| `scripts/measure_gripper_breakaway.py` | breakaway torque | yes |
| `scripts/sweep_gripper_friction.py` | residual vs `frictionloss` | no |
| `scripts/sweep_coupling_constraint.py` | residual vs `solref` / `solimp` | no |
| `scripts/sweep_timestep_convergence.py` | residual vs integration rate | no |

The two hardware scripts take `--dry-run`, which exercises the whole path
against live telemetry without enabling the motor. Run that first.
