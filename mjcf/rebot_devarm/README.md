# reBot DevArm (RobStride) — MJCF

MuJoCo model of the Seeed reBot DevArm, RobStride build (6 revolute joints +
2-finger parallel gripper, 8 DOF).

This package is **vendored from the MuJoCo Menagerie**
[`seeed_rebot_devarm`](https://github.com/google-deepmind/mujoco_menagerie/tree/da76818e269b82289eba39808e2fb91d679d6994/seeed_rebot_devarm)
model, pinned to commit
[`da76818`](https://github.com/google-deepmind/mujoco_menagerie/commit/da76818e269b82289eba39808e2fb91d679d6994).
That model was in turn derived from this repository's
[`urdf/00-arm-rs_asm-v3`](../../urdf/00-arm-rs_asm-v3), so the MJCF, the URDF
and the Isaac Sim USD asset all describe the same arm.

## Files

| file | description |
|---|---|
| `rebot_devarm.xml` | the model: bodies, joints, inertials, visual + decomposed collision geoms, position actuators, `home` / `raised` keyframes |
| `scene.xml` | `rebot_devarm.xml` + floor, lights, skybox |
| `assets/` | visual meshes (`*.STL`) and the convex-decomposition collision parts (`*_col_N.stl`) |
| `check_menagerie_sync.py` | verifies this package still matches the pinned Menagerie commit |
| `parity_mujoco_vs_pinocchio.py` | gravity-parity check, writes the JSON below |
| `parity_mujoco_vs_pinocchio.json` | gravity-parity evidence (generated, do not hand-edit) |

## Provenance & sync

The only intentional difference from upstream is the model name
(`rebot_devarm` instead of `seeed_rebot_devarm`), so the package can be diffed
against Menagerie mechanically:

```bash
python check_menagerie_sync.py            # byte-compare against the pinned SHA
python check_menagerie_sync.py --update   # re-vendor from the pinned SHA
```

To move to a newer upstream model, bump `MENAGERIE_SHA` in that script, run it
with `--update`, and re-run the parity check below.

Every link keeps its exact full URDF inertial, including off-diagonal tensor
terms, and `gripper_end` is kept as a separate body rather than merged into
`link6` — this keeps the MJCF bodies in one-to-one correspondence with the URDF
links so the two can be diffed link by link. (MuJoCo's URDF importer merges
that fixed joint correctly; an earlier version of this note claimed otherwise
and was wrong. The compiled merge matches a hand-computed composite-rigid-body
tensor to 7.4e-12 kg·m².)

## Collision & self-collision

Collision geometry is a convex **decomposition** (8–12 parts per link, 92
colliders), not one hull per link. A single hull inflates collision volume to
3.24× the true link volume; the decomposition brings that to 1.89×.

Self-collision is **enabled**. Eleven body pairs are excluded in `<contact>`:
nine adjacent links that genuinely interpenetrate at their shared motor
housings, plus `gripper_left`/`gripper_right` and `link2`/`link5`, which clear
each other by 0.15 mm and 0.77 mm in the source meshes — below what a convex
decomposition can represent. Both keyframes are contact-free.

## Actuation

Position actuators (`kp`/`kv` per motor class) with `ctrlrange` equal to the
joint range. Torque limits come from the actuator ratings, each read from the
motor firmware over CAN on a physical arm: RS-06 36 N·m (joints 1–3), RS-00
14 N·m (joints 4–6).

The two fingers are driven by a single 1:1 rack and pinion, so they are coupled
through an `<equality joint>` rather than being independent DOFs, and the
finger gains (`kp=5000`, `forcerange=±1904 N`) are derived from that
transmission rather than tuned by hand.

## Gravity parity

`qfrc_bias` (rest) vs Pinocchio `g(q)` from the same URDF, over 5 poses:

**max |MuJoCo − Pinocchio| = 5.9e-6 N·m**

Pinocchio is the cross-checked reference — it agrees with Isaac Sim (PhysX and
Newton) drive droop and with the real-arm PD-sweep measurements to 3 digits.
So this MJCF is gravity-consistent with the URDF, the USD asset, and hardware.

Reproduce: `python parity_mujoco_vs_pinocchio.py` (rewrites the JSON; the
committed JSON is exactly that script's output).

Full mass/CoM/inertia parity against the composed USD and URDF is checked by
`usd/RS-rebot-dev-arm/scripts/validate_physics_fidelity.py`; the committed
evidence is `usd/RS-rebot-dev-arm/evidence/physics_fidelity_validation.json`.

## Conventions

- `angle="radian"`, meters, `meshdir="assets"`.
- Joint sign convention follows the repo URDF: joint2/3 ∈ [−π, 0], joint4 ∈
  [−1.79, 1.69]. (The Seeed vendor control URDF mirrors all six axes;
  `q_repo = −q_vendor`.)
- Total mass 6.0085 kg. `home` = arm extended (URDF zero); `raised` = elbow-up
  L pose used for gravity validation.

## Load

```python
import mujoco
model = mujoco.MjModel.from_xml_path("mjcf/rebot_devarm/scene.xml")
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, 0)   # home
```
