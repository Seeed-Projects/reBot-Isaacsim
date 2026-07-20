# reBot DevArm (RobStride) — MJCF

MuJoCo model of the Seeed reBot DevArm, RobStride build (6 revolute joints +
2-finger parallel gripper, 8 DOF). Generated from
[`urdf/00-arm-rs_asm-v3`](../../urdf/00-arm-rs_asm-v3) so it stays in lockstep
with the URDF and the Isaac Sim USD asset.

## Files

| file | description |
|---|---|
| `rebot_devarm.xml` | the model: bodies, joints, inertials, visual + convex-collision geoms, position actuators, `home` / `raised` keyframes |
| `scene.xml` | `rebot_devarm.xml` + floor, lights, skybox |
| `assets/` | visual meshes (`*.STL` / `*_merged_*.obj`) and convex-hull collision meshes (`*_convex.stl`) |
| `build_mjcf.py` | regenerates `rebot_devarm.xml` from the URDF (stdlib-only, deterministic) |
| `parity_mujoco_vs_pinocchio.py` | gravity-parity check, writes the JSON below |
| `parity_mujoco_vs_pinocchio.json` | gravity-parity evidence (generated, do not hand-edit) |

## Provenance & regeneration

`build_mjcf.py` builds the XML **directly from
`urdf/00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf`** — no intermediate tool
output is needed. Every URDF value is carried over byte-verbatim (inertial
CoM/mass/full tensor including off-diagonals, joint origins/axes/ranges); only
the joint-origin rotations are converted to body quaternions. On top of that it
layers the menagerie conventions: fixed base, joint armature/damping, position
actuators with the hardware-validated PD gains (rs-06 shoulder/elbow, rs-00
wrist, gripper), `ctrlrange` equal to each joint range, and the two keyframes.

The meshes in `assets/` were produced once with
[discoverse-dev/urdf-to-mjcf](https://github.com/discoverse-dev/urdf-to-mjcf)
(`-ct convex_hull`): multi-part URDF visuals merged into per-link OBJs, plus
one convex collision hull per link. They are committed artifacts; the script
references them but does not regenerate geometry.

```bash
python build_mjcf.py                       # rewrites rebot_devarm.xml
git diff --exit-code -- rebot_devarm.xml   # must be empty: committed XML is byte-reproducible
```

Every link keeps its **exact full URDF inertial**, including off-diagonal
tensor terms, and `gripper_end` is kept as a **separate welded body** rather
than merged into `link6`. This matters: MuJoCo's own URDF importer mis-rotates
the `gripper_end` fixed-joint inertial (it drops the joint's rpy
`3.1416 -1.5708 0`), putting the merged CoM at `[-0.098, -0.144, 0.002]`
instead of `[0, 0, 0.047]` and inventing a ~1 N·m phantom gravity torque on
joint6. Keeping the body separate avoids it.

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

## Actuation

Position actuators (`kp`/`kv` per motor class) with `ctrlrange` equal to the
joint range. `forcerange` mirrors the URDF effort limits (±36 / ±14 / ±500 N).
Note that for the gripper's position servos the **static grip force is capped
by `kp`, not by `forcerange`**: max force ≈ `kp` × position error, i.e. 3.5 N
at 35 mm error and ≤ 7.15 N over the full 71.5 mm stroke at the current
`kp="100"` — the ±500 N `forcerange` is faithful to the URDF but unreachable.
The gains are kept identical to the
[menagerie `seeed_rebot_devarm` model](https://github.com/google-deepmind/mujoco_menagerie);
raise the gripper `kp` locally if you need stronger simulated grasps.

## Known limitations

- **Single convex hull per link.** Collision geometry is one raw convex hull
  per link (no decomposition or primitive pads). The finger hulls turn the
  flat ~122–132 mm gripper aperture into a wedge (~114 mm at the fingertips
  narrowing to ~40 mm at the finger root at full open; hull-vs-mesh deviation
  up to ~34 mm) with contact faces sloped ~27° relative to the real flat
  blades, so wide objects cannot seat deep and grasps are more ejection-prone
  than on hardware. Arm hulls are also bloated (link3 up to ~56 mm at the
  elbow), which degrades clearance checks.
- **Self-collision is disabled** (collision class `contype="0"
  conaffinity="1"`), and there are no `<contact>` excludes. This is deliberate:
  the shipped single hulls interpenetrate at the committed keyframes (~35 mm
  finger–finger and ~15 mm finger–palm overlap when closed), so enabling
  robot-robot contact without decomposed hulls produces instant deep
  penetration. Consequence: folded poses (joint2/joint3 both span π rad) can
  pass links through each other silently.
- **Undecimated assets.** The visual OBJs are full-resolution CAD
  tessellations (~72 MB total, e.g. `link3_merged_link3.obj` ≈ 18.8 MB), and
  `base_link.STL`/`link1.STL`/`link6.STL` are byte-identical copies of the
  URDF meshes. The menagerie variant of this model ships decimated meshes
  instead.

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
m = mujoco.MjModel.from_xml_path("scene.xml")
d = mujoco.MjData(m)
mujoco.mj_resetDataKeyframe(m, d, m.key("raised").id)
```
