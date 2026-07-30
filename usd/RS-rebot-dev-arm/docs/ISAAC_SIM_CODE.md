# Isaac Sim code snippets for this asset

## A. Move the arm live in the GUI (Script Editor)

Load the asset, press **Play FIRST**, then run:

```python
import numpy as np, math, omni.kit.app, omni.timeline
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager

print("engine:", SimulationManager.get_active_physics_engine())  # expect 'newton' or 'physx'

tl = omni.timeline.get_timeline_interface()
if not tl.is_playing():
    tl.play(); print("Play started — wait 1 s and re-run this cell")

art = Articulation("/tn__00armrs_asmv3_hJ6D/Geometry/base_link")
assert art.num_dofs == 8, "0 DOFs: press Play first, then re-run (recreate after every Stop)"

lower, upper = [x.numpy()[0] for x in art.get_dof_limits()]
center = (lower + upper) / 2
amp = np.minimum(0.3, 0.4 * (upper - lower) / 2)

t = 0.0
def on_update(e):
    global t
    t += 1/60
    tgt = center + amp * np.sin(2 * math.pi * 0.2 * t)   # gentle 0.2 Hz sweep
    art.set_dof_position_targets(tgt.reshape(1, -1).astype(np.float32))

sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
# stop with: sub.unsubscribe()
```

## B. Close / open the gripper (Script Editor, Play active)

```python
import numpy as np, omni.kit.app
from isaacsim.core.experimental.prims import Articulation

art = Articulation("/tn__00armrs_asmv3_hJ6D/Geometry/base_link")
assert art.num_dofs == 8
names = list(art.dof_names)
gl, gr = names.index("joint_left"), names.index("joint_right")
lower, upper = [x.numpy()[0] for x in art.get_dof_limits()]

tgt = art.get_dof_positions().numpy()[0].copy()
state = {"phase": "open", "i": 0, "steps": 90}   # 1.5 s per phase @60 Hz

def on_update(e):
    s = state
    a = min(1.0, (s["i"] + 1) / s["steps"])
    dest = upper if s["phase"] == "open" else lower
    for j in (gl, gr):
        tgt[j] = (1 - a) * tgt[j] + a * dest[j]
    art.set_dof_position_targets(tgt.reshape(1, -1).astype(np.float32))
    s["i"] += 1
    if s["i"] >= s["steps"] + 30:                 # 0.5 s hold, then flip
        s["phase"] = "close" if s["phase"] == "open" else "open"
        s["i"] = 0

sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(on_update)
# stop with: sub.unsubscribe()
```

## Gotchas (hard-won)

- Create `Articulation` AFTER `timeline.play()` or you get a 0-DOF view and
  `broadcast ... (1,0) vs (1,8)` errors. Recreate it after every Stop->Play.
- Newton headless needs `experience=isaacsim.exp.full.newton.kit`; enabling the
  extension after boot does NOT create the Newton tensor backend.
- Gain Tuner Snap-to-Limits: run ONE joint per sequence (GUI default). Commanding
  all joints to limits simultaneously folds the arm into itself.
- This package ships hybrid colliders: convexHull on body/links (engine-agreement
  validated), convexDecomposition on gripper_end/left/right (hulls filled the
  finger gap and caused false premature contact).
- **MuJoCo-Warp caps (updated 2026-07-07):** this asset persists
  `newton:solver:nconmax = 8192` / `newton:solver:njmax = 32768` as custom attrs on
  the articulation root (stock defaults 200/1200 overflow with these colliders —
  contacts silently dropped). Honored by the patched
  `newton_stage._apply_usd_solver_overrides()`; on stock develop set them manually
  before physics init. Newton takes `max(user, geometry estimate)`, so a high cap
  is safe; verify with `solver.mjw_data.naconmax`.
- **Self-collision (updated 2026-07-07):** asset ships
  `newton:selfCollisionEnabled = 0`. Do NOT rely on the Gain Tuner
  "Disable Self-Collisions" checkbox under Newton — it writes the PhysX attr
  (`physxArticulation:enabledSelfCollisions`) which Newton ignores. This collision
  decomposition interpenetrates at rest (6915 self-contacts at home), so enabling
  self-collision without filtering blocks joint2/joint4 at their limits. A
  filtered-pair variant (`00-arm-rs_asm-v3-plus`, 19 `physics:filteredPairs`,
  home contacts 6915 -> 34, needs a body-level expansion patch in
  newton_stage.py) was validated but is NOT shipped in this repo.
- Gain Tuner Snap-to-Limits validated 8/8 joints pass on this asset
  (self-collision off, manufacturer limits, unchanged gains; errors ~1e-5 rad).

## C. Verify solver buffers and self-contacts (Script Editor, Play active)

```python
import isaacsim.physics.newton as newton_ext

ns = newton_ext.acquire_stage()
d = ns.solver.mjw_data
print("nconmax:", d.naconmax, "| njmax:", d.njmax)   # expect 8192 / 32768

# Self-contact census by body pair (run at home pose)
model = ns.model
names = [b.split("/")[-1] for b in model.body_label]
sb = model.shape_body.numpy()
c = ns.contacts
n = int(c.rigid_contact_count.numpy()[0])
pairs = {}
for a, b in zip(c.rigid_contact_shape0.numpy()[:n], c.rigid_contact_shape1.numpy()[:n]):
    if a < 0 or b < 0: continue
    ba, bb = sb[a], sb[b]
    if ba < 0 or bb < 0 or ba == bb: continue
    k = tuple(sorted((names[ba], names[bb])))
    pairs[k] = pairs.get(k, 0) + 1
for k, v in sorted(pairs.items(), key=lambda kv: -kv[1])[:10]:
    print(k, v)
```
