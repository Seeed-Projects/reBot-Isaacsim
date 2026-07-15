# Gain Tuner + Newton analysis — 2026-07-07

Session data package. All raw numbers are real tool output from the live
Isaac Sim Newton session (reBot arm asset `00-arm-rs_asm-v3`).

## Files

| File | Content |
|---|---|
| `snap_test_results.csv` | Snap-to-Limits per joint across all 8 configurations tested (status + lower/upper position error in rad) |
| `snap_errors_by_config.png` | Bar chart (log scale) of per-joint errors for the 3 key configs vs 0.01 rad tolerance |
| `home_self_contacts.csv` | Self-contact pairs at HOME pose with self-collision ON: 6915 contacts / 34 pairs; which 19 pairs were filtered in the plus variant and what remains (34 contacts) |
| `home_contacts_filtering.png` | Before/after per-pair contact counts (log) |
| `joint_drives.csv` | Stiffness/damping/limits per joint (manufacturer values, validated 8/8) |
| `joint_gains.png` | Gains per joint (log) |
| `limit_reach_probes.csv` | Direct drive-to-limit probes: target vs reached angle and the stalling contact pair |
| `limit_reach_probes.png` | Evidence that the stall angle is pose-dependent (limit trimming cannot fix it) |
| `solver_buffers.csv` | nconmax/njmax: stock defaults, new defaults, USD persistence, verified allocation |
| `pr_property_query/fix_approaches.csv` | BFS (Alexandra) vs widening (Milad) vs parser descriptor (#696/#697) comparison |
| `pr_property_query/pr696_vs_pr697.csv` | The two open PRs side by side |
| `pr_property_query/harness_results.csv` | Verification harness (6/6) + live Gain Tuner results per variant |

## Key findings (summary)

1. **Gains are correct**: with self-collision OFF, 8/8 joints pass with
   errors ~1e-5 rad. `blocked` results were never a gains problem.
2. **Collision geometry interpenetrates at rest**: 6915 self-contacts at
   home pose (75% gripper_left<->gripper_right). CoACD decomposition
   artifact, not real collision.
3. **Limit trimming does not fix blocked joints**: the stall angle depends
   on the other joints' pose (base<->gripper contact), so a 1-D joint limit
   cannot encode it. Trims to 173/168/166 deg (j2) and -55/-40 deg (j4) all
   still blocked.
4. **Plus variant**: 19 body-level `physics:filteredPairs` remove the
   garbage (6915 -> 34 contacts) while keeping REAL fold self-collision
   active. Requires the extension-side body-level expansion
   (`_apply_usd_filtered_pairs` in newton_stage.py).
5. **Gain Tuner "Disable Self-Collisions" checkbox is a no-op on Newton**:
   it writes `physxArticulation:enabledSelfCollisions` (PhysX attr); Newton
   reads `newton:selfCollisionEnabled`.
6. **nconmax/njmax**: stock 200/1200 overflow instantly with this asset
   (contacts silently dropped). New defaults 8192/32768; also persisted in
   the asset USD via `newton:solver:nconmax`/`njmax` custom attrs (needs the
   `_apply_usd_solver_overrides` extension change to be honored).
