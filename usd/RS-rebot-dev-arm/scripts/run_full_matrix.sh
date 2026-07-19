#!/usr/bin/env bash
# Full per-joint SnapToLimits matrix on the re-exported asset, gain tuner 3.6.1.
# Strictly sequential (VALIDATION_SKILL.md: never overlap engine runs; never
# gate on pgrep of a self-matching pattern).
set -uo pipefail

: "${ISAACSIM_PATH:?set ISAACSIM_PATH to your Isaac Sim release dir}"
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USD=$PKG/00-arm-rs_asm-v3.usda
EV=$PKG/evidence

for engine in newton physx; do
  echo "=== ENGINE $engine start $(date -u +%H:%M:%S) ==="
  "$ISAACSIM_PATH/python.sh" "$PKG/scripts/gaintuner_perjoint_361.py" \
    "$USD" "$engine" "$EV/gt_pj_new_${engine}.json" \
    > "$EV/gt_pj_new_${engine}.log" 2>&1
  echo "=== ENGINE $engine exit=$? $(date -u +%H:%M:%S) ==="
done
echo "MATRIX_DONE"
