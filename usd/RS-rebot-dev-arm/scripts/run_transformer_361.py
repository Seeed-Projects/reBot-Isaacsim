"""Run isaacsim.asset.transformer with the LATEST develop rules (>=1.7.12)
forced onto the Isaac Sim 6.0.1 runtime via an isolated --ext-folder, to
emit the NVIDIA Isaac Sim layered asset structure (base/robot/physics/
physx/mujoco/none variant set + interface) from a converter output.

1.7.12 fixes the variant-default bug (build 1.7.10 sets a default "physx"
variant that doesn't exist when the source is Newton-only, breaking
composition).

Usage: python.sh run_transformer_361.py <input.usda> <profile.json> <out_dir>
Env:   TF_EXT_DIR  REQUIRED: isolated folder with symlinks to ONLY
                   isaacsim.asset.transformer[.rules] from the develop tree.
"""
import os
import sys

INPUT, PROFILE, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
TF_EXT_DIR = os.environ.get("TF_EXT_DIR")
assert TF_EXT_DIR and os.path.isdir(TF_EXT_DIR), "set TF_EXT_DIR (see docstring)"

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "extra_args": ["--ext-folder", TF_EXT_DIR]})

import omni.kit.app
from isaacsim.core.experimental.utils.app import enable_extension

enable_extension("isaacsim.asset.transformer")
enable_extension("isaacsim.asset.transformer.rules")
app.update()

_id = omni.kit.app.get_app().get_extension_manager().get_enabled_extension_id(
    "isaacsim.asset.transformer.rules"
)
print(f"[EXT] transformer.rules resolved: {_id}", flush=True)
assert "1.7.1" in str(_id) and int(str(_id).rsplit(".", 1)[-1]) >= 12, (
    f"need transformer.rules >= 1.7.12, got {_id}"
)

from isaacsim.asset.transformer import AssetTransformerManager, RuleProfile

with open(PROFILE, encoding="utf-8") as f:
    profile = RuleProfile.from_json(f.read())
print(f"[PROFILE] {profile.profile_name} v{profile.version} "
      f"({sum(1 for r in profile.rules if r.enabled)} enabled rules)", flush=True)

os.makedirs(OUT, exist_ok=True)
manager = AssetTransformerManager()
report = manager.run(input_stage=INPUT, profile=profile, package_root=OUT)
for r in report.results:
    print(f"[{'PASS' if r.success else 'FAIL'}] {r.rule_name}", flush=True)
ok = all(r.success for r in report.results)
print(f"[DONE] {len(report.results)} rules, all_pass={ok} -> {OUT}", flush=True)

app.close()
sys.exit(0 if ok else 1)
