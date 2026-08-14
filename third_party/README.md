# third_party provenance

## reBotArm_control_py

- **Upstream**: <https://github.com/Seeed-Projects/reBotArm_control_py>
- **Vendoring method**: git submodule (not a file snapshot)
- **Pinned commit**: `5ba28ac`
  (Seeed-Projects `main`, merge of PR #26 `feat/gravity-compensation-api`),
  which provides `reBotArm_control_py.controllers.GravityCompensation`

Clone or update:

```bash
git clone --recurse-submodules <this-repo>
# already cloned:
git submodule update --init --recursive
```

Bump to a newer upstream commit from the repo root:

```bash
cd third_party/reBotArm_control_py
git fetch origin
git checkout <sha>
cd ../..
git add third_party/reBotArm_control_py
```

Do not commit machine-local edits such as `config/rebotarm.yaml`
(`rebotarm_rs.yaml` vs `rebotarm_dm.yaml`) into this parent repository.
To switch this machine to RS (matches `usd/RS-rebot-dev-arm`), run
`python reBotArm_Isaacsim/set_hw_rs.py`. Senders that call `RebotArm()` use
that file for both motors and Pinocchio.
`.gitmodules` sets `ignore = dirty` so those local YAML edits do not show up
in the parent `git status` (a different submodule HEAD still will).
