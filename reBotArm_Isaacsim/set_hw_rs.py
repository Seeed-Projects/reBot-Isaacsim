#!/usr/bin/env python3
"""把 submodule 的 config/rebotarm.yaml 指到 RS（本机本地改动，不要提交）。

``gravity_joint_sender.py`` 使用 ``RebotArm()``，电机和 Pinocchio 都读这一份。
先运行本脚本，再启动发送端。

Set submodule config/rebotarm.yaml to RS. Machine-local; do not commit.
``gravity_joint_sender.py`` uses ``RebotArm()``, so motors and Pinocchio both
follow this file. Run this script before the sender.
"""

from __future__ import annotations

import re
from pathlib import Path

HW = "rebotarm_rs.yaml"
CFG = (
    Path(__file__).resolve().parents[1]
    / "third_party"
    / "reBotArm_control_py"
    / "config"
    / "rebotarm.yaml"
)


def main() -> None:
    rs = CFG.with_name(HW)
    if not CFG.exists():
        raise SystemExit(f"not found: {CFG}")
    if not rs.exists():
        raise SystemExit(f"not found: {rs}")

    text = CFG.read_text(encoding="utf-8")
    new, n = re.subn(
        r'^hardware_yaml:\s*.*$',
        f'hardware_yaml: "{HW}"',
        text,
        count=1,
        flags=re.M,
    )
    if n != 1:
        raise SystemExit(f"hardware_yaml not found in {CFG}")

    CFG.write_text(new, encoding="utf-8")
    print(f"{CFG} -> {HW}")
    print("local only; do not commit this yaml")


if __name__ == "__main__":
    main()
