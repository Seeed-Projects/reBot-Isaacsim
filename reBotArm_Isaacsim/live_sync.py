#!/usr/bin/env python3
"""reBotArm 与 Isaac Sim 双进程实时镜像入口说明。

本示例已改为两个独立脚本：
1. `gravity_joint_sender.py`：在当前工程 `uv` 环境中运行，负责真实机械臂重力补偿与关节角 UDP 发送。
2. `isaacsim_joint_receiver.py`：使用 Isaac 官方 `python.sh` 运行，负责启动 Isaac Sim、加载地面和机械臂，并接收关节角进行同步。

推荐启动顺序：
1. 先启动接收端。
2. 再启动发送端。
"""

from __future__ import annotations

from pathlib import Path


def main() -> None:
    example_dir = Path(__file__).resolve().parent
    sender = example_dir / "gravity_joint_sender.py"
    receiver = example_dir / "isaacsim_joint_receiver.py"

    print("=" * 72)
    print("  reBotArm + Isaac Sim 双进程实时镜像")
    print("=" * 72)
    print("请分别启动以下两个脚本：")
    print()
    print(f"1. 发送端（uv 环境）: {sender.name}")
    print(f"2. 接收端（Isaac 官方 python.sh）: {receiver.name}")
    print()
    print("推荐顺序：先启动接收端，再启动发送端。")


if __name__ == "__main__":
    main()
