#!/usr/bin/env python3
"""reBotArm POS_VEL 控制（全部关节，测试模式）。

用法:
    python example/4_pos_vel_control.py

输入: 全部关节角度（度），空格分隔
示例:
    0 0 0 0 0 0         # 仅 arm
    0 0 0 0 0 0 2.0     # arm + 夹爪（如果配置了 gripper）

所有关节统一 POS_VEL 模式，每周期同步发送。
"""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm

rebotarm = RebotArm()
rebotarm.connect()
rebotarm.arm.mode_pos_vel()
if rebotarm.has_gripper:
    rebotarm.gripper.mode_pos_vel()
rebotarm.enable_all()

n_arm = rebotarm.arm.num_joints
n_gripper = rebotarm.gripper.num_joints
n_total = n_arm + n_gripper
target_pos = np.zeros(n_total)


def pos_vel_controller(r: RebotArm, dt: float) -> None:
    r.arm.send_pos_vel(target_pos[:r.arm.num_joints])
    if r.has_gripper:
        r.gripper.send_pos_vel(target_pos[r.arm.num_joints:])


rebotarm.start_control_loop(pos_vel_controller)

print(f"关节数: {n_total} (arm={n_arm}, gripper={n_gripper}) | {rebotarm.rate}Hz")
gripper_hint = "（夹爪将忽略）" if n_gripper == 0 else ""
print(f"命令: {n_total}个角度(度)  q退出  state查看状态 {gripper_hint}\n")

while True:
    try:
        line = input("> ").strip()
    except EOFError:
        break

    if not line:
        continue
    if line.lower() in ("q", "quit", "exit"):
        break

    if line.lower() == "state":
        pos = rebotarm.get_positions()
        print(f"  pos (deg): {[f'{x:+.2f}' for x in np.degrees(pos)]}")
        continue

    tokens = line.split()
    if len(tokens) < n_total:
        print(f"需要 {n_total} 个值（{n_arm} 关节 + {n_gripper} 夹爪）")
        continue

    pos_deg = [float(x) for x in tokens[:n_total]]
    target_pos[:] = np.radians(pos_deg)
    print(f"  -> {[f'{x:+.1f}' for x in pos_deg]}")

rebotarm.disconnect()
