#!/usr/bin/env python3
"""机械臂零点校准 + 随动模式（MIT 零刚度）。

使能后，机械臂处于随动状态（MIT，kp=0, kd=0），可手动自由移动，
同时实时打印各关节角度（度）。

用法::

    python example/2_zero_and_read.py
      自动从 config/rebotarm.yaml 的 hardware_yaml 读取配置

    python example/2_zero_and_read.py rebotarm_dm.yaml
      强制指定硬件配置文件
"""
import time
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from reBotArm_control_py.actuator import RebotArm


_hw_yaml = sys.argv[1] if len(sys.argv) > 1 else None
rebotarm = RebotArm(_hw_yaml)
print(f"[{rebotarm.hardware_yaml}] 使用配置: {rebotarm.hardware_yaml}")
rebotarm.connect()
print("--- 连接成功 ---")
rebotarm.set_zero()
print("--- 零点已设置 ---\n")

n_arm = rebotarm.arm.num_joints
n_total = rebotarm.num_joints

# 随动模式：MIT 零刚度，机械臂可自由手动移动
_zeros_arm = np.zeros(n_arm)


def fresh_controller(r: RebotArm, dt: float) -> None:
    r.arm.send_mit(
        pos=_zeros_arm,
        vel=np.zeros(n_arm),
        kp=np.zeros(n_arm),
        kd=np.zeros(n_arm),
        tau=np.zeros(n_arm),
    )
    r.gripper.send_mit(
        pos=np.array([0.0]),
        vel=np.zeros(1),
        kp=np.zeros(1),
        kd=np.zeros(1),
        tau=np.zeros(1),
    )


print(f"--- 随动模式，实时角度（deg）Ctrl+C 退出 ---\n")
rebotarm.arm.mode_mit()
rebotarm.gripper.mode_mit()
rebotarm.enable_all()
rebotarm.start_control_loop(fresh_controller)
try:
    while True:
        positions = rebotarm.get_positions()
        row = "  ".join(f"{p*180/np.pi:+.2f}" for p in positions)
        print(f"\r{row}  ", end="", flush=True)
        time.sleep(0.002)
except (KeyboardInterrupt, EOFError):
    pass
finally:
    rebotarm.disconnect()
