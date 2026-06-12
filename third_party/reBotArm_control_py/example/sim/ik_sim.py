#!/usr/bin/env python3
"""逆运动学仿真 — 交互式输入目标位姿 + MeshCat 实时可视化。

用法:
    uv run python example/sim/ik_sim.py

控制:
    输入目标位置 x y z (米)
    可选: 姿态 roll pitch yaw (弧度)
    例: 0.25 0.0 0.15          (仅位置)
    例: 0.25 0.0 0.15 0 0 0    (位置+姿态)
    q / quit / exit: 退出
"""

import sys
import signal
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from reBotArm_control_py.kinematics.inverse_kinematics import compute_ik
from example.sim.visualizer import Visualizer

should_exit = False


def signal_handler(sig, frame):
    global should_exit
    should_exit = True


def main():
    signal.signal(signal.SIGINT, signal_handler)

    print("加载可视化器...")
    viz = Visualizer()

    viz.neutral()

    print("MeshCat 已打开. 输入目标位姿:")
    print("  x y z                      (仅位置，米)")
    print("  x y z roll pitch yaw       (位置+姿态，弧度)")
    print("  q/quit/exit: 退出\n")

    while not should_exit:
        time.sleep(0.01)

        try:
            line = input("目标位姿 > ").strip().lower()
        except EOFError:
            break

        if line in ("q", "quit", "exit", ""):
            break

        try:
            vals = [float(x) for x in line.split()]
            if len(vals) not in (3, 6):
                print("需要 3 个值（仅位置）或 6 个值（位置+姿态）\n")
                continue
        except ValueError:
            print("无效输入\n")
            continue

        target_pos = np.array(vals[:3])  # 获取位置
        target_rot = None
        if len(vals) == 6:
            r, p, y = vals[3], vals[4], vals[5]
            target_rot = pin.rpy.rpyToMatrix(r, p, y)  # 获取姿态

        result = compute_ik(None, target_pos, target_rot)

        viz.update(result.q)
        status = "收敛" if result.success else "未收敛"
        print(f"  [{status}] 迭代={result.iterations} 误差={result.error:.2e}m")
        print(f"  关节角度(deg): {np.degrees(result.q)}\n")


if __name__ == "__main__":
    main()
