#!/usr/bin/env python3
"""reBotArm 重力补偿控制演示。

使用 Pinocchio 计算当前关节构型下的广义重力向量 g(q)，
通过 MIT 模式的前馈力矩直接补偿重力。

控制律（MIT 模式）：
    rebotarm.arm 组: MIT 位置闭环 + 重力前馈
    rebotarm.gripper 组: MIT 控制（保持位置）

安全测试: 设置 ENABLED_JOINTS 只使能部分电机，其他电机保持失能状态。
默认为全部使能；修改为只包含要测试的关节名即可，例如 ["joint1", "joint2"]。
"""
import signal
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)

# ── 安全测试配置 ─────────────────────────────────────────────────────────────
# 只使能以下关节；留空 [] 则全部使能。用于逐个电机安全测试。
ENABLED_JOINTS: list[str] = []
# 示例: 只使能 joint1 进行单电机测试
# ENABLED_JOINTS: list[str] = ["joint1"]
# 示例: 只使能 joint1 和 joint2
# ENABLED_JOINTS: list[str] = ["joint1", "joint2"]
# ─────────────────────────────────────────────────────────────────────────────


_running = True


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C，准备停止...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


def gravity_compensation_controller(r: RebotArm, dt: float) -> None:
    q = r.arm.get_positions(request_feedback=False)
    tau_g = compute_generalized_gravity(q=q)
    tau_g[1] *= 1.55  # joint2 额外补偿
    tau_g[2] *= 1.55  # joint3 额外补偿

    r.arm.send_mit(
        pos=q,
        vel=np.zeros(r.arm.num_joints),
        kp=np.full(r.arm.num_joints, 2.0),
        kd=np.full(r.arm.num_joints, 1.0),
        tau=tau_g,
    )
    if r.has_gripper:
        r.gripper.send_mit(r.gripper.get_positions())

    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 20 == 0:
        print(
            f"[{gravity_compensation_controller._counter:4d}] "
            f"tau_g = " + "  ".join(f"{t:+.3f}" for t in tau_g) + "  N·m"
        )


gravity_compensation_controller._counter = 0


def main() -> None:
    print("=" * 60)
    print("  reBotArm 重力补偿演示")
    print("  预计行为: 机械臂维持位置不动，可以手动掰动至任何位置）")
    print("  Ctrl+C 停止并断开连接")
    print("=" * 60)

    model = load_dynamics_model()
    g_vec = get_default_gravity()
    print(f"\n[模型] nq={model.nq}, nv={model.nv}")
    print(f"[重力] {g_vec}  m/s²")

    rebotarm = RebotArm()
    rebotarm.connect()
    rebotarm.arm.mode_mit()
    rebotarm.gripper.mode_mit()
    rebotarm.disable_all()
    time.sleep(0.1)
    if ENABLED_JOINTS:
        for name in ENABLED_JOINTS:
            if name in rebotarm._motor_map:
                rebotarm._motor_map[name].enable()
        print(f"[安全模式] 仅使能电机: {ENABLED_JOINTS}")
    else:
        rebotarm.enable_all()
        print("[使能] 全部电机已使能")
    rebotarm.start_control_loop(gravity_compensation_controller, rate=rebotarm.rate)
    print(f"[控制循环] 启动 @ {rebotarm.rate} Hz")
    print("-" * 60)
    print(f"{'step':>4}  tau_g (N·m)")
    print("-" * 60)

    try:
        while _running:
            time.sleep(0.01)
    finally:
        print("\n[停止] 关闭控制循环...")
        rebotarm.disconnect()
        print("[完成] 已安全断开连接")


if __name__ == "__main__":
    main()
