#!/usr/bin/env python3
"""reBotArm 重力补偿控制演示（末端速度锁止版）。

在基础重力补偿的基础上，加入末端速度检测：
  - 持续计算末端执行器的线速度和角速度
  - 当末端速度 ||v_ee|| < 阈值时：目标关节角度保持锁定
  - 当末端速度 ||v_ee|| > 阈值时：目标关节角度更新为当前关节角度

控制律（MIT 模式）：
    rebotarm.arm 组: 重力前馈 + MIT 位置闭环
    rebotarm.gripper 组: MIT 控制

兼容: rebotarm_dm.yaml (Damiao 电机) 和 rebotarm_rs.yaml (RobStride 电机)
"""
import signal
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.dynamics import (
    load_dynamics_model,
    compute_generalized_gravity,
    get_default_gravity,
)
from reBotArm_control_py.kinematics import load_robot_model, get_end_effector_frame, pad_q_for_model

# ── 安全测试配置 ─────────────────────────────────────────────────────────────
# 只使能以下关节；留空 [] 则全部使能。用于逐个电机安全测试。
ENABLED_JOINTS: list[str] = []
# 示例: 只使能 joint1 进行单电机测试
# ENABLED_JOINTS: list[str] = ["joint1"]
# ─────────────────────────────────────────────────────────────────────────────

# ── 控制参数 ────────────────────────────────────────────────────────────────
_VEL_THRESHOLD = 0.04
_W_VEL_THRESHOLD = 0.08
_EE_FRAME: str | None = None  # 由运行时从配置读取
_EE_FRAME_ID: int | None = None
_KP = 8.0
_KD = 1.0
_GRIPPER_KP = 0.0
_GRIPPER_KD = 0.0

_running = True
_q_target: np.ndarray | None = None
_lock_counter = 0
_integral: np.ndarray | None = None
_model: pin.Model | None = None  # dynamics model (nq=8, includes gripper)
_kin_model: pin.Model | None = None  # kinematics model (same nq=8)
_kin_data: pin.Data | None = None
_data: pin.Data | None = None


def _sigint_handler(signum, frame):
    global _running
    print("\n[gravity_comp] 收到 Ctrl+C，准备停止...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


def _init_models() -> None:
    global _model, _kin_model, _data, _kin_data, _EE_FRAME, _EE_FRAME_ID
    if _model is not None:
        return
    _model = load_dynamics_model()
    _kin_model = load_robot_model()
    _data = _model.createData()
    _kin_data = _kin_model.createData()
    _EE_FRAME = get_end_effector_frame()
    _EE_FRAME_ID = _kin_model.getFrameId(_EE_FRAME)


def gravity_compensation_controller(r: RebotArm, dt: float) -> None:
    global _q_target, _lock_counter, _integral, _model, _data, _kin_model, _kin_data

    _init_models()

    q_arm = r.arm.get_positions()
    q_full = pad_q_for_model(_kin_model, q_arm, controlled_joints=r.arm.num_joints)
    qd_arm = r.arm.get_velocities()
    qd_full = pad_q_for_model(_kin_model, qd_arm, controlled_joints=r.arm.num_joints)
    n = r.arm.num_joints

    tau_g = compute_generalized_gravity(q=q_full)

    q_error = _q_target - q_arm

    if _integral is None:
        _integral = np.zeros(n)

    _integral += q_error * 1.0
    np.clip(_integral, -0.5, 0.5, out=_integral)

    pin.computeJointJacobians(_kin_model, _kin_data, q_full)
    pin.updateFramePlacements(_kin_model, _kin_data)
    J = pin.getFrameJacobian(_kin_model, _kin_data, _EE_FRAME_ID, pin.ReferenceFrame.WORLD)
    v_spatial = J @ qd_full
    v_ee_norm = float(np.linalg.norm(v_spatial[:3]))
    w_ee_norm = float(np.linalg.norm(v_spatial[3:]))

    if v_ee_norm > _VEL_THRESHOLD or w_ee_norm > _W_VEL_THRESHOLD:
        _q_target = q_arm.copy()
        _lock_counter = 0
        _integral *= 0.9
    else:
        _lock_counter += 1

    r.arm.send_mit(
        pos=_q_target,
        vel=np.zeros(n),
        kp=np.full(n, _KP),
        kd=np.full(n, _KD),
        tau=tau_g[:n] + _integral,
    )
    if r.has_gripper:
        gripper_q = r.gripper.get_positions()
        gripper_n = r.gripper.num_joints
        r.gripper.send_mit(
            pos=gripper_q,
            vel=np.zeros(gripper_n),
            kp=np.full(gripper_n, _GRIPPER_KP),
            kd=np.full(gripper_n, _GRIPPER_KD),
        )

    gravity_compensation_controller._counter += 1
    if gravity_compensation_controller._counter % 20 == 0:
        lock_status = "LOCKED" if _lock_counter > 0 else "UPDATE"
        print(
            f"[{gravity_compensation_controller._counter:4d}] "
            f"{lock_status}  "
            f"v={v_ee_norm:.4f}m/s  w={w_ee_norm:.4f}rad/s  "
            f"tau_g=" + "  ".join(f"{t:+.3f}" for t in tau_g[:n]) + "  N·m"
        )


gravity_compensation_controller._counter = 0


def main() -> None:
    global _q_target

    print("=" * 65)
    print("  reBotArm 重力补偿演示（末端速度锁止版）")
    print(f"  末端速度阈值: {_VEL_THRESHOLD} m/s")
    print("  预计行为: 机械臂锁止在当前位置，用力推才能改变目标角度")
    print("  Ctrl+C 停止并断开连接")
    print("=" * 65)

    dyn_model = load_dynamics_model()
    g_vec = get_default_gravity()
    print(f"\n[模型] nq={dyn_model.nq}, nv={dyn_model.nv}")
    print(f"[重力] {g_vec}  m/s²")
    ee_frame = get_end_effector_frame()
    print(f"[末端帧] {ee_frame}")

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
    _q_target = rebotarm.arm.get_positions()
    print(f"[目标角度] 初始锁定: {np.rad2deg(_q_target).round(2)} deg")

    rebotarm.start_control_loop(gravity_compensation_controller, rate=rebotarm.rate)
    print(f"[控制循环] 启动 @ {rebotarm.rate} Hz")

    try:
        while _running:
            time.sleep(0.01)
    finally:
        print("\n[停止] 关闭控制循环...")
        rebotarm.disconnect()
        print("[完成] 已安全断开连接")


if __name__ == "__main__":
    main()
