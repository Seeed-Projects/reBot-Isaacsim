#!/usr/bin/env python3
"""reBotArm 正运动学数据测试

用法:
  python example/5_fk_test.py

输入: model.nq 个关节角度，单位：度，空格分隔
输出: 末端位置 (x, y, z)，单位：米
      + 旋转矩阵 (3x3)
      + 欧拉角 (roll, pitch, yaw)，单位：度

配置: config/rebotarm.yaml
"""

import sys
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

from reBotArm_control_py.kinematics import (
    get_joint_count,
    load_robot_model,
    compute_fk,
    get_joint_names,
)

# ----------------------------------------------------------------------
# 打印
# ----------------------------------------------------------------------
def print_welcome(model, joint_names) -> None:
    n = get_joint_count()
    print("=" * 52)
    print("  reBotArm 正运动学测试")
    print("=" * 52)
    print(f"  机器人  : {model.name}")
    print(f"  关节    : {joint_names[:n]}")
    print(f"  nq = {model.nq} (URDF), 控制前 {n} 个关节")
    print()
    print(f"  输入 {n} 个关节角度（度），空格分隔。")
    print("-" * 52)
    print("> ", end="", flush=True)

def print_result(q_deg, position, rotation, euler_deg) -> None:
    print()
    print("=" * 52)
    print("  结果")
    print("=" * 52)
    print(f"  关节角度 (度): {q_deg}")
    print()
    print(f"  末端位置 (m):")
    print(f"    X = {position[0]:+.6f}")
    print(f"    Y = {position[1]:+.6f}")
    print(f"    Z = {position[2]:+.6f}")
    print()
    print(f"  旋转矩阵 (R_world^end):")
    for row in rotation:
        print(f"    [{row[0]:+.6f}  {row[1]:+.6f}  {row[2]:+.6f}]")
    print()
    print(f"  欧拉角 XYZ (横滚, 俯仰, 偏航) [度]:")
    print(f"    横滚  = {euler_deg[0]:+.4f}")
    print(f"    俯仰 = {euler_deg[1]:+.4f}")
    print(f"    偏航   = {euler_deg[2]:+.4f}")

def parse_joint_input(line: str, n: int) -> np.ndarray:
    tokens = line.split()
    if len(tokens) != n:
        print(f"错误: 需要 {n} 个值，输入了 {len(tokens)} 个")
        sys.exit(1)
    try:
        q_deg = [float(x) for x in tokens]
    except ValueError as e:
        print(f"错误: 无法解析数字 — {e}")
        sys.exit(1)
    return np.radians(q_deg)


# ----------------------------------------------------------------------
# 核心算法
# ----------------------------------------------------------------------
def compute_fk_from_deg(model, q_deg: list) -> tuple:
    q_rad = np.radians(q_deg)
    full_q = np.zeros(model.nq)
    full_q[:len(q_rad)] = q_rad
    position, rotation, homogeneous = compute_fk(model, full_q)
    euler_deg = np.degrees(pin.rpy.matrixToRpy(rotation))
    return position, rotation, homogeneous, euler_deg

# ----------------------------------------------------------------------
# main
# ----------------------------------------------------------------------
def main() -> None:
    model = load_robot_model()
    joint_names = get_joint_names(model)
    n_joints = get_joint_count()

    print_welcome(model, joint_names)

    try:
        line = input().strip()
    except EOFError:
        print("无输入，退出。")
        return

    q_rad = parse_joint_input(line, n_joints)
    q_deg = np.degrees(q_rad)

    position, rotation, homogeneous, euler_deg = compute_fk_from_deg(model, q_deg)

    print_result(q_deg, position, rotation, euler_deg)

if __name__ == "__main__":
    main()
