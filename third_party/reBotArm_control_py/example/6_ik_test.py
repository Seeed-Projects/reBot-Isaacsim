#!/usr/bin/env python3
"""reBotArm 逆运动学数据测试

用法:
  python example/6_ik_test.py

输入: 末端期望位置 (x y z)，单位：米
      可选：跟随姿态 (roll pitch yaw)，单位：度
输出: 求得的关节角度（度）
      + 收敛信息

配置: config/rebotarm.yaml
"""

import sys
import numpy as np
import pinocchio as pin

sys.path.insert(0, str(__file__).rsplit("/", 2)[0])

from reBotArm_control_py.kinematics import (
    get_joint_count,
    load_robot_model,
    compute_ik,
    get_joint_names,
)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams


# ----------------------------------------------------------------------
# 打印 / UI 相关
# ----------------------------------------------------------------------

def print_welcome(model, joint_names) -> None:
    print("=" * 52)
    print("  reBotArm 逆运动学测试")
    print("=" * 52)
    print(f"  机器人: {model.name}")
    print(f"  关节  : {joint_names}")
    print()
    print("  输入末端期望位姿:")
    print("    <x> <y> <z>                       (仅位置，米)")
    print("    <x> <y> <z> <roll> <pitch> <yaw>    (位置+姿态，度)")
    print()
    print("  示例:")
    print("    0.25 0.0 0.15                      (仅位置)")
    print("    0.25 0.0 0.15 0 0 0                (位置+姿态)")
    print("-" * 52)
    print("> ", end="", flush=True)


def print_result(result, target_pos, target_rot, joint_names, n_joints: int) -> None:
    print()
    print("=" * 52)
    print("  结果")
    print("=" * 52)
    print(f"  目标末端位置   : [{target_pos[0]:+.4f}, {target_pos[1]:+.4f}, {target_pos[2]:+.4f}] m")
    if target_rot is not None:
        euler_in = np.degrees(pin.rpy.matrixToRpy(target_rot))
        print(f"  目标末端姿态   : [{euler_in[0]:+.2f}, {euler_in[1]:+.2f}, {euler_in[2]:+.2f}] deg")
    print()
    print(f"  收敛状态  : {'是' if result.success else '否'}")
    print(f"  迭代次数 : {result.iterations}")
    print(f"  位置误差  : {result.error:.2e} m")
    print()
    print(f"  关节角度 (度) [前 {n_joints} 个控制关节]:")
    for name, deg, rad in zip(joint_names[:n_joints], np.degrees(result.q[:n_joints]), result.q[:n_joints]):
        print(f"    {name:10s} = {deg:+8.4f} deg  ({rad:+.4f} rad)")


def parse_pose_input(line: str) -> tuple:
    tokens = line.split()
    if len(tokens) not in (3, 6):
        print(f"错误: 需要 3 个值（仅位置）或 6 个值（位置+姿态），输入了 {len(tokens)} 个")
        sys.exit(1)
    try:
        vals = [float(x) for x in tokens]
    except ValueError as e:
        print(f"错误: 无法解析数字 — {e}")
        sys.exit(1)

    target_pos = np.array(vals[:3])
    target_rot = None
    if len(vals) == 6:
        roll, pitch, yaw = np.radians(vals[3:6])
        target_rot = pin.rpy.rpyToMatrix(roll, pitch, yaw)
    return target_pos, target_rot


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

    target_pos, target_rot = parse_pose_input(line)

    q_init = np.zeros(model.nq)

    ik_params = IKParams(max_iter=2000, damping=0.01)

    result = compute_ik(
        q_init=q_init,
        target_pos=target_pos,
        target_rot=target_rot,
        params=ik_params,
    )

    print_result(result, target_pos, target_rot, joint_names, n_joints)


if __name__ == "__main__":
    main()
