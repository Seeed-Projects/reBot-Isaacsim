#!/usr/bin/env python3
"""Isaac Sim 逆运动学控制发送端 / Isaac Sim IK control sender.

功能概述：
1. 循环读取用户输入的末端期望位姿（位置 / 位置+姿态）。
2. 调用 `reBotArm_control_py` 的 IK 求解器，得到前 6 个关节角。
3. 以与 `isaacsim_joint_test_sender.py` 相同的 JSON UDP 协议将关节角
   发送到 `isaacsim_joint_receiver.py`（默认 `127.0.0.1:5005`）。
4. 每次发送前携带可选的夹爪开合比（与 6_ik_test.py 保持一致）。

输入格式（每行一条，支持空行 / Ctrl+C / `quit` 退出）：
    <x> <y> <z>                          (位置, 米; 姿态保持当前)
    <x> <y> <z> <roll> <pitch> <yaw>     (位置+姿态, 米/度)
    gripper <ratio>                      (单独更新夹爪, ratio ∈ [0, 1])
    q <j1> <j2> ... <j6>                 (直接发送关节角, 度)

启动方式：
    # 先启动仿真端：
    ./run_isaacsim_receiver.sh
    # 再启动本脚本：
    python3 isaacsim_ik_sender.py
"""

from __future__ import annotations

import json
import select
import signal
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pinocchio as pin

# 兼容直接 python3 运行：把 reBotArm_control_py 加入 sys.path。
_THIRD_PARTY = Path(__file__).resolve().parents[1] / "third_party" / "reBotArm_control_py"
if str(_THIRD_PARTY) not in sys.path:
    sys.path.insert(0, str(_THIRD_PARTY))

from reBotArm_control_py.kinematics import (  # noqa: E402
    compute_ik,
    get_joint_count,
    get_joint_names,
    load_robot_model,
)
from reBotArm_control_py.kinematics.inverse_kinematics import IKParams  # noqa: E402

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_SEND_HZ = 60.0
DEFAULT_GRIPPER = 0.0

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[ik-sender] 收到 Ctrl+C，准备退出...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class IKSender:
    """循环读取位姿 → IK 求解 → UDP 发送关节角。"""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.host = host
        self.port = port

        self.model = load_robot_model()
        self.joint_names = get_joint_names(self.model)
        self.n_joints = get_joint_count()
        self.ik_params = IKParams(max_iter=2000, damping=0.01)
        self.q_prev = np.zeros(self.model.nq)
        self.gripper = DEFAULT_GRIPPER

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0

    # ---------- UDP 发送 ----------

    def _send(self, q_rad: np.ndarray, gripper: float) -> None:
        payload = {
            "sequence": self.sequence,
            "timestamp": time.time(),
            "joint_positions": q_rad.tolist(),
            "gripper_position": float(np.clip(gripper, 0.0, 1.0)),
        }
        packet = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(packet, (self.host, self.port))
        self.sequence += 1

    # ---------- 解析输入 ----------

    @staticmethod
    def _parse_pose(line: str) -> tuple[np.ndarray, np.ndarray | None]:
        tokens = line.split()
        if len(tokens) not in (3, 6):
            raise ValueError(f"需要 3 或 6 个数，实际 {len(tokens)} 个")
        pos = np.array([float(x) for x in tokens[:3]])
        rot = None
        if len(tokens) == 6:
            rpy = np.radians([float(x) for x in tokens[3:6]])
            rot = pin.rpy.rpyToMatrix(*rpy)
        return pos, rot

    def _handle_command(self, line: str) -> str | None:
        """处理一行输入：返回 None 表示继续循环，返回 'quit' 表示退出。"""
        line = line.strip()
        if not line:
            return None

        # 子命令：直接发关节角 / 单独设夹爪
        if line.startswith("q ") or line.startswith("q\t"):
            # 用户输入单位为度，必须先转弧度并取反，对齐电机 → 仿真端的符号约定。
            q_deg = np.array([float(x) for x in line[1:].split()], dtype=np.float64)
            if q_deg.shape != (ARM_JOINT_COUNT,):
                raise ValueError(f"q 需要 {ARM_JOINT_COUNT} 个数，实际 {q_deg.shape[0]} 个")
            q_rad = np.radians(q_deg)
            self._send(-q_rad, self.gripper)
            return self._log("直接下发", q_rad, None, None, None, unit="rad")

        if line.startswith("gripper ") or line.startswith("gripper\t"):
            # 只更新本地夹爪比例；不再发送关节包，
            # 否则接收端会把 latest_q 覆盖为零，让仿真臂瞬间回原点。
            # 新比例会在下一次 IK 或 `q` 命令随包一起下发。
            ratio = float(line.split(None, 1)[1])
            self.gripper = float(np.clip(ratio, 0.0, 1.0))
            return f"夹爪比例已更新为 {self.gripper:.2f}（下一次关节命令生效）"

        if line.lower() in {"quit", "exit", ":q"}:
            return "quit"

        # 默认按位姿输入解析
        target_pos, target_rot = self._parse_pose(line)
        result = compute_ik(
            q_init=self.q_prev,
            target_pos=target_pos,
            target_rot=target_rot,
            params=self.ik_params,
        )
        if not result.success:
            print(
                f"[ik-sender] 求解失败: err={result.error:.2e}, iter={result.iterations}，仍将下发当前解"
            )

        self.q_prev = result.q.copy()
        # 与 joint_reader_sender.py 保持一致：电机反馈 → 仿真端的符号约定为取反。
        self._send(-result.q[:ARM_JOINT_COUNT], self.gripper)
        return self._log(
            "IK 解", result.q[:ARM_JOINT_COUNT], result, target_pos, target_rot
        )

    def _log(self, title, q, result=None, target_pos=None, target_rot=None, unit: str = "deg") -> str:
        # `unit` 决定打印单位；所有显示都按 rad 显示，IK 路径再用度数辅助展示。
        q_arr = np.asarray(q, dtype=np.float64)
        q_rad = q_arr
        q_deg = np.degrees(q_arr)
        msg = [f"[{title}]"]
        if unit == "rad":
            msg.append(
                " q_rad="
                + "  ".join(f"{v:+.3f}" for v in q_rad)
                + "  q_deg="
                + "  ".join(f"{v:+7.2f}" for v in q_deg)
            )
        else:  # 默认按度显示
            msg.append(
                " q_deg="
                + "  ".join(f"{v:+7.2f}" for v in q_deg)
            )
        if result is not None:
            msg.append(f"  success={result.success}  err={result.error:.2e}  iter={result.iterations}")
        if target_pos is not None:
            msg.append(f"  target_pos=[{target_pos[0]:+.3f},{target_pos[1]:+.3f},{target_pos[2]:+.3f}]")
        if target_rot is not None:
            rpy = np.degrees(pin.rpy.matrixToRpy(target_rot))
            msg.append(f"  target_rpy=[{rpy[0]:+.1f},{rpy[1]:+.1f},{rpy[2]:+.1f}]")
        text = "".join(msg)
        print(text)
        return text

    # ---------- 主循环 ----------

    def run(self) -> None:
        print("=" * 64)
        print("  Isaac Sim IK 控制发送端")
        print(f"  机器人      : {self.model.name}")
        print(f"  关节名      : {self.joint_names}")
        print(f"  目标 (UDP)  : {self.host}:{self.port}")
        print("  输入位姿 (每行一条):")
        print("    x y z                  (位置, 米; 姿态保持当前)")
        print("    x y z r p y            (位置+姿态, 米/度)")
        print("    gripper <0~1>           (单独更新夹爪)")
        print("    q j1 j2 j3 j4 j5 j6    (直接发关节角, 度)")
        print("  退出: Ctrl+C / quit / :q")
        print("=" * 64)

        period = 1.0 / DEFAULT_SEND_HZ
        while _running:
            # 使用 select 监听 stdin，timeout 内可响应 Ctrl+C
            rlist, _, _ = select.select([sys.stdin], [], [], period * 0.5)
            if not rlist:
                continue
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:  # EOF
                print("\n[ik-sender] 输入结束，退出。")
                break
            try:
                ret = self._handle_command(line)
            except Exception as exc:  # 输入解析或求解异常
                print(f"[ik-sender] 错误: {exc}")
                continue
            if ret == "quit":
                print("[ik-sender] 收到退出指令。")
                break

        self.socket.close()


def main() -> None:
    sender = IKSender()
    sender.run()


if __name__ == "__main__":
    main()
