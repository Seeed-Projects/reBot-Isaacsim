#!/usr/bin/env python3
"""reBotArm 关节角只读 UDP 发送端 / Joint-angle read-only UDP sender.

功能概述：
1. 在当前工程 `uv` 环境中连接真实机械臂。
2. 仅读取关节角度（被动反馈模式），不发送任何控制指令。
3. 将前 6 个关节角和夹爪位置通过 UDP JSON 持续发送给 Isaac Sim 接收端。

推荐运行方式：
- 直接使用当前工程的 `uv` 环境运行本脚本。
- 再单独使用 Isaac 官方 `python.sh` 启动 `isaacsim_joint_receiver.py`。

Overview:
1. Connect to the physical robot arm using the current project's `uv` environment.
2. Read joint angles in passive feedback mode only - no control commands are sent.
3. Continuously send the first 6 joint angles and gripper position to the
   Isaac Sim receiver over UDP as JSON packets.

Recommended usage:
- Run this script inside the current project's `uv` environment.
- Separately start `isaacsim_joint_receiver.py` with the official Isaac
  `python.sh` launcher.
"""

from __future__ import annotations

import json
import signal
import socket
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
_THIRD_PARTY = REPO_ROOT / "third_party" / "reBotArm_control_py"
sys.path.insert(0, str(_THIRD_PARTY))

from reBotArm_control_py.actuator import RebotArm

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_SEND_HZ = 60.0
DEFAULT_REPORT_EVERY = 30
GRIPPER_POSITION_SCALE = 0.007
DEFAULT_MIT_ENABLED = False  # MIT 零命令发送开关 / MIT zero-command send toggle

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[sender] 收到 Ctrl+C，准备退出... / received Ctrl+C, preparing to exit...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class JointReaderSender:
    """关节角只读发送器。

    Joint-angle read-only sender - reads positions without sending control commands.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT, mit_enabled: bool = DEFAULT_MIT_ENABLED) -> None:
        self.host = host
        self.port = port
        self.mit_enabled = mit_enabled
        self.rebotarm = RebotArm()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.latest_q = np.zeros(ARM_JOINT_COUNT, dtype=np.float64)
        self.latest_q_raw = np.zeros(ARM_JOINT_COUNT, dtype=np.float64)
        self.latest_gripper_q = 0.0
        self.latest_gripper_position = 0.0

    @staticmethod
    def _format_joint_values(values: np.ndarray) -> str:
        q_rad = "  ".join(f"{value:+.3f}" for value in values)
        q_deg = "  ".join(f"{value:+7.2f}" for value in np.rad2deg(values))
        return f"rad=[{q_rad}]  deg=[{q_deg}]"

    def setup_hardware(self) -> None:
        self.rebotarm.connect()

        if self.mit_enabled:
            self.rebotarm.arm.mode_mit()
            if self.rebotarm.has_gripper:
                self.rebotarm.gripper.mode_mit()
            self.rebotarm.disable_all()
            time.sleep(0.1)
            self.rebotarm.enable_all()

            # 发送初始 MIT 命令触发电机回传
            init_pos = np.zeros(self.rebotarm.arm.num_joints, dtype=np.float64)
            init_tau = np.zeros(self.rebotarm.arm.num_joints, dtype=np.float64)
            self.rebotarm.arm.send_mit(
                pos=init_pos,
                vel=np.zeros(self.rebotarm.arm.num_joints, dtype=np.float64),
                kp=np.full(self.rebotarm.arm.num_joints, 0.0, dtype=np.float64),
                kd=np.full(self.rebotarm.arm.num_joints, 0.0, dtype=np.float64),
                tau=init_tau,
            )
            time.sleep(0.05)  # 等待电机回传

            q0 = self.rebotarm.arm.get_positions(request_feedback=True)
        else:
            q0 = self.rebotarm.arm.get_positions(request_feedback=True)  # 使用实时反馈

        if q0.shape[0] < ARM_JOINT_COUNT:
            raise RuntimeError(
                f"arm 组关节数不足 {ARM_JOINT_COUNT}，当前仅 {q0.shape[0]} 个 / "
                f"arm joint count is less than {ARM_JOINT_COUNT}, only {q0.shape[0]} available"
            )
        self.latest_q_raw[:] = q0[:ARM_JOINT_COUNT]
        self.latest_q[:] = -q0[:ARM_JOINT_COUNT]

        if self.rebotarm.has_gripper:
            # 发送初始 MIT 命令触发电爪回传
            init_gripper_pos = np.zeros(self.rebotarm.gripper.num_joints, dtype=np.float64)
            self.rebotarm.gripper.send_mit(
                pos=init_gripper_pos,
                vel=np.zeros(self.rebotarm.gripper.num_joints, dtype=np.float64),
                kp=np.zeros(self.rebotarm.gripper.num_joints, dtype=np.float64),
                kd=np.zeros(self.rebotarm.gripper.num_joints, dtype=np.float64),
                tau=np.zeros(self.rebotarm.gripper.num_joints, dtype=np.float64),
            )
            time.sleep(0.05)

            gripper_q0 = self.rebotarm.gripper.get_positions(request_feedback=True) if self.mit_enabled else self.rebotarm.gripper.get_positions(request_feedback=True)
            if gripper_q0.size > 0:
                self.latest_gripper_q = float(gripper_q0[0])
                self.latest_gripper_position = float(gripper_q0[0] * GRIPPER_POSITION_SCALE)

    def start(self) -> None:
        """启动 MIT 控制循环（仅在 mit_enabled=True 时生效）"""
        if self.mit_enabled:
            self.rebotarm.start_control_loop(self._mit_controller, rate=self.rebotarm.rate)

    def _mit_controller(self, robot: RebotArm, dt: float) -> None:
        """MIT 零命令控制回调 - 发送零力矩以触发电机回传"""
        del dt
        q = robot.arm.get_positions(request_feedback=True)
        q_arm = q[:ARM_JOINT_COUNT]

        pad_len = max(robot.arm.num_joints - ARM_JOINT_COUNT, 0)
        tau_zero = np.concatenate([np.zeros(ARM_JOINT_COUNT, dtype=np.float64), np.zeros(pad_len, dtype=np.float64)])

        robot.arm.send_mit(
            pos=q,
            vel=np.zeros(robot.arm.num_joints, dtype=np.float64),
            kp=np.full(robot.arm.num_joints, 0.0, dtype=np.float64),
            kd=np.full(robot.arm.num_joints, 0.0, dtype=np.float64),
            tau=tau_zero,
        )

        if robot.has_gripper:
            gripper_q = robot.gripper.get_positions(request_feedback=False)
            robot.gripper.send_mit(
                pos=gripper_q,
                vel=np.zeros(robot.gripper.num_joints, dtype=np.float64),
                kp=np.zeros(robot.gripper.num_joints, dtype=np.float64),
                kd=np.zeros(robot.gripper.num_joints, dtype=np.float64),
                tau=np.zeros(robot.gripper.num_joints, dtype=np.float64),
            )
            if gripper_q.size > 0:
                self.latest_gripper_q = float(gripper_q[0])
                self.latest_gripper_position = float(gripper_q[0] * GRIPPER_POSITION_SCALE)

        self.latest_q_raw[:] = q_arm
        self.latest_q[:] = -q_arm

    def read_positions(self) -> None:
        if self.mit_enabled:
            return  # 位置已由 MIT 控制循环更新

        q = self.rebotarm.arm.get_positions(request_feedback=True)
        if q.shape[0] >= ARM_JOINT_COUNT:
            self.latest_q_raw[:] = q[:ARM_JOINT_COUNT]
            self.latest_q[:] = -q[:ARM_JOINT_COUNT]

        if self.rebotarm.has_gripper:
            gripper_q = self.rebotarm.gripper.get_positions(request_feedback=True)
            if gripper_q.size > 0:
                self.latest_gripper_q = float(gripper_q[0])
                self.latest_gripper_position = float(gripper_q[0] * GRIPPER_POSITION_SCALE)

    def run(self, send_hz: float = DEFAULT_SEND_HZ) -> None:
        if send_hz <= 0:
            raise ValueError("send_hz 必须为正数 / send_hz must be a positive number")

        send_period = 1.0 / send_hz
        report_every = DEFAULT_REPORT_EVERY
        last_send_time = 0.0

        while _running:
            now = time.perf_counter()
            if now - last_send_time < send_period:
                time.sleep(send_period * 0.25)
                continue

            self.read_positions()

            payload = {
                "sequence": self.sequence,
                "timestamp": time.time(),
                "joint_positions": self.latest_q.tolist(),
                "gripper_position": self.latest_gripper_position,
            }
            packet = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.socket.sendto(packet, (self.host, self.port))

            if self.sequence % report_every == 0:
                print("[send] raw  " + self._format_joint_values(self.latest_q_raw))
                print("[send] send " + self._format_joint_values(self.latest_q))
                print(f"[send] gripper_q={self.latest_gripper_q:+.3f}  gripper_position={self.latest_gripper_position:+.4f}")

            self.sequence += 1
            last_send_time = now

    def shutdown(self) -> None:
        """关闭连接"""
        try:
            if self.mit_enabled:
                self.rebotarm.disconnect()
        finally:
            self.socket.close()


def main() -> None:
    import argparse

    parser = argparse.ArgumentParser(description="reBotArm Joint Reader Sender")
    parser.add_argument(
        "--mit", action="store_true",
        help="强制启用 MIT 零命令模式 / Force enable MIT zero-command mode"
    )
    parser.add_argument(
        "--no-mit", action="store_true",
        help="强制禁用 MIT 零命令模式 / Force disable MIT zero-command mode"
    )
    args = parser.parse_args()

    # 命令行参数覆盖默认值，但两者都指定时 no-mit 优先
    if args.mit and args.no_mit:
        mit_enabled = False
    elif args.mit:
        mit_enabled = True
    elif args.no_mit:
        mit_enabled = False
    else:
        mit_enabled = DEFAULT_MIT_ENABLED
    mit_status = "启用" if mit_enabled else "禁用"
    print("=" * 72)
    print("  reBotArm 关节角 UDP 发送端")
    print(f"  MIT 零命令: {mit_status}")
    print("  停止方式: Ctrl+C")
    print("=" * 72)
    print(f"[发送] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[关节] arm 前 {ARM_JOINT_COUNT} 个关节")
    print(f"[MIT] {mit_status}")
    print()
    print("=" * 72)
    print("  reBotArm joint-angle UDP sender")
    print(f"  MIT zero-command: {mit_status}")
    print("  To stop: press Ctrl+C")
    print("=" * 72)
    print(f"[sender] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[joints] first {ARM_JOINT_COUNT} arm joints")
    print(f"[MIT] {mit_status}")

    sender = JointReaderSender(mit_enabled=mit_enabled)
    try:
        sender.setup_hardware()
        print("[硬件] 已连接")
        print("[hardware] connected")

        if mit_enabled:
            sender.start()
            print("[MIT] 已启动零命令模式")
            print("[MIT] zero-command mode started")

        sender.run()
    finally:
        print("[停止] 正在关闭...")
        print("[stopping] shutting down...")
        sender.shutdown()
        print("[完成] 已安全退出")
        print("[done] exited safely")


if __name__ == "__main__":
    main()
