#!/usr/bin/env python3
"""reBotArm 重力补偿 + 关节角 UDP 发送端 / Gravity compensation + joint-angle UDP sender.

功能概述：
1. 在当前工程 `uv` 环境中连接真实机械臂。
2. 启动 MIT + 重力前馈补偿，允许用户手动掰动。
3. 将前 6 个关节角通过 UDP JSON 持续发送给 Isaac Sim 接收端。

推荐运行方式：
- 直接使用当前工程的 `uv` 环境运行本脚本。
- 再单独使用 Isaac 官方 `python.sh` 启动 `isaacsim_joint_receiver.py`。

Overview:
1. Connect to the physical robot arm using the current project's `uv` environment.
2. Enable MIT control with gravity feed-forward compensation so the arm can be
   moved freely by hand.
3. Continuously send the first 6 joint angles to the Isaac Sim receiver over
   UDP as JSON packets.

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
from reBotArm_control_py.dynamics import compute_generalized_gravity

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_SEND_HZ = 60.0
DEFAULT_REPORT_EVERY = 30
DEFAULT_POSITION_ALPHA = 0.2
GRIPPER_POSITION_SCALE = 0.03

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[sender] 收到 Ctrl+C，准备退出... / received Ctrl+C, preparing to exit...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class GravityCompensationSender:
    """真实机械臂重力补偿与关节角发送。

    Gravity compensation and joint-angle sender for the physical robot arm.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.host = host
        self.port = port
        self.rebotarm = RebotArm()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.latest_q = np.zeros(ARM_JOINT_COUNT, dtype=np.float64)
        self.latest_q_raw = np.zeros(ARM_JOINT_COUNT, dtype=np.float64)
        self.latest_gripper_q = 0.0
        self.latest_gripper_position = 0.0
        self.position_alpha = DEFAULT_POSITION_ALPHA

    @staticmethod
    def _format_joint_values(values: np.ndarray) -> str:
        q_rad = "  ".join(f"{value:+.3f}" for value in values)
        q_deg = "  ".join(f"{value:+7.2f}" for value in np.rad2deg(values))
        return f"rad=[{q_rad}]  deg=[{q_deg}]"

    @staticmethod
    def _gripper_q_to_position(gripper_q: float) -> float:
        return float(gripper_q * GRIPPER_POSITION_SCALE)

    def setup_hardware(self) -> None:
        self.rebotarm.connect()
        self.rebotarm.arm.mode_mit()
        if self.rebotarm.has_gripper:
            self.rebotarm.gripper.mode_mit()
        self.rebotarm.disable_all()
        time.sleep(0.1)
        self.rebotarm.enable_all()

        q0 = self.rebotarm.arm.get_positions(request_feedback=True)
        if q0.shape[0] < ARM_JOINT_COUNT:
            raise RuntimeError(
                f"arm 组关节数不足 {ARM_JOINT_COUNT}，当前仅 {q0.shape[0]} 个 / "
                f"arm joint count is less than {ARM_JOINT_COUNT}, only {q0.shape[0]} available"
            )
        self.latest_q[:] = q0[:ARM_JOINT_COUNT]
        self.latest_q_raw[:] = q0[:ARM_JOINT_COUNT]
        if self.rebotarm.has_gripper:
            gripper_q0 = self.rebotarm.gripper.get_positions(request_feedback=True)
            if gripper_q0.size > 0:
                self.latest_gripper_q = float(gripper_q0[0])
                self.latest_gripper_position = self._gripper_q_to_position(self.latest_gripper_q)

    def start(self) -> None:
        self.rebotarm.start_control_loop(self._gravity_controller, rate=self.rebotarm.rate)

    def _gravity_controller(self, robot: RebotArm, dt: float) -> None:
        del dt
        q = robot.arm.get_positions(request_feedback=True)
        q_arm = q[:ARM_JOINT_COUNT]
        tau_g = compute_generalized_gravity(q=q_arm)

        tau_g[1] *= 1.3  # joint2 额外补偿 / additional compensation for joint 2
        tau_g[2] *= 1.6  # joint3 额外补偿 / additional compensation for joint 3

        pad_len = max(robot.arm.num_joints - ARM_JOINT_COUNT, 0)
        tau_cmd = np.concatenate([tau_g, np.zeros(pad_len, dtype=np.float64)])

        robot.arm.send_mit(
            pos=q,
            vel=np.zeros(robot.arm.num_joints, dtype=np.float64),
            kp=np.full(robot.arm.num_joints, 1.0, dtype=np.float64),
            kd=np.full(robot.arm.num_joints, 0.5, dtype=np.float64),
            tau=tau_cmd,
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
                self.latest_gripper_position = self._gripper_q_to_position(self.latest_gripper_q)

        self.latest_q_raw[:] = q_arm
        filtered_q = (1.0 - self.position_alpha) * (-self.latest_q) + self.position_alpha * q_arm
        self.latest_q[:] = -filtered_q

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
                print(
                    f"[send] gripper_q={self.latest_gripper_q:+.3f}  "
                    f"gripper_position={self.latest_gripper_position:+.4f}"
                )

            self.sequence += 1
            last_send_time = now

    def shutdown(self) -> None:
        try:
            self.rebotarm.disconnect()
        finally:
            self.socket.close()


def main() -> None:
    print("=" * 72)
    print("  reBotArm 重力补偿 + 关节角 UDP 发送端")
    print("  预计行为: 用户可自由掰动真实机械臂，关节角持续发送给 Isaac Sim")
    print("  停止方式: Ctrl+C")
    print("=" * 72)
    print(f"[发送] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[关节] arm 前 {ARM_JOINT_COUNT} 个关节")

    print()
    print("=" * 72)
    print("  reBotArm gravity compensation + joint-angle UDP sender")
    print("  Expected behavior: the user can freely move the physical arm;")
    print("  joint angles are continuously sent to Isaac Sim.")
    print("  To stop: press Ctrl+C")
    print("=" * 72)
    print(f"[sender] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[joints] first {ARM_JOINT_COUNT} arm joints")

    sender = GravityCompensationSender()
    try:
        sender.setup_hardware()
        print(f"[硬件] 已连接，控制频率 {sender.rebotarm.rate:.1f} Hz")
        print(f"[hardware] connected, control rate {sender.rebotarm.rate:.1f} Hz")
        sender.start()
        print("[控制] 已启动重力补偿")
        print("[control] gravity compensation started")
        sender.run()
    finally:
        print("[停止] 正在关闭控制与发送...")
        print("[stopping] shutting down control loop and sender...")
        sender.shutdown()
        print("[完成] 已安全退出")
        print("[done] exited safely")

if __name__ == "__main__":
    main()
