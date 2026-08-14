#!/usr/bin/env python3
"""reBotArm 重力补偿 + 关节角 UDP 发送端 / Gravity compensation + joint-angle UDP sender.

重力补偿由 ``reBotArm_control_py.controllers.GravityCompensation`` 提供
（与上游 ``example/9_gravity_compensation.py`` 同一套控制律）。
本脚本只负责把关节角镜像到 Isaac Sim。

Gravity compensation is provided by
``reBotArm_control_py.controllers.GravityCompensation`` (same control law as
upstream ``example/9_gravity_compensation.py``). This script only mirrors joint
angles to Isaac Sim.

推荐运行方式 / Recommended usage:
- 本脚本：连接真实机械臂并发送 UDP。
- 另开终端：用 Isaac 官方 ``python.sh`` 启动 ``isaacsim_joint_receiver.py``。
- 不要同时再跑上游 ``example/9``，以免两个进程抢 CAN。
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

try:
    from reBotArm_control_py.actuator import RebotArm  # noqa: E402
    from reBotArm_control_py.controllers import GravityCompensation  # noqa: E402
except ImportError as exc:
    raise ImportError(
        "未找到 GravityCompensation，请先初始化 submodule:\n"
        "  git submodule update --init --recursive\n"
        "GravityCompensation not found; initialize the submodule first:\n"
        "  git submodule update --init --recursive"
    ) from exc

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
    """启动上游重力补偿，并将关节角 UDP 发给 Isaac Sim。

    Starts upstream gravity compensation and forwards joint angles to Isaac Sim.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.host = host
        self.port = port
        # 硬件 YAML 由 set_hw_rs.py 写入 submodule config/rebotarm.yaml。
        # Hardware YAML is set by set_hw_rs.py in submodule config/rebotarm.yaml.
        self.rebotarm = RebotArm()
        self.ctrl = GravityCompensation(self.rebotarm)
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

    def _snapshot_positions(self, *, filter_position: bool) -> None:
        q = self.rebotarm.arm.get_positions()
        if q.shape[0] < ARM_JOINT_COUNT:
            raise RuntimeError(
                f"arm 组关节数不足 {ARM_JOINT_COUNT}，当前仅 {q.shape[0]} 个 / "
                f"arm joint count is less than {ARM_JOINT_COUNT}, only {q.shape[0]} available"
            )
        q_arm = q[:ARM_JOINT_COUNT]
        self.latest_q_raw[:] = q_arm
        if filter_position:
            filtered_q = (1.0 - self.position_alpha) * (-self.latest_q) + self.position_alpha * q_arm
            self.latest_q[:] = -filtered_q
        else:
            # latest_q 是仿真坐标（q_sim = -q_motor）。
            self.latest_q[:] = -q_arm
        if self.rebotarm.has_gripper:
            gripper_q = self.rebotarm.gripper.get_positions()
            if gripper_q.size > 0:
                self.latest_gripper_q = float(gripper_q[0])
                self.latest_gripper_position = self._gripper_q_to_position(self.latest_gripper_q)

    def start(self) -> None:
        self.ctrl.start()
        self._snapshot_positions(filter_position=False)

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

            self._snapshot_positions(filter_position=True)

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
            self.ctrl.end()
        finally:
            self.socket.close()


def main() -> None:
    print("=" * 72)
    print("  reBotArm 重力补偿 + 关节角 UDP 发送端")
    print("  补偿: reBotArm_control_py.controllers.GravityCompensation")
    print("  预计行为: 用户可自由掰动真实机械臂，关节角持续发送给 Isaac Sim")
    print("  停止方式: Ctrl+C")
    print("=" * 72)
    print(f"[发送] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[关节] arm 前 {ARM_JOINT_COUNT} 个关节")

    print()
    print("=" * 72)
    print("  reBotArm gravity compensation + joint-angle UDP sender")
    print("  compensation: reBotArm_control_py.controllers.GravityCompensation")
    print("  Expected behavior: the user can freely move the physical arm;")
    print("  joint angles are continuously sent to Isaac Sim.")
    print("  To stop: press Ctrl+C")
    print("=" * 72)
    print(f"[sender] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[joints] first {ARM_JOINT_COUNT} arm joints")

    sender = GravityCompensationSender()
    print(f"[硬件] {sender.rebotarm.hardware_yaml}")
    print(f"[hardware] {sender.rebotarm.hardware_yaml}")
    try:
        sender.start()
        print(f"[硬件] 已连接，控制频率 {sender.rebotarm.rate:.1f} Hz")
        print(f"[hardware] connected, control rate {sender.rebotarm.rate:.1f} Hz")
        print("[控制] 已启动上游重力补偿")
        print("[control] upstream gravity compensation started")
        sender.run()
    finally:
        print("[停止] 正在关闭控制与发送...")
        print("[stopping] shutting down control loop and sender...")
        sender.shutdown()
        print("[完成] 已安全退出")
        print("[done] exited safely")


if __name__ == "__main__":
    main()
