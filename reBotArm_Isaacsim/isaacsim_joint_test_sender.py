#!/usr/bin/env python3
"""Isaac Sim 关节角 UDP 测试发送端。

功能概述：
1. 不依赖真实机械臂，直接向 `isaacsim_joint_receiver.py` 发送 6 维关节角。
2. 在几个预设姿态之间做缓慢线性插值，便于观察 Isaac Sim 侧是否稳定。
3. 可用于排查真实硬件数据抖动与 Isaac Sim 接收侧问题。
4. 附带发送单输入夹爪开合比，验证左右滑轨是否对称联动。
"""

from __future__ import annotations

import json
import signal
import socket
import time
from typing import Iterable

import numpy as np

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_SEND_HZ = 60.0
DEFAULT_SEGMENT_SECONDS = 3.0
DEFAULT_HOLD_SECONDS = 1.0
DEFAULT_REPORT_EVERY = 30

_running = True


# 单位：rad
# 根据 11b 接收端实测日志提取的角度轨迹，用于复现真实发送端的姿态变化。
POSES = [
    np.array([0.025, 0.099, 0.043, -0.082, 0.027, -0.019], dtype=np.float64),
    np.array([0.025, 0.346, 0.043, -0.082, 0.027, -0.019], dtype=np.float64),
    np.array([0.025, 0.470, 0.246, -0.092, 0.027, -0.019], dtype=np.float64),
    np.array([0.025, 0.497, 0.407, -0.095, 0.027, -0.019], dtype=np.float64),
    np.array([0.025, 0.563, 0.558, -0.138, 0.027, -0.019], dtype=np.float64),
    np.array([0.023, 0.541, 0.548, -0.185, 0.027, -0.019], dtype=np.float64),
    np.array([0.022, 0.462, 0.497, -0.227, 0.027, -0.019], dtype=np.float64),
    np.array([0.023, 0.263, 0.315, -0.227, 0.027, -0.019], dtype=np.float64),
    np.array([0.024, 0.247, 0.302, -0.228, 0.027, -0.019], dtype=np.float64),
]
GRIPPER_RATIOS = [0.35] * len(POSES)


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[test-sender] 收到 Ctrl+C，准备退出...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class IsaacJointTestSender:
    """向 Isaac Sim 接收端发送平滑测试关节角。"""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0

    def _send_packet(self, joint_positions: np.ndarray, gripper_ratio: float) -> None:
        payload = {
            "sequence": self.sequence,
            "timestamp": time.time(),
            "joint_positions": joint_positions.tolist(),
            "gripper_position": float(np.clip(gripper_ratio, 0.0, 1.0)),
        }
        packet = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(packet, (self.host, self.port))
        self.sequence += 1

    def _send_pose_for_duration(
        self,
        pose: np.ndarray,
        gripper_ratio: float,
        duration: float,
        send_hz: float,
    ) -> None:
        if duration <= 0:
            return
        steps = max(int(round(duration * send_hz)), 1)
        period = 1.0 / send_hz
        for _ in range(steps):
            if not _running:
                break
            self._send_packet(pose, gripper_ratio)
            if self.sequence % DEFAULT_REPORT_EVERY == 0:
                print(
                    "[hold] q = "
                    + "  ".join(f"{value:+.3f}" for value in pose)
                    + f"  gripper={gripper_ratio:.2f}"
                )
            time.sleep(period)

    def _interpolate_segment(
        self,
        start_pose: np.ndarray,
        end_pose: np.ndarray,
        duration: float,
        send_hz: float,
    ) -> Iterable[np.ndarray]:
        steps = max(int(round(duration * send_hz)), 2)
        for step in range(steps):
            alpha = step / (steps - 1)
            yield (1.0 - alpha) * start_pose + alpha * end_pose

    def _interpolate_ratio(
        self,
        start_ratio: float,
        end_ratio: float,
        duration: float,
        send_hz: float,
    ) -> Iterable[float]:
        steps = max(int(round(duration * send_hz)), 2)
        for step in range(steps):
            alpha = step / (steps - 1)
            yield (1.0 - alpha) * start_ratio + alpha * end_ratio

    def run(
        self,
        send_hz: float = DEFAULT_SEND_HZ,
        segment_seconds: float = DEFAULT_SEGMENT_SECONDS,
        hold_seconds: float = DEFAULT_HOLD_SECONDS,
    ) -> None:
        if send_hz <= 0:
            raise ValueError("send_hz 必须为正数")
        if segment_seconds <= 0:
            raise ValueError("segment_seconds 必须为正数")
        if hold_seconds < 0:
            raise ValueError("hold_seconds 不能为负数")

        period = 1.0 / send_hz
        print(f"[发送] udp://{self.host}:{self.port}")
        print(f"[频率] {send_hz:.1f} Hz")
        print(f"[插值] 每段 {segment_seconds:.1f} s，停留 {hold_seconds:.1f} s")
        print("[模式] 在预设姿态之间循环发送平滑关节角 + 单输入夹爪")

        while _running:
            for index, (start_pose, end_pose) in enumerate(zip(POSES[:-1], POSES[1:])):
                start_ratio = GRIPPER_RATIOS[index]
                end_ratio = GRIPPER_RATIOS[index + 1]
                pose_iter = self._interpolate_segment(start_pose, end_pose, segment_seconds, send_hz)
                ratio_iter = self._interpolate_ratio(start_ratio, end_ratio, segment_seconds, send_hz)
                for pose, gripper_ratio in zip(pose_iter, ratio_iter):
                    if not _running:
                        break
                    self._send_packet(pose, gripper_ratio)
                    if self.sequence % DEFAULT_REPORT_EVERY == 0:
                        print(
                            "[send] q = "
                            + "  ".join(f"{value:+.3f}" for value in pose)
                            + f"  gripper={gripper_ratio:.2f}"
                        )
                    time.sleep(period)
                if not _running:
                    break
                self._send_pose_for_duration(end_pose, end_ratio, hold_seconds, send_hz)
            if not _running:
                break

    def shutdown(self) -> None:
        self.socket.close()


def main() -> None:
    print("=" * 72)
    print("  Isaac Sim 关节角 UDP 测试发送端")
    print("  预计行为: 在几个预设关节姿态之间缓慢插值循环")
    print("  附带行为: 同时发送单输入夹爪开合比")
    print("  停止方式: Ctrl+C")
    print("=" * 72)

    sender = IsaacJointTestSender()
    try:
        sender.run()
    finally:
        print("[停止] 正在关闭测试发送端...")
        sender.shutdown()
        print("[完成] 已安全退出")


if __name__ == "__main__":
    main()
