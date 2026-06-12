#!/usr/bin/env python3
"""Isaac Sim 机械臂 + 地面 + UDP 关节角接收端。

功能概述：
1. 使用 Isaac 官方 Python 运行时启动 `SimulationApp`。
2. 创建地面并加载 `usd/RS-rebot-dev-arm` 机械臂资产。
3. 通过 UDP 接收真实机械臂前 6 个关节角，并实时同步到 Isaac Sim。
4. 将收到的夹爪角度乘以 `0.01` 后，作为双关节位置目标同步到仿真夹爪。
"""

from __future__ import annotations

import json
import signal
import socket
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]

try:
    from isaacsim import SimulationApp
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        "未检测到可用的 Isaac Sim Python 环境，请使用 Isaac 官方 python.sh 运行本脚本。"
    ) from exc

if not callable(SimulationApp):
    raise RuntimeError(
        "检测到了不完整的 Isaac Sim Python 运行时：`SimulationApp` 不可调用。"
        "请使用 Isaac 官方 python.sh 运行本脚本。"
    )

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_RENDER_HZ = 120.0
ASSET_RELATIVE_PATH = Path("usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda")
ROBOT_PRIM_PATH = "/World/reBotArm"
GRIPPER_JOINT_NAMES = ("joint_left", "joint_right")
GRIPPER_POSITION_SCALE = 0.01

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[receiver] 收到 Ctrl+C，准备退出...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class IsaacJointMirror:
    """接收 UDP 关节角并同步到 Isaac Sim。"""

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.asset_path = REPO_ROOT / ASSET_RELATIVE_PATH
        if not self.asset_path.exists():
            raise FileNotFoundError(f"Isaac Sim 资产不存在: {self.asset_path}")

        self.host = host
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.setblocking(False)

        self.sim_app = None
        self.world = None
        self.articulation = None
        self.latest_q = np.zeros(ARM_JOINT_COUNT, dtype=np.float64)
        self.last_sequence = -1
        self.last_packet_time = 0.0
        self.arm_joint_indices = np.arange(ARM_JOINT_COUNT, dtype=np.int64)
        self.gripper_joint_indices: np.ndarray | None = None
        self.gripper_limits = np.zeros(2, dtype=np.float64)
        self.gripper_target_position = 0.0
        self._last_gripper_command_signature: tuple[float, float, float] | None = None

    def setup_isaac_sim(self) -> None:
        self.sim_app = SimulationApp({"headless": False})

        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.prims import is_prim_path_valid
        from isaacsim.core.utils.stage import add_reference_to_stage

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()
        add_reference_to_stage(str(self.asset_path), ROBOT_PRIM_PATH)

        if not is_prim_path_valid(ROBOT_PRIM_PATH):
            raise RuntimeError(f"Isaac Sim 中未找到机器人 Prim: {ROBOT_PRIM_PATH}")

        self.articulation = SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="rebotarm_live")
        self.world.scene.add(self.articulation)
        self.world.reset()
        self.articulation.initialize()

        dof_names = list(self.articulation.dof_names)
        expected_names = [f"joint{i}" for i in range(1, ARM_JOINT_COUNT + 1)]
        if dof_names[:ARM_JOINT_COUNT] != expected_names:
            print(f"[warn] Isaac Sim DOF 顺序为: {dof_names}")
            print(f"[warn] 将按前 {ARM_JOINT_COUNT} 个自由度直接同步")

        self._setup_gripper_mapping(dof_names)

        self.articulation.set_joint_positions(self.latest_q, joint_indices=self.arm_joint_indices)
        self.articulation.set_joint_velocities(
            np.zeros(ARM_JOINT_COUNT, dtype=np.float64),
            joint_indices=self.arm_joint_indices,
        )
        self._apply_gripper_target(self.gripper_target_position)

    def _setup_gripper_mapping(self, dof_names: list[str]) -> None:
        missing_joints = [name for name in GRIPPER_JOINT_NAMES if name not in dof_names]
        if missing_joints:
            print(f"[warn] 未找到夹爪 DOF: {missing_joints}，将跳过夹爪联动")
            return

        self.gripper_joint_indices = np.array(
            [dof_names.index(name) for name in GRIPPER_JOINT_NAMES],
            dtype=np.int64,
        )
        lower_limits = np.asarray(self.articulation.dof_properties["lower"])
        upper_limits = np.asarray(self.articulation.dof_properties["upper"])
        self.gripper_limits = upper_limits[self.gripper_joint_indices]
        self.gripper_target_position = 0.0
        print(
            "[夹爪] DOF 映射 = "
            + "  ".join(
                f"{name}:index={index}, lower={lower_limits[index]:+.4f}m, upper={upper_limits[index]:+.4f}m"
                for name, index in zip(GRIPPER_JOINT_NAMES, self.gripper_joint_indices)
            )
        )
        print(
            "[夹爪] 位置控制已启用: "
            + "  ".join(f"{name} 显式接收位置目标" for name in GRIPPER_JOINT_NAMES)
        )
        print(
            "[夹爪] 行程上限 = "
            + "  ".join(f"{name}:{limit:.4f}m" for name, limit in zip(GRIPPER_JOINT_NAMES, self.gripper_limits))
        )

    def _apply_gripper_target(self, gripper_position: float) -> None:
        if self.gripper_joint_indices is None:
            return

        assert self.articulation is not None
        self.gripper_target_position = float(gripper_position)
        target_positions = np.clip(
            np.full(2, self.gripper_target_position, dtype=np.float64),
            0.0,
            self.gripper_limits,
        )
        command_signature = (
            round(float(self.gripper_target_position), 4),
            round(float(target_positions[0]), 4),
            round(float(target_positions[1]), 4),
        )
        if command_signature != self._last_gripper_command_signature:
            print(
                f"[夹爪] command_position={self.gripper_target_position:+.4f}m "
                + "  ".join(
                    f"{name}_target={position:+.4f}m"
                    for name, position in zip(GRIPPER_JOINT_NAMES, target_positions)
                )
            )
            self._last_gripper_command_signature = command_signature

        self.articulation.set_joint_positions(
            target_positions.astype(np.float64),
            joint_indices=self.gripper_joint_indices,
        )

    def _recv_latest_packet(self) -> tuple[np.ndarray, int, float | None] | None:
        latest_packet = None
        while True:
            try:
                packet, _ = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            payload = json.loads(packet.decode("utf-8"))
            joint_positions = np.asarray(payload["joint_positions"], dtype=np.float64)
            if joint_positions.shape != (ARM_JOINT_COUNT,):
                raise RuntimeError(
                    f"收到的关节角维度错误: {joint_positions.shape}，期望 {(ARM_JOINT_COUNT,)}"
                )
            gripper_value = payload.get("gripper_position")
            latest_packet = (joint_positions, int(payload["sequence"]), None if gripper_value is None else float(gripper_value))
        return latest_packet

    def run(self, render_hz: float = DEFAULT_RENDER_HZ) -> None:
        if render_hz <= 0:
            raise ValueError("render_hz 必须为正数")

        assert self.sim_app is not None
        assert self.world is not None
        assert self.articulation is not None

        render_period = 1.0 / render_hz
        step = 0

        while _running and self.sim_app.is_running():
            latest_packet = self._recv_latest_packet()
            if latest_packet is not None:
                self.latest_q, self.last_sequence, gripper_value = latest_packet
                self.last_packet_time = time.time()
                self.articulation.set_joint_positions(
                    self.latest_q,
                    joint_indices=self.arm_joint_indices,
                )
                if gripper_value is not None:
                    self._apply_gripper_target(gripper_value)
                if step % max(int(render_hz // 2), 1) == 0:
                    print(
                        "[recv] q = " + "  ".join(f"{value:+.3f}" for value in self.latest_q)
                    )
                    if self.gripper_joint_indices is not None:
                        gripper_positions = self.articulation.get_joint_positions(joint_indices=self.gripper_joint_indices)
                        print(
                            f"[recv] gripper_position = {self.gripper_target_position:+.4f}m  "
                            + "  ".join(
                                f"{name}={value:+.4f}m"
                                for name, value in zip(GRIPPER_JOINT_NAMES, gripper_positions)
                            )
                        )
                        print(
                            f"[sim] joint_left={gripper_positions[0]:+.4f}m  joint_right={gripper_positions[1]:+.4f}m"
                        )

            self.world.step(render=True)
            step += 1

            if self.last_packet_time > 0 and time.time() - self.last_packet_time > 2.0 and step % max(int(render_hz), 1) == 0:
                print("[warn] 超过 2 秒未收到新的关节角数据")

            time.sleep(render_period * 0.25)

    def shutdown(self) -> None:
        self.socket.close()
        if self.sim_app is not None:
            self.sim_app.close()
            self.sim_app = None


def main() -> None:
    print("=" * 72)
    print("  Isaac Sim 机械臂 + 地面 + UDP 关节角接收端")
    print("  预计行为: 接收真实机械臂关节角，并驱动仿真机械臂同步")
    print("  夹爪行为: 使用位置目标直接控制夹爪滑轨")
    print("  停止方式: 关闭 Isaac Sim 窗口或 Ctrl+C")
    print("=" * 72)
    print(f"[接收] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[资产] {ASSET_RELATIVE_PATH}")

    mirror = IsaacJointMirror()
    try:
        mirror.setup_isaac_sim()
        print("[仿真] Isaac Sim 已启动，地面和机械臂资产已加载")
        mirror.run()
    finally:
        print("[停止] 正在关闭接收与仿真...")
        mirror.shutdown()
        print("[完成] 已安全退出")


if __name__ == "__main__":
    main()
