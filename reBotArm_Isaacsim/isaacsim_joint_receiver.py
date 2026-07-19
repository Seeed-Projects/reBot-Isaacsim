#!/usr/bin/env python3
"""
Isaac Sim 机械臂 + 地面 + UDP 关节角接收端

Isaac Sim arm + ground + UDP joint-angle receiver.

功能概述：
1. 使用 Isaac 官方 Python 运行时启动 `SimulationApp`。
2. 创建地面并加载 `usd/RS-rebot-dev-arm` 机械臂资产。
3. 通过 UDP 接收真实机械臂前 6 个关节角，并实时同步到 Isaac Sim。
4. 将收到的夹爪角度乘以 `0.01` 后，作为双关节位置目标同步到仿真夹爪。

推荐运行方式：
- 使用 Isaac 官方 `python.sh` 启动本脚本。
- 先用 `uv run` 启动 `gravity_joint_sender.py` 推送关节角。

Overview:
1. Launch `SimulationApp` via the official Isaac Python runtime.
2. Create the ground plane and load the `usd/RS-rebot-dev-arm` robot asset.
3. Receive the first 6 joint angles from the physical arm over UDP and mirror
   them in Isaac Sim in real time.
4. Multiply the received gripper angle by `0.01` and use the result as a
   position target for the simulated two-joint gripper.

Recommended usage:
- Launch this script with the official Isaac `python.sh` runner.
- Start `gravity_joint_sender.py` via `uv run` to publish joint angles.
"""

from __future__ import annotations

import json
import signal
import socket
import struct
import sys
import time
import zlib
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]

try:
    from isaacsim import SimulationApp
except ImportError as exc:  # pragma: no cover
    raise RuntimeError(
        "未检测到可用的 Isaac Sim Python 环境，请使用 Isaac 官方 python.sh 运行本脚本 \n"
        "No usable Isaac Sim Python environment found; please run this script with the official Isaac python.sh \n"
    ) from exc

if not callable(SimulationApp):
    raise RuntimeError(
        "检测到了不完整的 Isaac Sim Python 运行时：`SimulationApp` 不可调用，请使用 Isaac 官方 python.sh 运行本脚本 \n"
        "Incomplete Isaac Sim Python runtime detected: `SimulationApp` is not callable, \n"
        "please run this script with the official Isaac python.sh \n"
    )

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_FEEDBACK_PORT = 5006
DEFAULT_RENDER_HZ = 120.0
ASSET_RELATIVE_PATH = Path("usd/RS-rebot-dev-arm/00-arm-rs_asm-v3.usda")
GRID_TEXTURE_RELATIVE_PATH = Path("reBotArm_Isaacsim/assets/grid_ground.png")
GRID_TEXTURE_CELLS = 10
GRID_TEXTURE_SIZE = 512
GRID_TEXTURE_SCALE = np.array([10.0, 10.0], dtype=np.float64)
ROBOT_PRIM_PATH = "/World/reBotArm"
GROUND_PLANE_PRIM_PATH = "/World/defaultGroundPlane"
DOME_LIGHT_PRIM_PATH = "/World/DomeLight"
DISTANT_LIGHT_PRIM_PATH = "/World/DistantLight"
DEFAULT_CAMERA_EYE = np.array([0.595, 0.532, 0.636], dtype=np.float64)
DEFAULT_CAMERA_TARGET = np.array([0.0, 0.0, 0.35], dtype=np.float64)
GRIPPER_JOINT_NAMES = ("joint_left", "joint_right")
GRIPPER_POSITION_SCALE = 0.01

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[receiver] 收到 Ctrl+C，准备退出... / received Ctrl+C, preparing to exit...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class IsaacJointMirror:
    """接收 UDP 关节角并同步到 Isaac Sim。

    Receive UDP joint angles and mirror them to the Isaac Sim articulation.
    """

    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT) -> None:
        self.asset_path = REPO_ROOT / ASSET_RELATIVE_PATH
        if not self.asset_path.exists():
            raise FileNotFoundError(
                f"Isaac Sim 资产不存在: {self.asset_path} / Isaac Sim asset not found: {self.asset_path}"
            )

        self.host = host
        self.port = port
        self.feedback_port = DEFAULT_FEEDBACK_PORT
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.host, self.port))
        self.socket.setblocking(False)
        self.feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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

    @staticmethod
    def _write_png_rgb(path: Path, rgb: np.ndarray) -> None:
        """Write an RGB PNG without external image dependencies."""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("PNG input must be uint8 RGB array")

        height, width = rgb.shape[:2]
        raw_data = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))
        compressed = zlib.compress(raw_data, level=9)

        def _chunk(tag: bytes, data: bytes) -> bytes:
            crc = zlib.crc32(tag + data) & 0xFFFFFFFF
            return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

        png = b"\x89PNG\r\n\x1a\n"
        png += _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        png += _chunk(b"IDAT", compressed)
        png += _chunk(b"IEND", b"")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(png)

    @classmethod
    def _ensure_grid_texture(cls) -> Path:
        """Create a local light-gray grid texture for the ground plane."""
        texture_path = REPO_ROOT / GRID_TEXTURE_RELATIVE_PATH

        size = GRID_TEXTURE_SIZE
        cells = GRID_TEXTURE_CELLS
        background = np.array([245, 245, 247], dtype=np.uint8)
        minor_line = np.array([220, 220, 224], dtype=np.uint8)
        major_line = np.array([190, 190, 198], dtype=np.uint8)
        image = np.tile(background, (size, size, 1))
        cell_size = size // cells

        for index in range(cells + 1):
            pos = min(index * cell_size, size - 1)
            line_color = major_line if index == cells // 2 else minor_line
            thickness = 2 if index == cells // 2 else 1
            end = min(pos + thickness, size)
            image[pos:end, :, :] = line_color
            image[:, pos:end, :] = line_color

        cls._write_png_rgb(texture_path, image)
        return texture_path

    @staticmethod
    def _add_local_ground_plane(world) -> None:
        """Create a local physics ground plane with an Isaac-style grid texture."""
        from isaacsim.core.api.materials.omni_pbr import OmniPBR
        from isaacsim.core.api.materials.physics_material import PhysicsMaterial
        from isaacsim.core.api.objects import GroundPlane
        from isaacsim.core.utils.prims import is_prim_path_valid
        from isaacsim.core.utils.string import find_unique_string_name

        physics_material_path = find_unique_string_name(
            initial_name="/World/Physics_Materials/physics_material",
            is_unique_fn=lambda x: not is_prim_path_valid(x),
        )
        physics_material = PhysicsMaterial(
            prim_path=physics_material_path,
            static_friction=0.5,
            dynamic_friction=0.5,
            restitution=0.8,
        )
        visual_material_path = find_unique_string_name(
            initial_name="/World/Looks/grid_ground_material",
            is_unique_fn=lambda x: not is_prim_path_valid(x),
        )
        grid_texture = IsaacJointMirror._ensure_grid_texture()
        visual_material = OmniPBR(
            prim_path=visual_material_path,
            texture_path=str(grid_texture),
            texture_scale=GRID_TEXTURE_SCALE,
            color=np.array([1.0, 1.0, 1.0], dtype=np.float64),
        )
        ground_plane = GroundPlane(
            prim_path=GROUND_PLANE_PRIM_PATH,
            name="default_ground_plane",
            z_position=0.0,
            physics_material=physics_material,
            visual_material=visual_material,
        )
        world.scene.add(ground_plane)

    @staticmethod
    def _add_default_lighting() -> None:
        """Add basic scene lighting without relying on Nucleus assets."""
        from isaacsim.core.utils.stage import get_current_stage
        from pxr import Gf, Sdf, UsdLux

        stage = get_current_stage()
        if not stage.GetPrimAtPath(DOME_LIGHT_PRIM_PATH).IsValid():
            dome = UsdLux.DomeLight.Define(stage, Sdf.Path(DOME_LIGHT_PRIM_PATH))
            dome.CreateIntensityAttr(450.0)
            dome.CreateColorAttr(Gf.Vec3f(0.96, 0.96, 0.98))

        if not stage.GetPrimAtPath(DISTANT_LIGHT_PRIM_PATH).IsValid():
            distant = UsdLux.DistantLight.Define(stage, Sdf.Path(DISTANT_LIGHT_PRIM_PATH))
            distant.CreateIntensityAttr(500.0)
            distant.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 45.0, 0.0))

    @staticmethod
    def _set_viewport_camera(sim_app) -> None:
        """Point the default viewport at the robot after the UI is ready."""
        from isaacsim.core.utils.viewports import set_camera_view

        for _ in range(3):
            sim_app.update()

        set_camera_view(
            eye=DEFAULT_CAMERA_EYE,
            target=DEFAULT_CAMERA_TARGET,
            camera_prim_path="/OmniverseKit_Persp",
        )

    def setup_isaac_sim(self) -> None:
        self.sim_app = SimulationApp({"headless": False})

        from isaacsim.core.api import World
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.prims import is_prim_path_valid
        from isaacsim.core.utils.stage import add_reference_to_stage

        # 创建纹理路径符号链接，解决 IsaacSim 从错误路径查找纹理的问题
        expected_tex_dir = Path.home() / "reBotArm_control_py" / "config" / "RS-rebot-dev-arm" / "Textures"
        if not expected_tex_dir.exists():
            expected_tex_dir.parent.mkdir(parents=True, exist_ok=True)
            actual_tex_dir = self.asset_path.parent / "Textures"
            expected_tex_dir.symlink_to(actual_tex_dir)

        self.world = World(stage_units_in_meters=1.0)
        self._add_local_ground_plane(self.world)
        self._add_default_lighting()
        add_reference_to_stage(str(self.asset_path), ROBOT_PRIM_PATH)

        if not is_prim_path_valid(ROBOT_PRIM_PATH):
            raise RuntimeError(
                f"Isaac Sim 中未找到机器人 Prim: {ROBOT_PRIM_PATH} / "
                f"robot prim not found in Isaac Sim: {ROBOT_PRIM_PATH}"
            )

        # 让 gripper 左右两指互不碰撞：把两指的 collider 放进一个 PhysicsCollisionGroup
        # 并把 group 的 filteredGroups 自引用——USD 的 PhysicsCollisionGroup 机制
        # 是"colliders 列表 ∩ filteredGroups 集合内的对象不互相接触"。同组内成员之
        # 间不发生接触。arm 6 个 link 之间不受影响。
        from pxr import Sdf, UsdPhysics
        from isaacsim.core.utils.stage import get_current_stage

        stage = get_current_stage()
        collision_group_path = Sdf.Path("/World/GripperFingerNoCollideGroup")
        collision_group = UsdPhysics.CollisionGroup.Define(stage, collision_group_path)
        colliders_api = collision_group.GetCollidersCollectionAPI()
        if colliders_api is None:
            colliders_api = UsdPhysics.CollectionAPI.Apply(collision_group.GetPrim(), "colliders")
        colliders_rel = colliders_api.GetIncludesRel()
        if colliders_rel is None:
            colliders_rel = colliders_api.CreateIncludesRel()
        finger_collider_paths = (
            f"{ROBOT_PRIM_PATH}/Geometry/base_link/link1/link2/link3/link4/link5/link6"
            f"/gripper_end/gripper_left/gripper_left",
            f"{ROBOT_PRIM_PATH}/Geometry/base_link/link1/link2/link3/link4/link5/link6"
            f"/gripper_end/gripper_right/gripper_right",
        )
        for finger_collider_path in finger_collider_paths:
            if stage.GetPrimAtPath(finger_collider_path).IsValid():
                colliders_rel.AddTarget(Sdf.Path(finger_collider_path))
        filtered_groups_rel = collision_group.GetFilteredGroupsRel()
        if filtered_groups_rel is None:
            filtered_groups_rel = collision_group.CreateFilteredGroupsRel()
        filtered_groups_rel.AddTarget(collision_group_path)
        print(
            f"[recv-setup] PhysicsCollisionGroup 已创建于 {collision_group_path}，"
            f"包含 colliders: {colliders_rel.GetTargets()}"
        )

        self.articulation = SingleArticulation(prim_path=ROBOT_PRIM_PATH, name="rebotarm_live")
        self.world.scene.add(self.articulation)
        self.world.reset()
        self.articulation.initialize()

        dof_names = list(self.articulation.dof_names)
        expected_names = [f"joint{i}" for i in range(1, ARM_JOINT_COUNT + 1)]
        if dof_names[:ARM_JOINT_COUNT] != expected_names:
            print(
                f"[warn] Isaac Sim DOF 顺序为: {dof_names} / Isaac Sim DOF order is: {dof_names}"
            )
            print(
                f"[warn] 将按前 {ARM_JOINT_COUNT} 个自由度直接同步 / "
                f"will mirror the first {ARM_JOINT_COUNT} DoFs directly"
            )

        self._setup_gripper_mapping(dof_names)

        self.articulation.set_joint_positions(self.latest_q, joint_indices=self.arm_joint_indices)
        self.articulation.set_joint_velocities(
            np.zeros(ARM_JOINT_COUNT, dtype=np.float64),
            joint_indices=self.arm_joint_indices,
        )
        self._apply_gripper_target(self.gripper_target_position)
        self._set_viewport_camera(self.sim_app)

    def _setup_gripper_mapping(self, dof_names: list[str]) -> None:
        missing_joints = [name for name in GRIPPER_JOINT_NAMES if name not in dof_names]
        if missing_joints:
            print(
                f"[warn] 未找到夹爪 DOF: {missing_joints}，将跳过夹爪联动 / "
                f"gripper DoFs not found: {missing_joints}; skipping gripper mirroring"
            )
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
            "[夹爪/gripper] DOF 映射 = "
            + "  ".join(
                f"{name}:index={index}, lower={lower_limits[index]:+.4f}m, upper={upper_limits[index]:+.4f}m"
                for name, index in zip(GRIPPER_JOINT_NAMES, self.gripper_joint_indices)
            )
        )
        print(
            "[夹爪/gripper] 位置控制已启用: "
            + "  ".join(f"{name} 显式接收位置目标 / {name} receives explicit position target" for name in GRIPPER_JOINT_NAMES)
        )
        print(
            "[夹爪/gripper] 行程上限 = "
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
                f"[夹爪/gripper] command_position={self.gripper_target_position:+.4f}m "
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
                packet, addr = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            payload = json.loads(packet.decode("utf-8"))

            # ── feedback_request：把当前关节角回传给发送端 ──
            if payload.get("type") == "feedback_request":
                feedback = {
                    "type": "feedback",
                    "joint_positions": self.latest_q.tolist(),
                    "timestamp": time.time(),
                }
                self.feedback_socket.sendto(
                    json.dumps(feedback, separators=(",", ":")).encode("utf-8"),
                    (addr[0], self.feedback_port),
                )
                continue

            joint_positions = np.asarray(payload["joint_positions"], dtype=np.float64)
            if joint_positions.shape != (ARM_JOINT_COUNT,):
                raise RuntimeError(
                    f"收到的关节角维度错误: {joint_positions.shape}，期望 {(ARM_JOINT_COUNT,)} / "
                    f"received joint angle has wrong shape: {joint_positions.shape}, expected {(ARM_JOINT_COUNT,)}"
                )
            gripper_value = payload.get("gripper_position")
            latest_packet = (joint_positions, int(payload["sequence"]), None if gripper_value is None else float(gripper_value))
        return latest_packet

    def run(self, render_hz: float = DEFAULT_RENDER_HZ) -> None:
        if render_hz <= 0:
            raise ValueError("render_hz 必须为正数 / render_hz must be a positive number")

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
                print(
                    "[warn] 超过 2 秒未收到新的关节角数据 / "
                    "no new joint-angle data received for more than 2 seconds"
                )

            time.sleep(render_period * 0.25)

    def shutdown(self) -> None:
        self.socket.close()
        self.feedback_socket.close()
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

    print()
    print("=" * 72)
    print("  Isaac Sim arm + ground + UDP joint-angle receiver")
    print("  Expected behavior: receive physical arm joint angles and")
    print("  drive the simulated arm in lockstep")
    print("  Gripper behavior: position targets directly control the gripper slide")
    print("  To stop: close the Isaac Sim window or press Ctrl+C")
    print("=" * 72)
    print(f"[receiver] udp://{DEFAULT_HOST}:{DEFAULT_PORT}")
    print(f"[asset] {ASSET_RELATIVE_PATH}")

    mirror = IsaacJointMirror()
    try:
        mirror.setup_isaac_sim()
        print("[仿真] Isaac Sim 已启动，地面和机械臂资产已加载")
        print("[sim] Isaac Sim started, ground plane and robot asset loaded")
        mirror.run()
    finally:
        print("[停止] 正在关闭接收与仿真...")
        print("[stopping] shutting down receiver and simulation...")
        mirror.shutdown()
        print("[完成] 已安全退出")
        print("[done] exited safely")


if __name__ == "__main__":
    main()
