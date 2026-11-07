#!/usr/bin/env python3
"""Isaac Sim 关节空间轨迹规划 UDP 发送端 / Isaac Sim joint-space trajectory sender.

功能概述：
1. 在终端输入末端期望位姿（位置 / 位置+姿态），通过 IK 求解得到关节角。
2. 在关节空间使用 MIN_JERK 插值，生成平滑且准确到达关节目标的轨迹。
3. 按轨迹点采样频率（默认 1000 Hz）逐点把关节角通过 UDP JSON 发送给
   `isaacsim_joint_receiver.py`，由仿真端的 drive 闭环平滑跟踪。
4. 同样支持 `q ...` 直发命令：直接规划一条关节空间直线轨迹（带时间剖面）
   从当前构型过渡到目标构型，避免位置阶跃。

与 `isaacsim_ik_sender.py` 的关键差别：
- IK 求解只发生在用户给出新目标时；中间是轨迹逐点回放。
- 关节角随时间按规划曲线变化（min-jerk），不再每帧直接写最新目标，
  从源头消除仿真端的"跳变"现象。

推荐运行方式：
    # 先启动仿真端：
    ./run_isaacsim_receiver.sh
    # 再启动本脚本：
    python3 isaacsim_traj_sender.py

输入格式（每行一条）：
    <x> <y> <z>                          (位置, 米; 姿态保持当前)
    <x> <y> <z> <roll> <pitch> <yaw>     (位置+姿态, 米/度)
    gripper <ratio>                      (单独更新夹爪, ratio ∈ [0, 1])
    q <j1> <j2> ... <j6>                 (直接发送关节角, 度)
    speed <scale>                        (速度比例 0.1–5.0, 默认 1.0)
    resync                               (读取仿真反馈)
    quit / :q                            退出
"""

from __future__ import annotations

import json
import select
import signal
import socket
import sys
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
import pinocchio as pin

# 把 third_party/reBotArm_control_py 加入 sys.path，兼容直接 python3 运行。
_THIRD_PARTY = Path(__file__).resolve().parents[1] / "third_party" / "reBotArm_control_py"
if str(_THIRD_PARTY) not in sys.path:
    sys.path.insert(0, str(_THIRD_PARTY))

from reBotArm_control_py.kinematics import (  # noqa: E402
    compute_ik,
    compute_fk,
    get_end_effector_frame_id,
    get_joint_count,
    get_joint_names,
    load_robot_model,
)
from reBotArm_control_py.trajectory import (  # noqa: E402
    TrajPlanParams,
    TrajProfile,
)
from reBotArm_control_py.trajectory.clik_tracker import (  # noqa: E402
    IKParams,
    JointTrajectoryPoint,
)

ARM_JOINT_COUNT = 6
DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 5005
DEFAULT_FEEDBACK_PORT = 5006
DEFAULT_FEEDBACK_TIMEOUT = 5.0          # 等待反馈的超时（秒）
DEFAULT_BROADCAST_HZ = 1000.0        # 轨迹点 UDP 广播频率
DEFAULT_TRAJ_DURATION = 3.0         # 默认轨迹时长（秒）
DEFAULT_SPEED_SCALE = 1.0           # 速度比例：1.0 = 使用默认时长，>1 更快
DEFAULT_JOINT_TOLERANCE = 1e-3      # 直发 q 模式下，目标与当前差距过小时跳过规划
DEFAULT_GRIPPER_MAX_OPENING_M = 0.045  # 夹爪完全打开时每指的滑动距离（米）；保守值，小于 USD upperLimit（两指均为 0.05）

_running = True


def _sigint_handler(signum, frame) -> None:
    del signum, frame
    global _running
    print("\n[traj-sender] 收到 Ctrl+C，准备退出...")
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class TrajSender:
    """IK → 关节空间轨迹规划 → UDP 逐点广播。"""

    def __init__(
        self,
        host: str = DEFAULT_HOST,
        port: int = DEFAULT_PORT,
    ) -> None:
        self.host = host
        self.port = port
        self.feedback_port = DEFAULT_FEEDBACK_PORT
        self.model = load_robot_model()
        self.data = self.model.createData()
        self.joint_names = get_joint_names(self.model)
        self.n_joints = get_joint_count()
        self.end_frame_id = get_end_effector_frame_id(self.model)

        # 当前关节配置（电机坐标 / URDF / Pinocchio 模型符号）。
        # IK 求解的 q_init、CLIK 跟踪的 q_start/q_end、以及广播后回填的
        # points[-1].q 都属于同一坐标系；发送阶段才取反成兼容的 UDP 协议坐标。
        self.q_prev = np.zeros(self.model.nq)
        self.gripper = 0.0
        self._synced = False

        self.traj_plan_params = TrajPlanParams(
            dt=1.0 / DEFAULT_BROADCAST_HZ,
            profile=TrajProfile.MIN_JERK,
            accel_ratio=0.25,
        )
        self.speed_scale = DEFAULT_SPEED_SCALE

        # 命令 socket（发送到接收端 5005）
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 反馈 socket（监听 5006，绑定以接收回传）
        self.feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.feedback_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.feedback_socket.bind(("0.0.0.0", self.feedback_port))
        self.feedback_socket.settimeout(0.0)
        self.sequence = 0

    # ---------- UDP 发送 ----------

    def _send(self, q_rad: np.ndarray, gripper: float) -> None:
        # gripper 入参约定：0~DEFAULT_GRIPPER_MAX_OPENING_M 米。如果调用方传入
        # 的值 > DEFAULT_GRIPPER_MAX_OPENING_M，会被解释为旧版"0~1 当米"的错误
        # 习惯用法，按 USD 行程上限裁剪到合理区间。
        gripper_m = float(np.clip(gripper, 0.0, DEFAULT_GRIPPER_MAX_OPENING_M))
        payload = {
            "sequence": self.sequence,
            "timestamp": time.time(),
            "joint_positions": q_rad.tolist(),
            "gripper_position": gripper_m,
        }
        packet = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(packet, (self.host, self.port))
        self.sequence += 1

    # ---------- 关节角反馈（从仿真端回读） ----------

    def _request_feedback(self) -> Optional[np.ndarray]:
        """优先请求仿真关节实测值；旧接收端回退到最近目标。

        UDP 协议保留旧资产约定 q_wire = -q_motor；接收端负责转换到
        所选 USD 的关节方向。返回值取反后可作为模型坐标的 q_prev。

        超时返回 None，禁止发送轨迹，直到 resync 成功。
        """
        request_payload = json.dumps({"type": "feedback_request"}, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(request_payload, (self.host, self.port))

        deadline = time.perf_counter() + DEFAULT_FEEDBACK_TIMEOUT
        while time.perf_counter() < deadline:
            if not _running:
                break
            try:
                packet, _ = self.feedback_socket.recvfrom(65535)
                payload = json.loads(packet.decode("utf-8"))
                if payload.get("type") == "feedback":
                    q_wire = np.asarray(payload.get("actual_joint_positions", payload["joint_positions"]), dtype=np.float64)
                    if q_wire.shape != (ARM_JOINT_COUNT,) or not np.isfinite(q_wire).all():
                        raise ValueError("仿真反馈关节角无效")
                    if "actual_joint_positions" not in payload:
                        print("[warn] 接收端仅提供最近目标，不是实测关节位置")
                    # 协议坐标 → 电机坐标（取反），再补零到 model.nq。
                    q_motor = -q_wire
                    q_model = np.concatenate(
                        [q_motor, np.zeros(max(0, self.model.nq - ARM_JOINT_COUNT))]
                    )
                    q_rad = q_motor[:ARM_JOINT_COUNT]
                    q_deg_str = "  ".join(f"{np.degrees(v):+7.2f}" for v in q_rad)
                    print(
                        f"[feedback] 收到仿真端关节反馈 (协议坐标): "
                        f"q_rad=[{'  '.join(f'{v:+.3f}' for v in q_wire)}]"
                    )
                    print(
                        f"[feedback] 已转换为电机坐标: "
                        f"q_rad=[{'  '.join(f'{v:+.3f}' for v in q_rad)}]  "
                        f"q_deg=[{q_deg_str}]"
                    )
                    return q_model
            except (BlockingIOError, socket.timeout):
                time.sleep(0.01)
                continue

        print("[warn] 等待反馈超时；请启动接收端后输入 resync，暂不发送运动命令")
        return None

    # ---------- 轨迹回放 ----------

    def _broadcast_trajectory(self, points: List[JointTrajectoryPoint]) -> None:
        """按轨迹点的绝对时间，逐点 UDP 广播；保证运动时长与规划一致。"""
        if not points:
            return

        # 采样间隔 = 1 / broadcast_hz；如果轨迹 dt 大于广播间隔，
        # 我们只发送那些相对起点的延迟 ≥ 一个广播周期的点，避免冗余发送。
        period = 1.0 / DEFAULT_BROADCAST_HZ
        start = time.perf_counter()
        next_deadline = 0.0

        for idx, pt in enumerate(points):
            if not _running:
                break
            # 既按"绝对时间"等时钟，也按"按顺序立刻发出"二选一：
            # 这里选立即按节奏发出，让仿真端的 drive 看到平滑目标序列。
            target_time = start + pt.time
            now = time.perf_counter()
            if target_time > now:
                time.sleep(target_time - now)

            if pt.time + 1e-9 < next_deadline:
                continue
            next_deadline = pt.time + period

            if not _running:
                break
            q_send = -pt.q[:ARM_JOINT_COUNT]
            self._send(q_send, self.gripper)
            self.q_prev[:] = pt.q
            if idx % 10 == 0 or not pt.ik_success:
                marker = "" if pt.ik_success else " !ik-fail"
                q_str = "  ".join(f"{v:+.3f}" for v in pt.q[:ARM_JOINT_COUNT])
                print(f"[traj] t={pt.time:5.2f}s  q=[{q_str}]{marker}")

    def _plan_joint_direct(
        self, q_start: np.ndarray, q_end: np.ndarray
    ) -> List[JointTrajectoryPoint]:
        """用最小加加速度曲线直接插值关节，准确到达指定关节角。"""
        duration = max(DEFAULT_TRAJ_DURATION / self.speed_scale, 0.05)
        n = max(1, int(np.ceil(duration / self.traj_plan_params.dt)))
        result = []
        for i in range(n + 1):
            u = i / n
            blend = 10*u**3 - 15*u**4 + 6*u**5
            result.append(JointTrajectoryPoint(
                time=duration*u, q=q_start + blend*(q_end-q_start), ik_success=True
            ))
        return result

    def _plan_pose(
        self, target_pos: np.ndarray, target_rot: Optional[np.ndarray]
    ) -> List[JointTrajectoryPoint]:
        """IK 解出终点后，在关节空间执行平滑轨迹。"""
        if not np.isfinite(target_pos).all() or (target_rot is not None and not np.isfinite(target_rot).all()):
            raise ValueError("目标位姿必须为有限数值")
        if target_rot is None:
            _, target_rot, _ = compute_fk(self.model, self.q_prev)
        result = compute_ik(
            q_init=self.q_prev, target_pos=target_pos, target_rot=target_rot,
            params=IKParams(max_iter=2000, damping=0.001),
        )
        if not result.success or not np.isfinite(result.q).all():
            raise ValueError(f"IK 求解失败 err={result.error:.2e}，未发送轨迹")
        print(f"[ik] success=True  err={result.error:.2e}  iter={result.iterations}")
        return self._plan_joint_direct(self.q_prev.copy(), result.q.copy())

    # ---------- 输入解析 ----------

    @staticmethod
    def _parse_pose(line: str) -> tuple[np.ndarray, Optional[np.ndarray]]:
        tokens = line.split()
        if len(tokens) not in (3, 6):
            raise ValueError(f"需要 3 或 6 个数，实际 {len(tokens)} 个")
        pos = np.array([float(x) for x in tokens[:3]])
        rot = None
        if len(tokens) == 6:
            rpy = np.radians([float(x) for x in tokens[3:6]])
            rot = pin.rpy.rpyToMatrix(*rpy)
        return pos, rot

    def _handle_command(self, line: str) -> Optional[str]:
        """返回 'quit' 表示退出，None 表示继续。"""
        line = line.strip()
        if not line:
            return None

        if line.lower() in {"quit", "exit", ":q"}:
            return "quit"

        if line.startswith("speed ") or line.startswith("speed\t"):
            speed = float(line.split(None, 1)[1])
            if not np.isfinite(speed) or not 0.1 <= speed <= 5.0:
                raise ValueError("speed 必须是 0.1–5.0 的有限数值")
            self.speed_scale = speed
            print(f"[traj-sender] speed_scale = {speed:.3f}")
            return None

        if line.startswith("gripper ") or line.startswith("gripper\t"):
            if not self._synced:
                raise RuntimeError("尚未同步仿真状态，请先执行 resync")
            ratio = float(line.split(None, 1)[1])
            if not np.isfinite(ratio):
                raise ValueError("夹爪比例必须为有限数值")
            ratio = float(np.clip(ratio, 0.0, 1.0))
            gripper_m = ratio * DEFAULT_GRIPPER_MAX_OPENING_M
            self.gripper = gripper_m
            # 立即广播一帧；既解决"等下次规划才生效"的延迟，也让"单独
            # 更新夹爪"的命令在不动 arm 时也能动 gripper。
            # 取上一次 joint 目标 q 的前 6 个当作静止参考（不触发任何臂规划）；
            # q_prev 是电机坐标，广播前取反成协议坐标（q_wire = -q_motor）。
            self._send(-self.q_prev[:ARM_JOINT_COUNT], gripper_m)
            print(f"夹爪比例已更新为 {ratio:.2f} → {gripper_m * 1000:.1f} mm（已立即广播）")
            return None

        if line.strip() == "resync":
            q_new = self._request_feedback()
            if q_new is None:
                return None
            self.q_prev[:] = q_new
            self._synced = True
            q_rad = self.q_prev[:ARM_JOINT_COUNT]
            q_deg_str = "  ".join(f"{np.degrees(v):+7.2f}" for v in q_rad)
            print(f"已同步 q_prev ← q_deg=[{q_deg_str}]")
            return None

        if not self._synced:
            raise RuntimeError("尚未同步仿真状态，请先执行 resync")

        if line.startswith("q ") or line.startswith("q\t"):
            q_deg = np.array([float(x) for x in line[1:].split()], dtype=np.float64)
            if q_deg.shape != (ARM_JOINT_COUNT,):
                raise ValueError(f"q 需要 {ARM_JOINT_COUNT} 个数，实际 {q_deg.shape[0]} 个")
            # 用户输入单位为度（电机坐标），直接转弧度送进 planner；
            # 广播阶段统一取反为协议坐标。
            q_target_model = np.zeros(self.model.nq)
            q_target_model[:ARM_JOINT_COUNT] = np.radians(q_deg)
            q_arm = q_target_model[:ARM_JOINT_COUNT]
            if not np.isfinite(q_arm).all():
                raise ValueError("关节角必须为有限数值")
            if np.any(q_arm < self.model.lowerPositionLimit[:ARM_JOINT_COUNT]) or np.any(q_arm > self.model.upperPositionLimit[:ARM_JOINT_COUNT]):
                raise ValueError("关节角超出 URDF 限位，未发送轨迹")

            if float(np.linalg.norm(q_target_model[:ARM_JOINT_COUNT]
                                    - self.q_prev[:ARM_JOINT_COUNT])) < DEFAULT_JOINT_TOLERANCE:
                print("[traj-sender] 目标与当前构型差距过小，跳过规划。")
                return None

            points = self._plan_joint_direct(self.q_prev, q_target_model)
            self._broadcast_trajectory(points)
            q_rad = self.q_prev[:ARM_JOINT_COUNT]
            q_deg_str = "  ".join(f"{np.degrees(v):+7.2f}" for v in q_rad)
            state = "完成" if _running else "已中断"
            print(f"[traj-sender] 关节轨迹{state}，最后发送 q_deg=[{q_deg_str}]")
            return None

        # 默认按位姿输入解析
        target_pos, target_rot = self._parse_pose(line)
        points = self._plan_pose(target_pos, target_rot)
        self._broadcast_trajectory(points)
        return None

    # ---------- 主循环 ----------

    def run(self) -> None:
        print("=" * 72)
        print("  Isaac Sim 关节空间轨迹规划发送端")
        print(f"  机器人     : {self.model.name}")
        print(f"  末端帧     : id={self.end_frame_id}")
        print(f"  目标 (UDP) : {self.host}:{self.port}")
        print(f"  反馈 (UDP) : {self.host}:{self.feedback_port}")
        print(f"  轨迹采样   : dt={self.traj_plan_params.dt:.3f}s, "
              f"profile={self.traj_plan_params.profile.value}, "
              f"accel_ratio={self.traj_plan_params.accel_ratio:.2f}")
        print(f"  广播频率   : {DEFAULT_BROADCAST_HZ:.0f} Hz")
        print("  输入位姿 (每行一条):")
        print("    x y z                       (位置, 米; 姿态保持当前)")
        print("    x y z r p y                 (位置+姿态, 米/度)")
        print("    gripper <0~1>                (单独更新夹爪, 立即广播)")
        print("    q j1 j2 j3 j4 j5 j6         (直接发关节角, 度)")
        print("    speed <scale>                (速度比例 0.1–5.0，默认 1.0)")
        print("    resync                       (读取仿真关节反馈；旧接收端仅返回目标)")
        print("  退出: Ctrl+C / quit / :q")
        print("=" * 72)

        # ── 启动时先从仿真端读取当前关节角，初始化 q_prev ──
        print("[traj-sender] 正在从仿真端请求当前关节角...")
        q_init = self._request_feedback()
        if q_init is not None:
            self.q_prev[:] = q_init
            self._synced = True

        while _running:
            # 用 select 等用户输入，给轨迹回放让出 CPU
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                continue
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                print("\n[traj-sender] 输入结束，退出。")
                break
            try:
                ret = self._handle_command(line)
            except Exception as exc:
                print(f"[traj-sender] 错误: {exc}")
                continue
            if ret == "quit":
                print("[traj-sender] 收到退出指令。")
                break

        self.socket.close()
        self.feedback_socket.close()


def main() -> None:
    sender = TrajSender()
    sender.run()


if __name__ == "__main__":
    main()