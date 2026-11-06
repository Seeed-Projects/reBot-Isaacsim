#!/usr/bin/env python3
"""RS arm gravity compensation, gripper hand-assist, and Isaac Sim mirror.

Default: model-only check, no CAN. --live enables real motors and requires
calibrated open/closed motor positions in radians. Stop with Ctrl+C: motors are
disabled without returning home; the arm/gripper can lose support.

Assist law and defaults follow Yang-Ci/ReBot_Arm_DigitalTwin_RS. Unlike a
position command, this is hand guidance: at rest it applies no assist torque.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import select
import signal
import socket
import sys
import threading
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "third_party" / "reBotArm_control_py"))
from reBotArm_control_py.actuator import RebotArm
from reBotArm_control_py.controllers import GravityCompensation
from reBotArm_control_py.dynamics import compute_generalized_gravity, load_dynamics_model

ARM_NAMES = tuple(f"joint{i}" for i in range(1, 7))
READ_TIMEOUT_MS = 200
MAX_SAMPLE_AGE = 0.5
LIMIT_MARGIN = 0.12
ASSIST_TORQUE = 0.04
ASSIST_KD = 0.001
SPEED_THRESHOLD = 0.02
SPEED_FULL = 0.22
SPEED_LIMIT = 0.8
BREAKAWAY_FRACTION = 0.30
GRIPPER_TARGET_RATE = 0.1  # rad/s, deliberately slower than the reference UI
GRIPPER_MAX_LEAD = 0.02   # rad: limit commanded error while waiting for feedback


def assist_torque(position, velocity, closed, opened):
    """Hand-assist: no drive at rest, excessive speed, or into an endpoint."""
    if not all(math.isfinite(v) for v in (position, velocity, closed, opened)):
        raise ValueError("夹爪输入不是有限数值")
    lower, upper = sorted((closed, opened))
    if upper - lower <= 2 * LIMIT_MARGIN:
        raise ValueError("夹爪校准行程过小")
    speed = abs(velocity)
    if speed <= SPEED_THRESHOLD or speed >= SPEED_LIMIT:
        return 0.0
    direction = 1.0 if velocity > 0 else -1.0
    if position <= lower + LIMIT_MARGIN and direction < 0:
        return 0.0
    if position >= upper - LIMIT_MARGIN and direction > 0:
        return 0.0
    ramp = min(1.0, (speed - SPEED_THRESHOLD) / (SPEED_FULL - SPEED_THRESHOLD))
    return direction * ASSIST_TORQUE * max(ramp, BREAKAWAY_FRACTION)


def gripper_sim_position(position, closed, opened):
    """Map the calibrated real opening to 0..0.045 m per simulated finger."""
    if not all(math.isfinite(v) for v in (position, closed, opened)):
        raise ValueError("夹爪输入不是有限数值")
    if abs(opened - closed) <= 2 * LIMIT_MARGIN:
        raise ValueError("夹爪校准行程过小")
    ratio = max(0.0, min(1.0, (position - closed) / (opened - closed)))
    return ratio * 0.045


def read_position(motor):
    """Explicit fresh RS mechPos: failures must not be interpreted as zero."""
    value = float(motor.robstride_get_param_f32(0x7019, READ_TIMEOUT_MS))
    if not math.isfinite(value):
        raise RuntimeError("电机位置反馈无效")
    return value


def known_conflicting_processes():
    names = ("joint_reader_sender.py", "gravity_joint_sender.py",
             "9_gravity_compensation.py", "gravity_gripper_assist.py")
    ancestors = {os.getpid()}
    parent = os.getppid()
    while parent > 1 and parent not in ancestors:
        ancestors.add(parent)
        try:
            stat = Path(f"/proc/{parent}/stat").read_text()
            parent = int(stat[stat.rfind(")") + 2:].split()[1])
        except (OSError, ValueError, IndexError):
            break
    found = []
    for proc in Path("/proc").iterdir():
        if not proc.name.isdigit() or int(proc.name) in ancestors:
            continue
        try:
            args = (proc / "cmdline").read_bytes().split(b"\0")
            if any(Path(a.decode(errors="ignore")).name in names for a in args):
                found.append(proc.name)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            pass
    return found


class AssistedGravity(GravityCompensation):
    """Example/9 gravity feedforward plus bounded hand assistance on J7."""
    def __init__(self, robot, closed, opened):
        super().__init__(robot)
        self.closed, self.opened = closed, opened
        self._lock = threading.Lock()
        self._feedback = None
        self._sample_time = 0.0
        self._feedback_fault = None
        self._stop_feedback = threading.Event()
        self._feedback_thread = None
        self._last_gripper = None
        self._last_gripper_time = None
        self._filtered_velocity = 0.0
        self._hold_q = None
        self._initial_kp = None
        self._initial_kd = None
        self._transition_start = None
        self._gripper_mode = "assist"
        self._gripper_goal = None
        self._gripper_command = None
        self._gripper_command_time = None

    def _read_snapshot(self, *, verify_start_range=False):
        motors = [self.rebotarm._motor_map[n] for n in ARM_NAMES]
        q = np.array([read_position(m) for m in motors])
        grip = read_position(self.rebotarm._motor_map["gripper"])
        if verify_start_range and not min(self.closed, self.opened) - 0.05 <= grip <= max(self.closed, self.opened) + 0.05:
            raise RuntimeError(f"夹爪反馈 {grip:.3f} rad 超出校准范围")
        stamp = time.monotonic()
        velocity = 0.0
        if self._last_gripper_time is not None:
            dt = stamp - self._last_gripper_time
            if dt > 0:
                velocity = (grip - self._last_gripper) / dt
        self._last_gripper, self._last_gripper_time = grip, stamp
        self._filtered_velocity = 0.7 * self._filtered_velocity + 0.3 * velocity
        with self._lock:
            self._feedback = (q.copy(), grip, self._filtered_velocity)
            self._sample_time = stamp

    def _poll_feedback(self):
        while not self._stop_feedback.is_set():
            try:
                self._read_snapshot()
            except Exception as exc:
                with self._lock:
                    self._feedback_fault = f"{type(exc).__name__}: {exc}"
                return
            self._stop_feedback.wait(0.05)

    def snapshot(self):
        with self._lock:
            if self._feedback_fault:
                raise RuntimeError("电机反馈失败: " + self._feedback_fault)
            if self._feedback is None or time.monotonic() - self._sample_time > MAX_SAMPLE_AGE:
                raise RuntimeError("电机反馈过期")
            q, grip, velocity = self._feedback
            return q.copy(), grip, velocity

    def start(self):
        robot = self.rebotarm
        if not robot.has_gripper or list(robot.arm.joint_names) != list(ARM_NAMES):
            raise RuntimeError("需要 RS 六关节和已配置的夹爪")
        robot.connect()
        try:
            # Read the real positions and validate calibration before any mode
            # change or enable command. No set-zero command is ever sent.
            self._read_snapshot(verify_start_range=True)
            q, grip, _ = self.snapshot()
            tau = np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float)
            if len(tau) != 6 or not np.isfinite(tau).all():
                raise RuntimeError("重力力矩无效")
            self._initial_kp = np.asarray(robot.arm._mit_kp, dtype=float).copy()
            self._initial_kd = np.asarray(robot.arm._mit_kd, dtype=float).copy()
            if self._initial_kp.shape != (6,) or self._initial_kd.shape != (6,):
                raise RuntimeError("关节保持增益无效")
            if not robot.arm.mode_mit() or not robot.gripper.mode_mit():
                raise RuntimeError("MIT 模式设置失败")
            robot.disable_all()
            time.sleep(0.1)
            # Mode switching can take long enough for the original reading to
            # go stale. Refresh before preloading position-hold targets.
            self._read_snapshot(verify_start_range=True)
            q, grip, _ = self.snapshot()
            tau = np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float)
            if len(tau) != 6 or not np.isfinite(tau).all():
                raise RuntimeError("重力力矩无效")
            self._hold_q = q.copy()
            # Reference implementation: preload the measured pose and gravity
            # before enabling. Start stiff and blend to compliant gains.
            for i, name in enumerate(ARM_NAMES):
                robot._motor_map[name].send_mit(float(q[i]), 0.0,
                    float(self._initial_kp[i]), float(self._initial_kd[i]), float(tau[i]))
            robot._motor_map["gripper"].send_mit(float(grip), 0.0,
                float(robot.gripper._mit_kp[0]), float(robot.gripper._mit_kd[0]), 0.0)
            for name in (*ARM_NAMES, "gripper"):
                robot._motor_map[name].enable()
            # Some motors do not retain a pre-enable command.
            for i, name in enumerate(ARM_NAMES):
                robot._motor_map[name].send_mit(float(q[i]), 0.0,
                    float(self._initial_kp[i]), float(self._initial_kd[i]), float(tau[i]))
            robot._motor_map["gripper"].send_mit(float(grip), 0.0,
                float(robot.gripper._mit_kp[0]), float(robot.gripper._mit_kd[0]), 0.0)
            self._read_snapshot(verify_start_range=True)
            self._hold_q = self.snapshot()[0]
            self._transition_start = time.monotonic()
            self._feedback_thread = threading.Thread(target=self._poll_feedback, daemon=True)
            self._feedback_thread.start()
            robot.start_control_loop(self._loop_cb, rate=125.0)
            self._running = True
        except BaseException:
            self._stop_feedback.set()
            if self._feedback_thread:
                self._feedback_thread.join(timeout=2)
            robot.disconnect()
            raise

    def _loop_cb(self, robot, dt):
        del dt
        q, grip, velocity = self.snapshot()
        tau = np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float)
        if len(tau) != 6 or not np.isfinite(tau).all():
            raise RuntimeError("重力力矩无效")
        elapsed = max(0.0, time.monotonic() - self._transition_start)
        ratio = min(1.0, elapsed / 0.5)
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        target = self._hold_q + blend * (q - self._hold_q)
        kp = self._initial_kp + blend * (1.5 - self._initial_kp)
        kd = self._initial_kd + blend * (1.0 - self._initial_kd)
        for i, name in enumerate(ARM_NAMES):
            robot._motor_map[name].send_mit(float(target[i]), 0.0,
                float(kp[i]), float(kd[i]), float(tau[i]))
        # Hand motion slightly outside the measured endpoints must not stop
        # arm gravity compensation. At an out-of-range reading, command zero
        # gripper assist and no gripper position target until it returns.
        outside = not min(self.closed, self.opened) - 0.05 <= grip <= max(self.closed, self.opened) + 0.05
        if outside:
            robot._motor_map["gripper"].send_mit(grip, 0.0, 0.0, ASSIST_KD, 0.0)
        elif blend < 1.0:
            robot._motor_map["gripper"].send_mit(grip, 0.0,
                float(robot.gripper._mit_kp[0]), float(robot.gripper._mit_kd[0]), 0.0)
        else:
            with self._lock:
                mode = self._gripper_mode
                if mode != "assist":
                    now = time.monotonic()
                    dt = min(max(now - self._gripper_command_time, 0.0), 0.05)
                    step = GRIPPER_TARGET_RATE * dt
                    remaining = self._gripper_goal - self._gripper_command
                    next_q = self._gripper_command + max(-step, min(step, remaining))
                    self._gripper_command = max(grip - GRIPPER_MAX_LEAD,
                                                min(grip + GRIPPER_MAX_LEAD, next_q))
                    self._gripper_command_time = now
                    target_grip = self._gripper_command
            if mode == "assist":
                drive = assist_torque(grip, velocity, self.closed, self.opened)
                robot._motor_map["gripper"].send_mit(grip, 0.0, 0.0, ASSIST_KD, drive)
            else:
                robot._motor_map["gripper"].send_mit(target_grip, 0.0,
                    float(robot.gripper._mit_kp[0]), float(robot.gripper._mit_kd[0]), 0.0)

    def set_gripper_mode(self, mode):
        if mode not in {"assist", "open", "close", "hold"}:
            raise ValueError("命令须为 open、close、hold 或 assist")
        _, grip, _ = self.snapshot()
        if mode != "assist" and not min(self.closed, self.opened) - 0.05 <= grip <= max(self.closed, self.opened) + 0.05:
            raise ValueError("夹爪超出校准范围，请先手动移回范围内")
        # No zero-setting and no assumed direction: these endpoints are supplied
        # from this specific gripper's measured calibration.
        goal = {"open": self.opened, "close": self.closed, "hold": grip,
                "assist": grip}[mode]
        with self._lock:
            self._gripper_mode = mode
            self._gripper_goal = goal
            self._gripper_command = grip
            self._gripper_command_time = time.monotonic()
        return goal

    def end(self):
        self._stop_feedback.set()
        if self._feedback_thread:
            self._feedback_thread.join(timeout=2)
        super().end()


def check_model(robot):
    if robot.hardware_yaml != "rebotarm_rs.yaml":
        raise RuntimeError(f"配置不是 RS: {robot.hardware_yaml}")
    if not robot.has_gripper or list(robot.arm.joint_names) != list(ARM_NAMES):
        raise RuntimeError("配置需要六个 arm 关节及夹爪")
    model = load_dynamics_model()
    torque = np.asarray(compute_generalized_gravity(q=np.zeros(6)))
    if model.nq < 6 or len(torque) < 6 or not np.isfinite(torque).all():
        raise RuntimeError("动力学模型检查失败")
    print(f"模型 {model.name}, nq={model.nq}, 夹爪已配置")
    print("零位估算力矩 (N·m):", np.round(torque[:6], 3))
    print("动力学模型未考虑额外负载，夹爪指关节按零位处理。")


def run_live(robot, closed, opened):
    if not all(math.isfinite(v) for v in (closed, opened)) or abs(opened - closed) <= 2 * LIMIT_MARGIN:
        raise ValueError("请提供有效的真实夹爪开、合校准值（弧度）")
    conflicts = known_conflicting_processes()
    if conflicts:
        raise RuntimeError("已有真机读写进程，请先停止: " + ", ".join(conflicts))
    ctrl = AssistedGravity(robot, closed, opened)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stopping = threading.Event()
    def on_signal(signum, frame):
        del signum, frame
        stopping.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    # Closing the terminal sends SIGHUP; run the same shutdown path instead of
    # abandoning the control loop without requesting motor disable.
    signal.signal(signal.SIGHUP, on_signal)
    try:
        print("即将使能真实机械臂和夹爪；Ctrl+C 停止并失能，不自动回零。", flush=True)
        ctrl.start()
        print("重力补偿及夹爪手动助力已启用。", flush=True)
        print("命令: open / close / hold / assist / quit（回车发送）", flush=True)
        seq = 0
        read_input = True
        while not stopping.wait(1 / 60):
            if not robot.control_loop_active:
                raise RuntimeError("控制线程停止")
            q, grip, velocity = ctrl.snapshot()
            payload = {"sequence": seq, "timestamp": time.time(),
                       "joint_positions": (-q).tolist(),
                       "gripper_position": gripper_sim_position(grip, closed, opened)}
            sock.sendto(json.dumps(payload).encode(), ("127.0.0.1", 5005))
            if read_input and select.select([sys.stdin], [], [], 0)[0]:
                command = sys.stdin.readline()
                if not command:
                    read_input = False
                else:
                    command = command.strip().lower()
                    if command in {"quit", "exit", "q"}:
                        stopping.set()
                    elif command:
                        try:
                            goal = ctrl.set_gripper_mode(command)
                            print(f"夹爪模式: {command}；参考目标 {goal:.3f} rad", flush=True)
                        except ValueError as exc:
                            print(str(exc), flush=True)
            if seq % 60 == 0:
                print("真机角度(deg):", np.round(np.degrees(q), 2),
                      "夹爪(rad):", round(grip, 3),
                      "夹爪速度(rad/s):", round(velocity, 3), flush=True)
                if not min(closed, opened) - 0.05 <= grip <= max(closed, opened) + 0.05:
                    print("夹爪超出校准范围：暂停夹爪助力和位置控制，机械臂补偿继续。", flush=True)
            seq += 1
    finally:
        ctrl.end()
        sock.close()
        print("控制已停止；失能后机械臂和夹爪可能失去支撑。", flush=True)


def main():
    parser = argparse.ArgumentParser(description="RS 机械臂重力补偿、夹爪助力与 Isaac Sim 同步")
    parser.add_argument("--live", action="store_true", help="连接并控制真实机械臂")
    parser.add_argument("--acknowledge-j2-drift", action="store_true",
                        help="确认 J2 下垂问题尚未解决，仍需进行受控测试")
    parser.add_argument("--closed-rad", type=float, help="真实夹爪完全闭合电机角度（弧度）")
    parser.add_argument("--open-rad", type=float, help="真实夹爪完全打开电机角度（弧度）")
    args = parser.parse_args()
    robot = RebotArm()
    check_model(robot)
    if not args.live:
        print("仅模型预检查，未连接 CAN；--live 才会控制真机。")
        return
    if args.closed_rad is None or args.open_rad is None:
        parser.error("--live 需要 --closed-rad 和 --open-rad 实测校准值")
    if not args.acknowledge_j2_drift:
        parser.error("实机曾观察到 J2 下垂，原因未确认；禁止直接运行。")
    run_live(robot, args.closed_rad, args.open_rad)


if __name__ == "__main__":
    main()
