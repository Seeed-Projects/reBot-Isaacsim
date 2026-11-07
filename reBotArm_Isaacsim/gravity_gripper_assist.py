#!/usr/bin/env python3
"""RS arm gravity compensation, gripper hand-assist, and Isaac Sim mirror.

Default: controls real motors. --check performs a model-only check without CAN.
The default gripper range is closed=0 and open=5 radians after zero calibration.
Home, quit and the first Ctrl+C request a smooth return. A second Ctrl+C stops
immediately and disables motors; the arm/gripper can lose support.

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
from reBotArm_control_py.kinematics import load_robot_model

ARM_NAMES = tuple(f"joint{i}" for i in range(1, 7))
READ_TIMEOUT_MS = 200
MAX_SAMPLE_AGE = 0.5
LIMIT_MARGIN = 0.12
DEFAULT_GRIPPER_CLOSED = 0.0
DEFAULT_GRIPPER_OPEN = 5.0
# Experimental reference scales; physical torque still requires calibration.
GRAVITY_SCALE = np.array([1.0, 0.92, 0.98, 1.0, 1.0, 1.0])
ASSIST_TORQUE = 0.04
ASSIST_KD = 0.001
SPEED_THRESHOLD = 0.02
SPEED_FULL = 0.22
SPEED_LIMIT = 0.8
BREAKAWAY_FRACTION = 0.30
GRIPPER_TARGET_RATE = 0.1  # rad/s, deliberately slower than the reference UI
GRIPPER_MAX_LEAD = 0.02   # rad: limit commanded error while waiting for feedback
HOME_MAX_VELOCITY = math.radians(30.0)
HOME_MAX_ACCELERATION = math.radians(25.0)
HOME_MAX_JERK = math.radians(60.0)
HOME_TARGET_MOTION_SECONDS = 4.0
HOME_TRACKING_ERROR = math.radians(3.0)
HOME_SETTLE_TOLERANCE = math.radians(0.75)
HOME_PREP_SECONDS = 0.8
HOME_SETTLE_SECONDS = 0.5
HOME_SETTLE_TIMEOUT = 5.0
HOME_MAX_LOOP_GAP = 0.15
HOME_MAX_START_SPEED = math.radians(2.0)
HOME_STILL_SECONDS = 0.6
HOME_WAIT_STILL_TIMEOUT = 15.0


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


def minimum_jerk(t):
    t = min(1.0, max(0.0, float(t)))
    return t ** 3 * (10.0 + t * (-15.0 + 6.0 * t))


class HomeNotStationary(RuntimeError):
    """The measured arm is still moving; keep waiting for it to settle."""


class HomePlan:
    """Bounded minimum-jerk trajectory with feedback tracking and settling."""
    def __init__(self, start_q, started_at):
        self.start_q = np.asarray(start_q, dtype=float).copy()
        if self.start_q.shape != (6,) or not np.isfinite(self.start_q).all():
            raise ValueError("回零起点无效")
        distance = float(np.max(np.abs(self.start_q)))
        # SDK safe_home uses a fixed minimum-jerk trajectory with a duration
        # of 2 * distance / max_vel. Bound acceleration and jerk as well.
        self.duration = max(HOME_TARGET_MOTION_SECONDS,
                            2.0 * distance / HOME_MAX_VELOCITY,
                            math.sqrt((10 / math.sqrt(3)) * distance / HOME_MAX_ACCELERATION),
                            (60 * distance / HOME_MAX_JERK) ** (1 / 3))
        self.distance = distance
        self.started_at = float(started_at)
        self.settled_at = None
        self.last_step_at = self.started_at
        self.motion_elapsed = 0.0
        self.target = self.start_q.copy()
        self.finished_at = None
        self.done = False

    def step(self, now, measured_q):
        q = np.asarray(measured_q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise RuntimeError("回零反馈无效")
        previous = self.last_step_at
        dt = float(now) - previous
        if dt < 0 or dt > HOME_MAX_LOOP_GAP:
            raise RuntimeError("控制循环延迟过长，停止回零")
        self.last_step_at = float(now)
        elapsed = max(0.0, float(now) - self.started_at)
        gain_blend = minimum_jerk(elapsed / HOME_PREP_SECONDS)
        error = float(np.max(np.abs(self.target - q)))
        if error > HOME_TRACKING_ERROR:
            worst = int(np.argmax(np.abs(self.target - q)))
            raise RuntimeError(f"J{worst + 1} 跟踪误差 {math.degrees(error):.2f}°（目标 {math.degrees(self.target[worst]):.2f}°，反馈 {math.degrees(q[worst]):.2f}°）超过 3°；停止回零")

        motion_dt = min(0.02, max(0.0, now - max(previous, self.started_at + HOME_PREP_SECONDS)))
        if motion_dt > 0 and self.motion_elapsed < self.duration:
            # Fixed-rate SDK-style trajectory: no adaptive time scaling.
            # A following error is handled by the guard above, not by forcing
            # a delayed joint to catch up or sending an unconditional zero.
            self.motion_elapsed = (self.duration if self.distance < 1e-9 else
                                   min(self.duration, self.motion_elapsed + motion_dt))
            self.target = self.start_q * (1.0 - minimum_jerk(self.motion_elapsed / self.duration))

        progress = min(1.0, self.motion_elapsed / self.duration)
        if progress >= 1.0:
            if self.finished_at is None:
                self.finished_at = now
            if np.max(np.abs(q)) <= HOME_SETTLE_TOLERANCE:
                if self.settled_at is None:
                    self.settled_at = now
                self.done = now - self.settled_at >= HOME_SETTLE_SECONDS
            else:
                self.settled_at = None
            if not self.done and now - self.finished_at > HOME_SETTLE_TIMEOUT:
                raise RuntimeError("回零未稳定到位，停止回零")
        elif elapsed > HOME_PREP_SECONDS + max(20.0, 4.0 * self.duration):
            raise RuntimeError("回零耗时过长，停止回零")
        return self.target.copy(), gain_blend


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
        self._last_arm_q = None
        self._last_arm_time = None
        self._arm_velocity = None
        self._hold_q = None
        self._initial_kp = None
        self._initial_kd = None
        self._transition_start = None
        self._gripper_mode = "assist"
        self._gripper_goal = None
        self._gripper_command = None
        self._gripper_command_time = None
        self._home_plan = None
        self._home_hold_q = None
        self._home_hold_kp = None
        self._home_hold_kd = None
        self._home_start_kp = None
        self._home_start_kd = None
        self._home_status = "idle"
        self._home_message = ""

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
            if self._last_arm_time is not None and stamp > self._last_arm_time:
                self._arm_velocity = (q - self._last_arm_q) / (stamp - self._last_arm_time)
            self._last_arm_q, self._last_arm_time = q.copy(), stamp
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
            with self._lock:
                is_homing = self._home_plan is not None
            # Keep normal gravity/assist polling unchanged; sample more often
            # only during home so the trajectory sees less stale feedback.
            self._stop_feedback.wait(0.015 if is_homing else 0.05)

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
            tau = (np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float) * GRAVITY_SCALE)
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
            tau = (np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float) * GRAVITY_SCALE)
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
        tau = (np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float) * GRAVITY_SCALE)
        if len(tau) != 6 or not np.isfinite(tau).all():
            raise RuntimeError("重力力矩无效")
        elapsed = max(0.0, time.monotonic() - self._transition_start)
        ratio = min(1.0, elapsed / 0.5)
        blend = ratio * ratio * (3.0 - 2.0 * ratio)
        target = self._hold_q + blend * (q - self._hold_q)
        kp = self._initial_kp + blend * (1.5 - self._initial_kp)
        kd = self._initial_kd + blend * (1.0 - self._initial_kd)
        with self._lock:
            plan = self._home_plan
            if plan is not None:
                try:
                    target, home_blend = plan.step(time.monotonic(), q)
                    kp = self._home_start_kp + home_blend * (self._initial_kp - self._home_start_kp)
                    kd = self._home_start_kd + home_blend * (self._initial_kd - self._home_start_kd)
                    if plan.done and self._home_status != "done":
                        self._home_status = "done"
                        self._home_message = f"回零已到位（最大绝对误差 ≤ {math.degrees(HOME_SETTLE_TOLERANCE):.2f}°），准备停止并失能。"
                except RuntimeError as exc:
                    # SDK-style stop: freeze the latest measured pose, keep the
                    # current holding gains and gravity feedforward, never send
                    # a zero target or drop into compliant hand-guidance.
                    gain_blend = minimum_jerk((time.monotonic() - plan.started_at) / HOME_PREP_SECONDS)
                    self._home_hold_kp = self._home_start_kp + gain_blend * (self._initial_kp - self._home_start_kp)
                    self._home_hold_kd = self._home_start_kd + gain_blend * (self._initial_kd - self._home_start_kd)
                    self._home_hold_q = q.copy()
                    self._home_plan = None
                    self._home_status = "aborted"
                    self._home_message = str(exc)
                    target = self._home_hold_q.copy()
                    kp, kd = self._home_hold_kp.copy(), self._home_hold_kd.copy()
            elif self._home_hold_q is not None:
                # Avoid an accumulating position error if the arm is displaced
                # while an operator handles a failed home; remain enabled.
                if np.max(np.abs(self._home_hold_q - q)) > HOME_TRACKING_ERROR:
                    self._home_hold_q = q.copy()
                target = self._home_hold_q.copy()
                kp, kd = self._home_hold_kp.copy(), self._home_hold_kd.copy()
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

    def home_stationary(self):
        with self._lock:
            return (self._arm_velocity is not None
                    and time.monotonic() - self._sample_time < 0.25
                    and np.isfinite(self._arm_velocity).all()
                    and np.max(np.abs(self._arm_velocity)) <= HOME_MAX_START_SPEED)

    def request_home(self):
        if time.monotonic() - self._transition_start < 0.5:
            raise RuntimeError("重力补偿仍在启动，请稍后再回零")
        model = load_robot_model()
        lower = np.asarray(model.lowerPositionLimit[:6], dtype=float)
        upper = np.asarray(model.upperPositionLimit[:6], dtype=float)
        self.snapshot()  # Propagate any feedback fault before taking the lock.
        with self._lock:
            if self._feedback_fault or self._feedback is None or time.monotonic() - self._sample_time >= 0.25:
                raise HomeNotStationary("反馈过期或不可用；等待新反馈")
            q, _, _ = self._feedback
            q = q.copy()
            if (not np.isfinite(q).all() or np.any(q < lower - 0.03)
                    or np.any(q > upper + 0.03) or np.any(lower > 0) or np.any(upper < 0)):
                raise RuntimeError("实际角度或零位超出模型限位，不能回零")
            if self._home_plan is not None and not self._home_plan.done:
                raise RuntimeError("正在回零")
            if self._arm_velocity is None or not np.isfinite(self._arm_velocity).all():
                raise HomeNotStationary("缺少有效速度反馈；等待机械臂保持静止")
            peak_speed_deg = float(np.degrees(np.max(np.abs(self._arm_velocity))))
            if peak_speed_deg > math.degrees(HOME_MAX_START_SPEED):
                raise HomeNotStationary(f"关节仍在运动（最大约 {peak_speed_deg:.1f}°/s）；等待机械臂保持静止")
            if (not np.isfinite(self._initial_kp).all() or not np.isfinite(self._initial_kd).all()
                    or np.any(self._initial_kp < 0) or np.any(self._initial_kd < 0)):
                raise RuntimeError("关节回零保持增益无效")
            plan = HomePlan(q, time.monotonic())
            if self._home_hold_q is None:
                self._home_start_kp = np.full(6, 1.5)
                self._home_start_kd = np.full(6, 1.0)
            else:
                self._home_start_kp = self._home_hold_kp.copy()
                self._home_start_kd = self._home_hold_kd.copy()
            self._home_hold_q = None
            self._home_plan = plan
            self._home_status = "running"
            self._home_message = "开始平稳回零"
        return plan.duration

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
        # The SDK's end() now holds position with motors enabled. This example
        # promises disconnect/disable on exit, so use the explicit robot API.
        if self._running:
            self.rebotarm.disconnect()
            self._running = False


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


class StopRequests:
    """First Ctrl+C requests home; a second or an external termination stops."""
    def __init__(self):
        self.graceful = threading.Event()
        self.immediate = threading.Event()

    def on_signal(self, signum, frame):
        del frame
        if signum == signal.SIGINT and not self.graceful.is_set():
            self.graceful.set()
        else:
            self.immediate.set()


def run_live(robot, closed, opened):
    if not all(math.isfinite(v) for v in (closed, opened)) or abs(opened - closed) <= 2 * LIMIT_MARGIN:
        raise ValueError("请提供有效的真实夹爪开、合校准值（弧度）")
    conflicts = known_conflicting_processes()
    if conflicts:
        raise RuntimeError("已有真机读写进程，请先停止: " + ", ".join(conflicts))
    ctrl = AssistedGravity(robot, closed, opened)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    requests = StopRequests()
    stopping = requests.immediate
    signal.signal(signal.SIGINT, requests.on_signal)
    signal.signal(signal.SIGTERM, requests.on_signal)
    # Do not start motion on terminal hangup or an external termination signal.
    signal.signal(signal.SIGHUP, requests.on_signal)
    try:
        print("即将使能真实机械臂和夹爪；home/quit/第一次 Ctrl+C 平稳回零；回零过程中再次 Ctrl+C 请求停止并失能。", flush=True)
        ctrl.start()
        print("重力补偿及夹爪手动助力已启用。", flush=True)
        print("命令: home（平稳回零） / cancel（取消等待） / open / close / hold / assist / quit", flush=True)
        seq = 0
        read_input = True
        last_home_status = "idle"
        exit_home_started = False
        home_pending = False
        pending_since = None
        still_since = None
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
                        requests.graceful.set()
                    elif command == "home":
                        # Demo 9 semantics: home completes by disconnecting,
                        # exactly as quit / the first Ctrl+C do.
                        requests.graceful.set()
                    elif command == "cancel":
                        if home_pending:
                            home_pending = False
                            requests.graceful.clear()
                            exit_home_started = False
                            print("已取消回零等待，维持当前控制状态。", flush=True)
                        else:
                            print("当前没有等待中的回零请求。", flush=True)
                    elif command:
                        try:
                            goal = ctrl.set_gripper_mode(command)
                            print(f"夹爪模式: {command}；参考目标 {goal:.3f} rad", flush=True)
                        except ValueError as exc:
                            print(str(exc), flush=True)
            with ctrl._lock:
                home_status, home_message = ctrl._home_status, ctrl._home_message
            if requests.graceful.is_set() and not exit_home_started:
                exit_home_started = True
                if home_status == "done":
                    stopping.set()
                elif home_status == "running":
                    print("正在回零，到位后退出；再次 Ctrl+C 请求停止并失能。", flush=True)
                elif not home_pending:
                    home_pending = True
                    pending_since, still_since = time.monotonic(), None
                    print("退出前回零：等待机械臂保持静止；请远离运动范围。输入 cancel 可取消等待。", flush=True)
            if home_pending:
                now = time.monotonic()
                if now - pending_since > HOME_WAIT_STILL_TIMEOUT:
                    home_pending = False
                    requests.graceful.clear()
                    exit_home_started = False
                    print("等待静止超时，未开始回零；维持当前控制状态，可稍后重试。", flush=True)
                elif ctrl.home_stationary():
                    if still_since is None:
                        still_since = now
                    if now - still_since >= HOME_STILL_SECONDS:
                        try:
                            duration = ctrl.request_home()
                        except HomeNotStationary:
                            still_since = None
                        except (ValueError, RuntimeError) as exc:
                            home_pending = False
                            requests.graceful.clear()
                            exit_home_started = False
                            print(f"回零未启动: {exc}；维持当前控制状态，可稍后重试。", flush=True)
                        else:
                            home_pending = False
                            print(f"机械臂已稳定，开始回零（名义轨迹 {duration:.1f} 秒，目标速度上限 {math.degrees(HOME_MAX_VELOCITY):.1f}°/s；跟随误差超限时停止回零并保持当前位置）。", flush=True)
                else:
                    still_since = None
            with ctrl._lock:
                home_status, home_message = ctrl._home_status, ctrl._home_message
            if requests.graceful.is_set() and exit_home_started and home_status == "done":
                stopping.set()
            if home_status != last_home_status:
                if home_status in {"done", "aborted"}:
                    print(f"[回零] {home_message}", flush=True)
                    if home_status == "aborted" and requests.graceful.is_set():
                        requests.graceful.clear()
                        exit_home_started = False
                        print("回零未完成，机械臂保持当前位置（电机仍使能）；请勿推动，排查后可再次 quit 或 Ctrl+C 重试。", flush=True)
                last_home_status = home_status
            if seq % 60 == 0:
                with ctrl._lock:
                    now = time.monotonic()
                    feedback_age = now - ctrl._sample_time
                    plan = ctrl._home_plan
                    if plan is None:
                        home_diagnostic = None
                    else:
                        elapsed = max(0.0, now - plan.started_at)
                        progress = float(np.clip(plan.motion_elapsed / plan.duration, 0.0, 1.0))
                        reference = plan.target.copy()
                        error_deg = float(np.degrees(np.max(np.abs(reference - q))))
                        remaining = max(0.0, plan.duration - plan.motion_elapsed) + max(0.0, HOME_PREP_SECONDS - elapsed)
                        phase = ("已到位" if plan.done else "保持过渡" if elapsed < HOME_PREP_SECONDS
                                 else "回零运动" if progress < 1.0 else "到位确认")
                        home_diagnostic = (phase, progress, remaining, error_deg)
                if home_diagnostic is not None:
                    phase, progress, remaining, error_deg = home_diagnostic
                    print(f"回零状态: {phase} | 轨迹进度 {progress * 100:.0f}% | 最少剩余 {remaining:.1f} 秒 | 最大跟踪误差 {error_deg:.2f}°", flush=True)
                model_tau = np.asarray(compute_generalized_gravity(q=q)[:6], dtype=float) * GRAVITY_SCALE
                print("诊断 J2/J3 模型前馈(Nm):", np.round(model_tau[1:3], 3),
                      "反馈年龄(ms):", round(feedback_age * 1000, 1), flush=True)
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
    parser.add_argument("--check", action="store_true", help="只检查模型，不连接 CAN 或控制真机")
    # Preserve compatibility with the formerly required launch flags.
    parser.add_argument("--live", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--acknowledge-j2-drift", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--closed-rad", type=float, default=DEFAULT_GRIPPER_CLOSED,
                        help="夹爪闭合位置（弧度，默认 0）")
    parser.add_argument("--open-rad", type=float, default=DEFAULT_GRIPPER_OPEN,
                        help="夹爪打开位置（弧度，默认 5）")
    args = parser.parse_args()
    robot = RebotArm()
    check_model(robot)
    if args.check:
        print("仅模型预检查，未连接 CAN，未控制真机。")
        return
    print("注意：即将控制真机；J2 曾出现下垂，请支撑机械臂并观察。", flush=True)
    run_live(robot, args.closed_rad, args.open_rad)


if __name__ == "__main__":
    main()
