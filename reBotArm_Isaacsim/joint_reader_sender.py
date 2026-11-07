#!/usr/bin/env python3
"""RS 关节只读映射：读取实时位置并发送 UDP，不切换模式或控制电机。"""
from __future__ import annotations

import argparse
import json
import signal
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

_THIRD_PARTY = Path(__file__).resolve().parents[1] / 'third_party' / 'reBotArm_control_py'
sys.path.insert(0, str(_THIRD_PARTY))
from reBotArm_control_py.actuator import RebotArm  # noqa: E402

ARM_JOINT_COUNT = 6
DEFAULT_HOST, DEFAULT_PORT = '127.0.0.1', 5005
DEFAULT_SEND_HZ = 60.0  # 目标上限，实际取决于 CAN 总线负载
GRIPPER_MAX_OPENING_M = 0.045  # 每根仿真手指最大行程
MECH_POS = 0x7019
READ_TIMEOUT_MS = 100
DEFAULT_CAN_HOST_ID = 0xFC
CAN_FRAME = struct.Struct("=IB3x8s")
PARAM_REQUEST = struct.pack("<H6x", MECH_POS)
_running = True


def _sigint_handler(signum, frame):
    del signum, frame
    global _running
    _running = False


signal.signal(signal.SIGINT, _sigint_handler)


class FeedbackError(RuntimeError):
    """A current position response was unavailable; no packet should be sent."""


class JointReaderSender:
    def __init__(self, host=DEFAULT_HOST, port=DEFAULT_PORT, can_host_id=DEFAULT_CAN_HOST_ID,
                 gripper_closed=0.0, gripper_open=5.0):
        self.host, self.port = host, port
        if not 0 <= can_host_id <= 255:
            raise ValueError("CAN host ID 必须在 0–255 之间")
        self.can_host_id = can_host_id
        if not np.isfinite([gripper_closed, gripper_open]).all() or abs(gripper_open - gripper_closed) < 1e-6:
            raise ValueError("夹爪开闭端点必须是不同的有限数值")
        self.gripper_closed = gripper_closed
        self.gripper_open = gripper_open
        self.can_socket = None
        self.motor_ids = []
        self.rebotarm = RebotArm()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sequence = 0
        self.latest_q_raw = np.zeros(ARM_JOINT_COUNT)
        self.latest_q = np.zeros(ARM_JOINT_COUNT)
        self.latest_gripper_q = 0.0
        self.latest_gripper_position = 0.0

    def setup_hardware(self):
        # RebotArm() reads YAML and builds groups without connecting or enabling.
        joints = list(self.rebotarm.arm._jcfgs[:ARM_JOINT_COUNT])
        if len(joints) != ARM_JOINT_COUNT:
            raise RuntimeError('arm 组少于 6 个关节')
        if self.rebotarm.has_gripper:
            if len(self.rebotarm.gripper._jcfgs) != 1:
                raise RuntimeError('夹爪组必须有一个电机')
            joints += list(self.rebotarm.gripper._jcfgs)
        if any(j.vendor != 'robstride' for j in joints):
            raise RuntimeError('只读实时查询目前仅支持 RS 电机')
        self.motor_ids = [int(j.motor_id) for j in joints]
        if (len(set(self.motor_ids)) != len(self.motor_ids)
                or any(not 1 <= mid <= 255 for mid in self.motor_ids)):
            raise RuntimeError('CAN 电机 ID 无效或重复')
        self.can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.can_socket.bind((self.rebotarm._channel,))
        response_id = socket.CAN_EFF_FLAG | (0x11 << 24) | self.can_host_id
        mask = socket.CAN_EFF_FLAG | 0x1f0000ff
        self.can_socket.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, struct.pack("=II", response_id, mask))
        self.can_socket.setblocking(False)

    def read_positions(self):
        # Type 17 only reads parameter 0x7019 (mechPos). Send the seven
        # requests first, then match every reply by host, motor and parameter.
        if self.can_socket is None:
            raise FeedbackError('CAN 尚未连接')
        can = self.can_socket
        try:
            can.setblocking(False)
            while True:  # Discard responses already queued before this sample.
                try:
                    can.recv(CAN_FRAME.size)
                except BlockingIOError:
                    break
            for mid in self.motor_ids:
                request_id = socket.CAN_EFF_FLAG | (0x11 << 24) | (self.can_host_id << 8) | mid
                can.send(CAN_FRAME.pack(request_id, 8, PARAM_REQUEST))
            deadline = time.monotonic() + READ_TIMEOUT_MS / 1000.0
            values = {}
            while len(values) < len(self.motor_ids):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise FeedbackError(f'位置反馈超时，缺少电机 {sorted(set(self.motor_ids) - values.keys())}')
                can.settimeout(remaining)
                try:
                    frame = can.recv(CAN_FRAME.size)
                except socket.timeout as exc:
                    raise FeedbackError(f'位置反馈超时，缺少电机 {sorted(set(self.motor_ids) - values.keys())}') from exc
                if len(frame) != CAN_FRAME.size:
                    continue
                can_id, length, data = CAN_FRAME.unpack(frame)
                if not can_id & socket.CAN_EFF_FLAG or can_id & (socket.CAN_RTR_FLAG | socket.CAN_ERR_FLAG):
                    continue
                can_id &= socket.CAN_EFF_MASK
                mid = (can_id >> 8) & 0xff
                expected_id = (0x11 << 24) | (mid << 8) | self.can_host_id
                if (can_id != expected_id or mid not in self.motor_ids or length != 8
                        or data[:4] != PARAM_REQUEST[:4] or mid in values):
                    continue
                value = struct.unpack('<f', data[4:])[0]
                if not np.isfinite(value):
                    raise FeedbackError(f'电机 {mid} 的位置反馈不是有限值')
                values[mid] = value
        except OSError as exc:
            raise FeedbackError(f'CAN 读写失败 ({exc})') from exc
        q = np.array([values[mid] for mid in self.motor_ids[:ARM_JOINT_COUNT]])
        gripper_q = values[self.motor_ids[ARM_JOINT_COUNT]] if self.rebotarm.has_gripper else 0.0
        # Publish an entire sample only after all required reads succeed.
        self.latest_q_raw[:] = q
        self.latest_q[:] = -q
        self.latest_gripper_q = float(gripper_q)
        ratio = (gripper_q - self.gripper_closed) / (self.gripper_open - self.gripper_closed)
        self.latest_gripper_position = float(np.clip(ratio, 0.0, 1.0) * GRIPPER_MAX_OPENING_M)

    def run(self, send_hz=DEFAULT_SEND_HZ):
        if not np.isfinite(send_hz) or send_hz <= 0:
            raise ValueError('send_hz 必须为正数')
        period = 1.0 / send_hz
        next_send = time.monotonic()
        last_error = 0.0
        failed = False
        gripper_range_warning = False
        while _running:
            delay = next_send - time.monotonic()
            if delay > 0:
                time.sleep(min(delay, 0.05))
                continue
            try:
                self.read_positions()
            except FeedbackError as exc:
                now = time.monotonic()
                if not failed or now - last_error >= 2.0:
                    print(f'[反馈异常] {exc}；暂停发送，等待恢复', flush=True)
                    last_error = now
                failed = True
                next_send = now + max(period, 0.1)
                continue
            if failed:
                print('[反馈恢复] 继续发送', flush=True)
                failed = False
            low, high = sorted((self.gripper_closed, self.gripper_open))
            outside_range = self.rebotarm.has_gripper and not low - 0.02 <= self.latest_gripper_q <= high + 0.02
            if outside_range and not gripper_range_warning:
                print(f'[夹爪] 反馈 {self.latest_gripper_q:+.3f} rad 超出映射范围 {low:.3f}–{high:.3f}；仿真已裁剪，请检查端点', flush=True)
            gripper_range_warning = outside_range
            payload = {'sequence': self.sequence, 'timestamp': time.time(),
                       'joint_positions': self.latest_q.tolist(),
                       'gripper_position': self.latest_gripper_position}
            self.socket.sendto(json.dumps(payload, separators=(',', ':')).encode(),
                               (self.host, self.port))
            if self.sequence % 30 == 0:
                print('[send] raw deg=' + np.array2string(np.degrees(self.latest_q_raw), precision=2)
                      + f' gripper_rad={self.latest_gripper_q:+.3f}', flush=True)
            self.sequence += 1
            next_send = max(next_send + period, time.monotonic())

    def shutdown(self):
        # SDK disconnect() calls disable_all(): never call it for a read-only client.
        # Releasing this process's socket/handles at exit leaves motor state alone.
        if self.can_socket is not None:
            self.can_socket.close()
        self.socket.close()


def main():
    parser = argparse.ArgumentParser(description='RS 关节只读映射到 Isaac Sim')
    parser.add_argument('--no-mit', action='store_true', help='兼容旧参数：只读模式')
    parser.add_argument('--mit', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--can-host-id', type=lambda x: int(x, 0), default=DEFAULT_CAN_HOST_ID,
                        help='CAN 查询 host ID，默认 0xfc；避免与其他控制程序冲突')
    parser.add_argument('--gripper-closed', type=float, default=0.0, help='夹爪闭合时电机角度 rad（仅仿真映射）')
    parser.add_argument('--gripper-open', type=float, default=5.0, help='夹爪全开时电机角度 rad（仅仿真映射）')
    args = parser.parse_args()
    if args.mit:
        parser.error('此程序只读，不支持 --mit；请使用专门的控制程序')
    print('[sender] RS 只读查询；不发送使能、失能或运动命令', flush=True)
    print(f'[sender] udp://{DEFAULT_HOST}:{DEFAULT_PORT}，目标 {DEFAULT_SEND_HZ:g} Hz', flush=True)
    print(f'[夹爪] 映射闭合={args.gripper_closed:+.3f}, 全开={args.gripper_open:+.3f} rad', flush=True)
    sender = JointReaderSender(can_host_id=args.can_host_id,
                               gripper_closed=args.gripper_closed, gripper_open=args.gripper_open)
    try:
        sender.setup_hardware()
        sender.run()
    finally:
        sender.shutdown()
        print('[sender] 已退出（未失能电机）', flush=True)


if __name__ == '__main__':
    main()
