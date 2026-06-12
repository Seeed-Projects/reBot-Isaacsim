#!/usr/bin/env python3
"""RobStride RS06 电机控制测试 — 直接使用 motorbridge SDK。

用法:
    python example/rs06_test.py

注意: RobStride 使用 CAN 总线通信，不是串口。
      确保 CAN 接口已配置 (如 can0) 并正确连接电机。

交互命令:
    ping                                   — ping 电机获取响应
    mit <pos_deg> [<vel> <kp> <kd> <tau>] — MIT 模式指令
    posvel <pos_deg> [<vlim> [<loc_kp>]]  — POS_VEL 模式指令
    vel <vel_rad_s>                       — 纯速度模式指令
    enable                                 — 使能
    disable                                — 去使能
    set_zero                               — 电机零位设置
    mode <mit|posvel|vel>                 — 切换控制模式
    state                                  — 打印当前状态
    clear_error                            — 清除错误
    ping                                   — ping 电机
    read_param <param_id> [<type>]        — 读取参数 (默认 f32)
    write_param <param_id> <value> [<type>] — 写入参数 (默认 f32)
    loop                                   — 进入循环控制模式（stat 查看状态）
    q / quit                               — 退出
"""

import math
import select
import signal
import statistics
import sys
import threading
import time
from collections import deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from motorbridge import Controller, Mode

CHANNEL = "can0"  # RobStride 使用 CAN 接口
MOTOR_ID = 0x01  # 电机 ID
HOST_ID = 0xFD  # RobStride host/feedback ID (默认 0xFD)
MODEL = "rs-06"  # RS06 电机型号

# RS06 电机 PID 参数配置
RS06_LOC_KP = 13.0  # 位置环 Kp (POS_VEL 模式)
RS06_SPD_KP = 12.0  # 速度环 Kp
RS06_SPD_KI = 0.1  # 速度环 Ki
RS06_MIT_KP = 50.0  # MIT 模式 Kp
RS06_MIT_KD = 3.0  # MIT 模式 Kd

PI_OVER_180 = math.pi / 180.0


def signal_handler(sig, frame):
    print("\n[ctrl+c] 退出")
    sys.exit(0)


def main() -> None:
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print(f"连接到 CAN 接口 {CHANNEL} ...")
    print(f"电机配置: id={MOTOR_ID:#04x} host_id={HOST_ID:#04x} model={MODEL}")

    ctrl = Controller(CHANNEL)
    motor = ctrl.add_robstride_motor(MOTOR_ID, HOST_ID, MODEL)
    print(f"电机已注册: motor_id={MOTOR_ID:#04x} host_id={HOST_ID:#04x} model={MODEL}")

    # ---- 命令处理器 -------------------------------------------------------

    def do_enable() -> None:
        ctrl.enable_all()
        time.sleep(0.3)
        print("电机已使能")

    def do_disable() -> None:
        ctrl.disable_all()
        print("电机已去使能")

    def do_set_zero() -> None:
        """设置软件零位。"""
        for _ in range(10):
            motor.request_feedback()
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
            time.sleep(0.05)
        motor.set_zero_position()  # type-6
        time.sleep(0.1)
        print("软件零位已设置")

    def do_clear_error() -> None:
        """清除电机错误。"""
        motor.clear_error()
        time.sleep(0.1)
        print("错误已清除")

    def do_ping() -> None:
        """Ping 电机。"""
        try:
            device_id, responder_id = motor.robstride_ping()
            print(f"Ping 成功: device_id={device_id} responder_id={responder_id:#04x}")
        except Exception as e:
            print(f"Ping 失败: {e}")

    def do_mode(args: list[str]) -> None:
        if not args:
            print("用法: mode <mit|posvel|vel>")
            return
        m = args[0].lower()
        if m == "mit":
            motor.ensure_mode(Mode.MIT, 1000)
            print("切换到 MIT 模式")
        elif m == "posvel":
            motor.robstride_write_param_f32(0x701F, RS06_SPD_KP)  # 速度环 Kp
            time.sleep(0.01)
            motor.robstride_write_param_f32(0x7020, RS06_SPD_KI)  # 速度环 Ki
            time.sleep(0.01)
            loc_kp = float(args[1]) if len(args) >= 2 else RS06_LOC_KP
            motor.robstride_write_param_f32(0x701E, loc_kp)  # 位置环 Kp
            time.sleep(0.02)
            print(f"PID 参数: 位置Kp={loc_kp}, 速度Kp={RS06_SPD_KP}, 速度Ki={RS06_SPD_KI}")
            motor.ensure_mode(Mode.POS_VEL, 1000)
            print("切换到 POS_VEL 模式")
        elif m == "vel":
            motor.ensure_mode(Mode.VEL, 1000)
            print("切换到 VEL 模式")
        else:
            print(f"未知模式: {m}，可用: mit / posvel / vel")

    def do_state() -> None:
        print(
            "[注意] motorbridge 内部协议配置问题：RS 电机无法像 DM 电机那样查询，"
            "可能会出现读取不到实际电机状态的情况，请以实际运动效果为准。"
        )
        st = None
        for _ in range(10):
            motor.request_feedback()
            ctrl.poll_feedback_once()
            time.sleep(0.005)
            st = motor.get_state()
            if st is not None and st.status_code == 0:
                break
        if st is None:
            print("无反馈数据")
            return
        print(
            f"pos={st.pos / PI_OVER_180:+.4f}deg  "
            f"vel={st.vel / PI_OVER_180:+.4f}deg/s  "
            f"torq={st.torq:+.4f}  "
            f"status={st.status_code}"
        )

    def do_mit(args: list[str]) -> None:
        if not args:
            print("用法: mit <pos_deg> [<vel> <kp> <kd> <tau>]")
            return
        pos = float(args[0]) * PI_OVER_180
        vel = float(args[1]) if len(args) > 1 else 0.0
        kp = float(args[2]) if len(args) > 2 else RS06_MIT_KP
        kd = float(args[3]) if len(args) > 3 else RS06_MIT_KD
        tau = 0.0
        motor.send_mit(pos, vel, kp, kd, tau)

    def do_posvel(args: list[str]) -> None:
        if not args:
            print("用法: posvel <pos_deg> [<vlim> [<loc_kp>]]")
            return
        pos = float(args[0]) * PI_OVER_180
        vlim = float(args[1]) if len(args) > 1 else 1.0
        motor.robstride_write_param_f32(0x7017, abs(vlim))  # limit_spd
        motor.robstride_write_param_f32(0x7016, pos)  # loc_ref
        print(f"POS_VEL: pos={pos:.4f}rad vlim={vlim}")

    def do_vel(args: list[str]) -> None:
        if not args:
            print("用法: vel <vel_rad_s>")
            return
        motor.send_vel(float(args[0]))

    def do_read_param(args: list[str]) -> None:
        """读取 RobStride 参数。"""
        if not args:
            print("用法: read_param <param_id> [<type>]")
            print("      type 可选: u8, u16, u32, f32 (默认 f32)")
            return
        param_id = int(args[0], 0)
        param_type = args[1] if len(args) > 1 else "f32"
        timeout_ms = 1000

        try:
            if param_type == "u8":
                value = motor.robstride_get_param_u8(param_id, timeout_ms)
            elif param_type == "u16":
                value = motor.robstride_get_param_u16(param_id, timeout_ms)
            elif param_type == "u32":
                value = motor.robstride_get_param_u32(param_id, timeout_ms)
            else:
                value = motor.robstride_get_param_f32(param_id, timeout_ms)
            print(f"param 0x{param_id:04X} = {value}")
        except Exception as e:
            print(f"读取参数失败: {e}")

    def do_write_param(args: list[str]) -> None:
        """写入 RobStride 参数。"""
        if len(args) < 2:
            print("用法: write_param <param_id> <value> [<type>]")
            print("      type 可选: u8, u16, u32, f32 (默认 f32)")
            return
        param_id = int(args[0], 0)
        value = args[1]
        param_type = args[2] if len(args) > 2 else "f32"

        try:
            if param_type == "u8":
                motor.robstride_write_param_u8(param_id, int(value, 0))
            elif param_type == "u16":
                motor.robstride_write_param_u16(param_id, int(value, 0))
            elif param_type == "u32":
                motor.robstride_write_param_u32(param_id, int(value, 0))
            else:
                motor.robstride_write_param_f32(param_id, float(value))
            print(f"param 0x{param_id:04X} 已写入值: {value}")
        except Exception as e:
            print(f"写入参数失败: {e}")

    # loop 共享状态
    loop_running = threading.Event()
    loop_lock = threading.Lock()
    _loop = {"mode": "mit", "pos": 0.0, "vel": 0.0, "vlim": 1.0}
    _hist: deque[float] = deque(maxlen=200)
    _lat_hist: deque[float] = deque(maxlen=200)

    def do_loop(_args: list[str]) -> None:
        if loop_running.is_set():
            print("loop 已在运行，q 退出")
            return
        loop_running.set()

        cnt, pos, vel, torq, freq, lat, std = 0, "---", "---", "---", 0, 0.0, 0.0
        last = time.perf_counter()
        while loop_running.is_set():
            t0 = time.perf_counter()
            with loop_lock:
                m, p, v, vl = _loop["mode"], _loop["pos"], _loop["vel"], _loop["vlim"]
            if m == "mit":
                motor.send_mit(p * PI_OVER_180, 0, RS06_MIT_KP, RS06_MIT_KD, 0)
            elif m == "posvel":
                motor.robstride_write_param_f32(0x7017, abs(vl))
                motor.robstride_write_param_f32(0x7016, p * PI_OVER_180)
            else:
                motor.send_vel(v)

            dt = t0 - last
            last = t0
            _hist.append(dt)
            n = len(_hist)
            freq = 1 / dt if dt > 0 else 0
            std = statistics.stdev(_hist) * 1000 if n >= 4 else 0.0
            lat = _lat_hist[-1] if _lat_hist else 0.0
            cnt += 1

            for _ in range(20):
                ctrl.poll_feedback_once()
                st = motor.get_state()
                if st and st.status_code == 0:
                    _lat_hist.append((time.perf_counter() - t0) * 1000)
                    pos = f"{st.pos / PI_OVER_180:+.2f}"
                    vel = f"{st.vel / PI_OVER_180:+.2f}"
                    torq = f"{st.torq:+.4f}"
                    break

            # 非阻塞检查输入
            if select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline()
                if not cmd:
                    break
                parts = cmd.strip().split()
                if parts:
                    cmd = parts[0].lower()
                    args = parts[1:]
                    if cmd == "q":
                        loop_running.clear()
                        break
                    elif cmd == "mit" and args:
                        _loop.update({"mode": "mit", "pos": float(args[0])})
                    elif cmd == "posvel" and args:
                        _loop.update(
                            {
                                "mode": "posvel",
                                "pos": float(args[0]),
                                "vlim": float(args[1]) if len(args) >= 2 else 1.0,
                            }
                        )
                    elif cmd == "vel" and args:
                        _loop.update({"mode": "vel", "vel": float(args[0])})
                    elif cmd == "stat":
                        print(
                            f"stat: cnt={cnt}  pos={pos}deg  vel={vel}  "
                            f"torq={torq}  {freq:.0f}Hz  lat={lat:.1f}ms  std={std:.1f}ms"
                        )
                        h, lh = list(_hist), list(_lat_hist)
                        lat_avg = statistics.mean(lh) if lh else 0
                        lat_std = statistics.stdev(lh) if len(lh) >= 4 else 0
                        avg_hz = f"{1 / statistics.mean(h):.0f}Hz" if h else "0Hz"
                        print(
                            f"avg={avg_hz}  min={min(h) * 1000:.1f}ms  "
                            f"max={max(h) * 1000:.1f}ms  "
                            f"std={statistics.stdev(h) * 1000:.1f}ms  "
                            f"lat={lat_avg:.1f}ms(std={lat_std:.1f})"
                        )
                    else:
                        print("[loop] mit/posvel/vel <参数>  stat  q")
            time.sleep(max(0, 0.002 - (time.perf_counter() - t0)))

        print("[loop] 退出")
        h, lh = list(_hist), list(_lat_hist)
        _hist.clear()
        _lat_hist.clear()
        if h:
            lat_avg = statistics.mean(lh) if lh else 0
            lat_std = statistics.stdev(lh) if len(lh) >= 4 else 0
            avg_hz = f"{1 / statistics.mean(h):.0f}Hz"
            print(
                f"[loop] avg={avg_hz}  min={min(h) * 1000:.1f}ms  "
                f"max={max(h) * 1000:.1f}ms  "
                f"std={statistics.stdev(h) * 1000:.1f}ms  "
                f"lat={lat_avg:.1f}ms(std={lat_std:.1f})\n"
            )

    # ---- 命令表 -----------------------------------------------------------
    NO_ARG_COMMANDS = frozenset(
        {do_enable, do_disable, do_set_zero, do_clear_error, do_ping, do_state}
    )
    COMMANDS: dict[str, tuple] = {
        "enable": (do_enable, "使能电机"),
        "disable": (do_disable, "去使能电机"),
        "set_zero": (do_set_zero, "设置软件零位"),
        "clear_error": (do_clear_error, "清除电机错误"),
        "ping": (do_ping, "Ping 电机"),
        "state": (do_state, "打印当前状态"),
        "mode": (do_mode, "mode <mit|posvel|vel>"),
        "mit": (do_mit, "mit <pos_deg> [<vel> <kp> <kd> <tau>]"),
        "posvel": (do_posvel, "posvel <pos_deg> [<vlim> [<loc_kp>]]"),
        "vel": (do_vel, "vel <vel_rad_s>"),
        "read_param": (do_read_param, "read_param <param_id> [<type>]"),
        "write_param": (do_write_param, "write_param <param_id> <value> [<type>]"),
        "loop": (do_loop, "loop"),
    }

    print(
        "\n命令: enable / disable / set_zero / clear_error / ping / state / "
        "mode / mit / posvel / vel / read_param / write_param / loop / q"
    )
    print(
        "提示: mode 会自动在下一条控制指令前生效；"
        "loop 进入实时控制循环模式\n"
    )

    try:
        while True:
            try:
                line = input("> ").strip()
            except EOFError:
                break
            if not line:
                continue

            parts = line.split()
            cmd = parts[0].lower()
            args = parts[1:]

            if cmd in ("q", "quit", "exit"):
                print("退出")
                break

            if cmd not in COMMANDS:
                print(f"未知命令: {cmd}，可用: {' / '.join(COMMANDS)}")
                continue

            fn, _ = COMMANDS[cmd]
            try:
                if fn in NO_ARG_COMMANDS:
                    fn()
                else:
                    fn(args)
            except Exception as e:
                print(f"错误: {e}")
    finally:
        ctrl.disable_all()
        ctrl.shutdown()
        ctrl.close()


if __name__ == "__main__":
    main()
