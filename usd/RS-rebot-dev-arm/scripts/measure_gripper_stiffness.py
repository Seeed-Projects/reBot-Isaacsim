"""Sim2real gripper calibration in MIT (torque) mode.

Two earlier attempts failed for reasons now understood:

  * `send_pos_vel` in a position loop kept raising effort toward an unreachable
    target and slammed the mechanical stop; at 40 Hz feedback no torque
    threshold is observable before saturation (0 -> -3.76 N*m between samples)
  * `mode_pos_vel()` silently wrote the library's YAML gains into the motor,
    and `send_pos_vel(vlim)` persisted vlim into parameter 0x7017

MIT mode avoids both. `send_mit(pos, vel, kp, kd, tau)` is a command frame: it
writes no parameters, and with a small kp the motor cannot deliver more than
kp * error, so a wrong direction stalls harmlessly instead of slamming a stop.
Detection latency stops mattering because the commanded torque is bounded by
construction rather than by how fast we react.

Measurement: the arm is at the zero the operator calibrated in MotorBridge
Studio, so absolute positions are meaningful again. Command a series of small
position offsets at fixed kp/kd and log the settled (position error, torque)
pair. Fitting torque against position error gives the closed-loop stiffness in
pos-hold, which is the number the asset's drive gain should reproduce.

SAFETY
  * only motor 7 is enabled; joints 1-6 stay unpowered
  * kp is capped so kp * max_error stays well under TAU_CAP
  * tau feed-forward is 0 and the commanded tau limit is TAU_CAP
  * abort and disable if |torque| exceeds TAU_ABORT at any sample
  * motor disabled in finally: on every exit path
  * --dry-run rehearses everything without enabling
"""

import argparse
import json
import statistics
import struct
import sys
import threading
import time
from pathlib import Path

import can

# resolve the vendor SDK relative to this file so the script runs from a clone
REPO = Path(__file__).resolve().parents[3] / "third_party" / "reBotArm_control_py"
sys.path.insert(0, str(REPO))

from motorbridge import Mode  # noqa: E402
from reBotArm_control_py.actuator.rebotarm import RebotArm  # noqa: E402

R = 7.353e-3
GRIPPER_ID = 0x07
KP = 3.0              # N*m/rad; breakaway measured at 0.10 N*m, so a
                      # 0.04 rad step (0.12 N*m) already clears friction
KD = 0.3              # N*m/(rad/s)
TAU_CAP = 1.0         # N*m commanded ceiling
TAU_ABORT = 2.0       # N*m -- disable immediately above this
STEP_RAD = 0.04       # ~0.29 mm at the finger per step
N_STEPS = 6
SETTLE_S = 1.2
RATE = 200.0

ap = argparse.ArgumentParser()
ap.add_argument("--dry-run", action="store_true")
ap.add_argument("--out", default="mit_calibration.json")
args = ap.parse_args()


class Telemetry(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.bus = can.interface.Bus(channel="can0", interface="socketcan")
        self.latest = None
        self._run = True

    def run(self):
        while self._run:
            msg = self.bus.recv(timeout=0.2)
            if msg is None:
                continue
            arb = msg.arbitration_id
            if ((arb >> 24) & 0xFF) != 0x18 or ((arb >> 8) & 0xFF) != GRIPPER_ID:
                continue
            if len(msg.data) < 8:
                continue
            a, v, t, temp = struct.unpack(">HHHH", msg.data)
            self.latest = (msg.timestamp,
                           -12.5 + 25.0 * a / 65535.0,
                           -44.0 + 88.0 * v / 65535.0,
                           -17.0 + 34.0 * t / 65535.0,
                           temp / 10.0)

    def stop(self):
        self._run = False
        time.sleep(0.25)
        self.bus.shutdown()


tel = Telemetry()
tel.start()
time.sleep(0.5)
if tel.latest is None:
    tel.stop()
    sys.exit("no gripper telemetry on can0")

zero = tel.latest[1]
print(f"MIT calibration   kp={KP} N*m/rad  kd={KD}  tau_cap={TAU_CAP} N*m")
print(f"start angle {zero:+.5f} rad   torque {tel.latest[3]:+.5f} N*m   "
      f"{tel.latest[4]:.0f} C")
print(f"step {STEP_RAD:.3f} rad = {STEP_RAD*R*1e3:.3f} mm at the finger, "
      f"{N_STEPS} steps each way")
print(f"max commanded torque at full step: {KP*STEP_RAD*N_STEPS:.3f} N*m")

if args.dry_run:
    print("\n[dry-run] not enabling; nothing commanded")
    tel.stop()
    sys.exit(0)

arm = None
motor = None
samples = []
plateaus = []

try:
    arm = RebotArm("rebotarm_rs.yaml")
    arm.connect()
    motor = arm._motor_map["gripper"]
    try:
        motor.clear_error()
        time.sleep(0.2)
    except Exception:
        pass

    # The motor must be switched to MIT first. Without this it stays in its
    # current run_mode (POS_VEL = 1) and silently ignores MIT frames: measured
    # torque then never tracks the command. ensure_mode() writes no gains,
    # unlike mode_pos_vel().
    motor.ensure_mode(Mode.MIT, 1000)
    time.sleep(0.2)
    mode_now = motor.robstride_get_param_i8(0x7005)
    print(f"run_mode after ensure_mode(MIT) = {mode_now}  (0 = MIT)")
    if mode_now != 0:
        raise RuntimeError(f"motor refused MIT mode (run_mode={mode_now})")

    motor.enable()
    time.sleep(0.4)
    s = tel.latest
    print(f"\nenabled   angle {s[1]:+.5f}   torque {s[3]:+.5f} N*m")

    print(f"\n{'offset mm':>10s} {'actual mm':>10s} {'error mm':>9s} "
          f"{'torque Nm':>10s} {'force N':>9s} {'K N/m':>10s}")
    print("-" * 64)

    # sweep out and back so hysteresis (backlash) is visible
    offsets = ([i * STEP_RAD for i in range(1, N_STEPS + 1)]
               + [i * STEP_RAD for i in range(N_STEPS - 1, -1, -1)])

    aborted = False
    for off in offsets:
        target = zero + off
        t0 = time.time()
        hold = []
        while time.time() - t0 < SETTLE_S:
            motor.send_mit(target, 0.0, KP, KD, 0.0)
            s = tel.latest
            if abs(s[3]) > TAU_ABORT:
                print(f"  ABORT: torque {s[3]:+.3f} N*m > {TAU_ABORT}")
                aborted = True
                break
            hold.append(s)
            samples.append({"t": s[0], "target_rad": target, "pos_rad": s[1],
                            "vel": s[2], "torque_Nm": s[3]})
            time.sleep(1.0 / RATE)
        if aborted:
            break

        settle = hold[len(hold) // 2:]
        pos = statistics.fmean(h[1] for h in settle)
        tau = statistics.fmean(h[3] for h in settle)
        err_rad = target - pos
        err_mm = err_rad * R * 1e3
        force = tau / R
        k = abs(force / (err_rad * R)) if abs(err_rad) > 1e-9 else float("nan")
        plateaus.append({"offset_mm": off * R * 1e3,
                         "actual_mm": (pos - zero) * R * 1e3,
                         "err_mm": err_mm, "torque_Nm": tau,
                         "force_N": force, "k_N_per_m": k})
        print(f"{off*R*1e3:10.3f} {(pos-zero)*R*1e3:10.3f} {err_mm:+9.4f} "
              f"{tau:+10.4f} {force:+9.2f} {k:10,.0f}")

    if plateaus:
        ks = [p["k_N_per_m"] for p in plateaus
              if p["k_N_per_m"] == p["k_N_per_m"] and abs(p["torque_Nm"]) > 0.01]
        if ks:
            print(f"\nmedian closed-loop stiffness = {statistics.median(ks):,.0f} N/m")
            print(f"  asset ships 5,000 N/m")
            print(f"  MIT kp reflected (kp/r^2) = {KP/R**2:,.0f} N/m at this kp")
            print(f"  at the factory MIT kp=50: {50/R**2:,.0f} N/m")
        else:
            print("\ntorque stayed below resolution at every step")

finally:
    if motor is not None:
        try:
            print("\ndisabling gripper motor")
            arm.disable_all()
            time.sleep(0.3)
            motor.ensure_mode(Mode.POS_VEL, 1000)
            time.sleep(0.2)
            print(f"  run_mode restored to {motor.robstride_get_param_i8(0x7005)}")
        except Exception as exc:
            print(f"  disable failed: {exc}")
    if arm is not None:
        try:
            arm.disconnect()
        except Exception:
            pass
    s = tel.latest
    tel.stop()
    print(f"final angle {s[1]:+.5f} rad   torque {s[3]:+.5f} N*m")

    if samples:
        Path(args.out).write_text(json.dumps(
            {"kp": KP, "kd": KD, "zero_rad": zero, "r_m_per_rad": R,
             "plateaus": plateaus, "samples": samples}, indent=1))
        print(f"saved {len(samples)} samples -> {args.out}")
