"""Find the torque that actually breaks the gripper free, then measure stiffness.

Previous MIT run: kp=3 N*m/rad over a 1.77 mm command range produced at most
0.72 N*m and the encoder moved 2 LSB (0.0056 mm) -- i.e. nothing. The drive
never overcame static friction in the pinion/rack and gearbox, so every
"stiffness" number it printed was noise divided by noise.

So measure the breakaway torque first: ramp the commanded torque slowly and
record the value at which the position starts to change by more than a few LSB.
That number is interesting in itself (it is friction the sim does not model),
and it sets the floor for any stiffness measurement.

Then, above breakaway, sweep position offsets and fit force vs deflection.

SAFETY
  * only motor 7; joints 1-6 untouched
  * torque ramps from 0 in small increments, abort at TAU_ABORT
  * movement bounded: abort if the jaws travel past TRAVEL_LIMIT_MM
  * motor disabled in finally: on every path
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
TAU_ABORT = 3.0          # N*m hard abort (motor limit is 14)
TAU_MAX_RAMP = 2.0       # N*m ceiling for the breakaway search
TAU_STEP = 0.05          # N*m per increment
STEP_HOLD_S = 0.35
MOVE_LSB = 6             # LSB of angle that count as "it moved"
TRAVEL_LIMIT_MM = 8.0    # abort if it runs this far
RATE = 200.0
LSB_RAD = 25.0 / 65535.0

ap = argparse.ArgumentParser()
ap.add_argument("--dry-run", action="store_true")
ap.add_argument("--out", default="breakaway.json")
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
    sys.exit("no gripper telemetry")

zero = tel.latest[1]
print(f"breakaway search: tau 0 -> {TAU_MAX_RAMP} N*m in {TAU_STEP} steps")
print(f"start angle {zero:+.5f} rad   {tel.latest[4]:.0f} C")
print(f"'moved' = {MOVE_LSB} LSB = {MOVE_LSB*LSB_RAD*R*1e3:.4f} mm at the finger")

if args.dry_run:
    print("\n[dry-run] not enabling")
    tel.stop()
    sys.exit(0)

arm = None
motor = None
log = []
breakaway = None

try:
    arm = RebotArm("rebotarm_rs.yaml")
    arm.connect()
    motor = arm._motor_map["gripper"]
    try:
        motor.clear_error()
        time.sleep(0.2)
    except Exception:
        pass
    # CRITICAL: the motor must be switched into MIT mode first. Without this
    # it stays in whatever run_mode it held (POS_VEL = 1) and silently ignores
    # MIT frames -- measured torque then never tracks the commanded value.
    # ensure_mode() only sets the mode; unlike mode_pos_vel() it writes no gains.
    motor.ensure_mode(Mode.MIT, 1000)
    time.sleep(0.2)
    mode_now = motor.robstride_get_param_i8(0x7005)
    print(f"run_mode after ensure_mode(MIT) = {mode_now}  (0 = MIT)")
    if mode_now != 0:
        raise RuntimeError(f"motor refused MIT mode (run_mode={mode_now})")

    motor.enable()
    time.sleep(0.4)
    base = tel.latest[1]
    print(f"enabled   angle {base:+.5f}   torque {tel.latest[3]:+.5f} N*m\n")

    print(f"{'cmd tau':>9s} {'pos mm':>9s} {'moved mm':>10s} {'meas tau':>10s}")
    print("-" * 42)

    tau = 0.0
    while tau < TAU_MAX_RAMP:
        tau += TAU_STEP
        t0 = time.time()
        peak_meas = 0.0
        while time.time() - t0 < STEP_HOLD_S:
            # pure torque: kp=0, kd=0, feed-forward tau only
            motor.send_mit(0.0, 0.0, 0.0, 0.0, tau)
            s = tel.latest
            peak_meas = max(peak_meas, abs(s[3]))
            log.append({"t": s[0], "cmd_tau": tau, "pos_rad": s[1],
                        "vel": s[2], "torque_Nm": s[3]})
            if abs(s[3]) > TAU_ABORT:
                print(f"  ABORT torque {s[3]:+.3f} N*m")
                tau = TAU_MAX_RAMP
                break
            moved_mm = abs(s[1] - base) * R * 1e3
            if moved_mm > TRAVEL_LIMIT_MM:
                print(f"  ABORT travel {moved_mm:.2f} mm")
                tau = TAU_MAX_RAMP
                break
            time.sleep(1.0 / RATE)

        s = tel.latest
        moved = (s[1] - base)
        moved_mm = moved * R * 1e3
        print(f"{tau:9.3f} {(s[1]-zero)*R*1e3:9.4f} {moved_mm:+10.4f} {peak_meas:10.4f}")

        if breakaway is None and abs(moved) > MOVE_LSB * LSB_RAD:
            breakaway = tau
            print(f"  --> BREAKAWAY at {tau:.3f} N*m "
                  f"({tau/R:.1f} N at the finger)")
            break

    # relax to zero torque
    for _ in range(60):
        motor.send_mit(0.0, 0.0, 0.0, 0.0, 0.0)
        time.sleep(0.005)

finally:
    if motor is not None:
        try:
            print("\ndisabling gripper motor")
            arm.disable_all()
            time.sleep(0.3)
            # leave the motor in POS_VEL, the mode the arm normally runs in
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

    if log:
        Path(args.out).write_text(json.dumps(
            {"zero_rad": zero, "breakaway_Nm": breakaway,
             "r_m_per_rad": R, "samples": log}, indent=1))
        print(f"saved {len(log)} samples -> {args.out}")
    if breakaway:
        print(f"\nbreakaway torque {breakaway:.3f} N*m = {breakaway/R:.1f} N at the finger")
        print("that is static friction the simulated gripper does not model")
    else:
        print(f"\nno movement up to {TAU_MAX_RAMP} N*m "
              f"({TAU_MAX_RAMP/R:.0f} N at the finger)")
