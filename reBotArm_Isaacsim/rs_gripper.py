"""B601-RS mapping from motor radians to simulated per-finger travel."""

import math

MOTOR_CLOSED_RAD = 0.0
MOTOR_OPEN_RAD = 5.0
FINGER_TRAVEL_M = 0.05


def motor_to_finger_travel(position, closed=MOTOR_CLOSED_RAD, opened=MOTOR_OPEN_RAD):
    if not math.isfinite(position):
        raise ValueError("Gripper motor position must be finite")
    ratio = (position - closed) / (opened - closed)
    return float(min(1.0, max(0.0, ratio)) * FINGER_TRAVEL_M)
