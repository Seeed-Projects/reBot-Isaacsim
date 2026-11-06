"""Offline checks for the gravity/hand-assist example; never connects to CAN."""
import math
import contextlib
import io
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "reBotArm_Isaacsim"))
import gravity_gripper_assist as app


class FakeControlTests(unittest.TestCase):
    def setUp(self):
        self.motors = {name: Mock() for name in (*app.ARM_NAMES, "gripper")}
        self.robot = SimpleNamespace(
            _motor_map=self.motors,
            gripper=SimpleNamespace(_mit_kp=[50.0], _mit_kd=[4.0]),
        )
        self.ctrl = app.AssistedGravity(self.robot, -0.4, 6.0)
        self.ctrl._feedback = (np.zeros(6), 2.0, 0.1)
        self.ctrl._sample_time = time.monotonic()
        self.ctrl._hold_q = np.zeros(6)
        self.ctrl._initial_kp = np.full(6, 50.0)
        self.ctrl._initial_kd = np.full(6, 3.0)
        self.ctrl._transition_start = time.monotonic() - 1

    def tick(self):
        with patch.object(app, "compute_generalized_gravity", return_value=np.arange(8.0)):
            self.ctrl._loop_cb(self.robot, 0.008)

    def test_hand_assist_direction_and_limits(self):
        self.assertEqual(app.assist_torque(2, 0, -0.4, 6), 0)
        self.assertGreater(app.assist_torque(2, 0.1, -0.4, 6), 0)
        self.assertLess(app.assist_torque(2, -0.1, -0.4, 6), 0)
        self.assertEqual(app.assist_torque(-0.35, -0.1, -0.4, 6), 0)
        self.assertEqual(app.assist_torque(5.95, 0.1, -0.4, 6), 0)
        self.assertEqual(app.assist_torque(2, 0.8, -0.4, 6), 0)

    def test_gripper_mapping_accepts_either_encoder_direction(self):
        self.assertAlmostEqual(app.gripper_sim_position(-0.4, -0.4, 6), 0)
        self.assertAlmostEqual(app.gripper_sim_position(6, -0.4, 6), 0.045)
        self.assertAlmostEqual(app.gripper_sim_position(2.8, 6, -0.4), 0.0225)
        with self.assertRaises(ValueError):
            app.gripper_sim_position(0, 0, 0)

    def test_soft_limit_keeps_arm_control_and_removes_gripper_assist(self):
        self.ctrl._feedback = (np.zeros(6), -0.6, -0.2)
        self.tick()
        for name in app.ARM_NAMES:
            self.motors[name].send_mit.assert_called_once()
        grip_command = self.motors["gripper"].send_mit.call_args.args
        self.assertEqual(grip_command[2:], (0.0, app.ASSIST_KD, 0.0))
        with self.assertRaises(ValueError):
            self.ctrl.set_gripper_mode("close")
        self.assertEqual(self.ctrl.set_gripper_mode("assist"), -0.6)

    def test_open_target_is_rate_and_feedback_lead_limited(self):
        self.assertEqual(self.ctrl.set_gripper_mode("open"), 6)
        self.ctrl._gripper_command_time -= 1
        self.tick()
        command = self.motors["gripper"].send_mit.call_args.args[0]
        self.assertGreaterEqual(command, 2)
        self.assertLessEqual(command, 2 + app.GRIPPER_MAX_LEAD)
        self.assertEqual(self.ctrl.set_gripper_mode("close"), -0.4)
        self.assertEqual(self.ctrl.set_gripper_mode("hold"), 2)

    def test_reference_j2_j3_gravity_scales(self):
        self.tick()
        self.assertAlmostEqual(self.motors["joint2"].send_mit.call_args.args[4], 0.92)
        self.assertAlmostEqual(self.motors["joint3"].send_mit.call_args.args[4], 1.96)
        self.assertAlmostEqual(self.motors["joint4"].send_mit.call_args.args[4], 3.0)

    def test_expired_feedback_sends_no_motor_command(self):
        self.ctrl._sample_time = time.monotonic() - app.MAX_SAMPLE_AGE - 1
        with self.assertRaises(RuntimeError):
            self.tick()
        for motor in self.motors.values():
            motor.send_mit.assert_not_called()

    def test_invalid_initial_gripper_position_rejected_before_enable(self):
        for name, motor in self.motors.items():
            motor.robstride_get_param_f32.return_value = -0.6 if name == "gripper" else 0
        with self.assertRaises(RuntimeError):
            self.ctrl._read_snapshot(verify_start_range=True)
        for motor in self.motors.values():
            motor.enable.assert_not_called()
            motor.send_mit.assert_not_called()
        self.ctrl._read_snapshot()
        self.assertTrue(math.isclose(self.ctrl.snapshot()[1], -0.6))

    def test_failed_feedback_is_not_replaced_by_zero(self):
        motor = Mock()
        motor.robstride_get_param_f32.side_effect = RuntimeError("CAN timeout")
        with self.assertRaisesRegex(RuntimeError, "CAN timeout"):
            app.read_position(motor)

    def test_end_disconnects_instead_of_sdk_position_hold(self):
        self.ctrl._running = True
        self.ctrl.rebotarm.disconnect = Mock()
        self.ctrl.end()
        self.ctrl.rebotarm.disconnect.assert_called_once_with()
        self.assertFalse(self.ctrl._running)

    def test_live_uses_reference_gripper_defaults(self):
        argv = ["gravity_gripper_assist.py"]
        with patch.object(sys, "argv", argv), patch.object(app, "RebotArm") as robot, \
             patch.object(app, "check_model"), patch.object(app, "run_live") as run:
            app.main()
        run.assert_called_once_with(robot.return_value, 0.0, 5.0)

    def test_check_does_not_connect_or_start_live(self):
        argv = ["gravity_gripper_assist.py", "--check"]
        with patch.object(sys, "argv", argv), patch.object(app, "RebotArm") as robot, \
             patch.object(app, "check_model") as check, patch.object(app, "run_live") as run, \
             contextlib.redirect_stdout(io.StringIO()):
            app.main()
        check.assert_called_once_with(robot.return_value)
        run.assert_not_called()

    def test_old_flags_remain_compatible(self):
        argv = ["gravity_gripper_assist.py", "--live", "--acknowledge-j2-drift"]
        with patch.object(sys, "argv", argv), patch.object(app, "RebotArm") as robot, \
             patch.object(app, "check_model"), patch.object(app, "run_live") as run:
            app.main()
        run.assert_called_once_with(robot.return_value, 0.0, 5.0)


if __name__ == "__main__":
    unittest.main()
