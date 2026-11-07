"""Offline checks for the gravity/hand-assist example; never connects to CAN."""
import math
import contextlib
import io
from pathlib import Path
import sys
import time
import threading
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

    def test_home_keeps_existing_gripper_mode_and_goal(self):
        self.ctrl._arm_velocity = np.zeros(6)
        self.ctrl.set_gripper_mode("open")
        original_command = self.ctrl._gripper_command
        model = SimpleNamespace(lowerPositionLimit=np.full(6, -math.pi),
                                upperPositionLimit=np.full(6, math.pi))
        with patch.object(app, "load_robot_model", return_value=model):
            self.ctrl.request_home()
        self.assertEqual(self.ctrl._gripper_mode, "open")
        self.assertEqual(self.ctrl._gripper_goal, 6.0)
        self.assertEqual(self.ctrl._gripper_command, original_command)
        self.tick()
        command = self.motors["gripper"].send_mit.call_args.args
        self.assertEqual(command[2], 50.0)
        self.assertEqual(command[3], 4.0)

    def test_home_does_not_change_gripper_assist_mode(self):
        self.ctrl._arm_velocity = np.zeros(6)
        model = SimpleNamespace(lowerPositionLimit=np.full(6, -math.pi),
                                upperPositionLimit=np.full(6, math.pi))
        with patch.object(app, "load_robot_model", return_value=model):
            self.ctrl.request_home()
        self.assertEqual(self.ctrl._gripper_mode, "assist")
        self.tick()
        cmd = self.motors["gripper"].send_mit.call_args.args
        self.assertEqual(cmd[2], 0.0)
        self.assertAlmostEqual(cmd[4], app.assist_torque(2.0, .1, -.4, 6.0))

    def test_home_uses_sdk_per_joint_holding_gains_without_zeroing_gravity(self):
        base = time.monotonic()
        self.ctrl._initial_kp = np.array([50, 150, 150, 50, 50, 50], dtype=float)
        self.ctrl._initial_kd = np.array([3, 10, 10, 5, 4, 4], dtype=float)
        self.ctrl._home_plan = app.HomePlan(np.zeros(6), base - app.HOME_PREP_SECONDS)
        self.ctrl._home_start_kp = np.full(6, 1.5)
        self.ctrl._home_start_kd = np.full(6, 1.0)
        self.ctrl._home_plan.last_step_at = base
        self.ctrl._feedback = (np.zeros(6), 2.0, 0.0)
        self.ctrl._sample_time = base
        with patch.object(app.time, "monotonic", return_value=base), \
             patch.object(app, "compute_generalized_gravity", return_value=np.arange(8.0)):
            self.ctrl._loop_cb(self.robot, .008)
        kp = [self.motors[n].send_mit.call_args.args[2] for n in app.ARM_NAMES]
        kd = [self.motors[n].send_mit.call_args.args[3] for n in app.ARM_NAMES]
        tau = [self.motors[n].send_mit.call_args.args[4] for n in app.ARM_NAMES]
        np.testing.assert_allclose(kp, self.ctrl._initial_kp)
        np.testing.assert_allclose(kd, self.ctrl._initial_kd)
        np.testing.assert_allclose(tau, np.arange(6.0) * app.GRAVITY_SCALE)

    def test_feedback_polls_faster_only_during_home(self):
        for is_homing, expected in ((False, .05), (True, .015)):
            with self.subTest(is_homing=is_homing):
                ctrl = app.AssistedGravity(self.robot, -.4, 6.0)
                ctrl._home_plan = object() if is_homing else None
                ctrl._read_snapshot = Mock()
                ctrl._stop_feedback = Mock()
                ctrl._stop_feedback.is_set.side_effect = [False, True]
                ctrl._poll_feedback()
                ctrl._read_snapshot.assert_called_once_with()
                ctrl._stop_feedback.wait.assert_called_once_with(expected)

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


class HomePlanTests(unittest.TestCase):
    def test_home_trajectory_is_smooth_bounded_and_settles(self):
        q0 = np.array([0.8, 0.4, 0.3, -0.2, 0.1, -0.5])
        plan = app.HomePlan(q0, 100.0)
        dt = .008
        previous = q0.copy()
        targets = []
        for i in range(1, 10000):
            target, _ = plan.step(100 + i * dt, previous)
            targets.append(target.copy())
            previous = target.copy()
            if plan.done:
                break
        self.assertTrue(plan.done)
        np.testing.assert_allclose(targets[-1], np.zeros(6), atol=1e-12)
        v = np.diff(targets, axis=0) / dt
        a = np.diff(v, axis=0) / dt
        jerk = np.diff(a, axis=0) / dt
        self.assertLessEqual(np.max(np.abs(v)), app.HOME_MAX_VELOCITY * 1.01)
        self.assertLessEqual(np.max(np.abs(a)), app.HOME_MAX_ACCELERATION * 1.05)
        self.assertLessEqual(np.max(np.abs(jerk)), app.HOME_MAX_JERK * 1.15)
        self.assertTrue(np.all(np.diff(np.array(targets)[:, 0]) <= 1e-10))

    def test_sdk_style_four_second_target_and_long_distance_speed_limit(self):
        start = np.radians([0, 58, 41, 0, 0, 0])
        plan = app.HomePlan(start, 0.0)
        self.assertAlmostEqual(plan.duration, 4.0)
        q = start.copy()
        for i in range(1, 1000):
            q, _ = plan.step(i * .008, q)
            if plan.finished_at is not None:
                break
        self.assertAlmostEqual(plan.finished_at - app.HOME_PREP_SECONDS, 4.0, delta=.016)
        self.assertAlmostEqual(q[1], 0.0, delta=1e-8)
        long_plan = app.HomePlan(np.radians([0, 90, 0, 0, 0, 0]), 0.0)
        self.assertGreater(long_plan.duration, 4.0)
        self.assertLessEqual(1.875 * long_plan.distance / long_plan.duration,
                             app.HOME_MAX_VELOCITY)

    def test_slow_motor_trips_tracking_guard_without_forced_zero(self):
        start = np.radians([0, 58, 41, 0, 0, 0])
        plan = app.HomePlan(start, 0.0)
        physical = start.copy()
        measured = physical.copy()
        with self.assertRaisesRegex(RuntimeError, "跟踪误差"):
            for i in range(1, 5000):
                if i % 10 == 0:
                    measured = physical.copy()
                target, _ = plan.step(i * .008, measured)
                physical += np.clip((target - physical) * 8 * .008,
                                    -math.radians(8) * .008,
                                    math.radians(8) * .008)
        self.assertGreater(abs(plan.target[1]), app.HOME_SETTLE_TOLERANCE)
        self.assertFalse(plan.done)

    def test_stalled_motor_holds_reference_and_never_forces_zero(self):
        start = np.radians([0, 30, 0, 0, 0, 0])
        plan = app.HomePlan(start, 0.0)
        with self.assertRaisesRegex(RuntimeError, "未跟上|跟踪误差"):
            for i in range(1, 3000):
                plan.step(i * .008, start)
        self.assertGreater(plan.target[1], 0)
        self.assertLessEqual(abs(plan.target[1] - start[1]), app.HOME_TRACKING_ERROR + app.HOME_MAX_VELOCITY * .008)
        self.assertFalse(plan.done)

    def test_tracking_error_aborts_without_forcing_home(self):
        plan = app.HomePlan(np.array([.1, 0, 0, 0, 0, 0]), 10)
        with self.assertRaisesRegex(RuntimeError, "跟踪误差"):
            plan.step(10.05, np.array([.2, 0, 0, 0, 0, 0]))
        self.assertFalse(plan.done)

    def test_measured_position_within_tolerance_must_stay_settled(self):
        plan = app.HomePlan(np.zeros(6), 10)
        end_t = 10 + app.HOME_PREP_SECONDS + plan.duration
        q = np.full(6, math.radians(.7))
        for now in np.arange(10.01, end_t + app.HOME_SETTLE_SECONDS + .05, .01):
            plan.step(float(now), q)
        self.assertTrue(plan.done)
        self.assertLessEqual(np.max(np.abs(q)), app.HOME_SETTLE_TOLERANCE)

    def test_failed_settle_raises_without_claiming_success(self):
        plan = app.HomePlan(np.zeros(6), 10)
        t = 10 + app.HOME_PREP_SECONDS + plan.duration + app.HOME_SETTLE_TIMEOUT + 2.0
        with self.assertRaisesRegex(RuntimeError, "未稳定到位"):
            for now in np.arange(10.01, t, .01):
                plan.step(float(now), np.full(6, app.HOME_SETTLE_TOLERANCE * 1.1))
        self.assertFalse(plan.done)

    def test_home_and_quit_disconnect_only_after_confirmed_home(self):
        for command in ("home", "quit"):
            with self.subTest(command=command):
                robot = SimpleNamespace(control_loop_active=True)
                ctrl = SimpleNamespace(_lock=threading.Lock(), _home_status="idle", _home_message="",
                    _sample_time=time.monotonic(), _home_plan=None, start=Mock(), end=Mock(),
                    snapshot=Mock(return_value=(np.zeros(6), 0.0, 0.0)), home_stationary=Mock(return_value=True))
                def request_home():
                    ctrl._home_status = "done"
                    ctrl._home_message = "到位"
                    return 2.0
                ctrl.request_home = Mock(side_effect=request_home)
                stream = io.StringIO(command + "\n")
                with patch.object(app, "AssistedGravity", return_value=ctrl), \
                     patch.object(app, "known_conflicting_processes", return_value=[]), \
                     patch.object(app.socket, "socket"), patch.object(app.signal, "signal"), \
                     patch.object(app.select, "select", return_value=([stream], [], [])), \
                     patch.object(sys, "stdin", stream), \
                     patch.object(app, "compute_generalized_gravity", return_value=np.zeros(8)), \
                     patch.object(app, "HOME_STILL_SECONDS", 0.0), \
                     contextlib.redirect_stdout(io.StringIO()):
                    app.run_live(robot, 0, 5)
                ctrl.request_home.assert_called_once_with()
                ctrl.end.assert_called_once_with()
                self.assertEqual(ctrl._home_status, "done")

    def test_quit_waits_for_measured_stillness_before_home(self):
        robot = SimpleNamespace(control_loop_active=True)
        ctrl = SimpleNamespace(_lock=threading.Lock(), _home_status="idle", _home_message="",
            _sample_time=time.monotonic(), _home_plan=None, start=Mock(), end=Mock(),
            snapshot=Mock(return_value=(np.zeros(6), 0.0, 0.0)))
        ctrl.home_stationary = Mock(side_effect=[False, False, False] + [True] * 100)
        def request_home():
            self.assertGreaterEqual(ctrl.home_stationary.call_count, 4)
            ctrl._home_status, ctrl._home_message = "done", "到位"
            return 2.0
        ctrl.request_home = Mock(side_effect=request_home)
        stream = io.StringIO("quit\n")
        with patch.object(app, "AssistedGravity", return_value=ctrl), \
             patch.object(app, "known_conflicting_processes", return_value=[]), \
             patch.object(app.socket, "socket"), patch.object(app.signal, "signal"), \
             patch.object(app.select, "select", return_value=([stream], [], [])), \
             patch.object(sys, "stdin", stream), patch.object(app, "HOME_STILL_SECONDS", .05), \
             patch.object(app, "compute_generalized_gravity", return_value=np.zeros(8)), \
             contextlib.redirect_stdout(io.StringIO()):
            app.run_live(robot, 0, 5)
        ctrl.request_home.assert_called_once_with()
        ctrl.end.assert_called_once_with()

    def test_interrupt_retries_after_rejected_or_aborted_home(self):
        import signal
        for failure in ("rejected", "aborted"):
            with self.subTest(failure=failure):
                robot = SimpleNamespace(control_loop_active=True)
                ctrl = SimpleNamespace(_lock=threading.Lock(), _home_status="idle", _home_message="",
                    _sample_time=time.monotonic(), _home_plan=None, end=Mock(), home_stationary=Mock(return_value=True))
                handlers = {}
                ctrl.start = Mock(side_effect=lambda: handlers[signal.SIGINT](signal.SIGINT, None))
                count = [0]
                retry_sent = [False]
                def snapshot():
                    count[0] += 1
                    if ctrl.request_home.call_count == 1 and count[0] > 1 and not retry_sent[0]:
                        retry_sent[0] = True
                        handlers[signal.SIGINT](signal.SIGINT, None)
                    return np.zeros(6), 0.0, 0.0
                ctrl.snapshot = Mock(side_effect=snapshot)
                attempts = [0]
                def request_home():
                    attempts[0] += 1
                    if attempts[0] == 1:
                        if failure == "rejected":
                            raise RuntimeError("关节仍在运动")
                        ctrl._home_status, ctrl._home_message = "aborted", "跟踪误差"
                    else:
                        ctrl._home_status, ctrl._home_message = "done", "到位"
                    return 2.0
                ctrl.request_home = Mock(side_effect=request_home)
                def register(sig, handler): handlers[sig] = handler
                with patch.object(app, "AssistedGravity", return_value=ctrl), \
                     patch.object(app, "known_conflicting_processes", return_value=[]), \
                     patch.object(app.socket, "socket"), patch.object(app.signal, "signal", side_effect=register), \
                     patch.object(app.select, "select", return_value=([], [], [])), \
                     patch.object(app, "compute_generalized_gravity", return_value=np.zeros(8)), \
                     patch.object(app, "HOME_STILL_SECONDS", 0.0), \
                     contextlib.redirect_stdout(io.StringIO()):
                    app.run_live(robot, 0, 5)
                self.assertEqual(ctrl.request_home.call_count, 2)
                self.assertEqual(ctrl._home_status, "done")
                ctrl.end.assert_called_once_with()

    def test_scheduling_gap_aborts_instead_of_jumping_to_a_later_target(self):
        plan = app.HomePlan(np.ones(6) * .1, 10.0)
        with self.assertRaisesRegex(RuntimeError, "延迟过长"):
            plan.step(10.3, np.ones(6) * .1)

    def test_first_interrupt_requests_home_second_stops_immediately(self):
        import signal
        requests = app.StopRequests()
        requests.on_signal(signal.SIGINT, None)
        self.assertTrue(requests.graceful.is_set())
        self.assertFalse(requests.immediate.is_set())
        requests.on_signal(signal.SIGINT, None)
        self.assertTrue(requests.immediate.is_set())
        other = app.StopRequests()
        other.on_signal(signal.SIGTERM, None)
        self.assertTrue(other.immediate.is_set())
        self.assertFalse(other.graceful.is_set())

    def test_aborted_home_holds_measured_pose_without_zero_command(self):
        motors = {name: Mock() for name in (*app.ARM_NAMES, "gripper")}
        robot = SimpleNamespace(_motor_map=motors, gripper=SimpleNamespace(_mit_kp=[50], _mit_kd=[4]))
        ctrl = app.AssistedGravity(robot, 0, 5)
        measured = np.array([.1, 0, 0, 0, 0, 0])
        ctrl._feedback = (measured, 0.5, 0.0)
        ctrl._sample_time = time.monotonic()
        ctrl._transition_start = time.monotonic() - 2
        ctrl._hold_q = measured.copy()
        ctrl._initial_kp = np.full(6, 50.0)
        ctrl._initial_kd = np.full(6, 3.0)
        ctrl._home_plan = app.HomePlan(np.zeros(6), time.monotonic())
        ctrl._home_start_kp = np.full(6, 1.5)
        ctrl._home_start_kd = np.full(6, 1.0)
        ctrl._home_status = "running"
        with patch.object(app, "compute_generalized_gravity", return_value=np.zeros(8)):
            ctrl._loop_cb(robot, .008)
        self.assertEqual(ctrl._home_status, "aborted")
        self.assertIsNone(ctrl._home_plan)
        self.assertAlmostEqual(motors['joint1'].send_mit.call_args.args[0], .1)
        np.testing.assert_allclose(ctrl._home_hold_q, measured)
        ctrl._feedback = (measured + np.array([.01, 0, 0, 0, 0, 0]), 0.5, 0.0)
        ctrl._sample_time = time.monotonic()
        with patch.object(app, "compute_generalized_gravity", return_value=np.zeros(8)):
            ctrl._loop_cb(robot, .008)
        self.assertAlmostEqual(motors['joint1'].send_mit.call_args.args[0], .1)
        self.assertEqual(ctrl._home_status, "aborted")


if __name__ == "__main__":
    unittest.main()
