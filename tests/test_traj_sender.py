"""Trajectory sender regression tests without real hardware or a simulator."""
import contextlib
import io
import json
import sys
from pathlib import Path
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "reBotArm_Isaacsim"))
import isaacsim_traj_sender as traj


class TrajectorySenderTest(unittest.TestCase):
    def setUp(self):
        self.socket_patch = patch("isaacsim_traj_sender.socket.socket")
        self.socket_patch.start()
        self.sender = traj.TrajSender()
        self.sender._synced = True

    def tearDown(self):
        self.socket_patch.stop()

    def test_min_jerk_reaches_exact_joint_target(self):
        q0 = np.zeros(self.sender.model.nq)
        q1 = q0.copy()
        q1[:6] = np.radians([6, 23, 34, 6, 6, 6])
        points = self.sender._plan_joint_direct(q0, q1)
        np.testing.assert_allclose(points[0].q, q0, atol=1e-12)
        np.testing.assert_allclose(points[-1].q, q1, atol=1e-12)
        self.assertAlmostEqual(points[-1].time, 3.0)
        self.assertTrue(all(p.ik_success for p in points))
        self.assertLess(np.linalg.norm(points[1].q-q0), np.linalg.norm(points[len(points)//2].q-points[len(points)//2-1].q))

    def test_limits_and_nonfinite_never_broadcast(self):
        with patch.object(self.sender, "_broadcast_trajectory") as broadcast:
            for cmd in ("q 0 -30 0 0 0 0", "q 0 0 0 0 0 360", "q nan 0 0 0 0 0"):
                with self.subTest(cmd=cmd), self.assertRaises(ValueError):
                    self.sender._handle_command(cmd)
            broadcast.assert_not_called()

    def test_unreachable_never_broadcasts(self):
        before = self.sender.q_prev.copy()
        with patch.object(self.sender, "_broadcast_trajectory") as broadcast:
            with self.assertRaisesRegex(ValueError, "IK"):
                self.sender._handle_command("9 9 9")
            broadcast.assert_not_called()
        np.testing.assert_array_equal(self.sender.q_prev, before)

    def test_requires_feedback_and_rejects_bad_speed(self):
        self.sender._synced = False
        with self.assertRaisesRegex(RuntimeError, "resync"):
            self.sender._handle_command("q 1 0 0 0 0 0")
        for cmd in ("speed nan", "speed 0", "speed 6"):
            with self.subTest(cmd=cmd), self.assertRaises(ValueError):
                self.sender._handle_command(cmd)

    def test_feedback_prefers_observed_and_fallbacks_to_target(self):
        fake = self.sender.feedback_socket
        for payload, expected in (
            ({"type":"feedback", "joint_positions":[-1]*6, "actual_joint_positions":[-.2]*6}, .2),
            ({"type":"feedback", "joint_positions":[-.3]*6}, .3),
        ):
            fake.recvfrom.return_value = (json.dumps(payload).encode(), ("127.0.0.1", 5005))
            with contextlib.redirect_stdout(io.StringIO()):
                q = self.sender._request_feedback()
            np.testing.assert_allclose(q[:6], np.full(6, expected))

    def test_interrupted_broadcast_retains_last_sent_point(self):
        p0 = traj.JointTrajectoryPoint(0, np.zeros(self.sender.model.nq), True)
        p1 = traj.JointTrajectoryPoint(1, np.ones(self.sender.model.nq)*.1, True)
        def stop_after_first(*_):
            traj._running = False
        with patch.object(self.sender, "_send", side_effect=stop_after_first):
            try:
                self.sender._broadcast_trajectory([p0, p1])
                np.testing.assert_array_equal(self.sender.q_prev, p0.q)
            finally:
                traj._running = True


class ReceiverFeedbackTest(unittest.TestCase):
    def test_backward_compatible_target_and_actual(self):
        import importlib.util
        stub = ModuleType("isaacsim")
        stub.SimulationApp = lambda *args, **kwargs: None
        spec = importlib.util.spec_from_file_location("receiver_feedback_test", ROOT / "reBotArm_Isaacsim/isaacsim_joint_receiver.py")
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, {"isaacsim": stub}), patch("signal.signal"):
            spec.loader.exec_module(module)
        obj = module.IsaacJointMirror.__new__(module.IsaacJointMirror)
        obj.socket = Mock()
        obj.socket.recvfrom.side_effect = [(b'{"type":"feedback_request"}', ("127.0.0.1", 12345)), BlockingIOError()]
        obj.feedback_socket = Mock()
        obj.feedback_port = 5006
        obj.wire_to_asset_sign = -1.0
        obj.latest_q = np.ones(6)*.4
        obj.arm_joint_indices = np.arange(6)
        obj.articulation = SimpleNamespace(get_joint_positions=Mock(return_value=np.ones(6)*.2))
        obj._recv_latest_packet()
        payload = json.loads(obj.feedback_socket.sendto.call_args.args[0])
        np.testing.assert_allclose(payload["joint_positions"], np.ones(6)*-.4)
        np.testing.assert_allclose(payload["actual_joint_positions"], np.ones(6)*-.2)


if __name__ == "__main__":
    unittest.main()
