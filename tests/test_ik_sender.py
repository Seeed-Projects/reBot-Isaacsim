"""IK sender protocol and failure behavior without a running robot or simulator."""
import json
import sys
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "reBotArm_Isaacsim"))
from isaacsim_ik_sender import IKSender, compute_fk


class TestIKSender(unittest.TestCase):
    def setUp(self):
        self.socket_patch = patch("isaacsim_ik_sender.socket.socket")
        self.socket_class = self.socket_patch.start()
        self.sender = IKSender()
        self.sock = self.socket_class.return_value

    def tearDown(self):
        self.socket_patch.stop()

    def packets(self):
        return [json.loads(call.args[0]) for call in self.sock.sendto.call_args_list]

    def test_direct_command_updates_seed_and_scales_gripper(self):
        self.sender._handle_command("gripper 0.5")
        self.sender._handle_command("q 10 20 30 0 0 0")
        np.testing.assert_allclose(self.sender.q_prev[:6], np.radians([10, 20, 30, 0, 0, 0]))
        pkt = self.packets()[-1]
        self.assertAlmostEqual(pkt["gripper_position"], 0.0225)
        np.testing.assert_allclose(pkt["joint_positions"], -self.sender.q_prev[:6])

    def test_position_command_preserves_last_command_orientation(self):
        self.sender._handle_command("q 10 30 40 10 -5 8")
        before = self.sender.q_prev.copy()
        pos, expected_rot, _ = compute_fk(self.sender.model, before)
        self.sender._handle_command(" ".join(map(str, pos)))
        _, actual_rot, _ = compute_fk(self.sender.model, self.sender.q_prev)
        np.testing.assert_allclose(actual_rot, expected_rot, atol=1e-4)

    def test_reachable_pose_near_singular_converges(self):
        import pinocchio as pin
        rng = np.random.default_rng(43)
        for _ in range(18):
            arm = [rng.uniform(-1, 1), rng.uniform(.15, 2.2),
                   rng.uniform(.15, 2.2), rng.uniform(-1, 1),
                   rng.uniform(-1, 1), rng.uniform(-1.5, 1.5)]
        q = np.zeros(self.sender.model.nq)
        q[:6] = arm
        pos, rot, _ = compute_fk(self.sender.model, q)
        rpy = np.degrees(pin.rpy.matrixToRpy(rot))
        result = self.sender._handle_command(" ".join(map(str, list(pos) + list(rpy))))
        self.assertIn("success=True", result)
        self.assertEqual(len(self.packets()), 1)
        solved_pos, solved_rot, _ = compute_fk(self.sender.model, self.sender.q_prev)
        np.testing.assert_allclose(solved_pos, pos, atol=1e-4)
        np.testing.assert_allclose(solved_rot, rot, atol=1e-4)

    def test_unreachable_does_not_send_or_change_seed(self):
        before = self.sender.q_prev.copy()
        self.sender._handle_command("9 9 9")
        self.assertEqual(len(self.packets()), 0)
        np.testing.assert_array_equal(self.sender.q_prev, before)

    def test_nonfinite_input_does_not_send(self):
        with self.assertRaises(ValueError):
            self.sender._handle_command("q nan 0 0 0 0 0")
        self.assertEqual(len(self.packets()), 0)


if __name__ == "__main__":
    unittest.main()
