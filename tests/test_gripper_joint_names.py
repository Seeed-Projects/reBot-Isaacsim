"""Check gripper joint aliases without starting Isaac Sim or real hardware."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
import xml.etree.ElementTree as ET
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
stub = ModuleType('isaacsim')
stub.SimulationApp = lambda *args, **kwargs: None
spec = importlib.util.spec_from_file_location('receiver_for_test', ROOT / 'reBotArm_Isaacsim/isaacsim_joint_receiver.py')
receiver = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {'isaacsim': stub}), patch('signal.signal'):
    spec.loader.exec_module(receiver)


class GripperNamesTest(unittest.TestCase):
    def make_mirror(self):
        mirror = receiver.IsaacJointMirror.__new__(receiver.IsaacJointMirror)
        mirror.articulation = SimpleNamespace(
            dof_properties={'lower': np.zeros(4), 'upper': np.array([0, 0.05, 0, 0.05])},
            set_joint_positions=Mock(),
        )
        mirror._last_gripper_command_signature = None
        return mirror

    def test_both_name_versions_map_and_drive_the_same_indices(self):
        for pair in (('gripper_joint1', 'gripper_joint2'), ('joint_left', 'joint_right')):
            with self.subTest(pair=pair), contextlib.redirect_stdout(io.StringIO()):
                mirror = self.make_mirror()
                mirror._setup_gripper_mapping(['joint1', pair[1], 'joint2', pair[0]])
                self.assertEqual(mirror.gripper_joint_names, pair)
                np.testing.assert_array_equal(mirror.gripper_joint_indices, [3, 1])
                mirror._apply_gripper_target(0.02)
                call = mirror.articulation.set_joint_positions.call_args
                np.testing.assert_allclose(call.args[0], [0.02, 0.02])
                np.testing.assert_array_equal(call.kwargs['joint_indices'], [3, 1])

    def test_wire_sign_round_trip_for_new_and_legacy_assets(self):
        for sign in (-1.0, 1.0):
            with self.subTest(sign=sign):
                mirror = self.make_mirror()
                mirror.wire_to_asset_sign = sign
                wire = np.array([-0.1, -0.3, -0.4, 0.2, 0.0, 0.1])
                command = json.dumps({"sequence": 42, "joint_positions": wire.tolist(), "gripper_position": 0.02}).encode()
                mirror.socket = Mock()
                mirror.socket.recvfrom.side_effect = [(command, ("127.0.0.1", 1000)), BlockingIOError()]
                positions, seq, grip = mirror._recv_latest_packet()
                np.testing.assert_allclose(positions, sign * wire)
                self.assertEqual((seq, grip), (42, 0.02))
                mirror.latest_q = positions
                mirror.feedback_port = 5006
                mirror.feedback_socket = Mock()
                request = json.dumps({"type": "feedback_request"}).encode()
                mirror.socket.recvfrom.side_effect = [(request, ("127.0.0.1", 1000)), BlockingIOError()]
                mirror._recv_latest_packet()
                response = json.loads(mirror.feedback_socket.sendto.call_args.args[0])
                np.testing.assert_allclose(response["joint_positions"], wire)

    def test_rs_urdf_limits_are_symmetric_in_project_and_sdk(self):
        for path in (ROOT/'urdf/RS/urdf/ReBot_Arm_RS.urdf',
                     ROOT/'third_party/reBotArm_control_py/urdf/RS/urdf/ReBot_Arm_RS.urdf'):
            with self.subTest(path=path):
                robot = ET.parse(path).getroot()
                for name in ('gripper_joint1', 'gripper_joint2'):
                    limit = robot.find(f"./joint[@name='{name}']/limit")
                    self.assertIsNotNone(limit)
                    self.assertAlmostEqual(float(limit.get('lower')), 0.0)
                    self.assertAlmostEqual(float(limit.get('upper')), 0.05)

    def test_oversized_gripper_command_clamped_for_both_fingers(self):
        mirror = self.make_mirror()
        with contextlib.redirect_stdout(io.StringIO()):
            mirror._setup_gripper_mapping(['joint1', 'gripper_joint2', 'joint2', 'gripper_joint1'])
            mirror._apply_gripper_target(0.0715)
        positions = mirror.articulation.set_joint_positions.call_args.args[0]
        np.testing.assert_allclose(positions, [0.05, 0.05])

    def test_incomplete_pair_does_not_control_gripper(self):
        mirror = self.make_mirror()
        with contextlib.redirect_stdout(io.StringIO()):
            mirror._setup_gripper_mapping(['gripper_joint1', 'joint_right'])
            mirror._apply_gripper_target(0.02)
        self.assertIsNone(mirror.gripper_joint_indices)
        mirror.articulation.set_joint_positions.assert_not_called()

if __name__ == '__main__':
    unittest.main()
