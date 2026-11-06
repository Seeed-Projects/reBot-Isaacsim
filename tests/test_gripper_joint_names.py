"""Check gripper joint aliases without starting Isaac Sim or real hardware."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
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

    def test_incomplete_pair_does_not_control_gripper(self):
        mirror = self.make_mirror()
        with contextlib.redirect_stdout(io.StringIO()):
            mirror._setup_gripper_mapping(['gripper_joint1', 'joint_right'])
            mirror._apply_gripper_target(0.02)
        self.assertIsNone(mirror.gripper_joint_indices)
        mirror.articulation.set_joint_positions.assert_not_called()

if __name__ == '__main__':
    unittest.main()
