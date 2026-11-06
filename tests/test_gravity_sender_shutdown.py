"""Ensure SDK version changes cannot silently keep this sender connected."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gravity_sender_for_test', root/'reBotArm_Isaacsim/gravity_joint_sender.py')
module = importlib.util.module_from_spec(spec)
with patch('signal.signal'):
    spec.loader.exec_module(module)

class SenderShutdownTests(unittest.TestCase):
    def test_disconnect_and_close(self):
        sender = module.GravityCompensationSender.__new__(module.GravityCompensationSender)
        sender.rebotarm = Mock()
        sender.socket = Mock()
        sender.shutdown()
        sender.rebotarm.disconnect.assert_called_once_with()
        sender.socket.close.assert_called_once_with()

    def test_socket_closed_even_if_disconnect_fails(self):
        sender = module.GravityCompensationSender.__new__(module.GravityCompensationSender)
        sender.rebotarm = Mock()
        sender.rebotarm.disconnect.side_effect = RuntimeError('disconnect failed')
        sender.socket = Mock()
        with self.assertRaisesRegex(RuntimeError, 'disconnect failed'):
            sender.shutdown()
        sender.socket.close.assert_called_once_with()

if __name__ == '__main__':
    unittest.main()
