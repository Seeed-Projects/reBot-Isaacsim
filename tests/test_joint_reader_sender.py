"""Read-only real-to-sim protocol tests, with no physical CAN socket."""
import json
import socket
import struct
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'reBotArm_Isaacsim'))
import joint_reader_sender as reader


def group(start, values):
    return SimpleNamespace(_jcfgs=[SimpleNamespace(name=f'j{i}', motor_id=i,
        vendor='robstride') for i in range(start, start+len(values))])


class FakeCAN:
    def __init__(self):
        self.commands = []
        self.values = {i: i / 10 for i in range(1, 8)}
        self.blocking = False
        self.pending = []
        self.missing = None
        self.inject = []
    def bind(self, addr): self.addr = addr
    def setsockopt(self, *args): pass
    def setblocking(self, b): self.blocking = b
    def settimeout(self, t): self.blocking = True
    def close(self): self.closed = True
    def send(self, frame):
        self.commands.append(frame)
        cid, length, data = reader.CAN_FRAME.unpack(frame)
        mid = cid & 0xff
        if mid != self.missing:
            resp_id = socket.CAN_EFF_FLAG | (0x11 << 24) | (mid << 8) | ((cid >> 8) & 0xff)
            self.pending.append(reader.CAN_FRAME.pack(resp_id, 8, data[:4] + struct.pack('<f', self.values[mid])))
    def recv(self, count):
        if not self.blocking: raise BlockingIOError()
        if self.inject: return self.inject.pop(0)
        if self.pending: return self.pending.pop(0)
        raise socket.timeout()


class JointReaderTests(unittest.TestCase):
    def setUp(self):
        self.robot = Mock()
        self.robot._channel = 'can0'
        self.robot.arm = group(1, range(6))
        self.robot.gripper = group(7, range(1))
        self.robot.has_gripper = True
        self.can = FakeCAN()
        self.udp = Mock()
        self.robot_patch = patch.object(reader, 'RebotArm', return_value=self.robot)
        self.socket_patch = patch.object(reader.socket, 'socket', side_effect=[self.udp, self.can])
        self.robot_patch.start(); self.socket_patch.start()
        self.addCleanup(self.robot_patch.stop); self.addCleanup(self.socket_patch.stop)
        self.sender = reader.JointReaderSender()
        self.sender.setup_hardware()
        reader._running = True
        self.addCleanup(setattr, reader, '_running', True)

    def test_only_read_requests_and_matching_feedback(self):
        self.sender.read_positions()
        np.testing.assert_allclose(self.sender.latest_q, -np.arange(1, 7)/10, atol=1e-6)
        self.assertAlmostEqual(self.sender.latest_gripper_position, .7*.009)
        self.assertEqual(len(self.can.commands), 7)
        for frame, motor in zip(self.can.commands, range(1, 8)):
            cid, length, data = reader.CAN_FRAME.unpack(frame)
            self.assertEqual(cid, socket.CAN_EFF_FLAG | 0x1100fc00 | motor)
            self.assertEqual((length, data), (8, bytes.fromhex('1970000000000000')))
        self.sender.shutdown()
        self.robot.connect.assert_not_called(); self.robot.disable_all.assert_not_called()
        self.robot.enable_all.assert_not_called(); self.robot.disconnect.assert_not_called()
        self.assertTrue(self.can.closed)

    def test_missing_or_invalid_reply_does_not_replace_sample(self):
        self.sender.read_positions(); old = self.sender.latest_q.copy()
        self.can.missing = 7
        with self.assertRaises(reader.FeedbackError): self.sender.read_positions()
        np.testing.assert_array_equal(self.sender.latest_q, old)
        self.can.missing = None; self.can.values[7] = float('nan')
        with self.assertRaises(reader.FeedbackError): self.sender.read_positions()

    def test_gripper_endpoint_mapping_is_only_a_visual_conversion(self):
        self.sender.gripper_closed, self.sender.gripper_open = -.479, 5.987
        for raw, expected in [(-1.0, 0), (-.479, 0), (2.754, .0225), (5.987, .045), (6.5, .045)]:
            self.can.values[7] = raw
            self.sender.read_positions()
            self.assertAlmostEqual(self.sender.latest_gripper_position, expected, places=5)
        self.robot.enable_all.assert_not_called()

    def test_wrong_host_or_motor_is_ignored(self):
        bad_id = socket.CAN_EFF_FLAG | 0x110001fd
        self.can.inject.append(reader.CAN_FRAME.pack(bad_id, 8, reader.PARAM_REQUEST[:4]+struct.pack('<f', 100)))
        self.sender.read_positions()
        self.assertAlmostEqual(self.sender.latest_q[0], -.1, places=6)

    def test_error_pauses_and_recovery_sends_one_packet(self):
        self.sender.read_positions = Mock(side_effect=[reader.FeedbackError('timeout'), None])
        self.udp.sendto.side_effect = lambda *args: setattr(reader, '_running', False)
        self.sender.run(send_hz=1000)
        self.assertEqual(self.sender.read_positions.call_count, 2)
        self.assertEqual(self.udp.sendto.call_count, 1)
        self.assertEqual(json.loads(self.udp.sendto.call_args.args[0])['sequence'], 0)

if __name__ == '__main__': unittest.main()
