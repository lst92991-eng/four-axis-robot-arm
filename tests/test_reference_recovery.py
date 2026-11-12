"""Exercise legacy J1 reference recovery without a CAN adapter."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'manual_control'))
from core import CLEAR, PACKET, PING, ZERO, Controller


class ReferenceRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.now = 1.
        self.sent = []
        self.config = json.loads((ROOT / 'config/manual-jog.json').read_text(encoding='utf-8'))
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name) / 'pending-reference.json'
        self.controller = self.new_controller()
        self.position = -20041.196
        self.counts = -123441
        self.status()

    def new_controller(self):
        controller = Controller(self.config, lambda n, data: self.sent.append(PACKET.unpack(data)),
                                clock=lambda: self.now, reference_path=self.path)
        controller.connected = True
        return controller

    def status(self, uptime=1000, fault=5, counts=None, position=None, referenced=True):
        values = [0] * 14
        values[0] = uptime
        values[1] = round(1000 * (self.position if position is None else position))
        values[2] = values[1]
        values[3] = (3 << 24 if fault else 0) | 3 << 16
        values[5] = self.counts if counts is None else counts
        values[6], values[7] = 10088, 1
        flags = 16 | 32 | 256 | (2 if referenced else 0)
        values[9] = (4 if fault else 0) | fault << 8 | flags << 16
        values[10] = struct.unpack('<i', bytes.fromhex('f88bb0d3'))[0]
        values[13] = 0x43414c31
        self.controller.receive(0x201, PACKET.pack(0x40, 1, 0, 3, 0, *values))

    def ack(self, result=0):
        command = self.sent[-1]
        self.controller.receive(0x481, PACKET.pack(command[0] | 0x80, 1, result, 3, command[4], *([0] * 14)))

    def begin_clear(self):
        self.controller.action('recover', client='test', order=1, node=1)
        self.ack()
        self.assertEqual([p[0] for p in self.sent], [PING, CLEAR])
        self.assertTrue(self.path.exists())
        self.ack()

    def test_preserves_multiturn_height_after_encoder_reset(self):
        self.begin_clear()
        self.status(uptime=1050, fault=0, counts=self.counts % 16384, position=388.25)
        self.assertEqual(self.sent[-1][0], ZERO)
        self.assertEqual(self.sent[-1][5], round(self.position * 1000))
        self.assertIsNotNone(self.controller.reference_restore)
        self.ack()
        self.status(uptime=1100, fault=0, counts=self.counts % 16384)
        self.assertIsNone(self.controller.reference_restore)
        self.assertIsNone(self.controller.recovering)
        self.assertFalse(self.path.exists())
        self.assertFalse(self.controller.armed)
        self.assertEqual([p[0] for p in self.sent], [PING, CLEAR, ZERO])

    def test_tracks_small_encoder_change_during_clear(self):
        self.begin_clear()
        self.status(uptime=1050, fault=0, counts=(self.counts + 3) % 16384)
        expected = round((self.position + 3 * 360 / 16384) * 1000)
        self.assertEqual(self.sent[-1][5], expected)

    def test_reference_survives_lost_reply_and_host_restart(self):
        self.begin_clear()
        self.status(uptime=1050, fault=0, counts=self.counts % 16384, position=388.25)
        self.now += .4
        self.status(uptime=1100, fault=0, counts=self.counts % 16384, position=388.25)
        self.controller.tick()
        self.assertIn('参考恢复', self.controller.reason(1))
        self.controller = self.new_controller()
        self.status(uptime=1100, fault=0, counts=self.counts % 16384, position=388.25)
        self.controller.action('recover', client='resumed', order=1, node=1)
        self.ack()
        self.assertEqual(self.sent[-1][0], ZERO)
        self.assertEqual(self.sent[-1][5], round(self.position * 1000))
        self.ack()
        self.status(uptime=1150, fault=0, counts=self.counts % 16384)
        self.assertIsNone(self.controller.reference_restore)

    def test_stop_cancels_reference_write_and_retains_recovery_record(self):
        self.begin_clear()
        self.controller.action('stop', client='operator', order=1, disarm=True)
        self.status(uptime=1050, fault=0, counts=self.counts % 16384)
        self.assertEqual(self.sent[-1][0], CLEAR)
        self.assertTrue(self.path.exists())
        self.assertIn('参考恢复', self.controller.reason(1))

    def test_board_restart_does_not_reuse_old_reference(self):
        self.begin_clear()
        self.status(uptime=20, fault=0, counts=self.counts % 16384)
        self.assertEqual(self.sent[-1][0], CLEAR)
        self.assertIsNone(self.controller.recovering)
        self.assertTrue(self.path.exists())

    def test_excessive_encoder_change_does_not_write_reference(self):
        self.begin_clear()
        self.status(uptime=1050, fault=0, counts=(self.counts + 300) % 16384)
        self.assertEqual(self.sent[-1][0], CLEAR)
        self.assertIsNone(self.controller.recovering)
        self.assertTrue(self.path.exists())
