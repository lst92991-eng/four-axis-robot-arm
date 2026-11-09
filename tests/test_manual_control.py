"""Offline safety/protocol regressions. These tests never open USB."""
import json
from pathlib import Path
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'manual_control'))
from core import Controller, MOVE, STOP, PING, PACKET


class JogTests(unittest.TestCase):
    def setUp(self):
        self.now, self.sent = 1., []
        config = json.loads((ROOT / 'config/manual-jog.json').read_text(encoding='utf-8'))
        for axis in config['axes'].values():
            axis.pop('diagnostic_step_deg', None)  # Exercise normal jogging separately from the diagnostic test.
            axis['minimum']=-180;axis['maximum']=180
            axis['manual_direction']=1;axis['max_speed_deg_s']=30
            axis['manual_mode']='position'
        self.c = Controller(config, lambda n, data: self.sent.append((n, PACKET.unpack(data))), clock=lambda: self.now)
        self.c.connected = True
        self.order = 0
        self.status()

    def status(self, node=1, state=0, flags=1 | 2 | 16 | 32 | 256, uptime=None, build=None, position=0, fault=0):
        v = [0] * 14
        v[0] = int(self.now * 1000) if uptime is None else uptime
        v[1] = round(position*1000)
        v[9] = state | fault << 8 | flags << 16
        v[10] = struct.unpack('<i', struct.pack('<I', build if build is not None else int(self.c.axes[node]['builds'][0], 16)))[0]
        self.c.receive(0x200+node, PACKET.pack(0x40, node, 0, 3, 0, *v))

    def action(self, name, **args):
        self.order += 1
        self.c.action(name, client='test', order=self.order, **args)

    def ack(self, index=-1, rejected=False):
        n, p = self.sent[index]
        self.c.receive(0x480+n, PACKET.pack(p[0] | 0x80, n, int(rejected), 3, p[4], *([0]*14)))

    def arm(self):
        self.action('arm')
        self.ack()

    def jog(self):
        self.action('jog', node=1, direction=1, speed=2, press='press1')

    def test_passive_connection_sends_nothing(self):
        self.now += 1
        self.c.tick()
        self.assertEqual(self.sent, [])

    def test_unconfigured_does_not_send_ping_or_move(self):
        self.status(flags=1 | 32 | 256)
        with self.assertRaisesRegex(ValueError, '没有满足'):
            self.arm()
        self.assertEqual(self.sent, [])

    def test_no_reference_is_not_displayed_as_ready(self):
        self.status(flags=1 | 16 | 32 | 256)
        self.assertIn('机械参考', self.c.reason(1))

    def test_jog_only_one_move_then_ping(self):
        self.arm(); self.jog(); self.ack()
        self.status(state=2, flags=1 | 2 | 16 | 32 | 64)
        for _ in range(10):
            self.now += .1
            self.status(state=2, flags=1 | 2 | 16 | 32 | 64)
            self.action('heartbeat', press='press1')
            self.c.tick(); self.ack()
        moves = [p for n, p in self.sent if p[0] == MOVE]
        self.assertEqual(len(moves), 1)
        self.assertEqual(moves[0][5:7], (180000, 2000))

    def test_release_before_first_move_status_still_sends_stop(self):
        self.arm(); self.jog()
        self.action('release', press='press1')
        self.assertEqual(self.sent[-1][1][0], STOP)
        self.assertIsNone(self.c.active)

    def test_lease_loss_stops_without_more_pings(self):
        self.arm(); self.jog(); self.ack()
        self.now += .301
        self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.sent[-1][1][0], STOP)
        count = len(self.sent)
        self.now += .1; self.c.tick()
        self.assertEqual(len(self.sent), count)

    def test_stale_status_stops_even_if_browser_healthy(self):
        self.arm(); self.jog(); self.ack()
        self.now += .251
        self.action('heartbeat', press='press1'); self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertEqual(self.sent[-1][1][0], STOP)

    def test_wrong_pressed_identity_stops(self):
        self.arm(); self.jog(); self.ack()
        self.action('heartbeat', press=None)
        self.assertFalse(self.c.armed)
        self.assertEqual(self.sent[-1][1][0], STOP)

    def test_late_start_cannot_overtake_stop(self):
        self.arm()
        self.c.action('stop', client='test', order=100)
        with self.assertRaises(ValueError):
            self.c.action('jog', client='test', order=99, node=1, direction=1, speed=1, press='late')
        self.assertNotIn(MOVE, [p[0] for _, p in self.sent])

    def test_ack_loss_never_retries_move(self):
        self.arm(); self.jog()
        self.now += .251
        self.status(state=2, flags=1 | 2 | 16 | 32 | 64)
        self.action('heartbeat', press='press1'); self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertEqual([p[0] for _, p in self.sent].count(MOVE), 1)
        self.assertEqual(self.sent[-1][1][0], STOP)

    def test_stop_ack_is_not_stop_confirmation(self):
        self.arm(); self.jog(); self.ack()
        self.status(state=2, flags=1 | 2 | 16 | 32 | 64)
        self.action('release', press='press1'); self.ack()
        self.assertIn(1, self.c.stopping)
        self.status(state=1, flags=1 | 2 | 16 | 32 | 64 | 256)
        self.assertNotIn(1, self.c.stopping)

    def test_old_status_before_stop_ack_cannot_unlock_next_move(self):
        self.arm(); self.jog(); self.ack()
        self.action('release', press='press1')
        self.status(state=0)
        self.assertIn(1, self.c.stopping)

    def test_full_stop_allows_next_explicit_press_after_confirmed_hold(self):
        self.arm(); self.jog(); self.ack()
        self.action('stop'); self.ack()
        self.status(state=1, flags=1 | 2 | 16 | 32 | 64 | 256)
        self.assertTrue(self.c.armed)
        self.jog()
        self.assertIsNotNone(self.c.active)

    def test_reboot_requires_explicit_rearm(self):
        self.arm(); self.jog(); self.ack()
        self.status(uptime=5)
        self.assertFalse(self.c.armed)
        self.assertNotIn(1, self.c.synced)

    def test_sequence_handshake_uses_only_ping(self):
        self.action('arm')
        for _ in range(3):
            self.ack(rejected=True)
        self.ack()
        self.assertIn(1, self.c.synced)
        self.assertEqual([p[0] for _, p in self.sent], [PING]*4)
        self.assertEqual(len({p[4] for _, p in self.sent}), 4)

    def test_limits_and_invalid_speed(self):
        self.arm()
        for speed in (0, -1, 31, float('nan'), float('inf')):
            with self.subTest(speed=speed), self.assertRaises(ValueError):
                self.action('jog', node=1, direction=1, speed=speed, press='p')
        self.status(position=180)
        with self.assertRaisesRegex(ValueError, '边界'):
            self.jog()
        self.assertNotIn(MOVE, [p[0] for _, p in self.sent])

    def test_second_browser_cannot_move(self):
        self.arm()
        with self.assertRaises(ValueError):
            self.c.action('jog', client='other', order=1, node=1, direction=1, speed=1, press='p')

    def test_unknown_build_blocked(self):
        self.status(build=123)
        self.assertIn('固件版本未核对', self.c.reason(1))

    def test_single_axis_at_a_time(self):
        self.status(node=2)
        self.action('arm'); self.ack(0); self.ack(1)
        self.jog()
        with self.assertRaisesRegex(ValueError, '一次只能'):
            self.action('jog', node=2, direction=-1, speed=2, press='p2')

    def test_no_off_zero_clear_command_in_normal_lifecycle(self):
        self.arm(); self.jog(); self.ack(); self.action('stop'); self.ack()
        self.assertTrue(all(p[0] in (MOVE, STOP, PING) for _, p in self.sent))

    def test_fault_is_visible_even_if_reference_was_invalidated(self):
        self.status(state=4, fault=4, flags=1 | 16 | 32)
        self.assertIn('故障 4', self.c.reason(1))

    def test_fault_shutdown_does_not_require_rejected_stop(self):
        self.arm(); self.jog(); self.ack()
        self.status(state=4, fault=5, flags=1 | 2 | 16 | 32)
        self.c.tick()
        self.assertFalse(self.c.armed)
        self.assertFalse(self.c.stopping)
        self.assertNotIn(STOP, [p[0] for _, p in self.sent])

    def test_fault_shutdown_during_braking_is_not_reported_as_holding(self):
        self.arm(); self.jog(); self.ack(); self.action('release', press='press1')
        self.status(state=4, fault=5, flags=1 | 2 | 16 | 32)
        self.assertFalse(self.c.stopping)
        self.assertTrue(any('未据此判定机械停稳' in e['message'] for e in self.c.events))

    def test_diagnostic_step_is_explicit_and_clamped_to_limits(self):
        self.c.axes[1]['diagnostic_step_deg']=.3
        self.arm(); self.jog()
        self.assertEqual(self.sent[-1][1][5],300)
        self.assertIn('诊断最多',self.c.message)

    def test_diagnostic_step_never_extends_past_firmware_limit(self):
        self.c.axes[1]['diagnostic_step_deg']=.3
        self.status(position=179.85)
        self.arm(); self.jog()
        self.assertEqual(self.sent[-1][1][5],180000)

    def test_reviewed_ten_turn_profile_uses_motor_degrees(self):
        config=json.loads((ROOT/'config/manual-jog.json').read_text(encoding='utf-8'))
        config['axes']['1'].update(minimum=-3600,maximum=3600,manual_direction=1,manual_mode='position')
        self.c=Controller(config,lambda n,data:self.sent.append((n,PACKET.unpack(data))),clock=lambda:self.now)
        self.c.connected=True;self.status();self.arm();self.jog()
        self.assertEqual(self.sent[-1][1][5],3600000)

    def test_j1_up_maps_to_decreasing_motor_angle(self):
        self.c.axes[1]['manual_direction']=-1
        self.arm();self.jog()
        self.assertEqual(self.sent[-1][1][5],-180000)
        self.status(position=-3)
        self.assertEqual(self.c.snapshot()['axes'][0]['display_position'],3)

    def test_velocity_mode_has_no_fixed_position_endpoint(self):
        self.c.axes[1].update(manual_mode='velocity',manual_direction=-1)
        self.status(position=8000)
        self.arm();self.jog()
        self.assertEqual(self.sent[-1][1][0],12)
        self.assertEqual(self.sent[-1][1][5],-2000)
        self.assertEqual(self.sent[-1][1][6],0)
        self.status(state=5,flags=1|2|16|32|64,position=8000)
        self.action('release',press='press1')
        self.assertEqual(self.sent[-1][1][0],STOP)

    def test_stop_deadline_uses_brake_rate(self):
        self.c.axes[1].update(brake_deceleration_deg_s2=10,max_speed_deg_s=180)
        self.c.config['max_speed_deg_s']=180
        self.arm();self.action('jog',node=1,direction=1,speed=90,press='p')
        self.action('release',press='p')
        self.assertGreaterEqual(self.c.stop_deadline,self.now+11)

    def test_j1_500_speed_ceiling_and_slow_default(self):
        self.c.config['max_speed_deg_s']=500
        self.c.axes[1].update(manual_mode='velocity',max_speed_deg_s=500)
        self.arm()
        with self.assertRaises(ValueError):
            self.action('jog',node=1,direction=1,speed=500.1,press='over')
        self.action('jog',node=1,direction=1,speed=500,press='at_limit')
        self.assertEqual(self.sent[-1][1][0],12)
        self.assertEqual(self.sent[-1][1][5],500000)
        self.assertLess(self.c.config['default_speed_deg_s'],500)


if __name__ == '__main__':
    unittest.main()
