"""离线验证点动的撤销/限时/租约及反馈边界，不连接USB。"""
import queue
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch
from panel import AutoSequence, Drive, HoldGate, HOLD_SECONDS, Intent, monitor, motion_command, speed_command, watchdog_command, parse_rpm, worker
from can_diagnostics import describe_can_error


class HoldTests(unittest.TestCase):
    def test_user_rpm_range_and_full_width_encoding(self):
        for text in ['1', '50', '256', '1000', '3000']:
            value = parse_rpm(text)
            with patch('panel.RPM', value):
                command = motion_command(0)
                self.assertEqual(int.from_bytes(bytes(command[2:4]), 'big'), value)
                self.assertEqual(int.from_bytes(bytes(command[5:9]), 'big'), 3200)
                self.assertEqual(speed_command(1), [246, 1, value >> 8, value & 255, 1, 0, 107])
        for text in ['', '0', '-1', '3001', '50.5', 'nan', '１２', '50e1']:
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_rpm(text)

    def test_watchdog_is_volatile_and_verified_before_speed(self):
        from unittest.mock import Mock
        self.assertEqual(watchdog_command(), [104, 56, 0, 0, 0, 1, 244, 107])
        drive = Mock()
        drive.query.side_effect = [[22, 0, 0, 0, 0, 107], [22, 0, 0, 1, 244, 107]]
        drive.reply.return_value = [104, 2, 107]
        Drive.ensure_watchdog(drive)
        self.assertTrue(drive.watchdog_ready)
        drive.send.assert_called_once_with(watchdog_command())
        drive.query.side_effect = [[22, 0, 0, 0, 0, 107], [22, 0, 0, 0, 0, 107]]
        with self.assertRaisesRegex(RuntimeError, '回读不符'):
            Drive.ensure_watchdog(drive)
        self.assertFalse(drive.watchdog_ready)

    def test_manual_move_uses_f6_once_with_watchdog(self):
        from unittest.mock import Mock
        drive = Mock()
        drive.watchdog_ready = False
        with patch('panel.AUTO_MODE', False), patch('panel.RPM', 60):
            with self.assertRaisesRegex(RuntimeError, '失联停机'):
                Drive.move(drive, 0)
            drive.send.assert_not_called()
            drive.watchdog_ready = True
            drive.reply.return_value = [246, 2, 107]
            Drive.move(drive, 0)
            drive.send.assert_called_once_with([246, 0, 0, 60, 1, 0, 107])
            drive.reply.assert_called_once_with(246)

    def test_manual_speed_is_display_only(self):
        with patch('panel.RPM', 50):
            for measured in [0, 20, 50, 55]:
                monitor((100, measured, 600, 1), 0, 0, 10, 50 * 6 * 34 + 360, manual=True)

    def test_manual_retains_fault_current_and_motion_bounds(self):
        for sample in [(100, 50, 600, 13), (100, 50, 2801, 1), (-2, 0, 600, 1), (400, 0, 600, 1)]:
            with self.subTest(sample=sample), self.assertRaises(RuntimeError):
                monitor(sample, 0, 0, 2, 360, manual=True)

    def test_idle_does_not_start(self):
        self.assertEqual(HoldGate().step(1, (None, 0, 0)), "idle")

    def test_release_stops_without_restart(self):
        gate = HoldGate()
        self.assertEqual(gate.step(0, (0, 1, .2)), "start")
        self.assertEqual(gate.step(.1, (None, 1, .2)), "stop")
        self.assertEqual(gate.step(.15, (0, 1, .3)), "idle")
        self.assertEqual(gate.step(.16, (0, 2, .3)), "start")

    def test_frozen_ui_expires(self):
        gate = HoldGate()
        gate.step(0, (0, 1, .2))
        self.assertEqual(gate.step(.21, (0, 1, .2)), "stop")

    def test_time_limit_requires_new_press(self):
        gate = HoldGate()
        gate.step(0, (0, 1, .2))
        self.assertEqual(gate.step(HOLD_SECONDS, (0, 1, HOLD_SECONDS + .2)), "stop")
        self.assertEqual(gate.step(HOLD_SECONDS + .1, (0, 1, HOLD_SECONDS + .3)), "idle")

    def test_manual_34_seconds_stops_and_requires_release(self):
        with patch('panel.HOLD_SECONDS', 34):
            gate = HoldGate()
            self.assertEqual(gate.step(0, (0, 1, .2)), 'start')
            self.assertEqual(gate.step(33.9, (0, 1, 34.1)), 'idle')
            self.assertEqual(gate.step(34, (0, 1, 34.2)), 'stop')
            self.assertEqual(gate.step(35, (0, 1, 35.2)), 'idle')

    def test_manual_monitor_can_pass_one_revolution_without_increasing_packet(self):
        with patch('panel.RPM', 25):
            monitor((1480, 25, 1800, 1), 0, 0, 10, 25 * 6 * 34 + 360)
            self.assertEqual(int.from_bytes(bytes(motion_command(0)[5:9]), 'big'), 3200)

    def test_50rpm_command(self):
        with patch('panel.RPM', 50):
            self.assertEqual(motion_command(0), [253, 0, 0, 50, 1, 0, 0, 12, 128, 2, 0, 107])

    def test_reverse_press_stops_first(self):
        gate = HoldGate()
        gate.step(0, (0, 1, .2))
        self.assertEqual(gate.step(.1, (1, 2, .3)), "stop")
        self.assertEqual(gate.step(.15, (1, 2, .3)), "idle")

    def test_finite_protocol(self):
        for direction in (0, 1):
            command = motion_command(direction)
            self.assertEqual(command, [253, direction, 0, 5, 1, 0, 0, 12, 128, 2, 0, 107])
        with self.assertRaises(ValueError):
            motion_command(2)

    def test_monitor_accepts_both_directions(self):
        monitor((30, 5, 559, 1), 0, 0, 1)
        monitor((-30, -5, 559, 1), 0, 1, 1)

    def test_monitor_rejects_fault_current_and_no_progress(self):
        for sample, elapsed in [((30, 5, 559, 13), 1), ((30, 5, 2801, 1), 1),
                                ((0, 0, 559, 1), 1), ((-2, 1, 559, 1), .1),
                                ((364, 5, 559, 1), 2), ((30, 9, 559, 1), 1)]:
            with self.subTest(sample=sample):
                with self.assertRaises(RuntimeError):
                    monitor(sample, 0, 0, elapsed)

    def test_25rpm_uses_acceleration_profile(self):
        with patch('panel.RPM', 25):
            monitor((75, 25, 1800, 1), 0, 0, .65)
            monitor((359.5, 0, 1800, 3), 0, 0, 2.8)
            with self.assertRaises(RuntimeError):
                monitor((50, 25, 1800, 1), 0, 0, 1)
            self.assertEqual(motion_command(1), [253, 1, 0, 25, 1, 0, 0, 12, 128, 2, 0, 107])


class WorkerTests(unittest.TestCase):
    def test_held_speed_does_not_repeat_command_after_one_second(self):
        from unittest.mock import Mock
        intent, closing, events = Intent(), threading.Event(), queue.Queue()
        fake = Mock()
        fake.validate.return_value = 24
        fake.sample.return_value = (0, 0, 100, 3)
        with patch('panel.Drive', return_value=fake), patch('panel.AUTO_MODE', False), patch('panel.append_diagnostic'):
            thread = threading.Thread(target=worker, args=(intent, closing, events))
            thread.start()
            try:
                self.assertEqual(events.get(timeout=1)[0], 'ready')
                intent.press(0)
                deadline = time.monotonic() + 1.35
                while time.monotonic() < deadline:
                    intent.renew()
                    time.sleep(.03)
                intent.release()
                deadline = time.monotonic() + 1
                while time.monotonic() < deadline:
                    if events.get(timeout=1)[0] == 'stopped':
                        break
                else:
                    self.fail('release did not stop')
                fake.move.assert_called_once_with(0)
                fake.stop.assert_called_once()
            finally:
                closing.set()
                thread.join(1)
                self.assertFalse(thread.is_alive())

    def test_apply_speed_is_idle_only_and_does_not_move(self):
        from unittest.mock import Mock
        intent, closing, events, settings = Intent(), threading.Event(), queue.Queue(), queue.Queue()
        fake = Mock()
        fake.validate.return_value = 24
        fake.sample.return_value = (0, 0, 100, 3)
        with patch('panel.Drive', return_value=fake), patch('panel.AUTO_MODE', False), patch('panel.RPM', 50), patch('panel.append_diagnostic'):
            thread = threading.Thread(target=worker, args=(intent, closing, events, settings))
            thread.start()
            try:
                self.assertEqual(events.get(timeout=1)[0], 'ready')
                settings.put(1000)
                kind, value = events.get(timeout=1)
                self.assertEqual((kind, value), ('settings_applied', 1000))
                fake.move.assert_not_called()
            finally:
                closing.set()
                thread.join(1)
                self.assertFalse(thread.is_alive())

    def run_case(self, mode):
        intent, closing, events = Intent(), threading.Event(), queue.Queue()
        calls = []

        class FakeDrive:
            def record(self, item):
                pass

            def validate(self):
                calls.append('validate')
                if mode == 'cancel_during_precheck' and calls.count('validate') == 2:
                    intent.release()
                return 24

            def sample(self):
                if mode == 'feedback_timeout' and 'move' in calls:
                    raise RuntimeError('feedback timeout')
                return 0, 0, 100, 3

            def move(self, direction):
                calls.append('move')
                if mode == 'ack_timeout':
                    raise RuntimeError('move ACK timeout')
                if mode == 'release':
                    intent.release()

            def stop(self):
                calls.append('stop')
                closing.set()

            def close(self):
                calls.append('close')

        with patch('panel.Drive', FakeDrive), patch('panel.append_diagnostic'):
            thread = threading.Thread(target=worker, args=(intent, closing, events))
            thread.start()
            self.assertEqual(events.get(timeout=1)[0], 'ready')
            intent.press(0)
            if mode == 'cancel_during_precheck':
                deadline = time.monotonic() + 1
                while calls.count('validate') < 2 and time.monotonic() < deadline:
                    time.sleep(.005)
                closing.set()
            thread.join(1)
            if thread.is_alive():
                closing.set()
                thread.join(1)
                self.fail('worker did not stop')
        return calls

    def test_release_and_expired_ui_lease_reach_stop(self):
        for mode in ['release', 'lease_expiry']:
            with self.subTest(mode=mode):
                calls = self.run_case(mode)
                self.assertEqual(calls.count('move'), 1)
                self.assertEqual(calls.count('stop'), 1)

    def test_missing_ack_or_feedback_reaches_stop(self):
        for mode in ['ack_timeout', 'feedback_timeout']:
            with self.subTest(mode=mode):
                calls = self.run_case(mode)
                self.assertIn('stop', calls)
                self.assertEqual(calls[-1], 'close')

    def test_release_during_validation_prevents_late_start(self):
        self.assertNotIn('move', self.run_case('cancel_during_precheck'))

    def test_reconnect_confirms_stop_without_replaying_motion(self):
        from unittest.mock import Mock
        intent, closing, events = Intent(), threading.Event(), queue.Queue()
        fake = Mock()
        fake.validate.return_value = 24
        fake.sample.return_value = (0, 0, 100, 3)
        with patch('panel.Drive', return_value=fake), patch('panel.append_diagnostic'):
            thread = threading.Thread(target=worker, args=(intent, closing, events, None, True))
            thread.start()
            try:
                self.assertEqual(events.get(timeout=1)[0], 'status')
                self.assertEqual(events.get(timeout=1)[0], 'ready')
                fake.stop.assert_called_once()
                fake.move.assert_not_called()
            finally:
                closing.set()
                thread.join(1)
                self.assertFalse(thread.is_alive())


class CanDiagnosticTests(unittest.TestCase):
    def test_only_exact_empty_controller_frame_is_notice(self):
        level, text = describe_can_error(0x20000004, [0] * 8)
        self.assertEqual(level, 'NOTICE')
        self.assertIn('0x20000004', text)
        for ident, payload in [(0x20000024, [0] * 8), (0x20000044, [0] * 8),
                               (0x20000004, [0, 16, 0, 0, 0, 0, 0, 0]),
                               (0x2000000C, [0, 0, 16, 0, 0, 0, 0, 0]),
                               (0x20000004, [0] * 7)]:
            with self.subTest(ident=ident, payload=payload):
                self.assertEqual(describe_can_error(ident, payload)[0], 'ERROR')
        self.assertIn('未收到ACK', describe_can_error(0x20000024, [0] * 8)[1])
        self.assertIn('隐性位', describe_can_error(0x2000000C, [0, 0, 16, 0, 0, 0, 0, 0])[1])

    def fake_drive(self, frames):
        from unittest.mock import Mock
        drive = Drive.__new__(Drive)
        drive.Frame = lambda: SimpleNamespace()
        drive.pending_replies = {}
        drive.log = SimpleNamespace(name='offline-test.jsonl')
        drive.record = Mock()
        drive.last_command = {'id': '0x80000100', 'data': [31, 107]}

        def read(frame, timeout):
            if not frames:
                return False
            ident, data = frames.pop(0)
            frame.can_id, frame.data, frame.can_dlc, frame.echo_id = ident, data, len(data), 0xFFFFFFFF
            return True
        drive.can = SimpleNamespace(read=read)
        return drive

    def test_empty_notice_does_not_replace_required_reply(self):
        frames = [(0x20000004, [0] * 8), (0x80000100, [31, 0, 203, 35, 20, 107])]
        drive = self.fake_drive(frames)
        with patch('panel.append_diagnostic'):
            self.assertEqual(drive.reply(31, timeout=.02), [31, 0, 203, 35, 20, 107])
            drive = self.fake_drive([(0x20000004, [0] * 8)])
            with self.assertRaisesRegex(RuntimeError, '超时'):
                drive.reply(31, timeout=.005)

    def test_real_ack_error_still_stops_waiting(self):
        drive = self.fake_drive([(0x20000024, [0] * 8), (0x80000100, [31, 0, 203, 35, 20, 107])])
        with patch('panel.append_diagnostic') as log:
            with self.assertRaisesRegex(RuntimeError, '未收到ACK'):
                drive.reply(31)
            self.assertEqual(log.call_args.args[0], 'ERROR')
            self.assertEqual(log.call_args.kwargs['last_command']['data'], [31, 107])

    def test_configuration_packets_can_arrive_out_of_order(self):
        packets = {i: [66, i, 107] for i in range(5)}
        drive = self.fake_drive([(0x80000100 | i, packets[i]) for i in [0, 1, 2, 4, 3]])
        with patch('panel.append_diagnostic'):
            self.assertEqual([drive.reply(66, packet=i) for i in range(5)], [packets[i] for i in range(5)])
            self.assertFalse(drive.pending_replies)

    def test_missing_configuration_packet_still_times_out(self):
        drive = self.fake_drive([(0x80000104, [66, 8, 107])])
        with patch('panel.append_diagnostic'):
            with self.assertRaisesRegex(RuntimeError, '第3包'):
                drive.reply(66, packet=3, timeout=.005)


class AutoTests(unittest.TestCase):
    def test_direction_check_stops_after_one_leg(self):
        auto = AutoSequence(True, legs=1)
        auto.due = 0
        self.assertEqual(auto.next(0), 0)
        auto.complete(12)
        self.assertIsNone(auto.next(20))
        self.assertEqual(auto.index, 1)
        self.assertFalse(auto.enabled)

    def test_stop_tolerates_one_stale_speed_then_requires_three_zero(self):
        from unittest.mock import Mock
        drive = Mock()
        drive.reply.return_value = [254, 2, 107]
        drive.sample.side_effect = [(0, 5, 100, 3), (0, 0, 100, 3), (0, 0, 100, 3), (0, 0, 100, 3)]
        Drive.stop(drive)
        drive.send.assert_called_once_with([254, 152, 0, 107])
        self.assertEqual(drive.sample.call_count, 4)

    def test_cancel_never_resumes(self):
        auto = AutoSequence(True)
        auto.due = 0
        self.assertEqual(auto.next(0), 0)
        auto.cancel()
        auto.complete(1)
        self.assertIsNone(auto.next(100))
        self.assertEqual(auto.index, 0)

    def test_six_bounded_legs_wait_for_confirmation(self):
        auto = AutoSequence(True)
        auto.due = 0
        for n in range(6):
            self.assertEqual(auto.next(n * 20), n % 2)
            self.assertIsNone(auto.next(n * 20 + 5))
            auto.complete(n * 20 + 12)
            self.assertIsNone(auto.next(n * 20 + 12.5))
        self.assertEqual(auto.index, 6)
        self.assertFalse(auto.enabled)


if __name__ == "__main__":
    unittest.main()
