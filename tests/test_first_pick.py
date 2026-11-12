"""Exercise placement timing without connecting to the arm or GPIO."""
from pathlib import Path
import sys
import unittest
from unittest.mock import ANY, Mock, call, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'manual_control'))
from first_pick import GPIO, Trial


class ReleaseTests(unittest.TestCase):
    def setUp(self):
        self.trial = Trial.__new__(Trial)
        self.trial.adb = ['adb', 'shell']
        self.trial.pulse = None
        self.trial.pick, self.trial.high = -100, -50
        self.trial.release_height = -88
        self.trial.release_pulse_ms = 250
        self.target = {'grid_id': 'e2'}
        self.events = Mock()
        self.trial.xy = self.events.xy
        self.trial.z = self.events.z
        self.trial.sample = self.events.sample
        self.trial.note = self.events.note
        self.process = Mock(name='adb_process', returncode=0)
        self.process.poll.return_value = 0
        self.process.communicate = self.events.read_levels
        self.process.communicate.return_value = (b'1\n0\n', b'')

    @patch('first_pick.time.sleep')
    @patch('first_pick.subprocess.Popen')
    def test_release_at_placement_height_then_raise(self, launch, _sleep):
        self.events.attach_mock(launch, 'launch')
        launch.return_value = self.process
        self.process.poll.side_effect = [None, 0]

        self.trial.place_piece(self.target)

        self.assertEqual(self.events.mock_calls, [
            call.xy(self.target),
            call.z(self.trial.release_height),
            call.launch(ANY, stdout=ANY, stderr=ANY, creationflags=ANY),
            call.sample(),
            call.read_levels(),
            call.note('release_pulse', grid='e2', duration_ms=250, levels=[1, 0]),
            call.z(self.trial.high),
        ])
        self.assertIn(f'echo 1 > {GPIO}; cat {GPIO}; busybox sleep 0.250; '
                      f'echo 0 > {GPIO}; cat {GPIO}', launch.call_args.args[0][-1])
        self.assertIsNone(self.trial.pulse)

    @patch('first_pick.subprocess.run')
    @patch('first_pick.subprocess.Popen')
    def test_unconfirmed_low_leaves_arm_at_placement_height(self, launch, force_low):
        launch.return_value = self.process
        self.process.communicate.return_value = (b'1\n', b'')

        with self.assertRaisesRegex(RuntimeError, '高低电平未确认'):
            self.trial.place_piece(self.target, height=-103)

        self.trial.z.assert_called_once_with(-103)
        force_low.assert_called_once_with(self.trial.adb + [f'echo 0 > {GPIO}'], timeout=2, check=True)
        self.assertIsNone(self.trial.pulse)

    @patch('first_pick.subprocess.run')
    @patch('first_pick.subprocess.Popen')
    def test_interrupted_pulse_turns_off_without_lifting(self, launch, force_low):
        launch.return_value = self.process
        self.process.poll.return_value = None
        self.trial.sample.side_effect = RuntimeError('试走已停止')

        with self.assertRaisesRegex(RuntimeError, '试走已停止'):
            self.trial.place_piece(self.target)

        self.trial.z.assert_called_once_with(self.trial.release_height)
        self.process.terminate.assert_called_once()
        force_low.assert_called_once_with(self.trial.adb + [f'echo 0 > {GPIO}'], timeout=2, check=True)
        self.assertIsNone(self.trial.pulse)

    def test_position_stops_when_feedback_is_stationary_at_goal(self):
        axis = dict(node=2, state=2, stationary=True, position=-50.506, target=-50.493)
        state = {'axes': [axis]}
        self.trial.at_rest = Mock(return_value=dict(position=-52.5, state=1, power=True))
        self.trial.begin = Mock()
        self.trial.end = Mock(return_value=state)

        def wait(predicate, _seconds, _watch_press):
            self.assertTrue(predicate(state))

        self.trial.wait = wait
        self.assertTrue(self.trial.position(2, -50.493, 8))
        self.trial.end.assert_called_once_with(2)
