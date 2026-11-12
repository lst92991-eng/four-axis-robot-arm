"""Replay misplaced and retained cannon outcomes with hardware calls mocked."""
from pathlib import Path
import json
import sys
import tempfile
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'manual_control'))
from tune_release import Tuning, moving_cannon, np


class CannonTrackingTests(unittest.TestCase):
    def test_matches_misplaced_cannon_even_when_it_is_between_intersections(self):
        detections = [dict(symbol='C', pixel=[4.3,1.4]), dict(symbol='C', pixel=[1.1,2.3])]
        cannon = moving_cannon(detections, np.eye(3), [1.1,2.3])
        self.assertEqual(cannon['pixel'], [4.3,1.4])

    def test_board_transform_keeps_other_cannon_identity(self):
        matrix = np.array([[0.,.1,-2.],[-.1,0.,12.],[0.,0.,1.]])
        detections = [dict(symbol='C', pixel=[97.,31.]), dict(symbol='C', pixel=[106.,63.])]
        cannon = moving_cannon(detections, matrix, [1.1,2.3])
        self.assertEqual(cannon['pixel'], [106.,63.])

    def test_retained_piece_is_distinguished_from_static_cannon(self):
        self.assertIsNone(moving_cannon([dict(symbol='C', pixel=[1.1,2.3])], np.eye(3), [1.1,2.3]))


class TuningLoopTests(unittest.TestCase):
    def setUp(self):
        self.tuning = Tuning.__new__(Tuning)
        self.tuning.state = dict(pulse_ms=20, lower_ms=0, upper_ms=100, target='e2',
                                 held=True, successes=0, trials=0, missing=0)
        self.tuning.trial = Mock()
        self.tuning.trial.pick = -100
        self.tuning.trial.plan = {'z': {'release_offset_mm':5}}
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.tuning.plan_path = Path(self.directory.name) / 'plan.json'
        self.tuning.plan_path.write_text(json.dumps({'z':{'pickup_deg':-100}}), encoding='utf-8')
        self.tuning.demo = Mock()
        self.tuning.demo.grid = dict(e2={'pixel_geometric':[0,0]}, h2={'pixel_geometric':[50,0]})
        self.tuning.demo.target.side_effect = lambda name,pixel=None: dict(
            grid_id=name, base_xy_mm=pixel if pixel is not None else self.tuning.demo.grid[name]['pixel_geometric'])
        self.tuning.park_target = {}
        self.tuning.park = Mock()
        self.tuning.frame = Mock()
        self.tuning.save = Mock()
        self.pulses = []
        self.tuning.trial.release_piece.side_effect = lambda _: self.pulses.append(self.tuning.trial.release_pulse_ms)

    def test_retries_from_actual_miss_and_verifies_three_alternating_landings(self):
        miss = {'pixel':[18,0]}
        e2 = {'pixel':[1,0]}
        h2 = {'pixel':[51,0]}
        scenes = [None,miss, miss,None,e2, e2,None,h2, h2,None,e2]
        self.tuning.scene = Mock(side_effect=[(c,[]) for c in scenes])
        self.tuning.run()
        picks = [c.args[0]['base_xy_mm'] for c in self.tuning.trial.pick_piece.call_args_list]
        self.assertEqual(picks, [[18,0],[1,0],[51,0]])
        self.assertEqual(self.pulses, [20,10,10,10])
        self.assertEqual(self.tuning.state['successes'], 3)
        saved = json.loads(self.tuning.plan_path.read_text(encoding='utf-8'))
        self.assertEqual(saved['release_pulse_ms'], 10)
        self.assertEqual(saved['z']['pickup_deg'], -100)

    def test_missing_piece_is_not_reported_as_success(self):
        self.tuning.scene = Mock(return_value=(None, []))
        with self.assertRaisesRegex(RuntimeError, '两次释放后'):
            self.tuning.run()
        self.assertEqual(self.pulses, [20,60])
        self.assertEqual(self.tuning.state['successes'], 0)
