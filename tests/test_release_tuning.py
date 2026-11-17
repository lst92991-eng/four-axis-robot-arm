"""Replay misplaced and retained cannon outcomes with hardware calls mocked."""
from pathlib import Path
import json
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'manual_control'))
from tune_release import Tuning, board_transform, transform_point, moving_cannon, np
from chess_demo import Demo


class CannonTrackingTests(unittest.TestCase):
    def test_matches_misplaced_cannon_even_when_it_is_between_intersections(self):
        detections = [dict(symbol='C', pixel=[4.3,1.4]), dict(symbol='C', pixel=[1.1,2.3])]
        cannon = moving_cannon(detections, np.eye(2, 3), [1.1,2.3])
        self.assertEqual(cannon['pixel'], [4.3,1.4])

    def test_board_transform_keeps_other_cannon_identity(self):
        matrix = np.array([[0.,.1,-2.],[-.1,0.,12.]])
        detections = [dict(symbol='C', pixel=[97.,31.]), dict(symbol='C', pixel=[106.,63.])]
        cannon = moving_cannon(detections, matrix, [1.1,2.3])
        self.assertEqual(cannon['pixel'], [106.,63.])

    def test_retained_piece_is_distinguished_from_static_cannon(self):
        self.assertIsNone(moving_cannon([dict(symbol='C', pixel=[1.1,2.3])], np.eye(2, 3), [1.1,2.3]))

    def test_affine_grid_uses_all_four_corners(self):
        grid = {name: dict(pixel_geometric=pixel) for name, pixel in
                zip(('a0','i0','i9','a9'), ((100,600),(340,616),(385,346),(145,330)))}
        matrix = board_transform(grid)
        self.assertEqual(matrix.shape, (2, 3))
        np.testing.assert_allclose(transform_point(matrix, [210,486]), [3,4], atol=1e-10)

    def test_affine_grid_rejects_collinear_corners(self):
        grid = {name: dict(pixel_geometric=[i*10, 50]) for i, name in enumerate(('a0','i0','i9','a9'))}
        with self.assertRaises(ValueError):
            board_transform(grid)

    def test_demo_target_uses_six_coefficients(self):
        demo = Demo.__new__(Demo)
        demo.pixel_to_base = np.array([[.4,.02,100],[-.01,.5,-80]])
        demo.trial = Mock(arm=dict(arm_lengths_mm=[275,275], elbow_ref_rad=.5, shoulder_ref_rad=0,
                                 j2_reference_deg=0, j2_scale=1, j2_sign=1,
                                 j3_reference_motor_deg=0, j3_ratio=1, j3_sign=1))
        self.assertEqual(demo.target('a0', [200,300])['base_xy_mm'], [186,68])

    @patch('chess_demo.Trial')
    @patch('chess_demo.cv2.imdecode')
    @patch('chess_demo.Path.read_bytes', return_value=b'')
    @patch('chess_demo.read')
    def test_demo_rejects_old_grid_before_hardware_setup(self, read, _bytes, _decode, trial):
        read.return_value = dict(geometry_valid=True, orientation_valid=True,
                                 points=[dict(column=c,row=r,base_xy_valid=True) for r in range(10) for c in range(9)])
        with self.assertRaises(ValueError):
            Demo(Path('unused-work'), 'unused-adb')
        trial.assert_not_called()


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
