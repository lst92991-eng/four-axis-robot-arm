"""台架入口的离线回归，所有硬件接口用替身。"""

import ctypes
import json
import os
import io
from contextlib import redirect_stdout
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
sys.path.insert(0, str(ROOT / 'bench'))
sys.path.insert(0, str(ROOT / 'bench/field'))
sys.path.insert(0, str(ROOT / 'temp'))
import flash_drive
import ota
import sweep
import run_game
import match_panel


class Entries(unittest.TestCase):
    @unittest.skipUnless(os.name == 'nt' and sys.version_info[:2] == (3, 13) and
                         (ROOT/'build/bench/bin/scara_usb_game.dll').is_file(), '需要已构建的 Windows 台架和 Python 3.13')
    def test_dll_plays_from_memory_and_emits_state_with_no_files(self):
        bins = ROOT / 'build/bench/bin'
        opening = json.loads((ROOT/'tests/fixtures/game-app/two-rounds.json').read_text(encoding='utf-8'))['initial_board']
        previous = Path.cwd()
        with tempfile.TemporaryDirectory() as tmp, os.add_dll_directory(str(bins)):
            try:
                os.chdir(tmp)
                dll = ctypes.PyDLL(str(bins/'scara_usb_game.dll'))
                dll.usb_installation.restype = ctypes.py_object
                cfg = run_game.compiled_config(dll.usb_installation())
                workspace = run_game.Bench(config=cfg).workspace
                self.assertGreater(len(workspace.angles), 0)
                park = workspace.angles_of(cfg['park_joints_deg'])
                self.assertAlmostEqual(workspace.opposite(park)[1], -park[1])
                run = dll.run_usb_game
                run.argtypes = ctypes.py_object, ctypes.c_char_p
                run.restype = ctypes.c_int
                for history, code in [('', 0), ('a3a4', 0), ('a3a4 xxxx', 1), ('', 0)]:
                    with self.subTest(history=history):
                        field = MagicMock()
                        field.resume_history.return_value = history
                        field.resume_board.return_value = opening
                        field.command.return_value = 'quit'
                        field.close.return_value = True
                        self.assertEqual(run(field, b'mock-model'), code)
                        field.execute.assert_not_called()
                        field.infer.assert_not_called()
                        field.close.assert_called_once()
                        phase, message, _, confirmed, board, _, _, pending, _, _ = field.state.call_args.args
                        self.assertEqual((phase, confirmed, pending), ('error' if code else 'stopped', '' if code else history, 0))
                        if history == 'a3a4':
                            self.assertEqual((board[6*9],board[5*9]), ('.','P'))
                        if code:
                            self.assertIn('xxxx',message)
                field.close.return_value = False
                self.assertEqual(run(field, b'mock-model'), 1)
                self.assertIn('机械停止', field.state.call_args.args[1])
                self.assertEqual(list(Path(tmp).iterdir()), [])
            finally:
                os.chdir(previous)

    def test_state_and_camera_use_the_panel_pipe(self):
        fixture = run_game.Bench.__new__(run_game.Bench)
        fixture.initial_board = list(match_panel.OPENING)
        flat = ''.join(fixture.initial_board)
        legal = [((6, 0), (5, 0))]
        picture = (ROOT/'tests/fixtures/camera/0.jpg').read_bytes()
        transport = io.StringIO()
        with redirect_stdout(transport):
            fixture.state('wait_human', '', '12', 'a3a4 b7e7', flat, flat, 0, 0, 1, legal)
            fixture.preview(picture)
        panel = match_panel.Match(False)
        proc = MagicMock()
        proc.stdout = io.StringIO(transport.getvalue())
        proc.wait.return_value = 0
        panel.proc = proc
        panel.read_output(proc)
        self.assertEqual(panel.preview, picture)
        self.assertEqual(panel.saved['legal_rc'], [[[6,0],[5,0]]])
        self.assertEqual(panel.saved['board'], match_panel.OPENING)
        self.assertEqual(panel.saved['initial_board'], match_panel.OPENING)

    def test_field_execution_obeys_explicit_grip_and_release_at_height(self):
        previous = Path.cwd()
        self.addCleanup(os.chdir, previous)
        os.chdir(ROOT / "bench/reference")
        bench = run_game.Bench(json.loads(Path('game.json').read_text(encoding='utf-8')))
        drive = MagicMock()
        bridge = MagicMock()
        high = bench.heights["z"]["travel_deg"]
        low = bench.heights["z"]["pickup_deg"]
        park = bench.config["park_joints_deg"]
        drive.states = {
            n: dict(position=q, power=False, stationary=True) for n, q in zip((1, 2, 3), (high, *park))
        }
        changes = []

        def z(target, **_):
            drive.states[1]["position"] = target

        def xy(j2, j3, *_):
            self.assertEqual(drive.states[1]["position"], high)
            drive.states[2]["position"] = j2
            drive.states[3]["position"] = j3

        drive.z.side_effect = z
        drive.xy.side_effect = xy
        bridge.send.side_effect = lambda cmd: changes.append((cmd, drive.states[1]["position"]))
        bench.drive = drive
        src, dst = (-30.0, 40.0), (-28.0, 43.0)
        steps = [
            (1, 7, high, *src, 0),
            (1, 7, low, *src, 0),
            (0, 7, 0, 0, 0, 420),
            (1, 7, high, *src, 0),
            (1, 7, high, *dst, 0),
            (1, 7, low, *dst, 0),
            (2, 7, 0, 0, 0, 0),
            (1, 7, high, *dst, 0),
            (1, 7, high, *park, 0),
        ]
        with patch.object(run_game, "Magnet", return_value=bridge):
            self.assertTrue(bench.execute("move", "c3c4", steps))
        self.assertEqual(changes[:2], [("hold", low), ("off", low)])
        self.assertTrue(all(action in ("hold", "off") for action, _ in changes))

    def test_single_node_update_write_selects_real_image_before_opening_bus(self):
        package = flash_drive.PACKAGE
        for node in range(1, 5):
            _, build = flash_drive.first_image(package, node)
            self.assertEqual(build, '9fc6a5de')
        firmware = package / 'joint-app.ota'
        bus = MagicMock()
        can = SimpleNamespace(Bus=lambda **_: bus)
        argv = ['ota.py', 'update', '2', str(firmware), '--write']
        with (
            patch.dict(sys.modules, {'can': can}),
            patch.object(sys, 'argv', argv),
            patch.object(ota, 'send_image') as send,
        ):
            ota.main()
        self.assertEqual(send.call_args.args[2:5], ota.image_for_node(firmware.read_bytes(), 2))

    def test_all_cells_and_vertical_transport(self):
        workspace = SimpleNamespace(route=lambda origin, target, fixed=False: ([target], False))
        bench = SimpleNamespace(
            workspace=workspace,
            config={'park_joints_deg': (-1, -1)},
            heights={'z': {'pickup_deg': 0, 'travel_deg': 100, 'operator_estimated_rise_mm': 40}},
        )
        points = {(r, c): ((r, c), (r, c)) for r in range(10) for c in range(9)}
        first = (3, 4)
        for carry in (False, True):
            actions = sweep.plan(bench, points, (-1, -1), first, carry)
            cells = [v for k, v in actions if k == 'record']
            self.assertEqual(len(cells), 90)
            self.assertEqual(set(cells), set(points))
            self.assertEqual(cells[-1] if carry else cells[0], first)
            self.assertEqual(actions.count(('magnet', 'hold')), 90 if carry else 0)
            self.assertEqual(actions.count(('magnet', 'off')), 90 if carry else 0)
            self.assertEqual(actions[-1][1][1], (-1, -1))
            height = None
            for kind, value in actions:
                if kind == 'z':
                    height = value
                if kind == 'xy':
                    self.assertEqual(height, 100)
                if kind == 'magnet':
                    self.assertEqual(height, 0)

    def test_missing_reference_sends_no_drive_or_magnet_commands(self):
        drive = MagicMock()
        args = SimpleNamespace(plan=False, case='pick90')
        with (
            patch.object(sweep, 'read_points'),
            patch.object(sweep, 'measured_config', return_value={}),
            patch.object(sweep, 'Bench'),
            patch.object(sweep, 'Drive', return_value=drive),
            patch.object(sweep, 'sample', return_value={}),
            patch.object(sweep, 'Magnet') as magnet,
        ):
            with self.assertRaisesRegex(RuntimeError, 'J1'):
                sweep.sweep(args, Path('unused'), Path('unused'), MagicMock())
        drive.read_initial.assert_not_called()
        drive.stop.assert_not_called()
        drive.close.assert_called_once()
        magnet.assert_not_called()


if __name__ == '__main__':
    unittest.main()
