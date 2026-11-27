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
    def test_live_default_site_requires_local_teaching(self):
        import teach
        with tempfile.TemporaryDirectory() as directory:
            installed = Path(directory) / 'installation.toml'
            with patch.object(teach, 'INSTALLATION', installed):
                with self.assertRaisesRegex(FileNotFoundError, '15-site'):
                    teach.default_site()
                installed.touch()
                self.assertEqual(teach.default_site(), installed)

    def test_measured_stops_are_in_joint_degrees(self):
        import teach
        values = dict(j2=[10, 30, 0.5], j3=[20, -10, -1/3], z=[0], observe=[0, 0],
                      discard=[100, 100], camera='rtsp://example.invalid/')
        self.assertNotIn('range_deg', teach.make_site(values, {})['j2'])
        values.update(j2_stops=[30, -10], j3_stops=[110, -40])
        result = teach.make_site(values, {})
        self.assertEqual(result['j2']['range_deg'], [20, 40])
        self.assertEqual(result['j3']['range_deg'], [-40, 10])

    def test_one_time_site_import(self):
        import subprocess
        import tomllib
        bins = Path(os.environ.get('SCARA_BENCH_BIN', ROOT/'build/bench/bin'))
        tool = bins/('scara_bench_points.exe' if os.name == 'nt' else 'scara_bench_points')
        if not tool.is_file():
            self.skipTest('需要已构建的 C++ 格点工具')
        fixtures = ROOT/'tests/fixtures/game-app'
        old = (fixtures/'installation-v1.toml').read_bytes()
        expected = (fixtures/'installation.toml').read_bytes()
        cli = [sys.executable, str(ROOT/'bench/run.py'), 'site']
        def query(flag, data):
            return subprocess.check_output([str(tool), flag], input=data)

        with tempfile.TemporaryDirectory() as work:
            installed = Path(work)/'installation.toml'
            # 使用原文件位置迁移。第二次安装保留文件及唯一的备份。
            installed.write_bytes(old)
            args = cli + ['--install', '--output', str(installed)]
            subprocess.run(args, check=True, capture_output=True)
            result = installed.read_bytes()
            self.assertEqual(query('-', result), query('-', expected))
            self.assertEqual(len(query('-', result).splitlines()), 90)
            self.assertEqual(query('--settings', result), query('--settings', expected))
            self.assertEqual(tomllib.loads(result.decode())['game']['think_ms'], 100)
            stamp = installed.stat().st_mtime_ns
            subprocess.run(args, check=True, capture_output=True)
            self.assertEqual(installed.stat().st_mtime_ns, stamp)
            self.assertEqual([p.read_bytes() for p in Path(work).glob('*.bak')], [old])

            # 旧矩形含奇数宽高时，沿用原来的截断方向计算 640 方框的左上角。
            source = Path(work)/'older.toml'
            source.write_bytes(old.replace(b'[vision]', b'[vision]\nroi = [300, 70, 651, 599]'))
            args = cli + [str(source), '--install', '--output', str(installed)]
            subprocess.run(args, check=True, capture_output=True)
            result = installed.read_bytes()
            self.assertEqual(tomllib.loads(result.decode())['camera']['crop_px'], [305, 50])
            source.write_bytes(old.replace(b'travel_deg = -19353.34', b'# missing measured height'))
            self.assertNotEqual(subprocess.run(args, capture_output=True).returncode, 0)
            self.assertEqual(installed.read_bytes(), result)

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
                site = (ROOT/'config/installation.toml').read_bytes()
                dll.usb_installation.argtypes = ctypes.c_char_p, ctypes.c_size_t
                dll.usb_installation.restype = ctypes.py_object
                cfg = run_game.compiled_config(dll.usb_installation(site, len(site)))
                self.assertEqual(cfg['skill_level'], 20)
                selected = site.replace(b'skill_level = 20', b'skill_level = 7')
                selected_cfg = run_game.compiled_config(dll.usb_installation(selected, len(selected)))
                bench = run_game.Bench(config=selected_cfg)
                engine = MagicMock()
                def reply(match, _timeout):
                    for line in ('a3a4: 1', 'Nodes searched: 1', 'bestmove a3a4'):
                        result = match(line)
                        if result is not None:
                            return result
                engine.read.side_effect = reply
                with patch.object(run_game, 'Lines', return_value=engine):
                    self.assertEqual(bench.engine('/'.join(opening) + ' w - - 0 1', '', True), (['a3a4'], 'a3a4'))
                commands = [call.args[0] for call in engine.send.call_args_list]
                self.assertLess(commands.index('setoption name Skill Level value 7'), commands.index('isready'))
                self.assertIn(f"go movetime {cfg['think_ms']}", commands)
                workspace = run_game.Bench(config=cfg).workspace
                self.assertGreater(len(workspace.angles), 0)
                park = workspace.angles_of(cfg['park_joints_deg'])
                self.assertAlmostEqual(workspace.opposite(park)[1], -park[1])
                run = dll.run_usb_game
                run.argtypes = ctypes.py_object, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_size_t, ctypes.c_char_p
                run.restype = ctypes.c_int
                camera_url = b'rtsp://example.invalid/live/0'
                for history, code, interrupted in [
                    ('', 0, None), ('a3a4', 0, None), ('a3a4 xxxx', 1, None),
                    ('a3a4', 0, EOFError), ('a3a4', 0, KeyboardInterrupt), ('', 0, None),
                ]:
                    with self.subTest(history=history, interrupted=interrupted):
                        field = MagicMock()
                        field.resume_history.return_value = history
                        field.resume_board.return_value = opening
                        field.execute.return_value = False
                        field.command.return_value = 'quit'
                        field.command.side_effect = interrupted
                        field.close.return_value = True
                        self.assertEqual(run(field, b'mock-model', site, len(site), camera_url), code)
                        if code:
                            field.execute.assert_not_called()
                        else:
                            field.execute.assert_called_once()
                            self.assertEqual(field.execute.call_args.args[0], 'prepare')
                        field.infer.assert_not_called()
                        field.close.assert_called_once()
                        message, frame, confirmed, board, observed, result, pending, stable, _ = field.state.call_args.args
                        self.assertEqual((frame, observed, stable), ('', '?' * 90, 0))
                        self.assertEqual((confirmed, result, pending), ('' if code else history, 0, 0))
                        if history == 'a3a4':
                            self.assertEqual((board[6*9],board[5*9]), ('.','P'))
                        if code:
                            self.assertIn('xxxx',message)

                # 同一个回观察位故障通过 false 或异常上报时，开局处理一致。
                for drive_error in (None, RuntimeError('mock CAN error')):
                    field = MagicMock()
                    field.resume_history.return_value = ''
                    field.execute.return_value = False
                    field.execute.side_effect = drive_error
                    field.command.return_value = 'quit'
                    field.close.return_value = True
                    self.assertEqual(run(field, b'mock-model', site, len(site), camera_url), 0)
                    field.open.assert_called_once_with('mock-model')
                    field.command.assert_called_once()
                    field.close.assert_called_once()
                    if drive_error:
                        self.assertTrue(any('mock CAN error' in call.args[0] for call in field.state.call_args_list))
                field.close.return_value = False
                self.assertEqual(run(field, b'mock-model', site, len(site), camera_url), 1)
                self.assertIn('机械停止', field.state.call_args.args[0])
                self.assertEqual(list(Path(tmp).iterdir()), [])
            finally:
                os.chdir(previous)

    def test_opening_returns_approval_without_chess_commands(self):
        bench = run_game.Bench.__new__(run_game.Bench)
        bench.expected = match_panel.OPENING
        bench.current_position = False
        bench.start_checks = 0
        bench.saved = None
        opening = ''.join(bench.expected)
        mismatch = '.' + opening[1:]
        with redirect_stdout(io.StringIO()):
            for _ in range(5):
                self.assertIs(bench.confirm_opening(mismatch), False)
            with self.assertRaisesRegex(RuntimeError, '实拍开局盘面仍有差异'):
                bench.confirm_opening(mismatch)
            self.assertIs(bench.confirm_opening(opening), True)

            bench.current_position = True
            with patch('builtins.input', side_effect=['', 'READY', 'draw', 'resign', 'quit', 'unknown']):
                for expected in (True, True, False, False, None):
                    self.assertIs(bench.confirm_opening(opening), expected)
                with self.assertRaisesRegex(RuntimeError, '未识别的终端命令'):
                    bench.confirm_opening(opening)

    def test_state_and_camera_use_the_panel_pipe(self):
        fixture = run_game.Bench.__new__(run_game.Bench)
        fixture.initial_board = list(match_panel.OPENING)
        flat = ''.join(fixture.initial_board)
        legal = [((6, 0), (5, 0))]
        picture = (ROOT/'tests/fixtures/camera/0.jpg').read_bytes()
        transport = io.StringIO()
        with redirect_stdout(transport):
            fixture.state('', '12', 'a3a4 b7e7', flat, flat, 0, 0, 1, legal)
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

        def xy(j2, j3, *_, force_sync=False):
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
            if not carry:
                self.assertEqual(cells[1], (4, 4))
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

    def test_field_profile_uses_installed_axis_limits(self):
        tuning = run_game.field_limits('unused', (4800, 72, 216), (5700, 220, 360))['motion_tuning']
        self.assertEqual(tuning['base'][2:8], [72, 216, 220, 360, 75, 125])
        self.assertEqual(tuning['z']['game_speed_deg_s'], 4800)
        self.assertEqual(tuning['z']['acceleration_deg_s2'], 5700)

    def test_synchronized_crossing_checks_the_whole_direct_path(self):
        import numpy as np
        from workspace import Workspace

        arm = dict(j2_reference_deg=0, j3_reference_motor_deg=0, joint_per_drive=[1, 1],
                   shoulder_ref_rad=0, elbow_ref_rad=0, arm_lengths_mm=[275, 275])
        angles = np.arange(-5., 6.)
        free = np.ones((len(angles), len(angles)), dtype=bool)
        free[1, 8] = False
        with tempfile.TemporaryDirectory() as directory:
            samples = Path(directory) / 'workspace.npz'
            np.savez(samples, angles=angles, connected=free)
            workspace = Workspace(arm, dict(samples=samples, neutral_shoulder_deg=0,
                                            registration_uncertainty_deg=0, transfer_shoulder_deg=-3,
                                            transfer_speed_deg_s=[18, 60]))
        start, goal = (-4., -4.), (4., 4.)
        self.assertFalse(workspace.free_box(start, goal))
        self.assertEqual(workspace.route(start, goal, fixed=True), ([goal], False))
        bench = run_game.Bench(dict(arm, joint_velocity_deg_s=[4800, 42, 106],
                                   joint_acceleration_deg_s2=[5700, 110, 180]))
        bench.workspace = workspace
        bench.drive = MagicMock()
        bench.move_xy(start, goal, False)
        bench.drive.xy.assert_called_once_with(*goal, 42, 106, force_sync=True)

        # 直达通道中间被占用时，经伸直姿态绕行。
        workspace.free[5, 5] = False
        route, transfer = workspace.route(start, goal, fixed=True)
        self.assertTrue(transfer)
        self.assertIn((-3., 0.), route[:-1])
        self.assertEqual(route[-1], goal)

    def test_motion_checks_accept_different_firmware_builds(self):
        from drive import Drive
        from quick import check

        drive = Drive.__new__(Drive)
        drive.states = {
            node: dict(build=build, state=0, power=False, fault=0, interlocked=False,
                       configured=True, referenced=True, encoder_valid=True, bus_mv=24000)
            for node, build in enumerate(('bc2d87b1', '9fc6a5de', '12345678'), 1)
        }
        drive.last_update = dict.fromkeys(drive.states, 10.0)
        check(drive.states, motion=True)
        with patch('drive.time.monotonic', return_value=10.0):
            drive.healthy()
            drive.states[2]['referenced'] = False
            with self.assertRaisesRegex(RuntimeError, 'J2'):
                drive.healthy()
        with self.assertRaisesRegex(RuntimeError, 'J2 需要接续机械参考'):
            check(drive.states, motion=True)

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
