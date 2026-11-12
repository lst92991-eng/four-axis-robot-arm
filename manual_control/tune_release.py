"""Retest a red cannon from its observed landing position using the existing NPU and motion panel."""
import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import time

from chess_demo import Capture, Demo, ROOT, cv2, disk, np, square_map
import host_preview as hp

MODEL_SHA = 'ce9c91f4668cb0ec58f836432546b7eb2aebfc675ba696ce9788968560dcbdd2'
ROI = [300, 70, 650, 600]


def board_transform(grid):
    corners = np.float32([grid[s]['pixel_geometric'] for s in ('a0', 'i0', 'i9', 'a9')])
    return cv2.getPerspectiveTransform(corners, np.float32([[0,0], [8,0], [8,9], [0,9]]))


def transform_point(matrix, pixel):
    x, y, w = matrix @ np.array([*pixel, 1.])
    return [float(x/w), float(y/w)]


def moving_cannon(detections, matrix, fixed_board):
    cannons = [dict(d, board=transform_point(matrix, d['pixel'])) for d in detections if d['symbol']=='C']
    if len(cannons) not in (1, 2):
        raise RuntimeError('画面中的红炮数量不明确')
    cannons.sort(key=lambda d: math.dist(d['board'], fixed_board))
    if math.dist(cannons[0]['board'], fixed_board)>.25:
        raise RuntimeError('另一枚红炮的位置发生变化，需要重新识别两枚炮')
    if len(cannons)==1:
        return None
    cannon = cannons[1]
    if math.dist(cannon['board'], cannons[0]['board'])<.75:
        raise RuntimeError('两枚红炮距离太近，当前视角无法可靠区分')
    if not (0<=cannon['board'][0]<=8 and 0<=cannon['board'][1]<=9):
        raise RuntimeError('红炮已落在棋盘之外，当前 Z 示教平面不覆盖落点')
    return cannon


class Tuning:
    def __init__(self, args):
        self.args = args
        self.demo = Demo(args.work, args.adb, args.reference, 15, True)
        self.trial = self.demo.trial
        self.park_target = self.demo.target('c4')
        self.directory = args.work / 'release-tuning'
        self.directory.mkdir(exist_ok=True)
        self.trial.log = self.directory / 'events.jsonl'
        self.checkpoint = self.directory / 'checkpoint.json'
        self.plan_path = ROOT/'output/first-pick-c3-c4-20261111/trial-plan.json'
        self.state = dict(pulse_ms=20, lower_ms=0, upper_ms=100, target='e2',
                          held=args.held, successes=0, trials=0, missing=0)
        if self.checkpoint.exists():
            self.state.update(json.loads(self.checkpoint.read_text(encoding='utf-8')))
        health = json.loads(hp.request(args.board, '/health'))
        if health['model_sha256'] != MODEL_SHA:
            raise RuntimeError('实际 NPU 模型与已验证模型不一致')
        initial = self.infer(self.demo.reference, 'reference')
        matrix = board_transform(self.demo.grid)
        cannons = [transform_point(matrix, d['pixel']) for d in initial if d['symbol']=='C']
        if not cannons:
            raise RuntimeError('参考画面缺少固定红炮')
        self.fixed_board = min(cannons, key=lambda p: math.dist(p, [1, 2]))
        self.demo.port = int(subprocess.check_output(self.demo.adb + ['forward','tcp:0','tcp:554'],
                                                    text=True, timeout=4).strip())
        try:
            self.demo.capture = Capture(f'rtsp://127.0.0.1:{self.demo.port}/live/0')
        except BaseException:
            self.demo.close()
            raise

    def save(self):
        temporary = self.checkpoint.with_suffix('.tmp')
        temporary.write_text(json.dumps(self.state, indent=2)+'\n', encoding='utf-8')
        temporary.replace(self.checkpoint)

    def infer(self, frame, name):
        crop = hp.transform(frame, ROI, 0)
        rgb, ratio, left, top = hp.letterbox(crop)
        frame_id = f'{name}-{time.monotonic_ns()}'
        found, _ = hp.decode(hp.request(self.args.board, '/infer', rgb, frame_id),
                             ratio, left, top, crop.shape, frame_id)
        for d in found:
            x1,y1,x2,y2 = d['bbox']
            d['pixel'] = [(x1+x2)/2+ROI[0], (y1+y2)/2+ROI[1]]
            if d['symbol']=='C':
                circle = disk(frame, d['pixel'])
                if circle and circle[0]=='red':
                    d['pixel'] = circle[1]
        return found

    def frame(self):
        began = time.monotonic()
        while time.monotonic()-began<5:
            if self.trial.owns_panel:
                self.trial.sample()
            if self.demo.capture.error:
                raise RuntimeError(self.demo.capture.error)
            frame, stamp, _ = self.demo.capture.latest()
            if frame is not None and stamp>began+.15 and time.monotonic()-stamp<.2:
                return frame
            time.sleep(.025)
        raise RuntimeError('没有新的相机画面')

    def inspect(self, config):
        path = self.directory / 'inspect.json'
        path.write_text(json.dumps(config, ensure_ascii=False), encoding='utf-8')
        output = self.directory / 'grid.json'
        result = subprocess.run([str(self.args.geometry.resolve()), 'inspect', str(path), str(output)],
                                capture_output=True, timeout=8,
                                creationflags=getattr(subprocess,'CREATE_NO_WINDOW',0))
        if result.returncode:
            raise RuntimeError(result.stderr.decode('utf-8', errors='replace'))
        grid = json.loads(output.read_text(encoding='utf-8'))
        if not grid['geometry_valid']:
            raise RuntimeError('当前帧未定位到完整格线')
        return grid

    def scene(self, label):
        if self.trial.owns_panel:
            raise RuntimeError('相机推理前先停稳并释放手动控制')
        frame = self.frame()
        hp.write_image(self.directory / 'source.png', frame)
        detections = self.infer(frame, label)
        localization = 'printed_reference'
        try:
            self.demo.observe(frame, ())
        except RuntimeError as exc:
            if str(exc) != '棋盘基准位置变化':
                raise
            config = dict(image='source.png', mode='auto', frame_id=label, orientation='unconfirmed',
                          camera=dict(id='sc3336-1280x720-fixed-focus-v1', width=1280, height=720))
            raw = self.inspect(config)
            grid = {f"{chr(97+p['column'])}{p['row']}":p for p in raw['points']}
            matrix = board_transform(grid)
            red = [transform_point(matrix, d['pixel'])[1] for d in detections if d['class_id']>=7]
            black = [transform_point(matrix, d['pixel'])[1] for d in detections if d['class_id']<7]
            if min(len(red), len(black))<3 or abs(float(np.median(red)-np.median(black)))<2:
                raise RuntimeError('当前局面的红黑端头不明确')
            config.update(orientation='manual_keep' if np.median(red)<np.median(black) else 'manual_flip',
                          plane_file=str((ROOT/'output/xy-registration-20261111/plane-map.yml').resolve()))
            self.demo.grid = square_map(self.inspect(config))
            self.demo.reference = frame
            self.demo.anchors = [(x,y,cv2.cvtColor(frame[y:y+h,x:x+w],cv2.COLOR_BGR2GRAY))
                                 for x,y,w,h in ((375,109,65,70),(900,109,60,70))]
            localization = 'native_grid'
        matrix = board_transform(self.demo.grid)
        cannon = moving_cannon(detections, matrix, self.fixed_board)
        second = self.frame()
        self.demo.observe(second, ())
        confirmed = moving_cannon(self.infer(second, label+'-confirm'), matrix, self.fixed_board)
        if (cannon is None)!=(confirmed is None) or (cannon is not None and
                math.dist(cannon['pixel'], confirmed['pixel'])>4):
            raise RuntimeError('连续两帧的红炮位置不一致，等待画面稳定后复测')
        hp.write_image(self.directory / f'{label}.jpg', frame)
        record = dict(detections=detections, cannon=cannon, grid=self.demo.grid, localization=localization)
        (self.directory/f'{label}.json').write_text(json.dumps(record, indent=2), encoding='utf-8')
        return cannon, detections

    def park(self):
        self.trial.acquire(require_raised=False)
        self.trial.z(self.trial.high)
        self.trial.xy(self.park_target)
        self.trial.stop()
        self.trial.owns_panel = False

    def run(self):
        # Each iteration observes the actual board after lifting and clearing it.
        while self.state['successes']<3 and self.state['trials']<16:
            self.park()
            label = f"test-{self.state['trials']:02d}-before"
            cannon, pieces = self.scene(label)
            if cannon is None and not self.state['held']:
                raise RuntimeError('盘面未找到测试红炮，吸头状态也未确认')
            target = self.demo.target(self.state['target'])
            if cannon is not None:
                target_pixel = self.demo.grid[self.state['target']]['pixel_geometric']
                for piece in pieces:
                    if piece is not cannon and math.dist(piece['pixel'], target_pixel)<24:
                        if piece['symbol']!='C' or math.dist(piece['pixel'], cannon['pixel'])>3:
                            raise RuntimeError('目标格被其他棋子占用')
                self.state['held'] = True
                self.save()
                self.trial.acquire()
                self.trial.pick_piece(self.demo.target('cannon', cannon['pixel']))
                self.trial.xy(self.park_target)
                self.trial.stop()
                self.trial.owns_panel = False
                remaining, _ = self.scene(f"test-{self.state['trials']:02d}-picked")
                if remaining is not None:
                    self.state['held'] = False
                    self.save()
                    raise RuntimeError('红炮仍在棋盘上，本次未吸起')
                self.state['held'] = True
                self.save()
                target = self.demo.target(self.state['target'])
            self.trial.acquire()
            self.trial.xy(target)
            self.demo.observe(self.frame(), ())
            self.trial.z(self.trial.release_height)
            self.demo.observe(self.frame(), ())
            self.trial.release_pulse_ms = self.state['pulse_ms']
            self.state['trials'] += 1
            self.save()
            self.trial.release_piece(self.state['target'])
            self.trial.xy(self.park_target)
            self.trial.stop()
            self.trial.owns_panel = False
            cannon, _ = self.scene(f"test-{self.state['trials']:02d}-landed")
            error = None
            if cannon is None:
                self.state['missing'] += 1
                self.state['successes'] = 0
                self.state['lower_ms'] = self.state['pulse_ms']
                if self.state['missing']>1:
                    raise RuntimeError('两次释放后盘面都未出现红炮，需检查吸头与盘外画面')
            else:
                self.state['held'] = False
                self.state['missing'] = 0
                actual = self.demo.target('cannon', cannon['pixel'])
                current_target = self.demo.target(self.state['target'])
                error = math.dist(actual['base_xy_mm'], current_target['base_xy_mm'])
                if error<=5:
                    self.state['successes'] += 1
                    self.state['target'] = 'h2' if self.state['target']=='e2' else 'e2'
                else:
                    self.state['successes'] = 0
                    self.state['upper_ms'] = self.state['pulse_ms']
            self.trial.note('release_result', pulse_ms=self.state['pulse_ms'], landing_error_mm=error,
                            actual_pixel=None if cannon is None else cannon['pixel'],
                            consecutive_successes=self.state['successes'])
            if not self.state['successes']:
                low, high = self.state['lower_ms'], self.state['upper_ms']
                if high-low<=2:
                    self.save()
                    raise RuntimeError('当前高度和供电下，脉冲范围内未找到稳定释放值')
                self.state['pulse_ms'] = (low+high)//2
            self.save()
        if self.state['successes']<3:
            raise RuntimeError('本轮测试尚未得到连续三次合格落点')
        plan = json.loads(self.plan_path.read_text(encoding='utf-8-sig'))
        if float(plan['z']['pickup_deg']) != self.trial.pick:
            raise RuntimeError('Z 示教在测试期间发生变化，测试值尚未写回')
        plan['release_pulse_ms'] = self.state['pulse_ms']
        plan['z']['release_offset_mm'] = self.trial.plan['z'].get('release_offset_mm',0)
        temporary = self.plan_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        temporary.replace(self.plan_path)
        self.trial.note('release_tuning_verified', pulse_ms=self.state['pulse_ms'],
                        release_offset_mm=self.trial.plan['z'].get('release_offset_mm',0),
                        trials=self.state['trials'])

    def close(self):
        self.demo.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('work', type=Path)
    parser.add_argument('--reference', required=True)
    parser.add_argument('--board', required=True)
    parser.add_argument('--adb', required=True)
    parser.add_argument('--geometry', type=Path, default=ROOT/'build/commit-verify/scara_geometry_tool.exe')
    parser.add_argument('--held', action='store_true')
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    tuning = Tuning(args)
    try:
        if args.run:
            for recovery in range(4):
                try:
                    tuning.run()
                    break
                except (RuntimeError, TimeoutError) as exc:
                    state = tuning.trial.request('/api/state')
                    recoverable = any(a['interlocked'] or
                                      (a['fault']==5 and a.get('motion_fault_detail')==3)
                                      for a in state['axes'])
                    rejected = '命令被驱动拒绝' in str(exc)
                    tuning.trial.stop()
                    tuning.trial.owns_panel = False
                    if recovery==3 or not (recoverable or rejected):
                        raise
                    tuning.trial.note('recover_before_retry', error=str(exc))
                    time.sleep(.3)
        else:
            cannon, _ = tuning.scene('observation')
            print(json.dumps(dict(cannon=cannon, motion=False), ensure_ascii=False))
    finally:
        tuning.close()


if __name__=='__main__':
    main()
