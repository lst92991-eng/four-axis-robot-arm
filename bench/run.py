"""编号 CMD 的公共入口。--plan 只读配置，真实动作各用一次现场预检。"""

import argparse
import hashlib
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import sys
import uuid

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
BIN = Path(os.environ.get('SCARA_BENCH_BIN', ROOT / 'build/bench/bin')).resolve()
REFERENCE = HERE / 'reference/game.json'
sys.path.insert(0, str(HERE / 'field'))


# ---------工具和测量记录区---------
def binary(name):
    path = BIN / (name + ('.exe' if os.name == 'nt' else ''))
    if not path.is_file():
        raise FileNotFoundError(f"缺少 {path.name}，先运行 01-build.cmd")
    return path


# 优先选显式指定的安装记录，离线规划可用保存的参考夹具。
def site_file(args):
    from teach import INSTALLATION, field_config

    named = args.site or os.environ.get('SCARA_SITE')
    site = Path(named).resolve() if named else INSTALLATION
    if not named and not site.exists() and site.with_suffix('.site').exists():
        site = site.with_suffix('.site')
    if not named and not site.exists() and args.plan:
        site = REFERENCE
    if not site.is_file():
        raise FileNotFoundError(f"缺少本机安装记录 {site}，请用 15-site 保存，或以 --site 指定现有记录")
    if site == REFERENCE and not args.plan:
        raise ValueError("历史扫描夹具只用于 --plan。实机请选择固定后已校验的安装配置。")
    return field_config(site)


# ---------编号命令分发区---------
def main():
    if sys.argv[1:] == ['menu']:
        return menu()
    if len(sys.argv) > 1 and sys.argv[1] == 'update':
        from flash_drive import update

        update(sys.argv[2:])
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == 'site':
        from teach import main as teach

        teach(sys.argv[2:], binary)
        return 0
    if len(sys.argv) > 1 and sys.argv[1] == 'flash':
        from flash_drive import main as flash

        flash(sys.argv[2:])
        return 0
    parser = argparse.ArgumentParser(description=__doc__)
    cases = parser.add_subparsers(dest='case', required=True)
    status = cases.add_parser('check', help='读取当前 CAN 状态')
    status.add_argument('--four', action='store_true', help='首装验收同时要求 J4 在线')
    frame = cases.add_parser('frame', help='读取实际相机的一张原图，不连接电机')
    frame.add_argument('--url', help='标定前可以直接填写 RTSP 地址')
    frame.add_argument('--site', type=Path)
    frame.add_argument('--show', action='store_true', help='打开本次原图')
    frame.set_defaults(plan=False)
    electrical = cases.add_parser('electrical',help='单轴电角标定，默认说明，--run 才通电标定')
    electrical.add_argument('axis',type=str.upper,choices=['J1','J2','J3','J4'])
    electrical.add_argument('--run',action='store_true')
    electrical.add_argument('--build',default='9fc6a5de')
    ref = cases.add_parser('reference',help='在实物已对齐标记时写回三轴参考角，不移动')
    select_ref = ref.add_mutually_exclusive_group(required=True)
    select_ref.add_argument('--angles',nargs=3,type=float,metavar=('J1','J2','J3'))
    select_ref.add_argument('--at-park',action='store_true',help='实物已对准该站点的搬运高度和观察位')
    ref.add_argument('--site',type=Path)
    ref.add_argument('--write',action='store_true')
    for key in ('grid90', 'pick90', 'game'):
        p = cases.add_parser(key)
        if key != 'game':
            p.add_argument('--site', type=Path)
        p.add_argument('--plan', action='store_true', help='离线检查，不打开硬件')
        if key == 'game':
            p.add_argument('--mock', action='store_true')
            p.add_argument('--no-browser', action='store_true')
            p.add_argument('--port', type=int, default=8121)
        else:
            p.add_argument(
                '--start',
                nargs=2,
                type=int,
                default=(0, 0),
                metavar=('ROW', 'COLUMN'),
                help='起点行列，21 的唯一测试棋子放在此处',
            )
    cases.add_parser('vision', help='保存图像与 NPU 张量的 C++ 回放')
    sim = cases.add_parser('sim', help='运行正式 C++ main 的离线棋局')
    sim.add_argument('--all', action='store_true', help='运行完整回归场景')
    args = parser.parse_args()
    if args.case == 'reference':
        args.plan = not args.write
    if args.case == 'electrical':
        args.plan = not args.run
    # 对局面板直接启动独立进程，实际开局由面板的开局按钮触发。
    if args.case == 'game':
        if args.plan:
            print("启动内存对局面板，点击开局才使用设备。" if not args.mock else "离线面板。")
            return 0
        cmd = [sys.executable, str(ROOT/'tools/match_panel.py'), '--port', str(args.port)]
        if args.mock:
            cmd.append('--mock')
        if args.no_browser:
            cmd.append('--no-browser')
        return subprocess.call(cmd, cwd=ROOT)
    run_id = datetime.now().strftime('%Y%m%d-%H%M%S') + '-' + uuid.uuid4().hex[:6]
    case_ids = {'check':'00','frame':'12','electrical':'13','reference':'14','grid90':'20','pick90':'21','vision':'30','sim':'40','game':'50'}
    # 诊断与回放各用独立目录，结果按行记录，包含本次命令编号。
    directory = ROOT / 'output/bench/runs' / (run_id + '-' + case_ids[args.case] + '-' + args.case)
    directory.mkdir(parents=True)
    os.environ['SCARA_RUN_DIR'] = str(directory)
    path = directory / 'result.jsonl'
    print(f"记录 {path}", flush=True)
    with path.open('x', encoding='utf-8') as stream:

        def log(event, **data):
            rec = dict(time=datetime.now().astimezone().isoformat(), event=event, **data)
            stream.write(json.dumps(rec, ensure_ascii=False) + '\n')
            stream.flush()

        log(
            'start',
            case=args.case,
            offline=bool(
                getattr(args, 'plan', False) or getattr(args, 'mock', False) or args.case in ('sim', 'vision')
            ),
        )
        try:
            # 从同一 USB-CAN 连接取得节点状态，核对后释放连接。
            if args.case == 'check':
                from drive import Drive
                from quick import sample, check

                drive = Drive()
                try:
                    states = sample(drive, args.four)
                    log('feedback', axes=states)
                    check(states, args.four)
                finally:
                    drive.close()
            elif args.case == 'electrical':
                print(f"{args.axis} 电角标定，预期版本 {args.build}，--run 将通电并保存该轴标定")
                if args.run:
                    subprocess.run([sys.executable,str(ROOT/'manual_control/calibrate_j1.py'),
                                    '--node',args.axis[1:],'--expected-build',args.build,'--electrical-only',
                                    '--serial',os.environ.get('SCARA_CAN_SERIAL',''),'--out',str(directory),'--run'],
                                   cwd=ROOT,check=True)
            elif args.case == 'reference':
                angles=args.angles
                if args.at_park:
                    site=site_file(args)
                    from sweep import measured_config
                    cfg = measured_config(site)
                    angles = [cfg['z']['travel_deg'], *cfg['park_joints_deg']]
                print("实物对准的 J1/J2/J3 参考角",*angles,flush=True)
                log('reference_request',angles_deg=angles,write=args.write)
                if args.write:
                    from drive import Drive
                    from quick import set_reference, sample
                    drive=Drive()
                    try:
                        set_reference(drive,sample(drive),angles,log)
                    finally:
                        drive.close()
            # 从 RTSP 取一张原图保存到本次目录，供相机视野和清晰度核对。
            elif args.case == 'frame':
                import cv2

                url = args.url
                if not url:
                    from sweep import measured_config
                    url = measured_config(site_file(args))['camera_url']
                camera = cv2.VideoCapture(
                    url,
                    cv2.CAP_FFMPEG,
                    [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000, cv2.CAP_PROP_READ_TIMEOUT_MSEC, 5000],
                )
                try:
                    ok, image = camera.read()
                    if not ok:
                        raise RuntimeError("相机没有返回图像，请核对 RTSP 地址和相机服务")
                    photo = directory / 'frame.jpg'
                    ok, jpeg = cv2.imencode('.jpg', image)
                    if not ok:
                        raise RuntimeError("原图编码失败")
                    photo.write_bytes(jpeg.tobytes())
                    log('frame', width=image.shape[1], height=image.shape[0], file=photo.name)
                    print(f"{image.shape[1]} × {image.shape[0]}  {photo}")
                    if args.show:
                        os.startfile(photo)
                finally:
                    camera.release()
            elif args.case == 'sim':
                exe = str(binary('scara_game_sim'))
                cmd = (
                    [sys.executable, str(ROOT / 'tests/run_match_program.py'), exe]
                    if args.all
                    else [exe, 'tests/fixtures/game-app/installation.toml', 'tests/fixtures/game-app/two-rounds.json']
                )
                subprocess.run(cmd, cwd=ROOT, check=True)
            # 逐帧回放原图和保存的张量，核对 RGB 输入、类别、分数及框的位置。
            elif args.case == 'vision':
                replay = binary('scara_bench_vision')
                corpus = Path('tests/fixtures/vision/review-corpus.json')
                subprocess.run(
                    [str(replay), str(corpus), str(directory.relative_to(ROOT))], cwd=ROOT, check=True
                )
                cases = json.loads((ROOT / corpus).read_text(encoding='utf-8'))['cases']
                total = 0
                for item in cases:
                    stem = directory / item['id']
                    actual = json.loads(stem.with_suffix('.json').read_text(encoding='utf-8'))['detections']
                    expected = item.get('expected', [])
                    if (
                        hashlib.sha256(stem.with_suffix('.rgb').read_bytes()).hexdigest()
                        != item['input_sha256']
                    ):
                        raise ValueError(f"{stem.name} 模型输入像素变化")
                    if len(actual) != len(expected):
                        raise ValueError(f"{stem.name} 检测数量变化")
                    for a, b in zip(actual, expected):
                        if (
                            a['class_id'] != b['class_id']
                            or abs(a['score'] - b['score']) > 0.0001
                            or max(abs(x - y) for x, y in zip(a['bbox'], b['bbox'])) > 0.01
                        ):
                            raise ValueError(f"{stem.name} 类别、分数或框变化")
                    total += len(actual)
                log('replay', frames=len(cases), detections=total)
                print(f"回放通过 {len(cases)} 帧，{total} 个检测", flush=True)
                subprocess.run(
                    [sys.executable, str(ROOT / 'tests/saved_tensor_input_test.py'), str(replay)],
                    cwd=ROOT,
                    check=True,
                )
            else:
                args.start = tuple(args.start)
                if args.start[0] not in range(10) or args.start[1] not in range(9):
                    raise ValueError("行须为 0 至 9，列须为 0 至 8")
                from sweep import sweep

                site = site_file(args)
                log('site', file=str(site))
                print(f"使用 {site}", flush=True)
                os.chdir(site.parent)
                sweep(args, site, binary('scara_bench_points'), log)
            log('finished')
        except KeyboardInterrupt:
            log('interrupted')
            return 130
        except Exception as error:
            log('failed', reason=str(error))
            print(f"失败  {error}", file=sys.stderr)
            return 1
    return 0


# ---------人工选择编号区---------
def menu():
    choices = {
        '00': ('读取 CAN 状态', 'check'),
        '10': ('单板 SWD 首装', 'flash'),
        '11': ('有引导的驱动板 CAN 升级', 'update'),
        '12': ('查看相机原图', 'frame'),
        '13': ('单轴电角标定','electrical'),
        '14': ('在实物标记处接续机械参考','reference'),
        '15': ('读取示教表并安装现场记录', 'site'),
        '20': ('90 点空吸头悬停定位', 'grid90'),
        '21': ('空盘上一颗棋子走过 90 格并回起点', 'pick90'),
        '30': ('视觉离线回放', 'vision'),
        '40': ('离线 C++ 棋局', 'sim'),
        '50': ('打开人机对弈页面', 'game'),
    }
    while True:
        print('\n' + '\n'.join(f'{n}  {title}' for n, (title, _) in choices.items()))
        number = input('输入编号，回车退出 > ').strip()
        if not number:
            return 0
        if number not in choices:
            continue
        args = [choices[number][1]]
        if number in ('10', '11'):
            axes = input('目标轴，例如 J1；11 可输入多个轴 > ').upper().split()
            args.extend(axes)
            if input('写入输入 W，回车只验证镜像 > ').strip().upper() == 'W':
                args.append('--write')
        if number == '12':
            url = input('RTSP 地址，已有现场记录可直接回车 > ').strip()
            if url:
                args.extend(['--url', url])
            args.append('--show')
        if number == '13':
            args.append(input('目标轴 J1/J2/J3/J4 > ').strip().upper())
            if input('通电标定并保存输入 R，回车只查看 > ').strip().upper()=='R':
                args.append('--run')
        if number == '14':
            values=input('已对齐的 J1 J2 J3 角度；已对齐观察位可回车 > ').split()
            args.extend(['--angles',*values] if values else ['--at-park'])
            if input('已在实物标记处，写参考输入 W，回车只查看 > ').strip().upper()=='W':
                args.append('--write')
        if number == '15':
            sheet = input('示教表路径，回车生成空表 > ').strip().strip('"')
            if not sheet:
                args.extend(['--template', 'site-measurements.txt'])
            else:
                args.append(sheet)
                if input('安装到本机输入 W，回车仅校验 > ').strip().upper() == 'W':
                    args.append('--install')
        if number in ('20', '21'):
            if number == '21':
                args.extend(['--start', *(input('唯一棋子的行 列 [0 0] > ').split() or ['0', '0'])])
            if input('开始实机测试输入 R，回车仅规划 > ').strip().upper() != 'R':
                args.append('--plan')
        status = subprocess.run([sys.executable, '-u', str(HERE / 'run.py'), *args]).returncode
        print(f"退出码 {status}")


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, OSError, KeyError, subprocess.CalledProcessError) as error:
        print(f"失败  {error}", file=sys.stderr)
        raise SystemExit(1)
