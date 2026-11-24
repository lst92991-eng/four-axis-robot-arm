"""按正式 C++ 格点走台架。20 只定位，21 搬同一颗测试棋子绕盘一圈。"""

import json
from pathlib import Path
import subprocess
import time
import tomllib

from drive import Drive
from quick import check, sample
from run_game import Bench, Magnet, field_limits


# ---------取得 90 个目标区---------
# 调用正式 C++ 格点工具，将每个（行,列）关联到基座毫米和关节角。
def read_points(tool, site, installation=None):
    reply = subprocess.run(
        [str(tool), '-' if installation is not None else site.name], cwd=site.parent,
        input=installation, capture_output=True, encoding='utf-8', errors='replace'
    )
    if reply.returncode:
        raise ValueError(reply.stderr.strip() or "C++ 格点规划失败")
    points = {}
    for line in reply.stdout.splitlines():
        row, column, x, y, j2, j3 = line.split()
        points[int(row), int(column)] = ((float(x), float(y)), (float(j2), float(j3)))
    if set(points) != {(r, c) for r in range(10) for c in range(9)}:
        raise ValueError("C++ 输出缺少棋盘格点")
    return points


# ---------规划整盘顺序区---------
def plan(bench, points, start_pose, first, carry):
    z = bench.heights['z']
    high, low = z['travel_deg'], z['pickup_deg']
    grip, release, hover = low, z.get('release_deg', low), z.get('hover_deg')
    if 'operator_estimated_rise_mm' in z:  # 历史夹具导入，正式 TOML 使用角度。
        deg_mm = (high - low) / z['operator_estimated_rise_mm']
        grip = low + deg_mm * z.get('pickup_gap_mm', 0)
        release = low + deg_mm * z.get('release_offset_mm', 0)
        hover = low + 2 * deg_mm
    if not carry and (hover is None or not 0 < (hover-low)/(high-low) <= 1):
        raise ValueError('20 号巡点需要在取子与搬运层之间实测 z.hover_deg')
    # 按行折返遍历棋盘，再旋转顺序，让操作者指定的格子排在第一位。
    order = [(r, c) for r in range(10) for c in (range(9) if r % 2 == 0 else reversed(range(9)))]
    cut = order.index(first)
    order = order[cut:] + order[:cut]
    actions = []
    pose = start_pose
    if bench.workspace is None:
        raise ValueError("全盘测试需要安装配置中的 cad_workspace 来选择角落的肘形")

    def horizontal(target, fixed=False):
        nonlocal pose
        path, transfer = bench.workspace.route(pose, target, fixed)
        for waypoint in path:
            if any(abs(a - b) > 0.0005 for a, b in zip(pose, waypoint)):
                actions.append(('xy', (pose, waypoint, transfer)))
            pose = waypoint

    # 先升到搬运层。带子模式每次把同一枚棋子送到下一格，最后送回起点。
    actions.append(('z', high))
    if carry:
        horizontal(points[first][1])
        for cell in order[1:] + [first]:
            actions.extend([('z', grip), ('magnet', 'hold'), ('z', high)])
            horizontal(points[cell][1])
            actions.extend([('z', release), ('magnet', 'off'), ('record', cell), ('z', high)])
    # 定位模式逐格下探并记录，在横移之前回到搬运层。
    else:
        for cell in order:
            horizontal(points[cell][1])
            actions.extend([('z', hover), ('record', cell), ('z', high)])
    horizontal(bench.config['park_joints_deg'], fixed=True)
    return actions


def measured_config(site, installation=None):
    if site.suffix.lower() == '.toml':
        setup = tomllib.loads(installation if installation is not None else site.read_text(encoding='utf-8-sig'))
        arm, motion = setup['arm'], setup['motion']
        j2, j3 = arm['reference_drive_deg']
        shoulder, elbow = arm['reference_joint_rad']
        return dict(arm_lengths_mm=arm['lengths_mm'], j2_reference_deg=j2, j3_reference_motor_deg=j3,
                    shoulder_ref_rad=shoulder, elbow_ref_rad=elbow, joint_per_drive=arm['joint_per_drive'],
                    z=setup['z'], park_joints_deg=motion['park_deg'],
                    joint_velocity_deg_s=motion['velocity_deg_s'], joint_acceleration_deg_s2=motion['acceleration_deg_s2'],
                    camera_url=setup['vision'].get('camera_url'), pickup_at_rest_ms=setup['z'].get('grip_dwell_ms', 420),
                    **field_limits(str(Path(__file__).resolve().parent/'reference/workspace.npz')))
    # 独立巡点命令也能导入早期分文件的实测夹具。
    config = json.loads(site.read_text(encoding='utf-8'))
    for key in ('arm_file', 'heights_file'):
        if config.get(key):
            values = json.loads((site.parent/config[key]).read_text(encoding='utf-8'))
            config.update(values)
    if config.get('cad_workspace'):
        cad = config['cad_workspace']
        cad['samples'] = str((site.parent/cad['samples']).resolve())
    if config.get('motion_tuning_file'):
        config['motion_tuning'] = json.loads((site.parent/config['motion_tuning_file']).read_text(encoding='utf-8'))
    return config


# ---------巡点执行与记录区---------
def sweep(args, site, tool, log):
    # 格点与台架动作使用同一份 TOML 快照。
    installation = site.read_text(encoding='utf-8-sig') if site.suffix.lower() == '.toml' else None
    points = read_points(tool, site, installation)
    config = measured_config(site, installation)
    if not args.plan and site.suffix.lower() == '.toml' and not config.get('camera_url'):
        raise ValueError('实机台架请选择含现场相机地址的 TOML')
    bench = Bench(config)
    drive = None
    bridge = None
    active = False
    started = time.monotonic()
    completed = 0
    try:
        # 实机从当前反馈姿态开始规划，离线规划从保存的观察姿态开始。
        pose = bench.config['park_joints_deg']
        if not args.plan:
            drive = bench.drive = Drive()
            check(sample(drive), motion=True)
            pose = tuple(drive.states[n]['position'] for n in (2, 3))
        actions = plan(bench, points, pose, args.start, args.case == 'pick90')
        if args.plan:
            log('plan', actions=actions, count=90, start=args.start)
            print(
                f"离线规划 90 格，横移 {sum(k == 'xy' for k, _ in actions)} 段，"
                f"正向吸合 {actions.count(('magnet', 'hold'))} 次，末段回观察位。"
            )
            return
        active = True
        drive.read_initial()
        bench.apply_drive_tuning()
        # 带子测试建立 H 桥会话，让等待 CAN 反馈期间也能维持连接。
        if args.case == 'pick90':
            bridge = bench.bridge = Magnet(drive.poll)
            drive.on_poll = bench.poll_bridge
        high = bench.heights['z']['travel_deg']
        speed = bench.config['joint_velocity_deg_s'][0]
        for kind, value in actions:
            if kind == 'z':
                drive.z(value, high=high, speed_limit=speed)
            elif kind == 'xy':
                origin, target, transfer = value
                rates = bench.xy_speed(origin, target)
                if transfer:
                    rates = [min(a, b) for a, b in zip(rates, bench.workspace.transfer_speed)]
                drive.xy(*target, *rates)
            elif kind == 'magnet':
                bench.bridge_command(value)
                if value == 'hold':
                    dwell = (bench.pickup_at_rest_ms or 170) / 1000
                    until = time.monotonic() + dwell
                    drive.wait(lambda: time.monotonic() >= until, dwell + 0.2)
            # 记录格号、目标毫米坐标和各轴实际反馈，随后继续下一格。
            elif kind == 'record':
                completed += 1
                log('point', number=completed, cell=value, target_mm=points[value][0], axes=drive.states)
                print(f"{completed:02d}/90 {value} 到位", flush=True)
        drive.finish_park()
        log('parked', points=completed, elapsed_s=time.monotonic() - started)
    # 实机中途退出时关断磁铁、停车并记录已完成的格数。
    except BaseException:
        if active:
            try:
                if bridge:
                    bench.bridge_command('off')
            finally:
                drive.on_poll = None
                log('stopped', confirmed=drive.stop(), completed_points=completed)
        raise
    finally:
        if drive:
            drive.on_poll = None
        try:
            if bridge:
                bridge.close()
        finally:
            if drive:
                drive.close()
