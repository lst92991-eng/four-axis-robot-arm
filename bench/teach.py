"""用三点实际对准时的 J2/J3 角度及两次 J1 示教值生成一份现场记录。"""

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
TEMPLATE = """# 黑方底线行0，红方底线行9，列0在红方视角左侧。用实际数值替换下划线。
# 三点依次记 行 列 原图u 原图v 对中时J2角 对中时J3电机角
point 9 0 _ _ _ _
point 9 8 _ _ _ _
point 0 0 _ _ _ _
# 已确定的机械参考：J2读数 J3电机读数 大臂物理角度 小臂相对大臂角度 J2读数换算比例
ref _ _ _ _ _
# J1 吸取高度角、搬运高度角、释放高度角，最后是吸取至搬运的实测毫米数
z _ _ _ _
# 观察位J2/J3角，盘外弃子点基座X/Y毫米
park _ _
discard _ _
# 固定相机的 RTSP 地址和已验证的 1280×720 原图识别 ROI
camera rtsp://_/live/0
roi _ _ _ _
"""


# ---------生成 C++ 安装参数区---------
def station_header(cfg, release, red_left_trim=(0, 0)):
    """把这一台的实测数值直接写进 Setup_game，学生也可以手写同样的赋值。"""
    number = lambda value: repr(float(value))
    vector = lambda values: ', '.join(map(number, values))
    data = cfg['points']['data']
    lines = ['#pragma once', '#include "app/game.hpp"', '', 'namespace scara', '{',
             '//---------安装参数区---------',
             'inline void Setup_game(GameConfig &config, Session &session)', '{',
             '    // (9,0)、(9,8)、(0,0)，每点依次写原图像素和基座毫米。']
    # 按红左、红右、黑左取出三组像素与毫米坐标，让 C++ 按相同顺序标定。
    for i, name in enumerate(('redLeft','redRight','blackLeft')):
        row = data[4*i:4*i+4]
        lines.append('    BoardPoint '+name+' = {{'+vector(row[:2])+'}, {'+vector(row[2:])+'}};')
    lines.append('    Geometry_calibrate(redLeft, redRight, blackLeft, session.layout);')
    if any(red_left_trim):
        lines.append('    // 这一机位的红左角有单格修正，只改机械臂到达的毫米位置。')
        lines.append('    session.layout.cell[9][0].xy += cv::Point2d(' + vector(red_left_trim) + ');')
    lines += ['', '    // 关节参考角和 J3 传动方向取自这台机械臂。', '    session.arm.upperMm = 275;', '    session.arm.forearmMm = 275;',
              '    session.arm.j3Ratio = 3;', '    session.arm.j3Sign = -1;']
    for name, value in [('arm.j2ReferenceDeg', cfg['j2_reference_deg']), ('arm.j3ReferenceMotorDeg', cfg['j3_reference_motor_deg']),
                        ('arm.j2Scale', cfg['j2_scale']), ('arm.shoulderReferenceRad',cfg['shoulder_ref_rad']),
                        ('arm.elbowReferenceRad',cfg['elbow_ref_rad']), ('heights.pickupDeg', cfg['z']['pickup_deg']),
                        ('heights.travelDeg',cfg['z']['travel_deg']), ('heights.releaseDeg',release)]:
        lines.append(f'    session.{name} = {number(value)};')
    lines += ['    // 吸取、搬运和释放层使用 J1 电机角度。这里选搬运层作为开始横移的高度。',
              '    session.heights.clearanceDeg = session.heights.travelDeg;',
              '    // 观察位用 J2 和 J3 的驱动角，盘外弃子点用基座 XY 毫米。',
              '    session.park = {' + vector(cfg['park_joints_deg']) + '};',
              '    session.capturePlaceMm = {' + vector(cfg['capture_place_mm']) + '};', '',
              '    // 推理 ROI 用原图像素。三轴限速和加速度均按 J1、J2、J3 排列。',
              '    config.cameraUrl = ' + json.dumps(cfg['camera_url'], ensure_ascii=False) + ';',
              '    config.visionRoi = {' + ', '.join(map(str,cfg['vision_roi'])) + '};',
              '    config.motion.velocity = {4800, 42, 106};',
              '    config.motion.acceleration = {5700, 110, 180};',
              '    config.engineDir = "engine/pikafish-2023-03-05";', '}', '}']
    return '\n'.join(lines) + '\n'


# ---------读取实物示教表区---------
def main(argv, binary):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('sheet', nargs='?', type=Path, help='填好的示教表')
    p.add_argument('--template', type=Path, help='生成待填写的示教表')
    p.add_argument('--install', action='store_true', help='核对后写入现场配置；默认只演算')
    p.add_argument('--output', type=Path, default=ROOT / 'config/site/installation.site')
    args = p.parse_args(argv)
    if args.template:
        with args.template.open('x', encoding='utf-8') as output:
            output.write(TEMPLATE)
        print(args.template.resolve())
        return
    if not args.sheet:
        p.error('请提供示教表，或用 --template 文件名 生成模板')
    values, points = {}, {}
    lengths = {'ref': 5, 'z': 4, 'park': 2, 'discard': 2, 'roi': 4}
    for number, line in enumerate(args.sheet.read_text(encoding='utf-8-sig').splitlines(), 1):
        words = line.partition('#')[0].split()
        if not words:
            continue
        key, data = words[0], words[1:]
        if key == 'camera' and len(data) == 1 and key not in values:
            values[key] = data[0]
            continue
        nums = list(map(float, data))
        if not all(map(math.isfinite, nums)):
            raise ValueError(f"示教表第 {number} 行需要有限数值")
        if key == 'point' and len(nums) == 6:
            cell = tuple(nums[:2])
            if cell not in ((9, 0), (9, 8), (0, 0)) or cell in points:
                raise ValueError(f"示教表第 {number} 行格点错误或重复")
            points[cell] = nums[2:]
        elif key in lengths and key not in values and len(nums) == lengths[key]:
            values[key] = nums
        else:
            raise ValueError(f"示教表第 {number} 行字段或数量错误")
    if len(points) != 3 or set(values) != set(lengths) | {'camera'}:
        raise ValueError("请填完三点、机械参考、工作高度、观察位、弃子点和相机资料")
    if not values['camera'].startswith('rtsp://') or '_' == values['camera'][7:].split('/')[0]:
        raise ValueError("请填写现场相机地址")
    u, v, w, h = values['roi']
    if not all(x.is_integer() for x in (u, v, w, h)) or not (
        0 <= u < u + w <= 1280 and 0 <= v < v + h <= 720
    ):
        raise ValueError("相机 ROI 超出原图")
    destination = args.output.resolve()
    reference = HERE / 'reference'
    if args.install and destination.suffix.lower() != '.site':
        raise ValueError("现场记录请使用 .site 扩展名")
    if destination.is_relative_to(reference):
        raise ValueError("请选择本机配置输出位置")
    cfg = json.loads((reference / 'game.json').read_text(encoding='utf-8'))
    j2, j3, shoulder, elbow, scale = values['ref']
    low, high, release, rise = values['z']
    if scale <= 0 or low == high or rise <= 0:
        raise ValueError("机械比例或 Z 层高无效")
    if not 0 <= (release - low) / (high - low) <= 1:
        raise ValueError("释放高度需要在吸取与搬运两层之间")
    cfg.update(
        j2_reference_deg=j2,
        j3_reference_motor_deg=j3,
        j2_scale=scale,
        shoulder_ref_rad=math.radians(shoulder),
        elbow_ref_rad=math.radians(elbow),
        camera_url=values['camera'],
        vision_roi=list(map(int, values['roi'])),
        park_joints_deg=values['park'],
        capture_place_mm=values['discard'],
    )
    cfg['z'].update(
        pickup_deg=low,
        travel_deg=high,
        clearance_deg=high,
        operator_estimated_rise_mm=rise,
        pickup_gap_mm=0,
        release_offset_mm=(release - low) / (high - low) * rise,
    )
    cfg['z'].pop('capture_release_deg', None)
    for key in (
        'placement_offsets_mm',
        'target_offsets_mm',
        'motion_tuning_joints_deg',
        'image',
        'frame_id',
        'note',
    ):
        cfg.pop(key, None)
    cfg['motion_tuning'].pop('legs', None)
    cfg['cad_workspace']['samples'] = os.path.relpath(reference / 'workspace.npz', destination.parent)
    cfg['work_directory'] = os.path.relpath(ROOT / 'output/bench/game-state', destination.parent)
    tool = binary('scara_bench_points')
    # 用 C++ 正解把三次对中时的 J2、J3 读数换成吸头 XY，再逐格核对逆解。
    with TemporaryDirectory(prefix='scara-site-') as temp:
        tmp = Path(temp) / 'game.json'
        tmp.write_text(json.dumps(cfg), encoding='utf-8')
        rows = []
        for cell in ((9, 0), (9, 8), (0, 0)):
            u, v, j2, j3 = points[cell]
            if not (0 <= u < 1280 and 0 <= v < 720):
                raise ValueError(f"{cell} 像素超出原图")
            result = subprocess.check_output(
                [str(tool), tmp.name, '--forward', str(j2), str(j3)], cwd=temp, text=True
            )
            xy = list(map(float, result.split()))
            rows.extend([u, v, *xy])
            print(f"{cell}  基座 XY {xy[0]:.3f}, {xy[1]:.3f} mm")
        cfg['points']['data'] = rows
        text = json.dumps(cfg, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
        tmp.write_text(text, encoding='utf-8')
        subprocess.run([str(tool), tmp.name], cwd=temp, stdout=subprocess.DEVNULL, check=True)
    # .site 供独立台架测量工具读取，对局只使用编入程序的 C++ 数值表。
    fmt = lambda row: ' '.join(format(float(value), '.16g') for value in row)
    records = [
        'SCARA1',
        *(fmt(rows[n : n + 4]) for n in range(0, 12, 4)),
        fmt(values['ref']),
        fmt((low, high, release)),
        fmt(values['park'] + values['discard']),
        values['camera'] + ' ' + fmt(values['roi']),
        fmt([rise]),
    ]
    # 带 --install 时保存巡点用的 .site 和对局编译用的同名 .hpp，两份都取自本次测量。
    if args.install:
        destination.parent.mkdir(parents=True, exist_ok=True)
        from tempfile import NamedTemporaryFile

        with NamedTemporaryFile('w', encoding='utf-8', dir=destination.parent, delete=False) as staged:
            staged.write('\n'.join(records) + '\n')
        Path(staged.name).replace(destination)
        header = destination.with_suffix('.hpp')
        with NamedTemporaryFile('w', encoding='utf-8', dir=header.parent, delete=False) as staged:
            staged.write(station_header(cfg, release))
        Path(staged.name).replace(header)
        print(f"本机示教表 {header} 已生成。运行 01-build，把这些数值编入对局。")
    else:
        print("已用 C++ 核对 90 个格点及逆解。--install 将保存这份测量。")


# ---------独立巡点工具导入区---------
def field_config(site):
    """仅供 Windows 过渡台架。学生编辑的 .site 与课堂 C++ 使用同一测量。"""
    from hashlib import sha256
    from tempfile import NamedTemporaryFile

    if site.suffix.lower() != '.site':
        return site
    source = site.read_bytes()
    fields = source.decode('utf-8-sig').split()
    if len(fields) != 31 or fields[0] != 'SCARA1':
        raise ValueError("现场记录需要三点、机械参考、高度、观察/弃子位、相机及搬运毫米数")
    nums = list(map(float, fields[1:25]))
    if not all(map(math.isfinite, nums)):
        raise ValueError("现场标定含无效数值")
    cfg = json.loads((HERE / 'reference/game.json').read_text(encoding='utf-8'))
    cfg['points']['data'] = nums[:12]
    j2, j3, shoulder, elbow, scale = nums[12:17]
    low, high, release = nums[17:20]
    rise = float(fields[-1])
    if not math.isfinite(rise) or rise <= 0 or low == high or scale <= 0:
        raise ValueError("现场记录缺少有效的运动参考")
    cfg.update(
        j2_reference_deg=j2,
        j3_reference_motor_deg=j3,
        j2_scale=scale,
        shoulder_ref_rad=math.radians(shoulder),
        elbow_ref_rad=math.radians(elbow),
        park_joints_deg=nums[20:22],
        capture_place_mm=nums[22:24],
        camera_url=fields[25],
        vision_roi=list(map(int, fields[26:30])),
    )
    cfg['z'] = dict(
        pickup_deg=low,
        travel_deg=high,
        release_deg=release,
        clearance_deg=high,
        operator_estimated_rise_mm=rise,
        release_offset_mm=(release - low) / (high - low) * rise,
        pickup_gap_mm=0,
    )
    for key in (
        'placement_offsets_mm',
        'target_offsets_mm',
        'motion_tuning_joints_deg',
        'image',
        'frame_id',
        'note',
    ):
        cfg.pop(key, None)
    cfg['motion_tuning'].pop('legs', None)
    # 按源记录的内容摘要区分缓存目录，供独立台架工具复用同一份现场测量。
    folder = ROOT / 'output/bench/sites' / sha256(source).hexdigest()[:16]
    cfg['cad_workspace']['samples'] = os.path.relpath(HERE / 'reference/workspace.npz', folder)
    cfg['work_directory'] = os.path.relpath(ROOT / 'output/bench/game-state', folder)
    folder.mkdir(parents=True, exist_ok=True)
    dest = folder / 'game.json'
    text = json.dumps(cfg, indent=2, ensure_ascii=False, allow_nan=False) + '\n'
    if not dest.exists() or dest.read_text(encoding='utf-8') != text:
        with NamedTemporaryFile('w', encoding='utf8', dir=folder, delete=False) as stage:
            stage.write(text)
        Path(stage.name).replace(dest)
    return dest
