"""把现场三点和直接示教角保存到同一份 TOML。"""

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import subprocess
from tempfile import NamedTemporaryFile, TemporaryDirectory
import tomllib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
INSTALLATION = ROOT / 'config/site/installation.toml'
CORNERS = {(9, 0): 'red_left', (9, 8): 'red_right', (0, 0): 'black_left'}
TEMPLATE = """# 从红方看。每个角填：原图 u v（像素）、吸头对中时的 J2 J3 驱动角（度）。
red_left _ _ _ _
red_right _ _ _ _
black_left _ _ _ _
# 同一参考姿态下填：驱动读数（度）、机械角（度）。使用本机转角比例；第三项可覆盖比例。
j2 _ _
j3 _ _
# 可选：两端已有的驱动读数（度，顺序任意）。工具按本机参考换成机械角保存。
# j2_stops _ _
# j3_stops _ _
# J1 贴子驱动角。本机默认抬升约 4.3 cm；已实测搬运角时可填第二项。
z _
observe _ _           # 观察位的 J2 J3 驱动角（度）
discard _ _           # 弃子点在基座上的 X Y（毫米）
camera rtsp://_/live/0
# 取景框固定 640×640。换了取景位置才填左上角：
# crop_px _ _
# 20 号下探巡点另填实测的 J1 悬停角：
# hover _
"""


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile('w', encoding='utf8', newline='', dir=path.parent, delete=False) as stage:
        stage.write(text)
    Path(stage.name).replace(path)


def site_text(site):
    notes = {
        'j2': '大臂。drive_deg 和 joint_deg 是同一姿态的驱动读数和机械角；ratio 是关节转角 / 驱动转角。可选 range_deg 记录相对基座的实测端点。',
        'j3': '小臂。机械角相对大臂，负比例表示转向相反。可选 range_deg 记录该关节的实测端点。',
        'z': 'J1 贴子角。缺省从这里抬升约 4.3 cm，有实测搬运角时用 carry_deg。',
        'motion': '观察位为 J2/J3 驱动角，弃子点为基座毫米，速度与加速度按 J1/J2/J3 排列。',
        'camera': 'crop_px 是 640×640 取景框的左上角像素。',
        'board': '从红方看，每个点依次是图像 u、v 像素和基座 X、Y 毫米。',
        'game': '本机对局设置，沿用旧表中的先手、执子方和引擎设置。',
    }
    sections = []
    for name, values in site.items():
        lines = [f'# {notes[name]}', f'[{name}]']
        for key, value in values.items():
            lines.append(f'{key} = ' + json.dumps(value, ensure_ascii=False, allow_nan=False))
        sections.append('\n'.join(lines))
    return '\n\n'.join(sections) + '\n'


def crop_origin(rect):
    if any(type(v) not in (int, float) or not math.isfinite(v) or int(v) != v for v in rect):
        raise ValueError('取景框需要整数像素')
    x, y, *size = map(int, rect)
    if size:  # 旧表填了整个矩形，用其中心恢复原来的方形取景位置。
        w, h = size
        if not (0 <= x and 0 <= y and 0 < w <= 1280-x and 0 < h <= 720-y):
            raise ValueError('旧取景框超出原图')
        x = min(640, max(0, x + int((w-640)/2)))
        y = min(80, max(0, y + int((h-640)/2)))
    if not (0 <= x <= 640 and 0 <= y <= 80):
        raise ValueError('640×640 取景框超出原图')
    return [x, y]


def make_site(values, board):
    defaults = tomllib.loads((ROOT/'config/installation.toml').read_text(encoding='utf8'))
    site = {}
    for axis in ('j2', 'j3'):
        drive, joint, *ratio = values[axis]
        site[axis] = dict(link_mm=defaults[axis]['link_mm'], drive_deg=drive, joint_deg=joint,
                          ratio=ratio[0] if ratio else defaults[axis]['ratio'])
        if axis + '_stops' in values:
            site[axis]['range_deg'] = sorted(joint + (reading-drive)*site[axis]['ratio']
                                             for reading in values[axis + '_stops'])
    touch, *extra = values['z']
    z = dict(touch_deg=touch)
    if extra:
        z['carry_deg'] = extra[0]
    if len(extra) >= 2 and extra[1] != touch:
        z['release_deg'] = extra[1]
    if 'hover' in values:
        z['hover_deg'] = values['hover'][0]
    elif len(extra) == 3:  # 旧表最后一项为实测抬升毫米数；20 号保留其 2 mm 悬停位。
        if extra[2] <= 0:
            raise ValueError('旧示教行程须为正')
        z['hover_deg'] = touch + 2*(extra[0]-touch)/extra[2]
    site['z'] = z
    site['motion'] = dict(defaults['motion'], observe_deg=values['observe'], discard_mm=values['discard'])
    site['camera'] = dict(url=values['camera'])
    if 'crop_px' in values:
        site['camera']['crop_px'] = crop_origin(values['crop_px'])
    site['board'] = board
    return site


def old_site(source):
    fields = source.decode('utf-8-sig').split()
    if len(fields) != 31 or fields[0] != 'SCARA1':
        raise ValueError('旧示教表字段不完整')
    nums = list(map(float, fields[1:25]))
    board = {key: nums[i*4:i*4+4] for i, key in enumerate(CORNERS.values())}
    j2, j3, shoulder, elbow, ratio = nums[12:17]
    values = dict(j2=[j2, shoulder, ratio], j3=[j3, elbow, -1/3], z=nums[17:20]+[float(fields[-1])],
                  observe=nums[20:22], discard=nums[22:24], camera=fields[25], crop_px=list(map(int, fields[26:30])))
    return make_site(values, board)


def import_toml(text):
    """把旧安装表的数值转成当前格式，现行表保留原注释与排版。"""
    site = tomllib.loads(text)
    changed = False
    def rename(section, old, new):
        nonlocal changed
        if old in section:
            if new in section:
                raise ValueError(f'安装表重复填写 {old} 和 {new}')
            section[new] = section.pop(old)
            changed = True

    if 'arm' in site:
        if {'j2', 'j3', 'board', 'camera'} & site.keys():
            raise ValueError('安装表混用了新旧分区，请核对原件')
        arm = site.pop('arm')
        keys = ('lengths_mm', 'reference_drive_deg', 'reference_joint_rad', 'joint_per_drive')
        columns = [arm.pop(key) for key in keys]
        if arm or any(not isinstance(c, list) or len(c) != 2 for c in columns):
            raise ValueError('旧 arm 分区需要两根臂长、两组参考角和两个转角比例')
        for name, (length, drive, radians, ratio) in zip(('j2','j3'), zip(*columns)):
            if type(radians) not in (int, float) or not math.isfinite(radians):
                raise ValueError('reference_joint_rad 须为有限的弧度数值')
            site[name] = dict(link_mm=length, drive_deg=drive, joint_deg=math.degrees(radians), ratio=ratio)
        changed = True

    z = site.get('z', {})
    required_carry = 'pickup_deg' in z
    rename(z, 'pickup_deg', 'touch_deg')
    rename(z, 'travel_deg', 'carry_deg')
    if required_carry and 'carry_deg' not in z:
        raise ValueError('旧表缺少实测搬运角，请核对原件')
    motion = site.get('motion', {})
    for old, new in [('park_deg','observe_deg'), ('discard_xy_mm','discard_mm'),
                     ('velocity_deg_s','speed_deg_s'), ('acceleration_deg_s2','accel_deg_s2')]:
        rename(motion, old, new)

    if 'vision' in site:
        vision = site.pop('vision')
        camera = {k: vision.pop(k) for k in ('camera_url','url','roi','crop_px') if k in vision}
        for group, fields in [('camera', camera), ('board', vision)]:
            if not fields:
                continue
            dest = site.setdefault(group, {})
            if dest.keys() & fields.keys():
                raise ValueError(f'{group} 的新旧参数重复，请核对原件')
            dest.update(fields)
        changed = True
    camera = site.get('camera', {})
    rename(camera, 'camera_url', 'url')
    if 'roi' in camera:
        rect = camera.pop('roi')
        if 'crop_px' in camera or not isinstance(rect, list) or len(rect) != 4:
            raise ValueError('旧 ROI 需要唯一一组四个整数')
        camera['crop_px'] = crop_origin(rect)
        changed = True
    board = site.get('board', {})
    if 'points' in board:
        if set(CORNERS.values()) & board.keys():
            raise ValueError('棋盘角点重复填写')
        board.update(zip(CORNERS.values(), board.pop('points'), strict=True))
        changed = True
    # 导入历史文件时丢弃已退役的字段，原件按原有备份流程保存。
    for key in ('offsets_mm', 'target_offsets_mm', 'placement_offsets_mm'):
        if key in board:
            del board[key]
            changed = True
    if not changed:
        return text
    sections = ('j2','j3','z','motion','camera','board','game')
    if site.keys() - set(sections):
        raise ValueError('旧表含未识别的分区，请核对原件')
    return site_text({name: site[name] for name in sections if name in site})


def default_site():
    if INSTALLATION.exists():
        return INSTALLATION
    old = INSTALLATION.with_suffix('.site')
    if old.exists():
        raise ValueError(f'旧示教表请先执行 15-site.cmd "{old}" --install')
    if INSTALLATION.with_suffix('.hpp').exists():
        raise ValueError('现场只有旧 C++ 示教表，请将其实测值填入 installation.toml')
    return ROOT / 'config/installation.toml'


def measurements(sheet):
    values = {}
    lengths = dict(red_left=(4,), red_right=(4,), black_left=(4,), j2=(2,3), j3=(2,3), j2_stops=(2,), j3_stops=(2,),
                   z=(1,2,3,4), hover=(1,), observe=(2,), discard=(2,), crop_px=(2,), roi=(4,), ref=(5,6))
    for line in sheet.read_text(encoding='utf-8-sig').splitlines():
        fields = line.partition('#')[0].split()
        if not fields:
            continue
        key, *raw = fields
        if key == 'camera' and len(raw) == 1 and key not in values:
            values[key] = raw[0]
            continue
        data = list(map(float, raw))
        if not all(map(math.isfinite, data)):
            raise ValueError('示教值须为有限数值')
        if key == 'point' and len(data) == 6:  # 兼容按行、列标记角点的实测表。
            key = CORNERS.get(tuple(data[:2]))
            data = data[2:]
        if key == 'park':
            key = 'observe'
        if key not in lengths or len(data) not in lengths[key]:
            raise ValueError(f'示教表字段或数量错误：{key}')
        if key == 'roi':
            key = 'crop_px'
        if key in values:
            raise ValueError(f'示教表重复填写：{key}')
        values[key] = data
    if 'ref' in values:
        if 'j2' in values or 'j3' in values:
            raise ValueError('参考姿态重复填写')
        j2, j3, shoulder, elbow, ratio, *k3 = values.pop('ref')
        values['j2'], values['j3'] = [j2, shoulder, ratio], [j3, elbow, k3[0] if k3 else -1/3]
    required = {'j2','j3','z','observe','discard','camera', *CORNERS.values()}
    if not required <= values.keys():
        raise ValueError('示教表缺少：' + ', '.join(sorted(required - values.keys())))
    if not values['camera'].startswith('rtsp://') or values['camera'][7:].startswith('_'):
        raise ValueError('请填写现场相机地址')
    return values


def main(argv, binary):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sheet', nargs='?', type=Path)
    parser.add_argument('--template', type=Path)
    parser.add_argument('--install', action='store_true')
    parser.add_argument('--output', type=Path, default=INSTALLATION)
    args = parser.parse_args(argv)
    if args.template:
        with args.template.open('x', encoding='utf8') as output:
            output.write(TEMPLATE)
        return
    if args.install and args.sheet is None:
        for source in (args.output, args.output.with_suffix('.site')):
            if source.is_file():
                args.sheet = source
                break
    if not args.sheet or (args.install and args.output.suffix.lower() != '.toml'):
        parser.error('请提供示教表，安装文件扩展名使用 .toml')
    tool = binary('scara_bench_points')
    with TemporaryDirectory(prefix='scara-site-') as directory:
        tmp = Path(directory)/'installation.toml'
        if args.sheet.suffix.lower() == '.site':
            text = site_text(old_site(args.sheet.read_bytes()))
        elif args.sheet.suffix.lower() == '.toml':
            original = args.sheet.read_bytes()
            source_text = original.decode('utf-8-sig')
            text = import_toml(source_text)
        else:
            values = measurements(args.sheet)
            site = make_site(values, {})
            tmp.write_text(site_text(site), encoding='utf8')
            for corner in CORNERS.values():
                u, v, j2, j3 = values[corner]
                if not (0 <= u < 1280 and 0 <= v < 720):
                    raise ValueError(f'{corner} 像素超出原图')
                xy = subprocess.check_output([str(tool), tmp.name, '--forward', str(j2), str(j3)],
                                             cwd=directory, text=True)
                site['board'][corner] = [u, v, *map(float, xy.split())]
            text = site_text(site)
        # 实测端点是装机记录；下棋路径由现有运动规划器检查。
        site = tomllib.loads(text)
        for axis in ('j2', 'j3'):
            if 'range_deg' in site[axis]:
                ends = site[axis]['range_deg']
                if not (isinstance(ends, list) and len(ends) == 2 and
                        all(type(n) in (int, float) and math.isfinite(n) for n in ends) and ends[0] < ends[1]):
                    raise ValueError(f'{axis}.range_deg 需要从小到大的两个有限机械角')
        tmp.write_bytes(text.encode('utf8'))
        subprocess.run([str(tool), tmp.name], cwd=directory, stdout=subprocess.DEVNULL, check=True)
    if args.install:
        if args.sheet.suffix.lower() == '.toml' and args.sheet.resolve() == args.output.resolve():
            if args.sheet.read_bytes() != original:
                raise ValueError('校验期间原安装表发生变化，请重新导入')
            if text == source_text:
                print(f'本机已是新版安装表 {args.output.resolve()}')
                return
            # 原件和安装目标同名时，先按原始字节留一份可识别的备份。
            backup = args.sheet.with_name(args.sheet.stem + '.' + sha256(original).hexdigest()[:12] + '.bak')
            if not backup.exists():
                with backup.open('xb') as stream:
                    stream.write(original)
            if backup.read_bytes() != original:
                raise ValueError(f'备份内容不符，保留原文件：{backup}')
            print(f'旧表原件 {backup}')
        save(args.output, text)
        print(f'已保存 {args.output.resolve()}')
    else:
        print('已核对 90 个格点及逆解。--install 保存实测配置。')
