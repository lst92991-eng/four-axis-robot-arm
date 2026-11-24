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
CELLS = ((9, 0), (9, 8), (0, 0))
TEMPLATE = """# 黑方底线行0，红方底线行9。数字用实测值替换。
# 每点依次为 行 列 原图u 原图v 对中时J2反馈角 J3电机角
point 9 0 _ _ _ _
point 9 8 _ _ _ _
point 0 0 _ _ _ _
# J2/J3驱动参考角、肩肘机械参考角（度）、J2/J3有符号换算系数
ref _ _ _ _ _ _
# J1吸取角、搬运角、释放角（度）。释放角可省略，默认用吸取角。
z _ _ _
# 可选。运行20号空吸头定位时，填实测的下探角。
# hover _
park _ _
discard _ _
camera rtsp://_/live/0
roi _ _ _ _
"""


def save(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with NamedTemporaryFile('w', encoding='utf8', dir=path.parent, delete=False) as stage:
        stage.write(text)
    Path(stage.name).replace(path)


def site_text(arm, heights, motion, vision):
    # 只写固定分区和数值。引号和数组由 JSON 编码生成兼容 TOML 的基本字面量。
    parts = []
    for name, note, values in (
        ('arm', '臂长为毫米，驱动参考为度，机械参考为弧度。换算系数带方向。', arm),
        ('z', 'J1 直接示教角，单位为度。释放默认同取子，净空默认从搬运层起。', heights),
        ('motion', '观察位为 J2/J3 驱动角，弃子点为基座毫米。限幅按 J1/J2/J3 排列。', motion),
        ('vision', '红左、红右、黑左三点。每点是原图像素 u/v 和基座 X/Y 毫米。', vision),
        ('game', '默认红方先走，玩家执红。可选 think_ms、human_red、red_to_move。', {}),
    ):
        lines = [f'# {note}', f'[{name}]']
        for key, value in values.items():
            lines.append(f'{key} = '+json.dumps(value, ensure_ascii=False, allow_nan=False))
        parts.append('\n'.join(lines))
    return '\n\n'.join(parts)+'\n'


def make_site(ref, z, park, discard, camera, roi, points, hover=None):
    j2, j3, shoulder, elbow, k2, *k3 = ref
    low, high, *release = z
    defaults = tomllib.loads((ROOT/'config/installation.toml').read_text(encoding='utf8'))
    arm = dict(lengths_mm=defaults['arm']['lengths_mm'], reference_drive_deg=[j2, j3],
               reference_joint_rad=[math.radians(shoulder), math.radians(elbow)],
               joint_per_drive=[k2, k3[0] if k3 else -1/3])
    heights = dict(pickup_deg=low, travel_deg=high)
    if release and release[0] != low:
        heights['release_deg'] = release[0]
    # 老示教表的最后一列仅在导入时换算一次，产出的 TOML 只保存角度。
    if hover is None and len(release) == 2:
        if release[1] <= 0:
            raise ValueError('旧示教行程须为正')
        hover = low + 2*(high-low)/release[1]
    if hover is not None:
        heights['hover_deg'] = hover
    motion = dict(defaults['motion'], park_deg=park, discard_xy_mm=discard)
    vision = dict(camera_url=camera, roi=roi, points=points)
    return arm, heights, motion, vision


def old_site(source):
    fields = source.decode('utf-8-sig').split()
    if len(fields) != 31 or fields[0] != 'SCARA1':
        raise ValueError('旧示教表字段不完整')
    nums = list(map(float, fields[1:25]))
    points = [nums[n:n+4] for n in range(0, 12, 4)]
    return make_site(nums[12:17], nums[17:20]+[float(fields[-1])], nums[20:22], nums[22:24],
                     fields[25], list(map(int, fields[26:30])), points)


def default_site():
    if INSTALLATION.exists():
        return INSTALLATION
    old = INSTALLATION.with_suffix('.site')
    if old.exists():
        return old
    if INSTALLATION.with_suffix('.hpp').exists():
        raise ValueError('现场只有旧 C++ 示教表，请将其实测值填入 installation.toml')
    return ROOT / 'config/installation.toml'


def field_config(site):
    if site.suffix.lower() != '.site':
        return site
    source = site.read_bytes()
    dest = ROOT/'output/bench/sites'/sha256(source).hexdigest()[:16]/'installation.toml'
    text = site_text(*old_site(source))
    if not dest.exists() or dest.read_text(encoding='utf8') != text:
        save(dest, text)
    return dest


def measurements(sheet):
    values, points = {}, {}
    lengths = {'ref': (5, 6), 'z': (2, 3, 4), 'hover': (1,), 'park': (2,), 'discard': (2,), 'roi': (4,)}
    for line in sheet.read_text(encoding='utf-8-sig').splitlines():
        fields = line.partition('#')[0].split()
        if not fields:
            continue
        key, raw = fields[0], fields[1:]
        if key == 'camera' and len(raw) == 1 and key not in values:
            values[key] = raw[0]
            continue
        data = list(map(float, raw))
        if not all(map(math.isfinite, data)):
            raise ValueError('示教值须为有限数值')
        if key == 'point' and len(data) == 6 and tuple(data[:2]) in CELLS and tuple(data[:2]) not in points:
            points[tuple(data[:2])] = data[2:]
        elif key not in values and len(data) in lengths.get(key, ()):
            values[key] = data
        else:
            raise ValueError(f'示教表字段或数量错误 {key}')
    required = (set(lengths)-{'hover'}) | {'camera'}
    if len(points) != 3 or not required <= values.keys():
        raise ValueError('请填完整三点、机械参考、工作高度、观察位和相机资料')
    if not values['camera'].startswith('rtsp://') or values['camera'][7:].startswith('_'):
        raise ValueError('请填写现场相机地址')
    if any(int(value) != value for value in values['roi']):
        raise ValueError('裁剪区域使用整数像素')
    return values, points


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
    if not args.sheet or (args.install and args.output.suffix.lower() != '.toml'):
        parser.error('请提供示教表，安装文件扩展名使用 .toml')
    tool = binary('scara_bench_points')
    with TemporaryDirectory(prefix='scara-site-') as directory:
        tmp = Path(directory)/'installation.toml'
        if args.sheet.suffix.lower() == '.site':
            text = site_text(*old_site(args.sheet.read_bytes()))
        elif args.sheet.suffix.lower() == '.toml':
            text = args.sheet.read_text(encoding='utf-8-sig')
        else:
            values, measured = measurements(args.sheet)
            arm, z, motion, vision = make_site(
                values['ref'], values['z'], values['park'], values['discard'], values['camera'],
                list(map(int, values['roi'])), [], values.get('hover', [None])[0])
            tmp.write_text(site_text(arm, z, motion, vision), encoding='utf8')
            for cell in CELLS:
                u, v, j2, j3 = measured[cell]
                if not (0 <= u < 1280 and 0 <= v < 720):
                    raise ValueError(f'{cell} 像素超出原图')
                xy = subprocess.check_output([str(tool), tmp.name, '--forward', str(j2), str(j3)],
                                             cwd=directory, text=True)
                vision['points'].append([u, v, *map(float, xy.split())])
            text = site_text(arm, z, motion, vision)
        tmp.write_text(text, encoding='utf8')
        subprocess.run([str(tool), tmp.name], cwd=directory, stdout=subprocess.DEVNULL, check=True)
    if args.install:
        save(args.output, text)
        print(f'已保存 {args.output.resolve()}')
    else:
        print('已核对 90 个格点及逆解。--install 保存实测配置。')
