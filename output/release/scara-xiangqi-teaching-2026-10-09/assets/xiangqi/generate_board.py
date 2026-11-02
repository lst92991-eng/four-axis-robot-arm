"""Read project dimensions and create grayscale Xiangqi calibration-board PDFs."""
import hashlib
import json
import math
from pathlib import Path
import re
import sys

from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
OUT = ROOT / 'output' / 'pdf'
CONFIG = ROOT / 'app' / 'config.hpp'
CONFIG_TEXT = CONFIG.read_text(encoding='utf-8')


def dimension(name):
    match = re.search(r'inline constexpr double\s+' + name + r'\s*=\s*([0-9.]+)\s*;', CONFIG_TEXT)
    if not match:
        raise ValueError(f'Missing literal dimension in app/config.hpp: {name}')
    return float(match.group(1))


VALUES = {name: dimension(name) for name in (
    'PieceDiameter', 'Pitch', 'BoardWidth', 'BoardHeight', 'GridStart', 'MarkerSide', 'MarkerInset')}
PIECE, PITCH = VALUES['PieceDiameter'], VALUES['Pitch']
WIDTH, HEIGHT, START = VALUES['BoardWidth'], VALUES['BoardHeight'], VALUES['GridStart']
MARKER, INSET = VALUES['MarkerSide'], VALUES['MarkerInset']
ROWS, COLS = 10, 9
END_X, END_Y = START + (COLS - 1) * PITCH, START + (ROWS - 1) * PITCH
QUIET = MARKER / 6  # 一格白边；48mm码的6格结构对应8mm。
CENTRES = [(INSET, INSET), (WIDTH-INSET, INSET),
           (WIDTH-INSET, HEIGHT-INSET), (INSET, HEIGHT-INSET)]
TILE_W, TILE_H = 180.0, 270.0
TILE_X, TILE_Y = [0.0, 150.0, 300.0], [0.0, 250.0]
PAGE_ORIGIN = (15.0, 13.5)
FONT = 'XiangqiChinese'
# OpenCV5 DICT_4X4_50的前4个编码，沿用已核验的旧生成器marker_bits。
MARKER_BYTES = [(181, 50), (15, 154), (51, 45), (153, 70)]
DICT_SOURCE = 'https://raw.githubusercontent.com/opencv/opencv/5.0.0/modules/objdetect/src/aruco/predefined_dictionaries.hpp'
PRODUCT_SOURCE = 'https://www.deliworld.com/product/educational-toy/'


def marker_bits(marker_id):
    bits = [int(bit) for byte in MARKER_BYTES[marker_id] for bit in f'{byte:08b}']
    return [bits[row * 4:row * 4 + 4] for row in range(4)]


def text(pdf, page_h, x, y, value, size=9, centered=False, gray=.12):
    pdf.setFillGray(gray)
    pdf.setFont(FONT, size)
    draw = pdf.drawCentredString if centered else pdf.drawString
    draw(x * mm, (page_h-y) * mm, value)


def draw_board(pdf, page_h, ox, oy):
    def line(x1, y1, x2, y2):
        pdf.line((ox+x1)*mm, (page_h-oy-y1)*mm, (ox+x2)*mm, (page_h-oy-y2)*mm)

    def rect(x, y, width, height, gray):
        pdf.setFillGray(gray)
        pdf.rect((ox+x)*mm, (page_h-oy-y-height)*mm, width*mm, height*mm, fill=1, stroke=0)

    rect(0, 0, WIDTH, HEIGHT, 1)
    pad = PITCH / 2
    rect(START-pad, START-pad, END_X-START+2*pad, END_Y-START+2*pad, .94)
    pdf.setStrokeGray(.15)
    pdf.setLineWidth(.35*mm)
    for row in range(ROWS):
        y = START + row*PITCH
        line(START, y, END_X, y)
        text(pdf, page_h, ox+START-24, oy+y+1.2, str(9-row), 10, True)
        text(pdf, page_h, ox+END_X+24, oy+y+1.2, str(9-row), 10, True)
    for col in range(COLS):
        x = START + col*PITCH
        if col in (0, 8):
            line(x, START, x, END_Y)
        else:
            line(x, START, x, START+4*PITCH)
            line(x, START+5*PITCH, x, END_Y)
        text(pdf, page_h, ox+x, oy+START-19, chr(97+col), 10, True)
        text(pdf, page_h, ox+x, oy+END_Y+23, chr(97+col), 10, True)
    # 上下九宫，以及炮位和兵卒位的传统角标。
    for row in (0, 7):
        top = START + row*PITCH
        line(START+3*PITCH, top, START+5*PITCH, top+2*PITCH)
        line(START+5*PITCH, top, START+3*PITCH, top+2*PITCH)
    points = [(row, col) for row in (2, 7) for col in (1, 7)]
    points += [(row, col) for row in (3, 6) for col in (0, 2, 4, 6, 8)]
    for row, col in points:
        x, y = START+col*PITCH, START+row*PITCH
        for dx in (-1, 1):
            if (col == 0 and dx < 0) or (col == 8 and dx > 0):
                continue
            for dy in (-1, 1):
                line(x+dx*2.5, y+dy*7.5, x+dx*2.5, y+dy*2.5)
                line(x+dx*2.5, y+dy*2.5, x+dx*7.5, y+dy*2.5)
    river_y = START + 4.5*PITCH + 7
    text(pdf, page_h, ox+START+2*PITCH, oy+river_y, '楚 河', 42, True, .25)
    text(pdf, page_h, ox+START+6*PITCH, oy+river_y, '漢 界', 42, True, .25)
    text(pdf, page_h, ox+WIDTH/2, oy+25, '中国象棋 · 视觉标定板', 16, True)
    text(pdf, page_h, ox+WIDTH/2, oy+44, '黑方在上 / row 0 / rank 9', 10, True)
    text(pdf, page_h, ox+WIDTH/2, oy+HEIGHT-38, '红方在下 / row 9 / rank 0', 10, True)
    text(pdf, page_h, ox+WIDTH/2, oy+HEIGHT-15,
         f'格距 {PITCH:g} mm · 实物棋子 {PIECE:g} mm · 机器人布局尚未标定', 9, True)
    # 白底最后绘制，保证完整白边，不让灰底、边框或辅助文字进入quiet zone。
    cell = MARKER / 6
    for marker_id, (cx, cy) in enumerate(CENTRES):
        left, top = cx-MARKER/2, cy-MARKER/2
        rect(left-QUIET, top-QUIET, MARKER+2*QUIET, MARKER+2*QUIET, 1)
        rect(left, top, MARKER, MARKER, 0)
        for row, bits in enumerate(marker_bits(marker_id)):
            for col, bit in enumerate(bits):
                if bit:
                    rect(left+cell*(col+1), top+cell*(row+1), cell, cell, 1)
        label_y = INSET + MARKER/2 + QUIET + 7 if cy < HEIGHT/2 else cy-MARKER/2-QUIET-5
        text(pdf, page_h, ox+cx, oy+label_y, f'ID {marker_id}' + (' / 左上' if marker_id == 0 else ''), 9, True)


def horizontal_ruler(pdf, page_h, x, y):
    pdf.setStrokeGray(0)
    pdf.setLineWidth(.25*mm)
    pdf.line(x*mm, (page_h-y)*mm, (x+100)*mm, (page_h-y)*mm)
    for n in range(11):
        depth = 3 if n in (0, 5, 10) else 1.5
        pdf.line((x+n*10)*mm, (page_h-y)*mm, (x+n*10)*mm, (page_h-y-depth)*mm)
    text(pdf, page_h, x+50, y+10, '此段实测应为 100 mm', 10, True)


def vertical_ruler(pdf):
    # 窄左边距内放100mm竖尺，所有墨迹仍距纸边至少10mm。
    pdf.setStrokeGray(.15)
    pdf.setLineWidth(.20*mm)
    pdf.line(10.5*mm, 197*mm, 10.5*mm, 97*mm)
    for n in range(11):
        length = 2 if n in (0, 5, 10) else 1
        pdf.line(10.5*mm, (197-n*10)*mm, (10.5+length)*mm, (197-n*10)*mm)
    pdf.saveState()
    pdf.translate(14.5*mm, 147*mm)
    pdf.rotate(90)
    pdf.setFillGray(.1)
    pdf.setFont(FONT, 5.5)
    pdf.drawCentredString(0, 0, '100 mm / 请用直尺核对 / 100%')
    pdf.restoreState()


def create_a4():
    path = OUT / 'xiangqi-board-a4-tiles.pdf'
    pdf = canvas.Canvas(str(path), pagesize=(210*mm, 297*mm), pageCompression=1)
    pdf.setTitle('中国象棋标定板 - 六页A4 - 单面100%打印')
    for row, y0 in enumerate(TILE_Y):
        for col, x0 in enumerate(TILE_X):
            number = row*3+col+1
            text(pdf, 297, 15, 12.8, f'中国象棋  {number}/6  ·  单面打印  ·  实际大小100%  ·  禁止适合页面', 7)
            pdf.saveState()
            clip = pdf.beginPath()
            clip.rect(PAGE_ORIGIN[0]*mm, (297-PAGE_ORIGIN[1]-TILE_H)*mm, TILE_W*mm, TILE_H*mm)
            pdf.clipPath(clip, stroke=0, fill=0)
            draw_board(pdf, 297, PAGE_ORIGIN[0]-x0, PAGE_ORIGIN[1]-y0)
            pdf.restoreState()
            vertical_ruler(pdf)
            text(pdf, 297, 15, 286.2, '排列 1 2 3 / 4 5 6 · 重叠30/20mm · 裁接缝白边，对齐重叠图案；外白边保留，胶带贴背面', 6.5)
            pdf.showPage()
    pdf.save()


def create_a1():
    path = OUT / 'xiangqi-board-a1.pdf'
    pdf = canvas.Canvas(str(path), pagesize=(594*mm, 841*mm), pageCompression=1)
    pdf.setTitle('中国象棋标定板 - A1整张 - 单面100%打印')
    text(pdf, 841, 57, 40, '中国象棋 · 视觉标定棋盘', 30)
    text(pdf, 841, 57, 55, f'外框 {WIDTH:g}×{HEIGHT:g} mm / 9列×10行 / 格距 {PITCH:g} mm / 实物棋子 {PIECE:g} mm', 13)
    text(pdf, 841, 57, 70, '单面打印 · 实际大小100% · 关闭适合页面、缩放和无边距扩印', 13)
    draw_board(pdf, 841, (594-WIDTH)/2, 110)
    horizontal_ruler(pdf, 841, 247, 665)
    notes = ['使用哑光纸并平整固定；不要裁入四角标记白边，四角必须完整入镜。',
             '真实棋子使用红黑字区分双方；打印棋盘本身只用灰度，不依赖彩色打印。',
             '打印几何读取 app/config.hpp；robot-placement 只是未标定模拟布局。',
             '35mm参考得力9566商品规格；40mm格距由本项目选择，并非唯一市场标准。',
             '商品规格来源：www.deliworld.com/product/educational-toy/']
    for i, value in enumerate(notes):
        text(pdf, 841, 57, 710+i*10, value, 11)
    text(pdf, 841, 57, 800, 'DICT_4X4_50 / ID 0,1,2,3 / 48mm黑块 / 8mm完整白边 / 所有坐标单位mm', 10)
    pdf.save()


def create_metadata():
    half = MARKER/2+QUIET
    pages = []
    for row, y0 in enumerate(TILE_Y):
        for col, x0 in enumerate(TILE_X):
            ids = [i for i, (x, y) in enumerate(CENTRES)
                   if x-half >= x0 and x+half <= x0+TILE_W and y-half >= y0 and y+half <= y0+TILE_H]
            pages.append({'page': row*3+col+1, 'global_origin_mm': [x0, y0], 'complete_marker_ids': ids})
    assert sorted(i for page in pages for i in page['complete_marker_ids']) == [0, 1, 2, 3]
    assert TILE_X[-1]+TILE_W == WIDTH and TILE_Y[-1]+TILE_H == HEIGHT
    assert START > INSET+half and END_X < WIDTH-INSET-half and END_Y < HEIGHT-INSET-half
    formula = re.search(r'inline scara::Pose squarePose[\s\S]*?return\s*\{([^}]+)\}', CONFIG_TEXT).group(1).strip()
    metadata = {
        'source': 'app/config.hpp', 'source_sha256': hashlib.sha256(CONFIG.read_bytes()).hexdigest(),
        'project_dimensions_mm': VALUES, 'board_size_mm': [WIDTH, HEIGHT],
        'grid': {'rows': ROWS, 'cols': COLS, 'origin_mm': [START, START], 'spacing_mm': PITCH,
                 'last_point_mm': [END_X, END_Y], 'line_width_mm': .35, 'palaces': [[0,2,3,5],[7,9,3,5]],
                 'river_between_rows': [4,5], 'pawn_cannon_corner_marks': True,
                 'orientation': 'row0 black top, row9 red bottom, col0 left from red viewpoint',
                 'uci': 'file=a+col; rank=9-row'},
        'aruco': {'dictionary': 'DICT_4X4_50', 'dictionary_source': DICT_SOURCE,
                  'black_square_mm': MARKER, 'quiet_zone_mm': QUIET, 'cell_mm': MARKER/6,
                  'markers': [{'id': i, 'center_mm': point, 'upright': True, 'inner_bits': marker_bits(i)} for i, point in enumerate(CENTRES)]},
        'a4': {'page_mm': [210,297], 'viewport_mm': [TILE_W,TILE_H], 'page_origin_mm': PAGE_ORIGIN,
               'overlap_mm': [TILE_W-(TILE_X[1]-TILE_X[0]), TILE_H-(TILE_Y[1]-TILE_Y[0])],
               'ruler_mm': 100, 'ruler_orientation': 'vertical', 'pages': pages},
        'a1': {'page_mm': [594,841], 'board_origin_mm': [(594-WIDTH)/2,110], 'ruler_mm': 100},
        'print': {'scale_percent': 100, 'single_sided': True, 'grayscale': True,
                  'instructions': '关闭适合页面/缩放；先量100mm标尺；拼接重叠30/20mm；保留外侧纸边和全部白边。'},
        'robot-placement': {'calibrated': False, 'status': 'uncalibrated simulation layout only',
                            'squarePose_source_expression': formula,
                            'note': 'No running calibration, mechanical zero, limits, or board height was changed.'},
        'piece_reference': {'model': 'Deli 9566', 'catalog_size_mm': 35, 'source_url': PRODUCT_SOURCE,
                            'note': 'One product example, not a unique market standard.'},
        'corner_stone_to_marker_quiet_mm': math.hypot(START-(INSET+half),START-(INSET+half))-PIECE/2,
    }
    (HERE / 'board.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    if len(sys.argv) > 2:
        raise SystemExit('Usage: generate_board.py [Chinese TrueType .ttf/.ttc path]')
    font_path = Path(sys.argv[1]) if len(sys.argv) == 2 else Path('C:/Windows/Fonts/msyh.ttc')
    pdfmetrics.registerFont(TTFont(FONT, str(font_path), subfontIndex=0))
    OUT.mkdir(parents=True, exist_ok=True)
    create_metadata()
    create_a4()
    create_a1()
    print('Created xiangqi-board-a4-tiles.pdf and xiangqi-board-a1.pdf from app/config.hpp.')
