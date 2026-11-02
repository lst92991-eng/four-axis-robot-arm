"""V2: grayscale board with 48 mm ArUco markers, A4 tiles and an A1 sheet."""
import json
from pathlib import Path

from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

from generate_board import DICT_SOURCE, MARKER_BYTES, marker_bits

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'output' / 'pdf'
DATA = Path(__file__).resolve().parent / 'v2'
SIZE, GRID_START, PITCH = 510, 80, 25
MARKER_SIZE, QUIET = 48, 8
CENTRES = [(36, 36), (474, 36), (474, 474), (36, 474)]
FONT = 'BoardChinese'


def text(pdf, page_h, x, y, value, size=9, centre=False):
    pdf.setFillGray(0.12)
    pdf.setFont(FONT, size)
    draw = pdf.drawCentredString if centre else pdf.drawString
    draw(x * mm, (page_h - y) * mm, value)


def draw_board(pdf, page_h, ox, oy):
    """Coordinates are millimetres measured from the board's top-left."""
    def rectangle(x, y, width, height, gray):
        pdf.setFillGray(gray)
        pdf.rect((ox+x)*mm, (page_h-oy-y-height)*mm,
                 width*mm, height*mm, fill=1, stroke=0)

    # White surround saves toner. Gray playing area separates white stones from paper.
    rectangle(0, 0, SIZE, SIZE, 1)
    rectangle(65, 65, 380, 380, 0.65)
    pdf.setStrokeGray(0.18)
    pdf.setLineWidth(0.30 * mm)
    for index in range(15):
        p = GRID_START + PITCH * index
        pdf.line((ox+GRID_START)*mm, (page_h-oy-p)*mm,
                 (ox+430)*mm, (page_h-oy-p)*mm)
        pdf.line((ox+p)*mm, (page_h-oy-GRID_START)*mm,
                 (ox+p)*mm, (page_h-oy-430)*mm)
        text(pdf, page_h, ox+p, oy+74, str(index), 7, True)
        text(pdf, page_h, ox+73, oy+p+0.9, str(index), 7, True)
    text(pdf, page_h, ox+255, oy+25, '五子棋 V2 / 15 × 15 / 格距 25 mm', 12, True)

    # Draw white pads AFTER the gray background: keep every quiet zone complete.
    cell = MARKER_SIZE / 6
    for marker_id, (cx, cy) in enumerate(CENTRES):
        left, top = cx-MARKER_SIZE/2, cy-MARKER_SIZE/2
        rectangle(left-QUIET, top-QUIET, MARKER_SIZE+2*QUIET, MARKER_SIZE+2*QUIET, 1)
        rectangle(left, top, MARKER_SIZE, MARKER_SIZE, 0)
        for row, bits in enumerate(marker_bits(marker_id)):
            for col, bit in enumerate(bits):
                if bit:
                    rectangle(left+cell*(col+1), top+cell*(row+1), cell, cell, 1)
        label_y = 74 if cy < SIZE/2 else 438
        label = f'ID {marker_id}' + (' / 左上' if marker_id == 0 else '')
        text(pdf, page_h, ox+cx, oy+label_y, label, 9, True)


def ruler(pdf, page_h, x, y):
    pdf.setStrokeGray(0.1)
    pdf.setLineWidth(0.22 * mm)
    pdf.line(x*mm, (page_h-y)*mm, (x+100)*mm, (page_h-y)*mm)
    for step in range(11):
        tick = 2 if step % 5 == 0 else 1.2
        px = (x+step*10)*mm
        pdf.line(px, (page_h-y)*mm, px, (page_h-y-tick)*mm)
    text(pdf, page_h, x+50, y+6, '打印后测量：此尺应为 100 mm', 7, True)


def crop_marks(pdf, page_h, x, y, width, height):
    pdf.setStrokeGray(0.35)
    pdf.setLineWidth(0.18 * mm)
    for px, py, dx, dy in [(x,y,-1,-1), (x+width,y,1,-1),
                           (x,y+height,-1,1), (x+width,y+height,1,1)]:
        pdf.line((px+dx)*mm, (page_h-py)*mm, (px+dx*3)*mm, (page_h-py)*mm)
        pdf.line(px*mm, (page_h-py-dy)*mm, px*mm, (page_h-py-dy*3)*mm)


def a4_tiles():
    pdf = canvas.Canvas(str(OUT/'gomoku-board-v2-a4-tiles.pdf'),
                        pagesize=(210*mm,297*mm), pageCompression=1)
    pdf.setTitle('五子棋 V2 - 48mm标记 - A4六页拼接 - 100%实际大小')
    for row, y in enumerate([0,250]):
        for col, x in enumerate([0,160,320]):
            n = row*3+col+1
            text(pdf, 297, 10, 9.5, f'五子棋 V2 / 第 {n} 张，共 6 张', 10)
            text(pdf, 297, 10, 16, f'第{row+1}行，第{col+1}列 / 单面 / 100%实际大小', 8)
            ruler(pdf, 297, 95, 12)
            pdf.saveState()
            clip = pdf.beginPath()
            clip.rect(10*mm, (297-22-260)*mm, 190*mm, 260*mm)
            pdf.clipPath(clip, stroke=0)
            draw_board(pdf, 297, 10-x, 22-y)
            pdf.restoreState()
            crop_marks(pdf, 297, 10, 22, 190, 260)
            text(pdf, 297, 10, 290,
                 '排列：1 2 3 / 4 5 6；横向重叠30mm、竖向10mm。只裁拼接白边，对齐网格，背面固定。', 7)
            pdf.showPage()
    pdf.save()


def a1_sheet():
    pdf = canvas.Canvas(str(OUT/'gomoku-board-v2-a1.pdf'),
                        pagesize=(594*mm,841*mm), pageCompression=1)
    pdf.setTitle('五子棋 V2 - 48mm标记 - A1整张 - 100%实际大小')
    text(pdf, 841, 42, 52, '五子棋 V2 / 大号定位标记', 27)
    text(pdf, 841, 42, 74, '黑白打印 | 棋盘外框510mm | 15×15交点 | 格距25mm | 适配18-20mm棋子', 12)
    text(pdf, 841, 42, 94, '每块标记黑色外框48×48mm，四周白色留边8mm；四个ID均保持正向。', 12)
    draw_board(pdf, 841, 42, 125)
    ruler(pdf, 841, 247, 675)
    text(pdf, 841, 42, 717, '请选择“实际大小 / 100%”，关闭“适合页面”和“缩小”。', 14)
    text(pdf, 841, 42, 739, '不要把本页缩到A4；A4打印请使用六页拼接版。', 14)
    text(pdf, 841, 42, 761, '打印后核对：标尺100mm、标记黑框48mm、相邻交点25mm。', 12)
    text(pdf, 841, 42, 805, '保留完整白边，使用哑光纸并贴平。ID 0所在角为棋盘左上角。', 10)
    pdf.save()


def metadata():
    data = {
        'revision': 'V2', 'units': 'mm', 'opencv_version_checked': '5.0.0',
        'board_size_mm': [SIZE,SIZE], 'grid_rows': 15, 'grid_cols': 15,
        'grid_origin_mm': [GRID_START,GRID_START], 'grid_spacing_mm': PITCH,
        'playing_area_mm': [65,65,380,380], 'background_gray': 0.65,
        'grid_line_width_mm': 0.30, 'stone_diameter_mm': [18,20],
        'aruco': {'dictionary': 'DICT_4X4_50', 'border_bits': 1,
                  'black_square_size_mm': MARKER_SIZE, 'quiet_zone_mm': QUIET,
                  'cell_size_mm': 8, 'source': DICT_SOURCE,
                  'markers': [{'id':i, 'center_mm':list(p), 'normal_rotation_bytes':list(MARKER_BYTES[i])}
                              for i,p in enumerate(CENTRES)]},
        'canonical': {'pixels_per_mm':2, 'size_px':[1020,1020],
                      'grid_start_px':160, 'grid_spacing_px':50,
                      'marker_centers_px':[[72,72],[948,72],[948,948],[72,948]],
                      'marker_corner_offsets_px':[[-48,-48],[48,-48],[48,48],[-48,48]]},
        'a4_tiles': {'page_mm':[210,297], 'viewport_mm':[190,260],
                     'page_origin_mm':[10,22], 'layout':[3,2], 'overlap_mm':[30,10],
                     'global_offsets_mm':[[x,y] for y in [0,250] for x in [0,160,320]]},
        'a1': {'page_mm':[594,841], 'board_origin_mm':[42,125]},
        'print_scale_percent': 100,
        'software_note': 'V2 marker geometry differs from V1. Switch calibration when switching printed boards.'
    }
    (DATA/'board.json').write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')


if __name__ == '__main__':
    pdfmetrics.registerFont(TTFont(FONT, 'C:/Windows/Fonts/msyh.ttc', subfontIndex=0))
    OUT.mkdir(parents=True, exist_ok=True)
    DATA.mkdir(parents=True, exist_ok=True)
    a4_tiles()
    a1_sheet()
    metadata()
    print('Created V2 A4 tiles, A1 sheet and matching geometry metadata.')
