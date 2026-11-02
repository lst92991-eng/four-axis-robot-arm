"""Generate a metric Gomoku board; no OpenCV installation is required.

Run with Python containing reportlab and Pillow. Geometry is in millimetres,
with the origin at the top-left of the 410 mm coloured board.
"""
import json
from pathlib import Path
from xml.sax.saxutils import escape

from PIL import Image, ImageDraw, ImageFont
from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
OUT = ROOT / 'output' / 'pdf'
SIZE = 410
SPACING = 25
OFFSET = 30
BACKGROUND = '#C7A76A'
INK = '#403528'
LABEL = '#5A4630'
CENTRES = [(10, 10), (400, 10), (400, 400), (10, 400)]
MARKER_SIZE = 18
QUIET = 3
# OpenCV 5.0.0, DICT_4X4_1000_BYTES: first 4 markers, normal rotation.
# DICT_4X4_50 uses the first 50 records of that array. MSB-first, row-major.
MARKER_BYTES = [(181, 50), (15, 154), (51, 45), (153, 70)]
DICT_SOURCE = ('https://raw.githubusercontent.com/opencv/opencv/5.0.0/'
               'modules/objdetect/src/aruco/predefined_dictionaries.hpp')
FONT_PATH = 'C:/Windows/Fonts/arial.ttf'


def marker_bits(marker_id):
    values = [int(bit) for byte in MARKER_BYTES[marker_id] for bit in f'{byte:08b}']
    return [values[row * 4:row * 4 + 4] for row in range(4)]


def board_scene():
    """A small shared drawing list keeps PDF, SVG and PNG coordinates identical."""
    items = [('rect', 0, 0, SIZE, SIZE, BACKGROUND)]
    for index in range(15):
        p = OFFSET + SPACING * index
        items.extend([
            ('line', OFFSET, p, SIZE - OFFSET, p, INK, 0.25),
            ('line', p, OFFSET, p, SIZE - OFFSET, INK, 0.25),
            ('text', p, 16, str(index), LABEL, 2.5),
            ('text', 16, p + 0.9, str(index), LABEL, 2.5),
        ])
    items.append(('text', SIZE / 2, 7, 'GOMOKU  /  15 x 15  /  25 mm', LABEL, 2.6))
    for marker_id, (cx, cy) in enumerate(CENTRES):
        left, top = cx - 9, cy - 9
        items.append(('rect', left - QUIET, top - QUIET,
                      MARKER_SIZE + 2 * QUIET, MARKER_SIZE + 2 * QUIET, '#FFFFFF'))
        items.append(('rect', left, top, MARKER_SIZE, MARKER_SIZE, '#000000'))
        for row, bits in enumerate(marker_bits(marker_id)):
            for col, bit in enumerate(bits):
                if bit:
                    items.append(('rect', left + 3 * (col + 1), top + 3 * (row + 1),
                                  3, 3, '#FFFFFF'))
        label_y = 25 if cy < SIZE / 2 else 385
        items.append(('text', cx, label_y, '0 START' if marker_id == 0 else str(marker_id),
                      LABEL, 2.2))
    return items


SCENE = board_scene()


def draw_pdf_board(pdf, page_height, origin_x, origin_y):
    for item in SCENE:
        kind, *v = item
        if kind == 'rect':
            x, y, w, h, fill = v
            pdf.setFillColor(colors.HexColor(fill))
            pdf.rect((origin_x + x) * mm, (page_height - origin_y - y - h) * mm,
                     w * mm, h * mm, fill=1, stroke=0)
        elif kind == 'line':
            x1, y1, x2, y2, fill, width = v
            pdf.setStrokeColor(colors.HexColor(fill))
            pdf.setLineWidth(width * mm)
            pdf.line((origin_x + x1) * mm, (page_height - origin_y - y1) * mm,
                     (origin_x + x2) * mm, (page_height - origin_y - y2) * mm)
        else:
            x, y, text, fill, font_mm = v
            pdf.setFillColor(colors.HexColor(fill))
            pdf.setFont('Helvetica', font_mm * mm)
            pdf.drawCentredString((origin_x + x) * mm, (page_height - origin_y - y) * mm, text)


def page_text(pdf, page_height, x, y, text, size=9, bold=False):
    pdf.setFillColor(colors.HexColor(INK))
    pdf.setFont('Helvetica-Bold' if bold else 'Helvetica', size)
    pdf.drawString(x * mm, (page_height - y) * mm, text)


def ruler(pdf, page_height, x, y):
    pdf.setStrokeColor(colors.black)
    pdf.setLineWidth(0.25 * mm)
    pdf.line(x * mm, (page_height - y) * mm, (x + 100) * mm, (page_height - y) * mm)
    for n in range(11):
        tick = 3 if n in (0, 5, 10) else 1.8
        px = (x + 10 * n) * mm
        pdf.line(px, (page_height - y) * mm, px, (page_height - y - tick) * mm)
    page_text(pdf, page_height, x, y + 7, '0', 7)
    page_text(pdf, page_height, x + 90, y + 7, '100 mm', 7)


def crop_marks(pdf, page_height, x, y, width, height):
    pdf.setStrokeColor(colors.HexColor('#777777'))
    pdf.setLineWidth(0.18 * mm)
    for px, py, dx, dy in [(x, y, -1, -1), (x + width, y, 1, -1),
                           (x, y + height, -1, 1), (x + width, y + height, 1, 1)]:
        pdf.line((px + dx * 2) * mm, (page_height - py) * mm,
                 (px + dx * 4) * mm, (page_height - py) * mm)
        pdf.line(px * mm, (page_height - py - dy * 2) * mm,
                 px * mm, (page_height - py - dy * 4) * mm)


def create_a2():
    path = OUT / 'gomoku-board-a2.pdf'
    pdf = canvas.Canvas(str(path), pagesize=(420 * mm, 594 * mm), pageCompression=1)
    pdf.setTitle('Gomoku board - A2 - actual size 100 percent')
    page_text(pdf, 594, 25, 30, 'GOMOKU / PRINT AT ACTUAL SIZE', 20, True)
    page_text(pdf, 594, 25, 43, 'A2 420 x 594 mm | Board 410 x 410 mm | 15 x 15 intersections | 25 mm spacing', 10)
    page_text(pdf, 594, 25, 53, '18-20 mm stones. ArUco DICT_4X4_50, IDs 0-3. ID 0 (START) is the board top-left.', 9)
    draw_pdf_board(pdf, 594, 5, 92)
    ruler(pdf, 594, 155, 530)
    page_text(pdf, 594, 25, 554, 'PRINT: Actual size / 100%. Disable Fit, Shrink, scaling and printer borderless enlargement.', 10, True)
    page_text(pdf, 594, 25, 566, 'Measure the ruler: exactly 100 mm. Keep at least 2 mm of white paper outside the coloured board.', 9)
    page_text(pdf, 594, 25, 578, 'Do not trim the corner-marker white quiet zones. Use matte paper and mount flat.', 9)
    pdf.save()


def create_a4():
    path = OUT / 'gomoku-board-a4-tiles.pdf'
    pdf = canvas.Canvas(str(path), pagesize=(210 * mm, 297 * mm), pageCompression=1)
    pdf.setTitle('Gomoku board - six A4 tiles - actual size 100 percent')
    for row, y0 in enumerate([0, 200]):
        for col, x0 in enumerate([0, 130, 260]):
            number = row * 3 + col + 1
            page_text(pdf, 297, 20, 15, f'GOMOKU / TILE {number} OF 6', 16, True)
            page_text(pdf, 297, 20, 24, 'A4 portrait | Actual size 100% | Do not Fit / Shrink', 9)
            page_text(pdf, 297, 20, 33,
                      f'Row {row + 1}, column {col + 1} | Global origin ({x0}, {y0}) mm | Window 150 x 210 mm', 8)
            # Preserve 2 mm of outer white quiet zone beyond the physical board.
            xmin = -2 if col == 0 else 0
            xmax = 152 if col == 2 else 150
            ymin = -2 if row == 0 else 0
            ymax = 212 if row == 1 else 210
            pdf.saveState()
            clip = pdf.beginPath()
            clip.rect((30 + xmin) * mm, (297 - 42 - ymax) * mm,
                      (xmax - xmin) * mm, (ymax - ymin) * mm)
            pdf.clipPath(clip, stroke=0)
            draw_pdf_board(pdf, 297, 30 - x0, 42 - y0)
            pdf.restoreState()
            crop_marks(pdf, 297, 30, 42, 150, 210)
            page_text(pdf, 297, 20, 261, 'Layout: 1 2 3 / 4 5 6. Overlap: 20 mm sideways; 10 mm vertically.', 8)
            ruler(pdf, 297, 55, 268)
            page_text(pdf, 297, 20, 283, 'Trim joining margins only; align duplicate grid lines. Keep the printed overlap.', 7.5)
            page_text(pdf, 297, 20, 290, 'Keep 2 mm outer white border. Check ruler = 100 mm before joining. Tape on the back.', 7.5)
            pdf.showPage()
    pdf.save()


def create_svg():
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="410mm" height="410mm" viewBox="0 0 410 410">',
           '<title>Gomoku board: 15 x 15, 25 mm spacing</title>',
           '<desc>Outer marker quiet zones extend to -2 and 412 mm in the printable PDFs. This board-only SVG clips them.</desc>']
    for item in SCENE:
        kind, *v = item
        if kind == 'rect':
            x, y, w, h, fill = v
            out.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}"/>')
        elif kind == 'line':
            x1, y1, x2, y2, fill, width = v
            out.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{fill}" stroke-width="{width}"/>')
        else:
            x, y, text, fill, font_mm = v
            out.append(f'<text x="{x}" y="{y}" text-anchor="middle" font-family="Arial, sans-serif" font-size="{font_mm}" fill="{fill}">{escape(text)}</text>')
    out.append('</svg>')
    (HERE / 'board.svg').write_text('\n'.join(out), encoding='utf-8')


def create_png():
    scale = 8  # Supersample text and 0.25 mm lines, then output exactly 2 px/mm.
    image = Image.new('RGB', (SIZE * scale, SIZE * scale), BACKGROUND)
    draw = ImageDraw.Draw(image)
    for item in SCENE:
        kind, *v = item
        if kind == 'rect':
            x, y, w, h, fill = v
            draw.rectangle((round(x * scale), round(y * scale),
                            round((x + w) * scale) - 1, round((y + h) * scale) - 1), fill=fill)
        elif kind == 'line':
            x1, y1, x2, y2, fill, width = v
            draw.line((x1 * scale, y1 * scale, x2 * scale, y2 * scale), fill=fill,
                      width=round(width * scale))
        else:
            x, y, text, fill, font_mm = v
            font = ImageFont.truetype(FONT_PATH, round(font_mm * scale))
            draw.text((x * scale, y * scale), text, font=font, fill=fill, anchor='ms')
    image.resize((820, 820), Image.Resampling.LANCZOS).save(HERE / 'board.png', dpi=(50.8, 50.8))


def create_metadata():
    data = {
        'schema_version': 1,
        'units': 'mm',
        'coordinate_system': {'origin': 'top-left of coloured board', 'x': 'right', 'y': 'down',
                              'indexing': 'zero-based row and column'},
        'board_width_mm': SIZE, 'board_height_mm': SIZE,
        'grid_rows': 15, 'grid_cols': 15, 'grid_origin_mm': [OFFSET, OFFSET],
        'grid_spacing_mm': SPACING, 'grid_span_mm': 350, 'grid_line_width_mm': 0.25,
        'grid_point_formula': 'x = 30 + 25 * col; y = 30 + 25 * row',
        'recommended_stone_diameter_mm': [18, 20], 'background_rgb_hex': BACKGROUND,
        'star_points': False,
        'aruco': {'dictionary': 'DICT_4X4_50', 'opencv_version_checked': '5.0.0',
                  'dictionary_source_url': DICT_SOURCE, 'border_bits': 1,
                  'black_square_size_mm': MARKER_SIZE, 'cell_size_mm': 3,
                  'quiet_zone_mm': QUIET, 'all_markers_rotation_degrees': 0,
                  'markers': [{'id': i, 'center_mm': list(p), 'normal_rotation_bytes': list(MARKER_BYTES[i]),
                               'inner_bits_row_major': marker_bits(i)} for i, p in enumerate(CENTRES)]},
        'a2': {'page_mm': [420, 594], 'board_origin_on_page_mm': [5, 92], 'scale': 1.0},
        'a4_tiles': {'page_mm': [210, 297], 'layout': [3, 2], 'viewport_mm': [150, 210],
                     'viewport_origin_on_page_mm': [30, 42], 'overlap_mm': [20, 10], 'scale': 1.0,
                     'global_offsets_mm': [[x, y] for y in [0, 200] for x in [0, 130, 260]],
                     'outer_quiet_zone_extension_mm': 2},
        'canonical_png': {'size_px': [820, 820], 'pixels_per_mm': 2,
                          'note': 'Board-only crop; outer quiet-zone extension is present in PDFs only.'},
        'print': {'scale_percent': 100, 'calibration_ruler_mm': 100,
                  'outer_white_paper_border_min_mm': 2,
                  'instruction': 'Disable fit/shrink/borderless enlargement; check ruler before assembly.'},
    }
    (HERE / 'board.json').write_text(json.dumps(data, indent=2), encoding='utf-8')


if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    create_a2()
    create_a4()
    create_svg()
    create_png()
    create_metadata()
    print('Created A2 PDF, six-page A4 PDF, board.png, board.svg and board.json.')
