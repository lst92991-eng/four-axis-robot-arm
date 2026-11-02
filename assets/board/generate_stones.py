"""Create three A4 sheets of 20 mm Gomoku stones at actual size.

Requires reportlab. Only the stone PDF is generated; board assets are untouched.
White stones have a 0.15 mm outline whose OUTER diameter is exactly 20 mm.
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / 'output' / 'pdf' / 'gomoku-stones-a4.pdf'
PAGE_W, PAGE_H = 210, 297
DIAMETER = 20
PITCH = 24
FIRST_X = (PAGE_W - 7 * PITCH) / 2
FIRST_Y = 42
OUTLINE = 0.15
PAGE_COUNTS = [(40, 40), (40, 40), (33, 32)]
FONT_CANDIDATES = [
    'C:/Windows/Fonts/msyh.ttc',
    'C:/Windows/Fonts/simsun.ttc',
    'C:/Windows/Fonts/simhei.ttf',
]


def register_font():
    required = '五子棋棋子剪裁版本页黑颗白全套直径实际大小打印关闭适合页面缩放校验尺沿外轮廓剪下可贴硬卡纸勿覆亮膜'
    for path in FONT_CANDIDATES:
        if not Path(path).exists():
            continue
        font = TTFont('StoneChinese', path, subfontIndex=0)
        if all(ord(character) in font.face.charToGlyph for character in required):
            pdfmetrics.registerFont(font)
            return path
    raise RuntimeError('No installed TrueType font covers the required Chinese text.')


def text(pdf, x, y, value, size=9, align='left'):
    pdf.setFillColor(colors.HexColor('#303030'))
    pdf.setFont('StoneChinese', size)
    draw = {'left': pdf.drawString, 'center': pdf.drawCentredString,
            'right': pdf.drawRightString}[align]
    draw(x * mm, (PAGE_H - y) * mm, value)


def stone_centres(black_count, white_count):
    """Keep black on the left and white on the right of every occupied row."""
    remaining = {'black': black_count, 'white': white_count}
    result = []
    for row in range(10):
        for col in range(8):
            colour = 'black' if col < 4 else 'white'
            if remaining[colour]:
                result.append((FIRST_X + col * PITCH, FIRST_Y + row * PITCH, colour))
                remaining[colour] -= 1
    assert not any(remaining.values())
    return result


def draw_stone(pdf, x, y, colour):
    cx, cy = x * mm, (PAGE_H - y) * mm
    if colour == 'black':
        pdf.setFillColor(colors.black)
        pdf.circle(cx, cy, DIAMETER / 2 * mm, fill=1, stroke=0)
    else:
        pdf.setFillColor(colors.white)
        pdf.setStrokeColor(colors.HexColor('#555555'))
        pdf.setLineWidth(OUTLINE * mm)
        radius = (DIAMETER - OUTLINE) / 2 * mm
        pdf.circle(cx, cy, radius, fill=1, stroke=1)


def draw_ruler(pdf):
    y = (PAGE_H - 276) * mm
    pdf.setStrokeColor(colors.HexColor('#303030'))
    pdf.setLineWidth(0.2 * mm)
    pdf.line(55 * mm, y, 155 * mm, y)
    for n in range(11):
        x = (55 + n * 10) * mm
        depth = 2 if n in (0, 5, 10) else 1.3
        pdf.line(x, y, x, y - depth * mm)
    text(pdf, 11, 277, '校验尺', 8)
    text(pdf, 55, 282, '0', 7)
    text(pdf, 155, 282, '100 mm', 7, 'right')


def main():
    register_font()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    pdf = canvas.Canvas(str(OUTPUT), pagesize=(PAGE_W * mm, PAGE_H * mm), pageCompression=1)
    pdf.setTitle('五子棋剪裁棋子 - 20 mm - A4实际尺寸')
    pdf.setSubject('113 black stones and 112 white stones; print at 100 percent.')
    for page, (black_count, white_count) in enumerate(PAGE_COUNTS, start=1):
        text(pdf, 11, 13, '五子棋棋子 · 剪裁版', 16)
        text(pdf, 199, 13, f'{page} / 3', 10, 'right')
        text(pdf, 11, 21, f'本页：黑 {black_count} 颗 / 白 {white_count} 颗     全套：黑 113 颗 / 白 112 颗', 9)
        text(pdf, 11, 28, '外径 20 mm | 实际大小 100% 打印，关闭适合页面 / 缩放', 9)
        for x, y, colour in stone_centres(black_count, white_count):
            draw_stone(pdf, x, y, colour)
        draw_ruler(pdf)
        text(pdf, 105, 290, '沿外轮廓剪下；可贴硬卡纸；勿覆亮膜。', 9, 'center')
        pdf.showPage()
    pdf.save()
    print(f'Created: {OUTPUT}')


if __name__ == '__main__':
    main()
