from pathlib import Path
import json
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from PIL import Image,ImageDraw
ROOT=Path(__file__).resolve().parents[2]
out=ROOT/'hardware/xiangqi30-old195275-2026-10-12/markers'
out.mkdir(parents=True,exist_ok=True)
data=json.loads((Path(__file__).parent/'markers.json').read_text('utf-8'))
assert data['opencv']=='5.0.0'
pdfmetrics.registerFont(TTFont('STSong-Light','C:/Windows/Fonts/simhei.ttf'))
def tile(c,id,x,y):
    c.setFillColorRGB(1,1,1);c.rect(x*mm,y*mm,60*mm,60*mm,fill=1,stroke=0)
    c.setFillColorRGB(0,0,0)
    for r,row in enumerate(data['markers'][id]):
        for col,white in enumerate(row):
            if not white:c.rect((x+6+8*col)*mm,(y+6+8*(5-r))*mm,8*mm,8*mm,fill=1,stroke=0)
    c.setStrokeColorRGB(.7,.7,.7);c.setLineWidth(.15*mm)
    for dx in [0,60]:
        for dy in [0,60]:
            signx=-1 if dx==0 else 1; signy=-1 if dy==0 else 1
            c.line((x+dx+signx)*mm,(y+dy)*mm,(x+dx+signx*3)*mm,(y+dy)*mm)
            c.line((x+dx)*mm,(y+dy+signy)*mm,(x+dx)*mm,(y+dy+signy*3)*mm)
    c.setFont('Helvetica',10);c.drawCentredString((x+30)*mm,(y-8)*mm,f'DICT_4X4_50 / ID {id}')
for id in range(4):
    c=canvas.Canvas(str(out/f'aruco_id_{id}_60mm.pdf'),pagesize=(80*mm,100*mm))
    c.setTitle(f'ArUco ID {id}: 48 mm code + 6 mm white margin')
    tile(c,id,10,26)
    c.setFont('Helvetica',8);c.drawCentredString(40*mm,8*mm,'Print 100% / Actual size')
    c.save()
    im=Image.new('RGB',(1200,1200),'white');d=ImageDraw.Draw(im)
    svg=['<svg xmlns="http://www.w3.org/2000/svg" width="60mm" height="60mm" viewBox="0 0 60 60">','<rect width="60" height="60" fill="white"/>']
    for r,row in enumerate(data['markers'][id]):
        for col,white in enumerate(row):
            if not white:
                x,y=120+160*col,120+160*r;d.rectangle((x,y,x+159,y+159),fill='black')
                svg.append(f'<rect x="{6+8*col}" y="{6+8*r}" width="8" height="8"/>')
    im.save(out/f'aruco_id_{id}_60mm.png');svg.append('</svg>')
    (out/f'aruco_id_{id}_60mm.svg').write_text(''.join(svg),'utf-8')
c=canvas.Canvas(str(out/'aruco_4_ids_A4_100percent.pdf'),pagesize=(210*mm,297*mm))
c.setTitle('SCARA chess ArUco markers - OpenCV 5.0.0')
c.setFont('STSong-Light',17);c.drawString(24*mm,273*mm,'中国象棋定位标记 / 4 个独立 ID')
for id,(x,y) in enumerate([(30,190),(120,190),(120,95),(30,95)]):tile(c,id,x,y)
c.setFont('STSong-Light',11)
for y,s in [(65,'按 100% 或“实际大小”打印，不要选择“适合页面”。'),(57,'黑色编码区 48 mm；四周白边各 6 mm；裁后外形 60 mm。'),(49,'不要剪掉白边，不要覆反光胶膜。四块按模型位置和方向摆放。')]:c.drawString(24*mm,y*mm,s)
c.setStrokeColorRGB(0,0,0);c.line(30*mm,32*mm,80*mm,32*mm)
for x in [30,80]:c.line(x*mm,30*mm,x*mm,34*mm)
c.setFont('Helvetica',10);c.drawString(89*mm,30*mm,'50 mm calibration bar')
c.save()
(out/'dictionary.json').write_text(json.dumps(data,indent=2),'utf-8')
print('5 PDFs + 4 PNGs + 4 SVGs; black 48mm, white border 6mm')
