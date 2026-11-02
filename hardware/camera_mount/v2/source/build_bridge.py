"""借用顶层铝板的双梁相机桥架。主板接口来自用户STEP。

只写v2目录。原机械臂只读取、不重建、不保存；尺寸单位mm。
SW2026 FeatureExtrusion2/FeatureCut3用法沿用本机v1已验证调用。
草图由技能的fully_define_sketch补尺寸，不用Fix锁死轮廓。
"""
import json
import os
from pathlib import Path
import sys

SKILL = Path(os.environ.get('SCARA_SW_SKILL', r'C:\Users\lst\.codex\tools\solidworks-automation-mcp'))
sys.path.insert(0, str(SKILL / 'scripts'))
from sw_connect import get_com_member as g, win32com_client
from sw_sketch_quality import fully_define_sketch, inspect_model_sketches
from sw_review import run_review

ROOT = Path(__file__).resolve().parents[1]
PARTS = ROOT / 'parts'
PRINT = ROOT / 'print'
for folder in (PARTS, PRINT): folder.mkdir(exist_ok=True)
SW = win32com_client.GetActiveObject('SldWorks.Application')
TEMPLATE = r'C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2026\templates\gb_part.prtdot'
records = []
sketch_records = []


def rect(a, b, c, d): return ('rectangle', a, b, c, d)
def circle(x, y, r): return ('circle', x, y, r)
def poly(points): return ('polygon', points)


def new_part(name):
    target = PARTS / (name + '.SLDPRT')
    old = SW.GetOpenDocumentByName(str(target))
    if old is not None:
        if g(old, 'GetSaveFlag'): raise RuntimeError('先保留自己的修改：' + name)
        SW.CloseDoc(g(old, 'GetTitle'))
    doc = SW.NewDocument(TEMPLATE, 0, 0., 0.)
    if doc is None: raise RuntimeError('新建失败')
    planes = []
    f = g(doc, 'FirstFeature')
    while f is not None:
        if g(f, 'GetTypeName2') == 'RefPlane': planes.append(g(f, 'Name'))
        f = g(f, 'GetNextFeature')
    return doc, planes


def feature(doc, planes, name, shapes, depth=1, start=0, plane=0, cut=False):
    doc.ClearSelection2(True)
    if not doc.FeatureByName(planes[plane]).Select2(False, 0): raise RuntimeError(name)
    sm = doc.SketchManager
    sm.InsertSketch(True)
    sm.AddToDB = True
    for shape in shapes:
        if shape[0] == 'rectangle':
            _, a, b, c, d = shape
            sm.CreateCornerRectangle(a/1000, b/1000, 0, c/1000, d/1000, 0)
        elif shape[0] == 'circle':
            _, x, y, r = shape
            sm.CreateCircleByRadius(x/1000, y/1000, 0, r/1000)
        else:
            points = shape[1]
            for a, b in zip(points, points[1:] + points[:1]):
                sm.CreateLine(a[0]/1000, a[1]/1000, 0, b[0]/1000, b[1]/1000, 0)
    sm.AddToDB = False
    evidence = fully_define_sketch(doc)
    sketch_records.append({'feature': name, **evidence})
    sm.InsertSketch(True)
    if cut:
        f = doc.FeatureManager.FeatureCut3(False, False, False, 1, 1, .5, .5,
            False, False, False, False, 0, 0, False, False, False, False,
            False, True, True, False, False, False, 0, 0, False)
    else:
        f = doc.FeatureManager.FeatureExtrusion2(True, False, False, 0, 0, depth/1000, 0,
            False, False, False, False, 0, 0, False, False, False, False,
            True, True, True, 3 if start else 0, abs(start)/1000, start < 0)
    if f is None: raise RuntimeError('特征失败：' + name)
    f.Name = name
    print(name, flush=True)


def save(doc, name, qty, note):
    doc.ClearSelection2(True)
    doc.ForceRebuild3(False)
    bodies = list(g(doc, 'GetBodies2', 0, False) or [])
    if len(bodies) != 1: raise RuntimeError('不是单一实体：' + name)
    mass = g(doc.Extension, 'CreateMassProperty')
    if mass.Volume <= 0: raise RuntimeError('体积无效')
    for path in (PARTS/(name+'.SLDPRT'), PARTS/(name+'.step'), PRINT/(name+'.STL')):
        status = doc.SaveAs3(str(path), 0, 1)
        if status != 0 or not path.exists(): raise RuntimeError(str(path) + ':' + str(status))
    quality = inspect_model_sketches(doc)
    (ROOT/(name+'-sketches.json')).write_text(json.dumps(quality, ensure_ascii=False, indent=2), encoding='utf-8')
    report, _ = run_review(doc, ROOT/'review', basename=name,
        expected_outputs=[str(PARTS/(name+'.SLDPRT')),str(PRINT/(name+'.STL'))])
    if doc.SaveAs3(str(PARTS/(name+'.SLDPRT')),0,1) != 0: raise RuntimeError('保存审查视图失败')
    records.append({'name': name, 'quantity': qty, 'volume_mm3': mass.Volume*1e9,
        'body_count': len(bodies), 'sketch_status': quality['status'],
        'review_evaluation': report['evaluation'], 'note': note})
    print('SAVED', name, round(mass.Volume*1e6,2), 'cm3', flush=True)


def clamp(hand):
    name = '01_clamp_right' if hand == 1 else '02_clamp_left'
    d,p = new_part(name)
    # u沿棋盘前后，v向上，w横向。右座w为正，左座镜像尺寸。
    def slab(label, shapes, low, high):
        a,b = sorted((hand*low, hand*high))
        feature(d,p,label,shapes,b-a,a)
    outline=[(-65,-5.4),(40,-5.4),(40,126),(-10,126),(-65,55)]
    slab('卡座外侧承力框', [poly(outline)], .2, 4.2)
    window=[(-55,15.2),(20,15.2),(20,105),(-4,105),(-55,49)]
    feature(d,p,'承力框减重窗口',[poly(window)],cut=True)
    slab('上卡唇',[rect(u-8,10.2,u+8,14.2) for u in (-55,15)],-8,4.2)
    slab('下卡唇螺母承力底',[rect(u-8,-5.4,u+8,-2.7) for u in (-55,15)],-10,4.2)
    nut_shapes=[]
    a,b=sorted((-4.2*hand,10*hand))
    for u in (-55,15):
        nut_shapes.append(rect(u-8,a,u+8,b))
        # 6.7x5.8矩形窝容纳M3六角螺母；5.8方向贴近两平面以防转。
        nut_shapes.append(rect(u-3.35,5*hand-2.9,u+3.35,5*hand+2.9))
    feature(d,p,'下卡唇M3螺母防转槽',nut_shapes,2.5,-2.7,plane=1)
    slab('梁根后挡',[rect(-10,81,-4,126)],.2,12.8)
    slab('梁根外夹片',[rect(-10,81,40,126)],8.8,12.8)
    feature(d,p,'梁根两孔3p4',[circle(30,v,1.7) for v in (91,116)],cut=True)
    # 顶视草图u/w的第二坐标为-w。通孔位于卡唇，压紧螺钉只从下方顶板。
    feature(d,p,'板边压紧孔3p4',[circle(u,5*hand,1.7) for u in (-55,15)],plane=1,cut=True)
    save(d,name,1,'10.4mm板厚卡槽；夹10mm顶板；两接触段避开下方角码。M3压紧，不靠摩擦插接单独承载。')


def beam():
    d,p = new_part('03_truss_200')
    feature(d,p,'45高4厚桁架毛坯',[rect(0,0,200,45)],4)
    triangles=[]
    for x in (22,75,128):
        triangles += [poly([(x,6),(x+43,6),(x,37)]),poly([(x+7,39),(x+50,8),(x+50,39)])]
    feature(d,p,'六个三角减重窗',triangles,cut=True)
    # 相邻同款梁叠接20mm；两片各1.9mm，名义中间间隙0.2mm。
    feature(d,p,'始末端半厚搭接',[rect(0,-4,20,-1.9),rect(180,-2.1,200,0)],plane=1,cut=True)
    feature(d,p,'搭接与梁根3p4孔',[circle(u,v,1.7) for u in (10,30,190) for v in (10,35)],cut=True)
    # 竖向固定孔需要局部加宽，不能在4mm宽梁上直接打3.4mm孔。
    feature(d,p,'顶面局部固定耳',[rect(u-5,42,u+5,45) for u in (65,95,125,155,175)],12,-4)
    feature(d,p,'顶面固定耳3p4孔',[circle(u,-2,1.7) for u in (65,95,125,155,175)],plane=1,cut=True)
    save(d,'03_truss_200',4,'200x45x4；同款梁搭接20mm。梁根用u30孔，接缝用u10/190孔。孔均3.4mm。')


def camera():
    d,p = new_part('04_camera_B_crossbar')
    feature(d,p,'相机横梁146x48',[rect(-73,-32,73,16)],3)
    feature(d,p,'横梁左右减重窗',[rect(-58,-26,-18,10),rect(18,-26,58,10)],cut=True)
    feature(d,p,'背面6mm间隔柱',[circle(x,y,2.7) for x in (-10.5,10.5) for y in (-6.75,6.75)],6,-6)
    holes=[circle(x,y,1.4) for x in (-10.5,10.5) for y in (-6.75,6.75)]
    holes += [circle(x,-25,1.7) for x in (-66.6,66.6)]
    feature(d,p,'相机2p8孔与梁端3p4孔',holes,cut=True)
    save(d,'04_camera_B_crossbar',1,'相机孔阵21x13.5；孔径2.8仅指打印座，不代表PCB孔径。')


def pcb_support():
    d,p = new_part('05_pcb_support')
    feature(d,p,'主板轻量横条146x10',[rect(-73,-5,73,5)],3)
    feature(d,p,'主板12mm支柱',[circle(x,0,3) for x in (-24.5,24.5)],12,3)
    # 主板孔距58x49。横条按58mm装，两端长孔吸收与梁耳60mm间距的差值。
    feature(d,p,'主板M2p5通孔',[circle(x,0,1.4) for x in (-24.5,24.5)],cut=True)
    feature(d,p,'梁上M3调节槽',[rect(x-1.7,-2.7,x+1.7,2.7) for x in (-66.6,66.6)],cut=True)
    save(d,'05_pcb_support',2,'146x10x15；主板孔径2.8、孔距49，两横条相隔58。梁端矩形槽3.4x5.4；底面留12mm间隔。')


if __name__ == '__main__':
    # 单独重建所需零件，避免改动已经审核的其他零件。
    old_t={k:SW.GetUserPreferenceToggle(k) for k in (69,70,71)}
    old_i={k:SW.GetUserPreferenceIntegerValue(k) for k in (78,211)}
    try:
        for k,v in {69:True,70:False,71:True}.items(): SW.SetUserPreferenceToggle(k,v)
        SW.SetUserPreferenceIntegerValue(78,2);SW.SetUserPreferenceIntegerValue(211,0)
        for command in sys.argv[1:] or ['clamps']:
            if command == 'clamps': clamp(1);clamp(-1)
            elif command == 'camera': camera()
            elif command == 'beam': beam()
            elif command == 'pcb': pcb_support()
    finally:
        for k,v in old_t.items(): SW.SetUserPreferenceToggle(k,v)
        for k,v in old_i.items(): SW.SetUserPreferenceIntegerValue(k,v)
        (ROOT/'partial-parts.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
