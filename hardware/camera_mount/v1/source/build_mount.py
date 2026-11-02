"""在运行中的SolidWorks 2026创建新相机支架；不编辑原SCARA装配。

尺寸单位mm。长杆也为打印件；只使用普通螺栓/螺母连接。
相机孔阵来自Luckfox官方B版图，PCB孔径和打印配合须实物试装。
"""
import json
import os
from pathlib import Path
import site
import sys

ROOT = Path(__file__).resolve().parents[1]
CAD_RUNTIME = Path(os.environ.get('SCARA_CAD_PYTHON', str(Path(os.environ['TEMP'])/'scara-cad-python')))
if CAD_RUNTIME.is_dir():
    site.addsitedir(str(CAD_RUNTIME))
    sys.path.extend(str(CAD_RUNTIME / p) for p in ('win32', 'win32/lib', 'Pythonwin'))
    os.add_dll_directory(str(CAD_RUNTIME / 'pywin32_system32'))
import pythoncom
import win32com.client

SW = win32com.client.GetActiveObject('SldWorks.Application')
TEMPLATE = os.environ.get('SCARA_PART_TEMPLATE', 'C:/ProgramData/SOLIDWORKS/SOLIDWORKS 2026/templates/gb_part.prtdot')
PARTS = ROOT / 'parts'
PRINT = ROOT / 'print'
PARTS.mkdir(exist_ok=True)
PRINT.mkdir(exist_ok=True)
records = []


def rect(x1, y1, x2, y2):
    return ('rectangle', x1, y1, x2, y2)


def circle(x, y, r):
    return ('circle', x, y, r)


def polygon(points):
    return ('polygon', points)


def new_part(name):
    existing = [d for d in SW.GetDocuments if d.GetPathName.lower() == str(PARTS / (name + '.SLDPRT')).lower()]
    for d in existing:
        if d.GetSaveFlag:
            raise RuntimeError('零件有未保存修改，请先处理：' + name)
        SW.CloseDoc(d.GetTitle)
    doc = SW.NewDocument(os.path.normpath(TEMPLATE), 0, 0.0, 0.0)
    if not doc:
        raise RuntimeError('无法创建新零件：' + TEMPLATE)
    planes = []
    feature = doc.FirstFeature
    while feature:
        if feature.GetTypeName2 == 'RefPlane':
            planes.append(feature.Name)
        feature = feature.GetNextFeature
    return doc, planes


def feature(doc, planes, name, shapes, depth=1, start=0, plane=0, cut=False):
    # Front草图u/v为X/Y；Top草图u/v为X/-Z，法向为+Y。
    doc.ClearSelection2(True)
    if not doc.FeatureByName(planes[plane]).Select2(False, 0):
        raise RuntimeError('无法选基准面：' + name)
    sketch = doc.SketchManager
    sketch.InsertSketch(True)
    sketch.AddToDB = True
    for shape in shapes:
        if shape[0] == 'rectangle':
            _, x1, y1, x2, y2 = shape
            sketch.CreateCornerRectangle(x1/1000, y1/1000, 0, x2/1000, y2/1000, 0)
        elif shape[0] == 'circle':
            _, x, y, r = shape
            sketch.CreateCircleByRadius(x/1000, y/1000, 0, r/1000)
        else:
            points = shape[1]
            for a, b in zip(points, points[1:] + points[:1]):
                sketch.CreateLine(a[0]/1000, a[1]/1000, 0, b[0]/1000, b[1]/1000, 0)
    sketch.AddToDB = False
    sketch.InsertSketch(True)
    if cut:
        f = doc.FeatureManager.FeatureCut3(False, False, False, 1, 1, .3, .3,
            False, False, False, False, 0, 0, False, False, False, False,
            False, True, True, False, False, False, 0, 0, False)
    else:
        f = doc.FeatureManager.FeatureExtrusion2(True, False, False, 0, 0, depth/1000, 0,
            False, False, False, False, 0, 0, False, False, False, False,
            True, True, True, 3 if start else 0, abs(start)/1000, start < 0)
    if not f:
        raise RuntimeError('特征创建失败：' + name)
    f.Name = name
    print(name, flush=True)


def save(doc, name, quantity, note):
    doc.ClearSelection2(True)
    doc.ForceRebuild3(False)
    bodies = list(doc.GetBodies2(0, False) or [])
    if len(bodies) != 1:
        raise RuntimeError(f'{name}: expected one solid, got {len(bodies)}')
    mass = doc.Extension.CreateMassProperty
    box = [v * 1000 for v in bodies[0].GetBodyBox()]
    if mass.Volume <= 0:
        raise RuntimeError('零件体积无效：' + name)
    for path in (PARTS / (name + '.SLDPRT'), PARTS / (name + '.step'), PRINT / (name + '.stl')):
        result = doc.SaveAs3(str(path), 0, 1)
        if result != 0 or not path.exists():
            raise RuntimeError(f'导出失败 {path}: {result}')
    records.append(dict(name=name, quantity=quantity, box_mm=box,
                        volume_mm3=mass.Volume*1e9, solid_bodies=len(bodies), note=note))
    print('SAVED', name, [round(box[i+3]-box[i], 2) for i in range(3)], flush=True)


def base():
    d,p = new_part('01_base')
    feature(d,p,'锚固底板140x140', [rect(-70,-70,70,70)], 10)
    feature(d,p,'立柱定位凸台39p6', [rect(-19.8,-19.8,19.8,19.8),rect(-15,-15,15,15)], 8,10)
    holes=[circle(x,y,2.75) for x in (-55,55) for y in (-55,55)]
    holes += [circle(x,y,2.25) for x in (-32,32) for y in (-32,32)]
    feature(d,p,'桌面M5与立柱M4通孔',holes,cut=True)
    save(d,'01_base',1,'必须锚固到刚性桌板/底板，不能自由摆放。')


def mast(length, name, quantity):
    d,p = new_part(name)
    feature(d,p,'50方管壁厚4p8',[rect(-25,-25,25,25),rect(-20.2,-20.2,20.2,20.2)],length)
    feature(d,p,'下连接法兰',[rect(-40,-40,40,40),rect(-20.2,-20.2,20.2,20.2)],8)
    feature(d,p,'上连接法兰与承台',[rect(-40,-40,40,40),rect(-15,-15,15,15)],8,length-8)
    feature(d,p,'定位凸台39p6',[rect(-19.8,-19.8,19.8,19.8),rect(-15,-15,15,15)],8,length)
    feature(d,p,'四个M4法兰通孔',[circle(x,y,2.25) for x in (-32,32) for y in (-32,32)],cut=True)
    save(d,name,quantity,'堆叠净增高%dmm；凸台插入下一件，不重复计入总高。'%length)


def rear_boom():
    d,p = new_part('04_rear_braced_boom')
    feature(d,p,'立柱顶部安装环',[rect(-40,-40,40,40),rect(-20.2,-20.2,20.2,20.2)],8,plane=1)
    outline=[(-30,8),(25,8),(140,80),(150,80),(150,120),(-30,120)]
    feature(d,p,'一体三角撑与后臂',[polygon(outline)],20,-10)
    feature(d,p,'三角减重窗口',[polygon([(25,30),(100,80),(25,80)])],cut=True)
    feature(d,p,'后臂减重孔',[circle(60,100,9),circle(90,100,9)],cut=True)
    feature(d,p,'悬臂拼接M4',[circle(125,90,2.25),circle(125,110,2.25)],cut=True)
    feature(d,p,'柱顶四个M4',[circle(x,z,2.25) for x in (-32,32) for z in (-32,32)],plane=1,cut=True)
    save(d,'04_rear_braced_boom',1,'后臂150mm；三角撑与连接板合一。')


def front_boom():
    d,p = new_part('05_front_boom')
    feature(d,p,'前臂200x40x20',[rect(0,-20,200,20)],20,-10)
    holes=[circle(x,0,9) for x in (70,110,145)]
    holes += [circle(x,y,2.25) for x in (25,182) for y in (-10,10)]
    feature(d,p,'减重与拼接相机孔',holes,cut=True)
    feature(d,p,'主板托盘竖向M4',[circle(50,0,2.25),circle(160,0,2.25)],plane=1,cut=True)
    save(d,'05_front_boom',1,'与后臂端面对接，两侧拼接板夹紧；200mm长边须确认打印机空间。')


def splice():
    d,p = new_part('06_splice_plate')
    feature(d,p,'双面拼接板',[rect(-40,-20,40,20)],6)
    feature(d,p,'拼接M4孔',[circle(x,y,2.25) for x in (-25,25) for y in (-10,10)],cut=True)
    save(d,'06_splice_plate',2,'悬臂接口两侧各一片，用4根M4贯穿。')


def camera():
    d,p = new_part('07_camera_B_carrier')
    feature(d,p,'相机背面托板',[rect(20,-40,90,-34)],60,-30)
    for z in (-16.3,10.3):
        feature(d,p,'夹臂侧板'+str(z),[rect(0,-40,30,20)],6,z)
    feature(d,p,'夹臂M4通孔',[circle(12,-10,2.25),circle(12,10,2.25)],cut=True)
    pads=[circle(x,z,2.7) for x in (60-6.75,60+6.75) for z in (-10.5,10.5)]
    feature(d,p,'相机6mm绝缘支柱',pads,6,-46,plane=1)
    holes=[circle(x,z,1.4) for x in (60-6.75,60+6.75) for z in (-10.5,10.5)]
    feature(d,p,'21x13p5相机孔阵',holes,plane=1,cut=True)
    save(d,'07_camera_B_carrier',1,'打印座通孔2.8；PCB实际孔径未标注，螺钉须与PCB核对。孔阵中心为安装参考，光轴位置需试拍微调。')


def tray():
    d,p = new_part('08_board_tray')
    feature(d,p,'主板通用托盘160x120',[rect(-80,-60,80,60)],6)
    holes=[circle(-55,0,2.25),circle(55,0,2.25)]
    holes += [circle(x,y,5) for x in (-35,0,35) for y in (-30,0,30)]
    holes += [rect(x-10,y-1.75,x+10,y+1.75) for x in (-60,60) for y in (-48,48)]
    feature(d,p,'安装孔散热孔与绑带槽',holes,cut=True)
    save(d,'08_board_tray',1,'主板用绝缘垫与绑带固定；不预设boomPI孔位，CSI接口朝相机侧。')


def main():
    old_toggles={k:SW.GetUserPreferenceToggle(k) for k in (69,70,71)}
    old_ints={k:SW.GetUserPreferenceIntegerValue(k) for k in (78,211)}
    try:
        SW.SetUserPreferenceToggle(69,True)
        SW.SetUserPreferenceToggle(70,False)
        SW.SetUserPreferenceToggle(71,True)
        SW.SetUserPreferenceIntegerValue(78,2)
        SW.SetUserPreferenceIntegerValue(211,0)
        base()
        mast(150,'02_mast_150',4)
        mast(50,'03_height_spacer_50',0)
        rear_boom()
        front_boom()
        splice()
        camera()
        tray()
        report={'units':'mm','solidworks_version':SW.RevisionNumber,'parts':records,
                'default_mast_stack_mm':600,'base_height_mm':10,'beam_center_above_mast_mm':100,
                'camera_roof_below_beam_mm':40,'camera_spacer_mm':6,'pcb_thickness_mm':1,
                'lens_front_from_pcb_front_mm':16.6,'nominal_lens_height_mm':646.4,
                'mast_xy_robot_mm':[0,630],'camera_hole_array_center_xy_robot_mm':[0,250],
                'camera_optical_xy_note':'孔阵中心作为布局参考；官方图未给光轴偏置，实拍校核。',
                'source_camera_outline':'https://www.luckfox.com/image/catalog/accBoard/MIS5001-5MP-Camera/MIS5001-5MP-Camera-details-size-B.jpg',
                'actual_ffc_length_mm':160,'status':'CAD prototype; not physically print-fit/load/calibration validated'}
        (ROOT/'parts-manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    finally:
        for k,v in old_toggles.items():SW.SetUserPreferenceToggle(k,v)
        for k,v in old_ints.items():SW.SetUserPreferenceIntegerValue(k,v)


if __name__=='__main__':
    main()
