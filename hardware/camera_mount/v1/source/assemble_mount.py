"""在新装配中布置打印件，并另建参考布局；保持原SCARA文件不变。"""
import json
import math
from pathlib import Path
import sys
import os
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_mount as b
import pythoncom
import win32com.client

ROOT=b.ROOT
SW=b.SW
MATH=SW.GetMathUtility
MATH._FlagAsMethod('CreateTransform')
NULL=win32com.client.VARIANT(pythoncom.VT_DISPATCH,None)
IDENTITY=[1,0,0,0,1,0,0,0,1]
# 部件X沿悬臂向棋盘，Y向上，Z为宽度，变换保持右手系。
BOOM=[0,-1,0,0,0,1,-1,0,0]
TRAY=[0,-1,0,1,0,0,0,0,1]
placed=[]


def assembly():
    doc=SW.NewDocument(os.path.normpath('C:/ProgramData/SOLIDWORKS/SOLIDWORKS 2026/templates/gb_assembly.asmdot'),0,0.0,0.0)
    if not doc:raise RuntimeError('无法新建装配')
    doc._FlagAsMethod('UnfixComponent','FixComponent','ViewZoomtofit2')
    return doc


def add(doc,name,xyz,rotation=IDENTITY,label=None,source_path=None):
    path=Path(source_path) if source_path else b.PARTS/(name+'.SLDPRT')
    comp=doc.AddComponent5(str(path),0,'',False,'',0,0,0)
    if not comp:raise RuntimeError('无法插入'+name)
    values=rotation+[v/1000 for v in xyz]+[1,0,0,0]
    transform=MATH.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,values))
    # 首个零件默认固定；先解除固定，写入变换，再固定场景位置。
    doc.ClearSelection2(True)
    comp.Select4(False,NULL,False)
    doc.UnfixComponent()
    comp.Transform2=transform
    doc.ClearSelection2(True)
    comp.Select4(False,NULL,False)
    doc.FixComponent()
    if label:comp.Name2=label
    actual=list(comp.Transform2.ArrayData)
    if any(abs(actual[i]-values[i])>1e-7 for i in range(13)):
        raise RuntimeError('装配变换没有生效：'+name)
    placed.append({'part':name,'label':label or comp.Name2,'xyz_mm':xyz,'rotation':rotation,'transform':actual})
    return comp


def save(doc,name):
    doc.ClearSelection2(True)
    doc.ForceRebuild3(True)  # 只重建新顶层，避免对子装配触发重建保存标记。
    for ext in ('.SLDASM','.step'):
        status=doc.SaveAs3(str(ROOT/(name+ext)),0,1)
        if status!=0:raise RuntimeError('装配保存失败：'+name+ext+' '+str(status))
    doc._FlagAsMethod('GraphicsRedraw2')
    f=[v/math.sqrt(2.64) for v in (1,-1,.8)]
    r=[2**-.5,2**-.5,0]
    u=[f[1]*r[2]-f[2]*r[1],f[2]*r[0]-f[0]*r[2],f[0]*r[1]-f[1]*r[0]]
    view=[x for k in range(3) for x in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
    doc.ActiveView.Orientation3=MATH.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,view))
    doc.ViewZoomtofit2()
    doc.GraphicsRedraw2()
    if not doc.SaveBMP(str(ROOT/(name+'.bmp')),2800,1800):
        raise RuntimeError('无法导出装配预览')
    from PIL import Image
    Image.open(ROOT/(name+'.bmp')).save(ROOT/(name+'.png'))
    if doc.SaveAs3(str(ROOT/(name+'.SLDASM')),0,1)!=0:raise RuntimeError('保存装配视图失败')


def make_mount():
    d=assembly()
    add(d,'01_base',[0,630,0],label='锚固底脚')
    for i in range(4):add(d,'02_mast_150',[0,630,10+i*150],label='立柱'+str(i+1))
    add(d,'04_rear_braced_boom',[0,630,610],BOOM,label='带三角撑后臂')
    add(d,'05_front_boom',[0,480,710],BOOM,label='前臂')
    add(d,'06_splice_plate',[-10,480,710],BOOM,label='拼接板左')
    add(d,'06_splice_plate',[16,480,710],BOOM,label='拼接板右')
    add(d,'07_camera_B_carrier',[0,310,710],BOOM,label='MIS5001_B安装座')
    add(d,'08_board_tray',[0,375,730],TRAY,label='短排线主板托盘')
    save(d,'camera_mount')
    (ROOT/'assembly-layout.json').write_text(json.dumps({'units':'mm','components':placed,
        'camera_lens_reference_mm':[0,250,646.4],'board_outer_mm':{'x':[-260,260],'y':[10,490]},
        'note':'参考光轴按孔阵中心；尚未实测光轴偏置、打印配合、承载和实机扫掠。'},ensure_ascii=False,indent=2),encoding='utf-8')
    return d


if __name__=='__main__':
    make_mount()
