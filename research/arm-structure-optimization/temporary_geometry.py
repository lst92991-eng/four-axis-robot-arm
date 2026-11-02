"""原CAD临时实体的截段/接长试验；不创建、激活或保存SolidWorks文档。"""
import sys,json
from pathlib import Path
import numpy as np
import pythoncom,win32com.client
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'research'))
from probe_cad_collision import SW,DOC,COMPS,S,E,MATH,MODELER,get,body,transform,variant
MODELER._FlagAsMethod('CreateBodyFromBox3')
OUT=Path(__file__).resolve().parent

def wrap(b):return b if hasattr(b,'_oleobj_') else win32com.client.Dispatch(b)
def duplicate(b):return get(wrap(b),'Copy')
def shift(b,robot_y_mm=0,z_mm=0):
    c=duplicate(b)
    if not c.ApplyTransform(transform(0,S,np.array([0,-robot_y_mm/1000,z_mm/1000]))):raise RuntimeError('translate failed')
    return c
def box(xlo,xhi,ylo,yhi,zlo,zhi):
    # x/y是以肩轴为原点的俯视坐标，z仍为总装绝对坐标；全部毫米。
    center=[S[0]-(xlo+xhi)/2000,S[1]-(ylo+yhi)/2000,zlo/1000]
    return MODELER.CreateBodyFromBox3(variant([*center,0,0,1,(xhi-xlo)/1000,(yhi-ylo)/1000,(zhi-zlo)/1000]))
def boolean(a,b,operation):
    error=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    target=wrap(a);target._FlagAsMethod('Operations2')
    result=target.Operations2(operation,wrap(b),error)
    if error.value!=0 or not result:raise RuntimeError(f'body boolean {operation} failed: {error.value}')
    return [wrap(c) for c in result]
def half(b,ymin,ymax):
    result=boolean(duplicate(b),box(-100,100,ymin,ymax,1000,1500),15901)
    if len(result)!=1:raise RuntimeError('split piece disconnected')
    return result[0]
def unite(a,b):
    result=boolean(a,b,15903)
    if len(result)!=1:raise RuntimeError('splice not a connected solid')
    return result[0]
def resize_arm(b,cut,delta,strips,zrange):
    """在等截面槽段修改长度，保留两端关节孔及厚度；不是缩放孔径。"""
    if not delta:return duplicate(b)
    low=half(b,-300,cut)
    high=half(b,cut+max(0,-delta),800)
    high=shift(high,delta)
    if delta>0:
        for xlo,xhi in strips:low=unite(low,box(xlo,xhi,cut,cut+delta,*zrange))
    return unite(low,high)

def describe(b):
    b=wrap(b);bounds=np.array(get(b,'GetBodyBox'))*1000
    return {'bounds_cad_mm':bounds.tolist(),'size_mm':(bounds[3:]-bounds[:3]).tolist(),
            'face_count':get(b,'GetFaceCount'),'volume_mm3':b.GetMassProperties(1)[3]*1e9}

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    before=get(DOC,'GetSaveFlag')
    a=body('J1机械臂-2')[0];b=body('第二机械臂-1')[0]
    az=np.array(get(a,'GetBodyBox'))[[2,5]]*1000;bz=np.array(get(b,'GetBodyBox'))[[2,5]]*1000
    records={'before':{'J1':describe(a),'J2':describe(b)}}
    for delta in [20,30,40]:
        print('resizing delta',delta,flush=True)
        aa=resize_arm(a,120,delta,[(-30,-15),(15,30)],az)
        bb=resize_arm(b,330,-delta,[(-30,-12.5),(12.5,30)],bz)
        records[str(delta)]={'J1':describe(aa),'J2':describe(bb)}
        print(json.dumps(records[str(delta)]),flush=True)
    records['save_flag_before']=before;records['save_flag_after']=get(DOC,'GetSaveFlag')
    (OUT/'temporary-arm-resize-evidence.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(records,ensure_ascii=False))
