"""在SolidWorks临时实体副本上验算碰撞，不移动或保存装配。"""
import sys,json,math
from pathlib import Path
import pythoncom,win32com.client
import numpy as np
from audit_workspace_constraints import get
ROOT=Path(__file__).resolve().parents[1]
REF=json.loads((ROOT/'research/scara-kinematics-geometry.json').read_text('utf-8'))
SW=win32com.client.GetActiveObject('SldWorks.Application')
DOC=SW.GetOpenDocumentByName(REF['assembly'])
COMPS={c.Name2:c for c in DOC.GetComponents(False)}
MATH=get(SW,'GetMathUtility');MATH._FlagAsMethod('CreateTransform')
MODELER=get(SW,'GetModeler')
S=np.array([0.000508123003337,-0.155215865757977,0])
E=S+np.array([0,-.195,0])

def variant(values):return win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,list(values))
def transform(angle,center,shift=None):
    a=math.radians(angle);c=math.cos(a);s=math.sin(a)
    rot=np.array([[c,s,0],[-s,c,0],[0,0,1]])
    tr=center-center@rot
    if shift is not None:tr+=shift
    return MATH.CreateTransform(variant([*rot.ravel(),*tr,1,0,0,0]))
def body(name):
    c=COMPS[name];p=get(c,'GetModelDoc2');result=[]
    for b in p.GetBodies2(0,False) or []:
        cp=get(b,'Copy')
        if not cp.ApplyTransform(c.Transform2):raise RuntimeError('ApplyTransform failed')
        result.append(cp)
    return result
def moved(b,a=0,center=S):
    cp=get(b,'Copy')
    if a and not cp.ApplyTransform(transform(a,center)):raise RuntimeError('rotation failed')
    return cp

_body_dispids={}
def raw_copy_transform(b,t=None):
    """复用Body2的DISPID，避免每个临时副本反复查询整个类型库。"""
    raw=b._oleobj_ if hasattr(b,'_oleobj_') else b
    if not _body_dispids:
        _body_dispids.update({n:raw.GetIDsOfNames(n) for n in ('Copy','ApplyTransform')})
    cp=raw.InvokeTypes(_body_dispids['Copy'],0,pythoncom.DISPATCH_METHOD,(pythoncom.VT_DISPATCH,0),())
    if t is not None:
        ok=cp.InvokeTypes(_body_dispids['ApplyTransform'],0,pythoncom.DISPATCH_METHOD,(pythoncom.VT_BOOL,0),((pythoncom.VT_DISPATCH,1),),t._oleobj_)
        if not ok:raise RuntimeError('temporary body transform failed')
    return cp
def clash(b1,b2):
    f1=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None)
    f2=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None)
    bs=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None)
    status=MODELER.CheckInterference(b1,b2,False,f1,f2,bs)
    out=bs.value
    return {'status':status,'bodies':len(out) if out else 0,'faces':len(f1.value) if f1.value else 0}

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    b1=body('J1机械臂-2')[0];b2=body('铝型材挤压角座-1')[0]
    for a in [0,60,90,100,110,120,130,150,180]:
        print(a,clash(moved(b1,a),moved(b2)),flush=True)
