"""修正中心矩形变长引起的中槽/电机槽组漂移，只操作独立副本。"""
import sys,json
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'research/arm-structure-optimization'))
from edit_equal_275_copy import SW,OUT,ASSEMBLY,protected,save,iv,cylinders
from audit_workspace_constraints import get,walk
import pythoncom,win32com.client
def apply(p,t):
    t=np.array(t);return np.array(p)@t[:9].reshape(3,3)*t[12]+t[9:12]
def invert(p,t):
    t=np.array(t);return (np.array(p)-t[9:12])@t[:9].reshape(3,3).T/t[12]
def sketch_centers(component):
    p=get(component,'GetModelDoc2');f=p.FeatureByName('草图19');s=get(f,'GetSpecificFeature2')
    centers=[]
    for seg in get(s,'GetSketchSegments'):
        if get(seg,'GetType')==1:
            point=get(seg,'GetCenterPoint2');centers.append((point,[point.X,point.Y,point.Z]))
    ms=list(s.ModelToSketchTransform.ArrayData);cm=list(component.Transform2.ArrayData)
    world=np.array([apply(invert(x,ms),cm) for _,x in centers])
    return p,f,s,centers,ms,cm,world
def main():
    source=json.loads((OUT/'source-before-copy.json').read_text('utf-8'))['source']
    old=SW.GetOpenDocumentByName(source);asm=SW.GetOpenDocumentByName(str(ASSEMBLY))
    oc=next(c for c in old.GetComponents(False) if Path(get(c,'GetPathName')).name=='J1机械臂.SLDPRT')
    nc=next(c for c in asm.GetComponents(False) if Path(get(c,'GetPathName')).name=='EQ275_J1机械臂.SLDPRT')
    op,of,osk,ocent,oms,ocm,ow=sketch_centers(oc)
    p,f,s,cent,ms,cm,nw=sketch_centers(nc);protected(p)
    delta=ow.mean(axis=0)-nw.mean(axis=0)
    if len(ow)!=8 or len(nw)!=8:raise RuntimeError('电机槽数量改变')
    if np.max(np.abs((ow-ow.mean(axis=0))-(nw-nw.mean(axis=0))))>1e-7:raise RuntimeError('槽组不只是平移，需要重新检查')
    targets=[apply(invert(v+delta,cm),ms) for v in nw]
    slot=p.Parameter('D3@草图22');oldslot=slot.SystemValue;slot.SystemValue=.060
    SW.ActivateDoc3(get(p,'GetTitle'),False,0,iv());p.ClearSelection2(True)
    if not f.Select2(False,0):raise RuntimeError('不能选中副本电机草图')
    p._FlagAsMethod('EditSketch');p.EditSketch()
    # 只移动8个圆弧中心，保留草图的外部参考线、孔槽宽度和各槽相对距离。
    responses=[]
    for (point,_),target in zip(cent,targets):responses.append(point.SetCoords(*[float(v) for v in target]))
    p.SketchManager.InsertSketch(True)
    p._FlagAsMethod('EditRebuild3');p.EditRebuild3()
    _,_,_,_,_,_,after=sketch_centers(nc)
    error=float(np.max(np.linalg.norm(after-ow,axis=1))*1000)
    if error>1e-4:raise RuntimeError('电机槽位置未回到原肩轴位置: '+str(error))
    save(p);SW.ActivateDoc3(get(asm,'GetTitle'),False,0,iv())
    asm._FlagAsMethod('ForceRebuild3');asm.ForceRebuild3(False);save(asm)
    errors=[]
    for doc in [p,asm]:
        for feat,depth in walk(doc):
            w=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BOOL,False);code=feat.GetErrorCode2(w)
            if code:errors.append({'doc':get(doc,'GetTitle'),'feature':feat.Name,'code':code,'warning':w.value})
    result={'slot_D3_before_mm':oldslot*1000,'slot_D3_after_mm':60,'motor_slot_translation_cad_mm':(delta*1000).tolist(),
        'motor_slot_return_error_mm':error,'SetCoords_results':responses,'feature_errors':errors,'J1_axes':cylinders(nc)}
    (OUT/'native-sketch-position-fix.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
