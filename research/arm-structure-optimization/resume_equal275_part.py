"""恢复等臂副本：只打开G1副本，避免重复加载原装配。"""
import sys,json
from pathlib import Path
import numpy as np
import pythoncom,win32com.client
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get,walk
OUT=ROOT/'hardware/arm_equal_275/v1';PATH=OUT/'native/EQ275_J1机械臂.SLDPRT'
sw=win32com.client.GetActiveObject('SldWorks.Application')
def iv():return win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
def apply(p,t):
    t=np.array(t);return np.array(p)@t[:9].reshape(3,3)*t[12]+t[9:12]
def slots(part):
    t=json.loads((OUT/'feature-comparison-before-fix.json').read_text('utf-8'))['copy']['transform']
    rows=[]
    for b in part.GetBodies2(0,False):
        for f in get(b,'GetFaces'):
            s=get(f,'GetSurface')
            if get(s,'IsCylinder'):
                p=s.CylinderParams
                if abs(p[6]-.015)<1e-7:
                    world=apply(p[:3],t)*1000
                    rows.append(float(-155.215865757977-world[1]))
    return sorted(set(round(v,6) for v in rows))
def main():
    p=sw.GetOpenDocumentByName(str(PATH)) or sw.OpenDoc6(str(PATH),1,1,'',iv(),iv())
    assert p and Path(get(p,'GetPathName')).resolve()==PATH.resolve()
    d=p.Parameter('D3@草图22');start=d.SystemValue
    p._FlagAsMethod('EditRebuild3')
    before=slots(p);d.SystemValue=start+.001;p.EditRebuild3();after=slots(p)
    print('slot D3 response',start,before,after,flush=True)
    slope=(after[0]-before[0])/.001
    assert abs(slope)>100
    target=start+(100-before[0])/slope
    d.SystemValue=target;p.EditRebuild3()
    print('slot corrected',target,slots(p),flush=True)
    assert np.allclose(slots(p),[100,230],atol=1e-4)
    feature=p.FeatureByName('草图19');sketch=get(feature,'GetSpecificFeature2')
    centers=[]
    for seg in get(sketch,'GetSketchSegments'):
        if get(seg,'GetType')==1:
            pt=get(seg,'GetCenterPoint2');centers.append((pt,[pt.X,pt.Y,pt.Z]))
    assert len(centers)==8
    # 原生中心矩形变长使零件坐标原点相对肩轴平移40mm；槽组在草图X方向补偿40mm。
    sw.ActivateDoc3(get(p,'GetTitle'),False,0,iv());p.ClearSelection2(True)
    assert feature.Select2(False,0)
    p._FlagAsMethod('EditSketch');p.EditSketch()
    flags=[]
    for pt,xyz in centers:flags.append(pt.SetCoords(xyz[0]+.04,xyz[1],xyz[2]))
    p.SketchManager.InsertSketch(True);p.EditRebuild3()
    actual=[]
    for seg in get(sketch,'GetSketchSegments'):
        if get(seg,'GetType')==1:
            pt=get(seg,'GetCenterPoint2');actual.append([pt.X,pt.Y,pt.Z])
    expected=np.array([xyz for _,xyz in centers])+[.04,0,0]
    error=float(np.max(np.linalg.norm(np.array(actual)-expected,axis=1))*1000)
    errors=[]
    for f,depth in walk(p):
        w=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BOOL,False);code=f.GetErrorCode2(w)
        if code:errors.append({'feature':f.Name,'code':code,'warning':w.value})
    report={'slot_D3_mm':target*1000,'slot_centers_shoulder_y_mm':slots(p),'motor_slot_max_error_mm':error,'SetCoords_results':flags,'part_feature_errors':errors}
    (OUT/'part-repair-check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False),flush=True)
    if error>1e-4 or any(not e['warning'] for e in errors):raise RuntimeError('修正尚未通过，未保存')
    er,wa=iv(),iv();assert p.Save3(1,er,wa)
    print('Saved repaired G1 copy',er.value,wa.value,flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
