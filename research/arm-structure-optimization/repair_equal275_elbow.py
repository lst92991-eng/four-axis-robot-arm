"""恢复肘部四孔定位，并将依赖原来抑制传动总成的附件随肘轴前移。"""
import sys,json,hashlib
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get,walk
OUT=ROOT/'hardware/arm_equal_275/v1';NATIVE=OUT/'native';sw=win32com.client.GetActiveObject('SldWorks.Application')
def iv():return win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
def apply(p,t):
    t=np.array(t);return np.array(p)@t[:9].reshape(3,3)*t[12]+t[9:12]
def invert(p,t):
    t=np.array(t);return (np.array(p)-t[9:12])@t[:9].reshape(3,3).T/t[12]
def save(p):
    assert Path(get(p,'GetPathName')).resolve().is_relative_to(NATIVE.resolve())
    assert p.Save3(1,iv(),iv())
def main():
    asm=sw.GetOpenDocumentByName(str(NATIVE/'EQ275_SCARA四轴机械臂总成.SLDASM'))
    cs={c.Name2.replace('EQ275_',''):c for c in asm.GetComponents(False)};c=cs['J1机械臂-2'];p=get(c,'GetModelDoc2')
    f=p.FeatureByName('草图18');s=get(f,'GetSpecificFeature2');cm=list(c.Transform2.ArrayData);ms=list(s.ModelToSketchTransform.ArrayData)
    centers=[]
    for seg in get(s,'GetSketchSegments'):
        if get(seg,'GetType')==1:
            pt=get(seg,'GetCenterPoint2');centers.append((pt,[pt.X,pt.Y,pt.Z]))
    world=np.array([apply(invert(xyz,ms),cm) for _,xyz in centers])
    assert len(world)==4
    target_xy=np.array([.000508123003337,-.430215865757977]);delta=np.r_[target_xy-world[:,:2].mean(axis=0),0]
    targets=[apply(invert(w+delta,cm),ms) for w in world]
    sw.ActivateDoc3(get(p,'GetTitle'),False,0,iv());p.ClearSelection2(True);assert f.Select2(False,0)
    p._FlagAsMethod('EditSketch');p.EditSketch()
    rm=s.RelationManager;rm._FlagAsMethod('DeleteRelation','AddRelation')
    fixed=[]
    for relation in rm.GetRelations(0):
        if get(relation,'GetRelationType')==17:
            fixed.append(get(relation,'GetEntities'))
            assert rm.DeleteRelation(relation)
    flags=[pt.SetCoords(*map(float,target)) for (pt,_),target in zip(centers,targets)]
    for entities in fixed:
        rm.AddRelation(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_DISPATCH,list(entities)),17)
    p.SketchManager.InsertSketch(True);p._FlagAsMethod('EditRebuild3');p.EditRebuild3();save(p)
    sw.ActivateDoc3(get(asm,'GetTitle'),False,0,iv());asm._FlagAsMethod('ForceRebuild3');asm.ForceRebuild3(False)
    # 先让原生配合求解，随后只检查仍留在旧肘轴位置的附件。
    old=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'))
    omap={c['name']:c for c in old['components']};groups=json.loads((ROOT/'output/constrained-workspace-2026-10-09/rigid-groups.json').read_text('utf-8'))['groups']
    mathutil=get(sw,'GetMathUtility');mathutil._FlagAsMethod('CreateTransform');moved=[]
    for comp in asm.GetComponents(False):
        name=comp.Name2.replace('EQ275_','')
        if name not in groups or name=='J1机械臂-2' or get(comp,'IsSuppressed'):continue
        o=omap[name];t=list(comp.Transform2.ArrayData);expected=list(o['transform'])
        shift=80 if groups[name]==2 or (groups[name]==1 and (-155.215865757977-(o['box'][1]+o['box'][4])*500)>140) else 0
        expected[10]-=shift/1000
        error=np.linalg.norm(np.array(t[9:12])-expected[9:12])*1000
        if error>1e-3:
            assert shift==80 and abs(error-80)<1e-3
            xform=mathutil.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,expected))
            ok=comp.SetTransformAndSolve2(xform)
            actual=list(comp.Transform2.ArrayData)
            moved.append({'component':name,'method':'SetTransformAndSolve2','returned':ok,'residual_mm':float(np.linalg.norm(np.array(actual[9:12])-expected[9:12])*1000)})
    asm.ForceRebuild3(False);save(asm)
    result={'elbow_pattern_translation_cad_mm':(delta*1000).tolist(),'point_moves':flags,'component_moves':moved,
        'G1_volume_mm3':p.GetBodies2(0,False)[0].GetMassProperties(1)[3]*1e9}
    (OUT/'elbow-repair.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    initial=json.loads((OUT/'source-before-copy.json').read_text('utf-8'))
    hashes={path:hashlib.sha256(Path(path).read_bytes()).hexdigest() for path in initial['sha256'] if Path(path).is_file()}
    (OUT/'source-after-restart-baseline.json').write_text(json.dumps({'sha256':hashes,'difference_since_initial_copy':[path for path,h in hashes.items() if h!=initial['sha256'][path]],'note':'恢复时只打开EQ275副本；保留重启间隔发生的源文件变化，不回滚。'},ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False),flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
