"""核对等臂副本装配，保存原生装配、STEP和模型预览；不打开原装配。"""
import sys,json,hashlib,math
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get,walk,safe
OUT=ROOT/'hardware/arm_equal_275/v1';NATIVE=OUT/'native';ASM=NATIVE/'EQ275_SCARA四轴机械臂总成.SLDASM'
SW=win32com.client.GetActiveObject('SldWorks.Application')
def iv():return win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
def feature_errors(doc):
    rows=[]
    for f,d in walk(doc):
        w=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BOOL,False);code=f.GetErrorCode2(w)
        if code:rows.append({'feature':f.Name,'code':code,'warning':w.value})
    return rows
def main():
    print('Opening resolved independent assembly',flush=True)
    asm=SW.GetOpenDocumentByName(str(ASM)) or SW.OpenDoc6(str(ASM),2,1|64|512,'',iv(),iv())
    assert asm and Path(get(asm,'GetPathName')).resolve()==ASM.resolve()
    SW.ActivateDoc3(get(asm,'GetTitle'),False,0,iv())
    print('resolve',asm.ResolveAllLightWeightComponents(False),flush=True)
    asm._FlagAsMethod('ForceRebuild3');asm.ForceRebuild3(False)
    old=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'))
    omap={c['name']:c for c in old['components']}
    groups=json.loads((ROOT/'output/constrained-workspace-2026-10-09/rigid-groups.json').read_text('utf-8'))['groups']
    rows=[];mismatches=[];axes={};part=None
    for c in asm.GetComponents(False):
        name=c.Name2;canonical=name.replace('EQ275_','');path=get(c,'GetPathName')
        row={'name':name,'canonical':canonical,'path':path,'suppressed':safe(c,'IsSuppressed'),'transform':list(c.Transform2.ArrayData)}
        rows.append(row)
        if canonical not in groups or row['suppressed'] or '/' in canonical:continue
        assert Path(path).resolve().is_relative_to(NATIVE.resolve())
        oc=omap[canonical];t=np.array(row['transform']);ot=np.array(oc['transform'])
        if canonical!='J1机械臂-2':
            shift=80 if groups[canonical]==2 or (groups[canonical]==1 and (-155.215865757977-(oc['box'][1]+oc['box'][4])*500)>140) else 0
            expected=ot[9:12]+[0,-shift/1000,0]
            error=float(np.linalg.norm(t[9:12]-expected)*1000)
            if error>1e-3 or not np.allclose(t[:9],ot[:9],atol=1e-6):mismatches.append({'component':canonical,'position_error_mm':error})
        if canonical in ['J1机械臂-2','第二机械臂-1']:
            p=get(c,'GetModelDoc2');found=[]
            if canonical=='J1机械臂-2':part=p
            for b in p.GetBodies2(0,False):
                for f in get(b,'GetFaces'):
                    s=get(f,'GetSurface')
                    if get(s,'IsCylinder'):
                        a=s.CylinderParams;axis=np.array(a[3:6])@t[:9].reshape(3,3)
                        if abs(axis[2])>.999:
                            xyz=(np.array(a[:3])@t[:9].reshape(3,3)*t[12]+t[9:12])*1000
                            found.append([*xyz[:2],a[6]*1000])
            axes[canonical]=found
    first=axes['J1机械臂-2'];second=axes['第二机械臂-1']
    shoulder=np.array(next(v[:2] for v in first if abs(v[2]-10)<1e-5))
    elbow=np.array(next(v[:2] for v in first if abs(v[2]-14.25)<1e-5))
    elbow2=np.array(next(v[:2] for v in second if abs(v[2]-4.25)<1e-5))
    wrist=np.array(next(v[:2] for v in second if abs(v[2]-12.25)<1e-5))
    lengths=[float(np.linalg.norm(elbow-shoulder)),float(np.linalg.norm(wrist-elbow2))]
    assert np.allclose(lengths,[275,275],atol=1e-4) and np.linalg.norm(elbow-elbow2)<1e-4
    es=feature_errors(asm);ps=feature_errors(part)
    original=json.loads((OUT/'feature-comparison-before-fix.json').read_text('utf-8'))['original']['errors']
    original_keys={(e['feature'],e['code'],e['warning']) for e in original}
    added=[e for e in es if (e['feature'],e['code'],e['warning']) not in original_keys]
    b=part.GetBodies2(0,False)[0];volume=b.GetMassProperties(1)[3]*1e9
    snapshot=json.loads((OUT/'source-after-restart-baseline.json').read_text('utf-8')) if (OUT/'source-after-restart-baseline.json').exists() else json.loads((OUT/'source-before-copy.json').read_text('utf-8'))
    changed=[p for p,h in snapshot['sha256'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    report={'assembly':str(ASM),'links_mm':lengths,'joint_axes_cad_mm':{'shoulder':shoulder.tolist(),'elbow':elbow.tolist(),'wrist':wrist.tolist()},
        'G1_volume_mm3':volume,'part_errors':ps,'inherited_assembly_errors':es,'new_assembly_errors':added,'component_position_mismatches':mismatches,
        'source_files_changed':changed,'source_drift_during_restart':snapshot.get('difference_since_initial_copy',[]),'components':rows}
    (OUT/'final-model-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ['components']},ensure_ascii=False),flush=True)
    assert not ps and not added and not mismatches and not changed
    assert abs(volume-136224.851137143)<.02
    e,w=iv(),iv();assert asm.Save3(1,e,w)
    asm.ClearSelection2(True)
    export_status=asm.SaveAs3(str(OUT/'SCARA_equal_275.step'),0,1)
    if export_status!=0:raise RuntimeError('STEP导出失败: '+str(export_status))
    mathutil=get(SW,'GetMathUtility');mathutil._FlagAsMethod('CreateTransform')
    f=np.array([1.,-1.,.8]);f/=np.linalg.norm(f);r=np.array([2**-.5,2**-.5,0.]);u=np.cross(f,r)
    arr=[v for k in range(3) for v in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
    asm.ActiveView.Orientation3=mathutil.CreateTransform(win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr))
    asm._FlagAsMethod('ViewZoomtofit2','GraphicsRedraw2');asm.ViewZoomtofit2();asm.GraphicsRedraw2()
    assert asm.SaveBMP(str(OUT/'model-isometric.bmp'),1800,1400)
    from PIL import Image
    Image.open(OUT/'model-isometric.bmp').save(OUT/'model-isometric.png')
    assert asm.Save3(1,iv(),iv())
    print('Verified and exported native assembly, STEP and preview',flush=True)
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
