"""只编辑Pack and Go副本的原生草图尺寸，回读孔轴与装配位置。"""
import json,sys,hashlib
from pathlib import Path
import numpy as np
import pythoncom,win32com.client
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import get,walk,safe
OUT=ROOT/'hardware/arm_equal_275/v1';NATIVE=OUT/'native'
ASSEMBLY=NATIVE/'EQ275_SCARA四轴机械臂总成.SLDASM'
SW=win32com.client.GetActiveObject('SldWorks.Application')
def iv():return win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
def protected(doc):
    p=Path(get(doc,'GetPathName')).resolve()
    if not p.is_relative_to(NATIVE.resolve()):raise RuntimeError('禁止编辑副本目录外文件: '+str(p))
def save(doc):
    protected(doc);e,w=iv(),iv()
    if not doc.Save3(1,e,w):raise RuntimeError(f'保存失败 {e.value} {w.value}')
    return {'errors':e.value,'warnings':w.value}
def cylinders(c):
    p=get(c,'GetModelDoc2');t=np.array(c.Transform2.ArrayData);r=t[:9].reshape(3,3);tr=t[9:12];result=[]
    for b in p.GetBodies2(0,False):
        for f in get(b,'GetFaces'):
            s=get(f,'GetSurface')
            if get(s,'IsCylinder'):
                q=s.CylinderParams;direction=np.array(q[3:6])@r
                if abs(direction[2])>.999999:
                    xyz=(np.array(q[:3])@r*t[12]+tr)*1000
                    row=[*xyz[:2],q[6]*1000]
                    if not any(np.linalg.norm(np.array(row)-v)<1e-5 for v in result):result.append(row)
    return result
def main():
    err,warn=iv(),iv();asm=SW.GetOpenDocumentByName(str(ASSEMBLY))
    if not asm:asm=SW.OpenDoc6(str(ASSEMBLY),2,1|16|64|512,'',err,warn)
    if not asm:raise RuntimeError('副本打开失败: '+str(err.value))
    protected(asm);outside=[];refs=[]
    cs=asm.GetComponents(False)
    for c in cs:
        p=safe(c,'GetPathName');v=safe(c,'IsVirtual')
        refs.append({'name':c.Name2,'path':p,'virtual':v,'suppressed':safe(c,'IsSuppressed')})
        if p and not v and not Path(p).resolve().is_relative_to(NATIVE.resolve()):outside.append(p)
    (OUT/'copied-reference-audit.json').write_text(json.dumps(refs,ensure_ascii=False,indent=2),encoding='utf-8')
    if outside:raise RuntimeError('副本仍引用外部原件: '+str(outside))
    SW.ActivateDoc3(get(asm,'GetTitle'),False,0,iv())
    print('resolve copy components',asm.ResolveAllLightWeightComponents(False),flush=True)
    cs=asm.GetComponents(False)
    c=next(c for c in cs if Path(get(c,'GetPathName')).name=='EQ275_J1机械臂.SLDPRT')
    part=get(c,'GetModelDoc2');protected(part)
    # 外形长250->330；槽的直段50->130，端部孔/圆角/板宽/厚度不变。
    changes=[]
    for name,old,new in [('D1@草图1',.250,.330),('D1@草图22',.050,.130)]:
        dim=part.Parameter(name);before=dim.SystemValue
        if abs(before-old)>1e-8:raise RuntimeError('输入尺寸与基准不符: '+name+' '+str(before))
        dim.SystemValue=new;changes.append({'dimension':name,'before_mm':before*1000,'after_mm':dim.SystemValue*1000})
    part._FlagAsMethod('EditRebuild3');part_result=part.EditRebuild3();part_save=save(part)
    asm._FlagAsMethod('ForceRebuild3');asm_result=asm.ForceRebuild3(False)
    cs=asm.GetComponents(False);key_axes={};components=[]
    for comp in cs:
        name=comp.Name2;path=safe(comp,'GetPathName')
        components.append({'name':name,'path':path,'transform':list(comp.Transform2.ArrayData),'suppressed':safe(comp,'IsSuppressed')})
        if Path(path or '').name in ['EQ275_J1机械臂.SLDPRT','EQ275_第二机械臂.SLDPRT','EQ275_DLE42-L2BJ-长款50.SLDPRT']:
            key_axes[name]=cylinders(comp)
    errors=[]
    for f,depth in walk(asm):
        if str(safe(f,'GetTypeName2')).startswith('Mate'):
            iswarning=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BOOL,False)
            err=f.GetErrorCode2(iswarning)
            if err:errors.append({'feature':f.Name,'code':err,'warning':iswarning.value})
    assembly_save=save(asm)
    source=json.loads((OUT/'source-before-copy.json').read_text('utf-8'))
    changed=[p for p,h in source['sha256'].items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    report={'assembly':str(ASSEMBLY),'modified_part':get(part,'GetPathName'),'parameter_changes':changes,
        'rebuild_result':{'part':part_result,'assembly':asm_result},'save':{'part':part_save,'assembly':assembly_save},
        'key_axes_xy_radius_mm':key_axes,'mate_errors':errors,'source_files_changed':changed,'components':components}
    (OUT/'native-edit-verification.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ['components']},ensure_ascii=False),flush=True)
    if changed:raise RuntimeError('原件哈希发生变化')
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
