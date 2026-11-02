"""独立重开总装，核对轴距/棋具变换/引用，导出STEP并准备视图。"""
import sys,json,shutil,hashlib,math
from pathlib import Path
import numpy as np,pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13'
OUT=ROOT/'hardware/xiangqi30-long275275-2026-10-13';DEST=OUT/'SolidWorks'
TARGET=DEST/'SCARA_FRONT_275_275_CHESS30.SLDASM'
SOURCE=ROOT/'hardware/arm_equal_275/v1/native'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document
sys.path.insert(0,str(ROOT/'research'))
from audit_workspace_constraints import walk
sw=win32com_client.GetActiveObject('SldWorks.Application');sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
owned=[]
for d in g(sw,'GetDocuments') or []:
    path=g(d,'GetPathName')
    if path and (Path(path).is_relative_to(DEST) or Path(path).is_relative_to(SOURCE)):owned.append(g(d,'GetTitle'))
for title in owned:sw.CloseDoc(title)
rewrites=[]
for p in DEST.glob('*.SLDASM'):
    if p.name.startswith('~$'):continue
    for prior in list(sw.GetDocumentDependencies2(str(p),False,False,False) or [])[1::2]:
        assert '^' not in Path(prior).name,('virtual dependency still present',p,prior)
        target=DEST/Path(prior).name;assert target.exists(),prior
        if Path(prior)!=target:
            ok=sw.ReplaceReferencedDocument(str(p),prior,str(target));assert ok
            rewrites.append({'assembly':p.name,'old':prior,'new':str(target)})
stored=list(sw.GetDocumentDependencies2(str(TARGET),True,False,False) or [])[1::2]
assert all(Path(p).is_relative_to(DEST) and Path(p).exists() for p in stored),stored
sw._FlagAsMethod('SetCurrentWorkingDirectory');sw.SetCurrentWorkingDirectory(str(DEST))
m=open_document(sw,str(TARGET),read_only=True,silent=True,raise_on_error=True)
m.ResolveAllLightWeightComponents(False)
top=list(g(m,'GetComponents',True));assert len(top)==38
refs=[]
for c in g(m,'GetComponents',False):
    path=g(c,'GetPathName');virtual=bool(g(c,'IsVirtual'));suppressed=g(c,'GetSuppression')==0
    assert not virtual,('unpackaged virtual',path)
    assert Path(path).is_relative_to(DEST) and Path(path).exists(),path
    if not suppressed and g(c,'GetModelDoc2') is None:c.SetSuppression2(2)
    assert suppressed or g(c,'GetModelDoc2') is not None,path
    refs.append({'name':g(c,'Name2'),'file':Path(path).name,'suppressed':suppressed})
layout=json.loads((OLD/'assembly-layout.json').read_text('utf-8'))
baseline={Path(r['file']).name:r for r in layout['components'] if not r['file'].lower().endswith('.sldasm')}
expected={}
for r in layout['components']:
    if not r['file'].lower().endswith('.sldasm'):expected.setdefault(Path(r['file']).name,[]).append(r['transform'])
actual={}
for c in top:
    name=Path(g(c,'GetPathName')).name
    if name.startswith('LONG_'):actual.setdefault(name.removeprefix('LONG_'),[]).append(list(g(g(c,'Transform2'),'ArrayData')))
assert set(expected)==set(actual)
for name,poses in expected.items():
    assert len(poses)==len(actual[name])
    for pose in poses:assert min(np.max(np.abs(np.array(pose)-p)) for p in actual[name])<1e-8,('layout drift',name)
arm=sw.GetOpenDocumentByName(str(DEST/'EQ275_SCARA四轴机械臂总成.SLDASM'))
axes={};volume=None
for c in g(arm,'GetComponents',False):
    name=g(c,'Name2').replace('EQ275_','')
    if name not in ('J1机械臂-2','第二机械臂-1'):continue
    p=g(c,'GetModelDoc2');t=np.array(list(g(g(c,'Transform2'),'ArrayData')));found=[]
    bodies=p.GetBodies2(0,False)
    if name=='J1机械臂-2':volume=bodies[0].GetMassProperties(1)[3]*1e9
    for b in bodies:
        for f in g(b,'GetFaces'):
            s=g(f,'GetSurface')
            if not g(s,'IsCylinder'):continue
            a=np.array(g(s,'CylinderParams'));axis=a[3:6]@t[:9].reshape(3,3)
            if abs(axis[2])>.999:
                xyz=(a[:3]@t[:9].reshape(3,3)*t[12]+t[9:12])*1000;found.append([*xyz[:2],a[6]*1000])
    axes[name]=found
first=axes['J1机械臂-2'];second=axes['第二机械臂-1']
shoulder=np.array(next(v[:2] for v in first if abs(v[2]-10)<1e-5));elbow=np.array(next(v[:2] for v in first if abs(v[2]-14.25)<1e-5))
elbow2=np.array(next(v[:2] for v in second if abs(v[2]-4.25)<1e-5));wrist=np.array(next(v[:2] for v in second if abs(v[2]-12.25)<1e-5))
lengths=[float(np.linalg.norm(elbow-shoulder)),float(np.linalg.norm(wrist-elbow2))]
assert np.allclose(lengths,[275,275],atol=1e-4) and np.linalg.norm(elbow-elbow2)<1e-4
assert abs(volume-136224.851137143)<.02
source_verification=json.loads((SOURCE.parent/'final-model-verification.json').read_text('utf-8'))
original_errors={(e['feature'],e['code'],e['warning']) for e in source_verification['inherited_assembly_errors']}
errors=[]
for f,depth in walk(arm):
    warning=VARIANT(pythoncom.VT_BYREF|pythoncom.VT_BOOL,False);code=f.GetErrorCode2(warning)
    if code:errors.append({'feature':g(f,'Name'),'code':int(code),'warning':bool(warning.value)})
added_errors=[e for e in errors if (e['feature'],e['code'],e['warning']) not in original_errors]
assert not added_errors,added_errors
layout['assembly']=str(TARGET);layout['arm_links_mm']=lengths
layout['components']=[{'name':g(c,'Name2'),'file':Path(g(c,'GetPathName')).name,'transform':list(g(g(c,'Transform2'),'ArrayData'))} for c in top]
layout['board'].pop('old_collision_grid_supported_points',None)
(OUT/'assembly-layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2),'utf-8')
source_hash={};archive_hash_checks=[]
for folder,manifest_name in ((OLD,'SHA256SUMS.json'),(SOURCE.parent,'DELIVERY-SHA256.json')):
    manifest=json.loads((folder/manifest_name).read_text('utf-8'))
    names=[name for name in manifest if name.lower().endswith(('.sldasm','.sldprt'))]
    assert all((folder/name).exists() and hashlib.sha256((folder/name).read_bytes()).hexdigest()==manifest[name] for name in names),'Original archive hash mismatch'
    archive_hash_checks.append({'manifest':str(folder/manifest_name),'native_files_checked':len(names),'changed':[]})
for folder in (SOURCE,OLD/'SolidWorks'):
    for p in folder.iterdir():
        if p.suffix.lower() in ('.sldprt','.sldasm') and not p.name.startswith('~$'):source_hash[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
# 与第一次复制时留存快照核对；快照若已写出，则绝不重新定义基准。
baseline_path=OUT/'source-sha256.json'
if baseline_path.exists():assert json.loads(baseline_path.read_text('utf-8'))==source_hash,'Source hash changed'
else:baseline_path.write_text(json.dumps(source_hash,ensure_ascii=False,indent=2),'utf-8')
report={'independently_reopened':True,'top_components':len(top),'all_components':len(refs),'stored_dependencies':stored,'outside_package_references':[],'virtual_components':0,'links_mm':lengths,'joint_axes_cad_mm':{'shoulder':shoulder.tolist(),'elbow':elbow.tolist(),'wrist':wrist.tolist()},'G1_volume_mm3':volume,'chess_and_marker_transforms_unchanged':True,'components':refs,'reference_rewrites':rewrites,'native_screw_head_height_mm':6,'low_head_workspace_is_analysis_variant_only':True,'source_archive_hash_checks':archive_hash_checks,'inherited_mate_errors':'Original 13 mate errors retained; see source final-model-verification.json'}
report.update({'arm_feature_errors':errors,'new_arm_feature_errors':added_errors})
(OUT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
for name in ('markers','reference'):shutil.copytree(OLD/name,OUT/name,dirs_exist_ok=True)
shutil.copy2(OLD/'piece-dimensions.json',OUT/'piece-dimensions.json')
shutil.copy2(ROOT/'hardware/arm_equal_275/v1/final-model-verification.json',OUT/'source-arm-verification.json')
print('Verified independent reopen, 275/275 axes, 37 chess/marker poses, no external refs',flush=True)
sw.ActivateDoc3(str(TARGET),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
status=m.SaveAs3(str(OUT/'SCARA_FRONT_275_275_CHESS30.step'),0,1);assert status==0,status
print('STEP exported',flush=True)
sw.Visible=True;sw.FrameState=1
mu=g(sw,'GetMathUtility');mu._FlagAsMethod('CreateTransform')
f=np.array([1.,-1.,.8]);f/=np.linalg.norm(f);r=np.array([2**-.5,2**-.5,0.]);u=np.cross(f,r)
arr=[v for k in range(3) for v in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
m.ActiveView.EnableGraphicsUpdate=True
m.ActiveView.Orientation3=mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr))
m.ViewZoomtofit2();m.GraphicsRedraw2()
print('READY FOR PREVIEW',flush=True)
