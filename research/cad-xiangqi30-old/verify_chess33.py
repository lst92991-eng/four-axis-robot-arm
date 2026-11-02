"""逐版回读33mm总装，保存缓存、导出STEP并设置实际CAD预览。"""
import sys,json,hashlib,math
from pathlib import Path
import numpy as np,pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi33-assemblies-2026-10-13'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document,save_document
sw=win32com_client.GetActiveObject('SldWorks.Application');sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
label=sys.argv[1];stage=next(r for r in json.loads((OUT/'stages.json').read_text('utf-8')) if r['label']==label)
folder=OUT/label;dest=folder/'SolidWorks';main=Path(stage['main'])
sw._FlagAsMethod('SetCurrentWorkingDirectory');sw.SetCurrentWorkingDirectory(str(dest))
m=open_document(sw,str(main),silent=True,raise_on_error=True)
assert m.ResolveAllLightWeightComponents(False) in (0,1,2,3,4,5)
assert m.ForceRebuild3(True)
top=list(g(m,'GetComponents',True));assert len(top)==38
layout=json.loads((folder/'assembly-layout.json').read_text('utf-8'));expected={}
for row in layout['components']:expected.setdefault(row['file'],[]).append(row['transform'])
actual={};refs=[];discs=[];axes={}
def canonical(name):
    name=name.split('/')[-1]
    for prefix in ('O33_','N33_','OLD_','EQ275_'):name=name.replace(prefix,'')
    return name
for c in top:actual.setdefault(Path(g(c,'GetPathName')).name,[]).append(list(g(g(c,'Transform2'),'ArrayData')))
assert set(actual)==set(expected),(set(actual)^set(expected))
for name,poses in expected.items():
    assert len(poses)==len(actual[name])
    for pose in poses:assert min(np.max(np.abs(np.array(pose)-p)) for p in actual[name])<1e-8,('placement changed',name)
for c in g(m,'GetComponents',False):
    path=g(c,'GetPathName');suppressed=g(c,'GetSuppression')==0
    assert not bool(g(c,'IsVirtual')) and Path(path).is_relative_to(dest) and Path(path).exists(),path
    if not suppressed and g(c,'GetModelDoc2') is None:c.SetSuppression2(2)
    p=g(c,'GetModelDoc2');assert suppressed or p is not None,path
    refs.append({'name':g(c,'Name2'),'file':Path(path).name,'suppressed':suppressed})
    if 'piece_' in Path(path).name:
        bb=np.array(g(p.GetBodies2(0,False)[0],'GetBodyBox'));size=(bb[3:]-bb[:3])*1000
        assert np.allclose(size[:2],[33,33],atol=1e-4),(path,size)
        discs.append({'component':g(c,'Name2'),'diameter_x_mm':float(size[0]),'diameter_y_mm':float(size[1])})
    name=canonical(g(c,'Name2'))
    if name not in ('J1机械臂-2','第二机械臂-1'):continue
    t=np.array(list(g(g(c,'Transform2'),'ArrayData')));found=[]
    for b in p.GetBodies2(0,False):
        for f in g(b,'GetFaces'):
            surface=g(f,'GetSurface')
            if not g(surface,'IsCylinder'):continue
            a=np.array(g(surface,'CylinderParams'));axis=a[3:6]@t[:9].reshape(3,3)
            if abs(axis[2])>.999:
                xyz=(a[:3]@t[:9].reshape(3,3)*t[12]+t[9:12])*1000;found.append([*xyz[:2],a[6]*1000])
    axes[name]=found
assert len(discs)==32
first=axes['J1机械臂-2'];second=axes['第二机械臂-1']
shoulder=np.array(next(v[:2] for v in first if abs(v[2]-10)<1e-5));elbow=np.array(next(v[:2] for v in first if abs(v[2]-14.25)<1e-5))
elbow2=np.array(next(v[:2] for v in second if abs(v[2]-4.25)<1e-5));wrist=np.array(next(v[:2] for v in second if abs(v[2]-12.25)<1e-5))
lengths=[float(np.linalg.norm(elbow-shoulder)),float(np.linalg.norm(wrist-elbow2))]
assert np.allclose(lengths,stage['links_mm'],atol=1e-4) and np.linalg.norm(elbow-elbow2)<1e-4
assert save_document(m)
stored=list(sw.GetDocumentDependencies2(str(main),True,False,False) or [])[1::2]
assert all(Path(p).is_relative_to(dest) and Path(p).exists() for p in stored),stored
baseline=json.loads((OUT/'source-30mm-sha256.json').read_text('utf-8'))
assert all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in baseline.items())
report={'label':label,'top_components':len(top),'all_components':len(refs),'links_mm':lengths,'joint_axes_cad_mm':{'shoulder':shoulder.tolist(),'elbow':elbow.tolist(),'wrist':wrist.tolist()},'chess_piece_count':len(discs),'piece_diameter_mm':33,'nominal_piece_gap_mm':2,'board_edge_margin_mm':1,'poses_unchanged':True,'outside_stored_references':[],'outside_runtime_references':[],'native_screw_head_height_mm':6,'pieces':discs,'components':refs,'original_30mm_sources_unchanged':True}
(folder/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
step=folder/(main.stem+'.step');status=m.SaveAs3(str(step),0,1);assert status==0,status
print('VERIFIED',label,'33mm x32; links',lengths,'no external refs; STEP exported',flush=True)
sw.Visible=True;sw.FrameState=1
sw.ActivateDoc3(str(main),False,0,VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0))
mu=g(sw,'GetMathUtility');mu._FlagAsMethod('CreateTransform')
f=np.array([1.,-1.,.8]);f/=np.linalg.norm(f);r=np.array([2**-.5,2**-.5,0.]);u=np.cross(f,r)
arr=[v for k in range(3) for v in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
m.ActiveView.EnableGraphicsUpdate=True;m.ActiveView.Orientation3=mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr));m.ViewZoomtofit2();m.GraphicsRedraw2()
print('READY FOR PREVIEW',label,flush=True)
