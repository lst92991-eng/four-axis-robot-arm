import sys,json,hashlib,zipfile,shutil
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
D=OUT/'delivery-front-2026-10-13';DEST=D/'SolidWorks'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document
sw=win32com_client.GetActiveObject('SldWorks.Application');sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
target=DEST/'SCARA_FRONT_195_275_CHESS30.SLDASM'
owned=[]
for doc in g(sw,'GetDocuments') or []:
    p=g(doc,'GetPathName')
    if p and (Path(p).is_relative_to(OUT/'native') or Path(p).is_relative_to(DEST)):owned.append(g(doc,'GetTitle'))
for title in owned:sw.CloseDoc(title)
sw._FlagAsMethod('SetCurrentWorkingDirectory');sw.SetCurrentWorkingDirectory(str(DEST))
def dependencies(p,traverse=False):
    raw=list(sw.GetDocumentDependencies2(str(p),traverse,False,False) or [])
    return [raw[i] for i in range(1,len(raw),2)]
# 只改交付目录内关闭的副本的引用；不依赖原路径仍存在或SolidWorks的搜索兜底。
replaced=[]
for p in DEST.glob('*.SLDASM'):
    assert sw.GetOpenDocumentByName(str(p)) is None,'Package must be closed for reference rewriting'
    for old in dependencies(p,True):
        new=DEST/Path(old).name
        if '^' in Path(old).name:continue # 内嵌虚拟子装配，不是外部依赖。
        if not new.exists():
            assert Path(old).exists() and Path(old).suffix.lower() in ('.sldprt','.sldasm'),('missing package file',old)
            shutil.copy2(old,new)
        if Path(old)!=new:
            ok=sw.ReplaceReferencedDocument(str(p),old,str(new))
            replaced.append({'document':p.name,'old':old,'new':new.name,'api_result':bool(ok)})
    print('Rewired',p.name,flush=True)
stored=dependencies(target,True)
outside=[p for p in stored if '^' not in Path(p).name and not Path(p).is_relative_to(DEST)]
print('Stored outside refs',len(outside),'replacement results',sum(x['api_result'] for x in replaced),'/',len(replaced),flush=True)
assert not outside,('Stored external reference',outside)
owned=[]
for doc in g(sw,'GetDocuments') or []:
    p=g(doc,'GetPathName')
    if p and Path(p).is_relative_to(OUT/'native'):owned.append(g(doc,'GetTitle'))
for title in owned:sw.CloseDoc(title)
sw._FlagAsMethod('SetCurrentWorkingDirectory');sw.SetCurrentWorkingDirectory(str(DEST))
m=open_document(sw,str(target),read_only=True,silent=True,raise_on_error=True)
m.ResolveAllLightWeightComponents(False)
top=list(g(m,'GetComponents',True));assert len(top)==38
refs=[]
for c in g(m,'GetComponents',False):
    p=g(c,'GetPathName');virtual=bool(g(c,'IsVirtual'));suppressed=g(c,'GetSuppression')==0
    if not virtual:
        assert Path(p).is_relative_to(DEST) and Path(p).exists(),p
    if not suppressed and g(c,'GetModelDoc2') is None:c.SetSuppression2(2)
    if not suppressed:assert g(c,'GetModelDoc2') is not None,('unresolved',p)
    refs.append({'component':g(c,'Name2'),'file':Path(p).name,'virtual':virtual,'suppressed':suppressed})
board=next(c for c in top if 'board_315' in g(c,'GetPathName'))
t=list(g(g(board,'Transform2'),'ArrayData'))
assert max(abs(a-b) for a,b in zip(t[9:12],[.000508123,-.435215866,1]))<1e-7
report={'reopened':True,'top_components':38,'all_components':len(refs),'outside_package_references':[],
        'legacy_virtual_stored_paths':outside,
        'reference_rewrites':replaced,'stored_dependencies':stored,'components':refs,
        'board_center_cad_m':t[9:12],'board_reference_pose_verified':True,
        'marker_check':'OpenCV 5.0.0 decoded all four IDs from rendered A4 PDF',
        'geometry_scope':'front reference layout, old-screw collision grid supports 86/90; 4 points need exact recheck',
        'source_note':'source SLDASM resaved by native Pack and Go; 44 source part hashes unchanged',
        'original_arm_warning':'original internal mates/interferences retained; not a mechanical safety acceptance',
        'cam_data':'User approved discarding CAM machining metadata for reference copies'}
(D/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
files=[p for p in D.rglob('*') if p.is_file() and not p.name.startswith('~$') and p.suffix.lower()!='.bmp' and p.name!='SHA256SUMS.json']
assert all(p.stat().st_size>0 for p in files)
manifest={p.relative_to(D).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(files)}
(D/'SHA256SUMS.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf-8');files.append(D/'SHA256SUMS.json')
archive=ROOT/'output/SCARA_front_195-275_30mm_chess_2026-10-13.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for p in files:z.write(p,'SCARA_front_30mm/'+p.relative_to(D).as_posix())
with zipfile.ZipFile(archive) as z:assert z.testzip() is None
print(json.dumps({'zip':str(archive),'files':len(files),'bytes':archive.stat().st_size,
 'sha256':hashlib.sha256(archive.read_bytes()).hexdigest()},ensure_ascii=False),flush=True)
