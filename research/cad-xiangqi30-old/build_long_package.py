"""复制既有棋具与275+275原生模型；只修改本次交付副本的引用。"""
import sys,json,shutil,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OLD=ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13'
SOURCE=ROOT/'hardware/arm_equal_275/v1/native'
OUT=ROOT/'hardware/xiangqi30-long275275-2026-10-13'
DEST=OUT/'SolidWorks';DEST.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,open_document
sw=win32com_client.GetActiveObject('SldWorks.Application')
sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
owned=[]
for d in list(g(sw,'GetDocuments') or []):
    path=g(d,'GetPathName')
    if path and (Path(path).is_relative_to(DEST) or Path(path).is_relative_to(SOURCE)):
        owned.append(g(d,'GetTitle'))
for title in owned:sw.CloseDoc(title)
source_hash={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for folder in (SOURCE,OLD/'SolidWorks') for p in folder.iterdir() if p.suffix.lower() in ('.sldasm','.sldprt') and not p.name.startswith('~$')}
for p in SOURCE.iterdir():
    if p.suffix.lower() in ('.sldasm','.sldprt') and not p.name.startswith('~$'):shutil.copy2(p,DEST/p.name)
mapping={}
for p in (OLD/'SolidWorks').iterdir():
    if p.name.startswith(('piece_','board_','aruco_')):
        target=DEST/('LONG_'+p.name);shutil.copy2(p,target);mapping[p.name]=target
main=DEST/'SCARA_FRONT_275_275_CHESS30.SLDASM'
shutil.copy2(OLD/'SolidWorks/SCARA_FRONT_195_275_CHESS30.SLDASM',main)
mapping['OLD_SCARA四轴机械臂总成.SLDASM']=DEST/'EQ275_SCARA四轴机械臂总成.SLDASM'
rewrites=[]
for p in DEST.glob('*.SLDASM'):
    if p.name.startswith('~$'):continue
    assert sw.GetOpenDocumentByName(str(p)) is None
    raw=list(sw.GetDocumentDependencies2(str(p),p!=main,False,False) or [])
    for prior in raw[1::2]:
        name=Path(prior).name
        if '^' in name:continue
        dest=mapping.get(name,DEST/name)
        if not dest.exists():
            raise RuntimeError(('missing dependency',p.name,prior))
        if Path(prior)!=dest:
            ok=sw.ReplaceReferencedDocument(str(p),prior,str(dest))
            rewrites.append({'assembly':p.name,'old':prior,'new':str(dest),'ok':bool(ok)})
            assert ok,(p.name,prior)
    print('Rewired',p.name,flush=True)
raw=list(sw.GetDocumentDependencies2(str(main),True,False,False) or [])
outside=[p for p in raw[1::2] if '^' not in Path(p).name and not Path(p).is_relative_to(DEST)]
assert not outside,outside
model=open_document(sw,str(main),silent=True,raise_on_error=True)
model.ResolveAllLightWeightComponents(False)
top=list(g(model,'GetComponents',True));assert len(top)==38
refs=[]
for c in g(model,'GetComponents',False):
    path=g(c,'GetPathName');virtual=bool(g(c,'IsVirtual'));suppressed=g(c,'GetSuppression')==0
    if not suppressed and g(c,'GetModelDoc2') is None:c.SetSuppression2(2)
    if not suppressed:assert g(c,'GetModelDoc2') is not None,('unresolved',g(c,'Name2'),path)
    if not virtual:assert Path(path).is_relative_to(DEST) and Path(path).exists(),path
    refs.append({'name':g(c,'Name2'),'file':path,'virtual':virtual,'suppressed':suppressed})
board=next(c for c in top if 'board_' in g(c,'GetPathName'))
pose=list(g(g(board,'Transform2'),'ArrayData'));assert max(abs(a-b) for a,b in zip(pose[9:12],[.000508123,-.435215866,1]))<1e-7
layout=json.loads((OLD/'assembly-layout.json').read_text('utf-8'))
layout['assembly']=str(main);layout['arm_links_mm']=[275,275]
layout['components']=[{'name':g(c,'Name2'),'file':Path(g(c,'GetPathName')).name,'transform':list(g(g(c,'Transform2'),'ArrayData'))} for c in top]
layout['board'].pop('old_collision_grid_supported_points',None)
(OUT/'assembly-layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2),'utf-8')
report={'opened':True,'top_components':len(top),'all_components':len(refs),'outside_stored_references':outside,'rewrites':rewrites,'components':refs,'board_pose_cad_m':pose[9:12],'head_height_mm':6,'source_files_unchanged':all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h for p,h in source_hash.items())}
assert report['source_files_unchanged']
(OUT/'validation-build.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
(OUT/'source-sha256.json').write_text(json.dumps(source_hash,ensure_ascii=False,indent=2),'utf-8')
for name in ('markers','reference'):shutil.copytree(OLD/name,OUT/name,dirs_exist_ok=True)
shutil.copy2(OLD/'piece-dimensions.json',OUT/'piece-dimensions.json')
print('READY',len(refs),'components; source hashes unchanged',flush=True)
