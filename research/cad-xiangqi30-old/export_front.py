import sys,json,math,shutil
from pathlib import Path
import pythoncom
from win32com.client import VARIANT
from PIL import Image
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
DELIVERY=OUT/'delivery-front-2026-10-13';DEST=DELIVERY/'SolidWorks';DEST.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g
from sw_review import save_review_previews
from sw_export import export_to_step
sw=win32com_client.GetActiveObject('SldWorks.Application');sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
source=OUT/'native/SCARA_FRONT_195_275_CHESS30.SLDASM'
m=sw.GetOpenDocumentByName(str(source));assert m is not None
top=list(g(m,'GetComponents',True));assert len(top)==38
rows=[]
for c in top:
    rows.append({'name':g(c,'Name2'),'file':str(Path(g(c,'GetPathName')).relative_to(OUT)),
                 'transform':list(g(g(c,'Transform2'),'ArrayData'))})
board=next(r for r in rows if 'board_315' in r['file'])
assert max(abs(a-b) for a,b in zip(board['transform'][9:12],[.000508123,-.435215866,1]))<1e-7
assert abs(board['transform'][0]-1)<1e-7
layout=json.loads((OUT/'assembly-layout.json').read_text('utf-8'))
layout['assembly']=str(source);layout['components']=rows
layout['board'].update({'world_center_mm':[0,280],'world_angle_deg':180,'cad_center_m':board['transform'][9:12],
 'piece_display_clearance_mm':.1,'user_placement':'2026-10-12 annotated photo; front centered; black near robot',
 'old_collision_grid_supported_points':86,'total_grid_points':90})
(OUT/'assembly-layout-front.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2),'utf-8')
print('Verified front placement: 38 components',flush=True)
# 导出已有保存模型，不重新保存或重建原机械臂。
assert export_to_step(m,str(DELIVERY/'SCARA_FRONT_195_275_CHESS30.step'))
save_review_previews(m,str(DELIVERY/'preview'),basename='front_layout')
f=[v/math.sqrt(2.64) for v in (1,-1,.8)];r=[2**-.5,2**-.5,0]
u=[f[1]*r[2]-f[2]*r[1],f[2]*r[0]-f[0]*r[2],f[0]*r[1]-f[1]*r[0]]
arr=[x for k in range(3) for x in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
mu=g(sw,'GetMathUtility');mu._FlagAsMethod('CreateTransform')
m.ActiveView.Orientation3=mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr))
m.ViewZoomtofit2();m.GraphicsRedraw2();assert m.SaveBMP(str(DELIVERY/'preview/front_layout_z_up.bmp'),1800,1200)
for f in (DELIVERY/'preview').glob('*.bmp'):Image.open(f).save(f.with_suffix('.png'))
names=set()
for f in (OUT/'native').rglob('*'):
    if not f.is_file() or f.name.startswith('~$') or f.suffix.lower() not in ('.sldprt','.sldasm'):continue
    if f.name=='SCARA_195_275_CHESS30.SLDASM':continue
    assert f.stat().st_size>0 and f.name.casefold() not in names
    names.add(f.name.casefold());shutil.copy2(f,DEST/f.name)
shutil.copytree(OUT/'markers',DELIVERY/'markers',dirs_exist_ok=True)
shutil.copy2(OUT/'assembly-layout-front.json',DELIVERY/'assembly-layout.json')
shutil.copy2(OUT/'piece-dimensions.json',DELIVERY/'piece-dimensions.json')
for name in ['front-layout-check.json','screw-head-3p82-analysis.json','source-parts-hash-check.json']:
    shutil.copy2(Path(__file__).parent/name,DELIVERY/name)
notes=(Path(__file__).parent/'delivery-notes.md').read_text('utf-8').replace('SolidWorks/SCARA_195_275_CHESS30.SLDASM','SolidWorks/SCARA_FRONT_195_275_CHESS30.SLDASM')
(DELIVERY/'README_先读我.md').write_text(notes,'utf-8')
pcb=Path('C:/Users/lst/Downloads/3D_PCB_20260622_2026-10-09.step')
if pcb.exists():
    (DELIVERY/'reference').mkdir(exist_ok=True);shutil.copy2(pcb,DELIVERY/'reference/RV1106_PCB_user_supplied.step')
print('STEP, views, PDFs and',len(names),'native files ready',flush=True)
