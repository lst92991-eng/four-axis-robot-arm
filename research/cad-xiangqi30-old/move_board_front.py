import sys,math,json,faulthandler
faulthandler.dump_traceback_later(15,repeat=True)
from pathlib import Path
import pythoncom
from win32com.client import VARIANT
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,save_document,create_empty_dispatch_variant
from sw_review import run_review
sw=win32com_client.GetActiveObject('SldWorks.Application')
sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
asm=sw.GetOpenDocumentByName(str(OUT/'native/SCARA_195_275_CHESS30.SLDASM'))
layout=json.loads((OUT/'assembly-layout.json').read_text('utf-8'))
old=layout['board']['cad_center_m'];oldAngle=math.radians(layout['board']['world_angle_deg']+180)
new=[.000508123,-.435215866,1.0]
parts=[c for c in g(asm,'GetComponents',True) if '/native/chess/' in g(c,'GetPathName').replace('\\','/')]
assert len(parts)==37
def select_all():
    asm.ClearSelection2(True)
    for i,c in enumerate(parts):assert c.Select4(i>0,create_empty_dispatch_variant(),False)
print('Selecting group',flush=True);select_all();print('Unfixing group',flush=True);asm.UnfixComponent();print('Group released',flush=True)
records=[]
for c in parts:
    print('Moving',g(c,'Name2'),flush=True)
    tr=g(c,'Transform2')
    baseline=next(e for e in layout['components'] if e['name']==g(c,'Name2'))
    a=list(baseline['transform'])
    if 'piece_' in baseline['file']:a[11]=1.0081
    dx,dy=a[9]-old[0],a[10]-old[1];co,si=math.cos(oldAngle),math.sin(oldAngle)
    x,y=co*dx+si*dy,-si*dx+co*dy
    angle=math.atan2(a[1],a[0])-oldAngle
    a[:9]=[math.cos(angle),math.sin(angle),0,-math.sin(angle),math.cos(angle),0,0,0,1]
    a[9:11]=[new[0]+x,new[1]+y]
    tr.ArrayData=VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,a);c.Transform2=tr
    actual=list(g(g(c,'Transform2'),'ArrayData'))
    assert max(abs(v-w) for v,w in zip(a,actual))<1e-8
    records.append({'name':g(c,'Name2'),'file':str(Path(g(c,'GetPathName')).relative_to(OUT)),'transform':actual})
print('Fixing group',flush=True);select_all();asm.FixComponent();asm.ClearSelection2(True)
assert asm.ForceRebuild3(True);assert save_document(asm)
arm=[x for x in layout['components'] if '/arm/' in x['file'].replace('\\','/')]
layout['components']=arm+records
layout['board'].update({'world_center_mm':[0,280],'world_angle_deg':180,'cad_center_m':new,'piece_display_clearance_mm':.1,
                      'user_placement':'2026-10-12 annotated photo; centered in front of arm; black near robot',
                      'old_collision_grid_supported_points':86,'total_grid_points':90})
(OUT/'assembly-layout.json').write_text(json.dumps(layout,ensure_ascii=False,indent=2),'utf-8')
review,path=run_review(asm,str(OUT/'preview'),'assembly',expected_outputs=[str(OUT/'native/SCARA_195_275_CHESS30.SLDASM')])
f=[v/math.sqrt(2.64) for v in (1,-1,.8)];r=[2**-.5,2**-.5,0]
u=[f[1]*r[2]-f[2]*r[1],f[2]*r[0]-f[0]*r[2],f[0]*r[1]-f[1]*r[0]]
arr=[x for k in range(3) for x in (r[k],u[k],f[k])]+[0.,0.,0.,1.,0.,0.,0.]
mu=g(sw,'GetMathUtility');mu._FlagAsMethod('CreateTransform')
asm.ActiveView.Orientation3=mu.CreateTransform(VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,arr))
asm.ViewZoomtofit2();asm.GraphicsRedraw2();assert asm.SaveBMP(str(OUT/'preview/assembly_z_up.bmp'),1800,1200)
assert save_document(asm)
print('Moved 37 chess/board/marker components; arm unchanged; front placement saved',flush=True)
