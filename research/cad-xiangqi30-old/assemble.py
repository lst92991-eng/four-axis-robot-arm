import sys,json,math,hashlib,faulthandler
faulthandler.dump_traceback_later(20,repeat=False)
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'hardware/xiangqi30-old195275-2026-10-12'
NATIVE=OUT/'native'
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import win32com_client,get_com_member as g,new_document,open_document,save_document,create_empty_dispatch_variant
from sw_assembly import add_component,_activate_model_document
from sw_review import run_review
import pythoncom
from win32com.client import VARIANT
sw=win32com_client.GetActiveObject('SldWorks.Application')
sw=win32com_client.dynamic.DumbDispatch(sw._oleobj_,'SldWorks.Application')
geom=json.loads((ROOT/'research/scara-kinematics-geometry.json').read_text('utf-8'))
origin=geom['joint_axes_xy_mm']['shoulder']
angle=math.radians(210)
center=[(origin[0]-256)/1000,(origin[1]-148)/1000,1.0]
target=NATIVE/'SCARA_195_275_CHESS30.SLDASM'
if target.exists():
    assembly=sw.GetOpenDocumentByName(str(target))
    assert assembly is not None,'Owned assembly must be open to resume'
else:
    assembly=new_document(sw,'assembly',max_documents=128)
    assert save_document(assembly,str(target))
records=[]
def add(path,xyz,angle=0,label=None):
    # 插件封装负责解析文件；随后明确设置零件原点变换，不能把包围盒中心当原点。
    print('Loading',path.name,flush=True)
    doc=sw.GetOpenDocumentByName(str(path))
    if doc is None:doc=open_document(sw,str(path),silent=True)
    assert doc is not None
    print('Activating assembly',flush=True)
    errors=VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    sw.ActivateDoc3(str(target),False,0,errors)
    print('Inserting',flush=True)
    existing=list(g(assembly,'GetComponents',True) or [])
    comp=next((c for c in existing if g(c,'Name2')==label or
        (path.suffix=='.SLDASM' and g(c,'GetPathName')==str(path))),None)
    if comp is None:comp=assembly.AddComponent5(str(path),0,'',False,'',0.,0.,0.)
    print('Inserted',flush=True)
    assert comp is not None,str(path)
    assembly.ClearSelection2(True)
    assert comp.Select4(False,create_empty_dispatch_variant(),False)
    assembly.UnfixComponent()
    c,s=math.cos(angle),math.sin(angle)
    values=[c,s,0,-s,c,0,0,0,1,*xyz,1,0,0,0]
    transform=g(comp,'Transform2')
    transform.ArrayData=VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,values)
    comp.Transform2=transform
    read=list(g(g(comp,'Transform2'),'ArrayData'))
    assert max(abs(a-b) for a,b in zip(read,values))<1e-8,'Transform mismatch'
    if label:comp.Name2=label
    assembly.ClearSelection2(True)
    assert comp.Select4(False,create_empty_dispatch_variant(),False)
    assembly.FixComponent()
    records.append({'name':g(comp,'Name2'),'file':str(path.relative_to(OUT)), 'transform':read})
    assembly.ClearSelection2(True)
    # 只关闭本脚本因插入新打开的副本文档；装配仍持有其引用。
    child=sw.GetOpenDocumentByName(str(path))
    if child is not None and str(path).startswith(str(NATIVE)):
        sw.CloseDoc(g(child,'GetTitle'))
    _activate_model_document(sw,assembly)
    print('Added',label or path.name,flush=True)
    return comp
def at(x,y,z=0):
    c,s=math.cos(angle),math.sin(angle)
    return [center[0]+(c*x-s*y)/1000,center[1]+(s*x+c*y)/1000,center[2]+z/1000]
arm=NATIVE/'arm/OLD_SCARA四轴机械臂总成.SLDASM'
add(arm,[0,0,0],label='OLD_ARM_195_275')
add(NATIVE/'chess/board_315x350_pitch35.SLDPRT',center,angle,'BOARD_315x350_P35')
rows=['rnbakabnr','.........','.c.....c.','p.p.p.p.p','.........','.........','P.P.P.P.P','.C.....C.','.........','RNBAKABNR']
for r,row in enumerate(rows):
    for col,code in enumerate(row):
        if code=='.':continue
        side='red' if code.isupper() else 'black'
        path=NATIVE/f'chess/piece_{side}_{code}.SLDPRT'
        add(path,at((col-4)*35,(4.5-r)*35,8.1),angle+(0 if code.isupper() else math.pi),f'{side}_{code}_{r}_{col}')
for id,(x,y) in enumerate([(-195,212.5),(195,212.5),(195,-212.5),(-195,-212.5)]):
    add(NATIVE/f'chess/aruco_id_{id}_60mm.SLDPRT',at(x,y),angle,f'ARUCO_ID_{id}')
assert len(records)==38
rebuilt=assembly.ForceRebuild3(False)
assert save_document(assembly)
manifest={'assembly':str(target),'top_components':len(records),'components':records,'rebuild_return':bool(rebuilt),
  'board':{'size_mm':[315,350,14],'pitch_mm':35,'grid_span_mm':[280,315], 'piece_diameter_mm':30,'piece_height_mm':8,
           'world_center_mm':[256,148],'world_angle_deg':30,'cad_center_m':center,'board_top_cad_z_mm':1000},
  'markers':{'dictionary':'DICT_4X4_50','ids':[0,1,2,3],'black_side_mm':48,'white_margin_mm':6,'outer_side_mm':60,
             'coplanar_support_height_mm':14,'total_local_envelope_mm':[450,485]},
  'purpose':'Reference assembly for camera fixture; no actuator commands; reference added parts fixed at measured transforms.'}
(OUT/'assembly-layout.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),'utf-8')
review,reviewpath=run_review(assembly,str(OUT/'preview'),'assembly',expected_outputs=[str(target)])
print('Review',reviewpath,review.get('evaluation'),flush=True)
