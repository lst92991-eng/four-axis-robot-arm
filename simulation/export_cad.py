"""Windows离线工具：只读SolidWorks三角网格，导出旧机本体，绝不保存原CAD。"""
import hashlib,json,struct,sys
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path.home()/'.codex/tools/solidworks-automation-mcp/scripts'))
from sw_connect import win32com_client,get_com_member as get

ref=json.loads((ROOT/'research/scara-kinematics-geometry.json').read_text('utf-8'))
groups=json.loads((ROOT/'output/constrained-workspace-2026-10-09/rigid-groups.json').read_text('utf-8'))
audit=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'))
sw=win32com_client.GetActiveObject('SldWorks.Application')
doc=sw.GetOpenDocumentByName(ref['assembly'])
if not doc:raise RuntimeError('请先用COM只读打开原始195+275装配')
before=get(doc,'GetSaveFlag')
tags={n:(g+1 if g else 0) for n,g in groups['groups'].items()}
carriage=['MGW12H滑块-1','T型丝杆螺母-1','L型支架底座-1','L型支架竖板-1',
          '铝型材挤压角座-1','铝型材挤压角座-2','方键条销-1','8mm键槽法兰-1']
for name in carriage:tags[name]=1
# 原CAD这六枚紧固件没有配合记录，但其位置在升降底板/丝杆螺母安装面上。
for name in ['M4内六角x16mm-19','M4内六角x16mm-20','M4内六角x16mm-21',
             'M3内六角小头x16mm-6','M3内六角小头x16mm-7','M3内六角小头x16mm-8']:tags[name]=1
for name in ['T型丝杆500mm-1','8-16联轴器-1']:tags[name]=4
links={n:set() for n in tags}
for mate in audit['documents'][0]['mates']:
    if mate.get('suppressed') is True:continue
    names={e.get('component') for e in mate.get('entities',[])}
    for n in names:
        if n in links:links[n].update(names-{n})
# 只补充与升降支架直接或通过其他紧固件相连的螺钉；旋转组沿用原CAD配合审计。
for _ in range(4):
    for name in tags:
        if tags[name]==0 and any(w in name for w in ['螺','内六角','垫']):
            if any(tags.get(other)==1 for other in links[name]):tags[name]=1

origin=np.array([*ref['joint_axes_xy_mm']['shoulder'],0])/1000
out=ROOT/'simulation/assets/arm';out.mkdir(parents=True,exist_ok=True)
vertices=[[] for _ in range(5)];colours=[[] for _ in range(5)];records=[];skipped=[]
light=np.array([.3,-.4,.85]);light/=np.linalg.norm(light)
shaft_tip=None;hashes={}
for component in get(doc,'GetComponents',False) or []:
    name=get(component,'Name2')
    if name not in tags:
        skipped.append({'name':name,'reason':'原CAD被抑制的传动子装配，不擅自恢复'});continue
    raw=component.GetTessTriangles(True)
    if not raw:raise RuntimeError('可见组件没有三角网格：'+name)
    local=np.asarray(raw,dtype=np.float64).reshape(-1,3)
    transform=np.asarray(get(get(component,'Transform2'),'ArrayData'))
    world=local@transform[:9].reshape(3,3)*transform[12]+transform[9:12]
    bounds=np.array(component.GetBox(False,False)).reshape(2,3)
    outside=max(0,float(np.max(bounds[0]-world.min(0))),float(np.max(world.max(0)-bounds[1])))
    if outside>.0001:raise RuntimeError('组件网格坐标未匹配装配包络：'+name)
    if name=='42步进电机L40-1':shaft_tip=float(world[:,2].min()-1.)
    mapped=world-origin;mapped[:,:2]*=-1;mapped[:,2]-=1. # CAD板面Z=1000mm，XY旋转180°。
    group=tags[name]
    if group==3:mapped[:,1]-=.195 # 肘轴局部原点，参考姿态沿+Y。
    if group==4:mapped[:,1]+=.075 # 固定丝杆轴在肩轴后方75mm。
    normal=np.cross(mapped[1::3]-mapped[0::3],mapped[2::3]-mapped[0::3])
    normal/=np.maximum(np.linalg.norm(normal,axis=1,keepdims=True),1e-12)
    base=np.array([.68,.72,.77])
    try:
        material=get(component,'MaterialPropertyValues')
        if material and len(material)>=3 and min(material[:3])>=0:base=np.array(material[:3])
    except Exception:pass
    if '步进电机' in name:base=np.array([.22,.24,.27])
    rgb=np.clip(base[None,:]*(.38+.62*np.abs(normal@light))[:,None]*255,0,255).astype(np.uint8)
    rgba=np.c_[np.repeat(rgb,3,axis=0),np.full(len(mapped),255,dtype=np.uint8)]
    vertices[group].append(mapped.astype('<f4'));colours[group].append(rgba)
    path=get(component,'GetPathName')
    if path not in hashes:hashes[path]=hashlib.sha256(Path(path).read_bytes()).hexdigest()
    records.append({'name':name,'group':group,'source':path,'sha256':hashes[path],
                    'triangles':len(mapped)//3,'bbox_outside_mm':outside*1000})
with open(out/'arm.mesh','wb') as f:
    f.write(b'SCARA3D1');f.write(struct.pack('<I',5))
    for i in range(5):
        v=np.concatenate(vertices[i]);c=np.concatenate(colours[i])
        f.write(struct.pack('<I',len(v)));f.write(v.tobytes());f.write(c.tobytes())
after=get(doc,'GetSaveFlag')
assert after==before,'只读提取改变了文档状态，需要复核'
manifest={'assembly':ref['assembly'],'assembly_sha256':hashlib.sha256(Path(ref['assembly']).read_bytes()).hexdigest(),
          'links_mm':[195,275],'group_names':['fixed','Z carriage','shoulder','elbow','screw'],
          'cad_to_world':'XY围绕肩轴旋转180度；板面选CAD Z=1000mm；模型单位m',
          'shaft_tip_reference_m':shaft_tip,'board_plane_cad_m':1.,
          'z_shift_range_m':[-.223196156862,.104303843137],
          'scope':'原始CAD可见三角网格；紧固件分组供显示，未作为新的连续碰撞认证；末端电磁铁20mm为显示假设',
          'save_flag_before':before,'save_flag_after':after,'parts':records,'skipped':skipped,
          'mesh_sha256':hashlib.sha256((out/'arm.mesh').read_bytes()).hexdigest()}
(out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'parts':len(records),'triangles':sum(p['triangles'] for p in records),
                 'shaft_tip_reference_m':shaft_tip,'mesh_bytes':(out/'arm.mesh').stat().st_size},ensure_ascii=False))
