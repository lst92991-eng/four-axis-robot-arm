"""离线核对实际STL、装配变换、参考视锥及全PCB器件包络；不控制设备。"""
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
load=lambda p:json.loads(p.read_text('utf-8-sig'))
qty={'01_clamp_right':1,'02_clamp_left':1,'03_truss_200':4,'04_camera_B_crossbar':1,'05_pcb_support':2}

def mesh(path):
    data=path.read_bytes();n=int.from_bytes(data[80:84],'little')
    a=np.frombuffer(data, dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84,count=n)
    return a['v'].astype(float)

def planes_box(lo,hi):
    a=[]
    for k in range(3):
        normal=np.eye(3)[k];a += [(normal,-lo[k]),(-normal,hi[k])]
    return a

def intersect_surface(triangles,planes):
    hits=0
    for tri in triangles:
        if any(np.all(tri@n+d < -1e-6) for n,d in planes):continue
        p=list(tri)
        for n,d in planes:
            out=[]
            for a,b in zip(p,p[1:]+p[:1]):
                fa=a@n+d;fb=b@n+d
                if fa>=0:out.append(a)
                if (fa<0)!=(fb<0):out.append(a+(b-a)*fa/(fa-fb))
            p=out
            if len(p)<3:break
        if len(p)>=3 and sum(np.linalg.norm(np.cross(p[k]-p[0],p[k+1]-p[0])) for k in range(1,len(p)-1))>1e-6:hits+=1
    return hits

meshes={};parts=[]
for name,count in qty.items():
    t=mesh(ROOT/'print'/(name+'.STL'));meshes[name]=t
    unique,inv=np.unique(np.round(t.reshape(-1,3),5),axis=0,return_inverse=True)
    f=inv.reshape(-1,3);edges=np.sort(np.concatenate([f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]]),axis=1)
    _,incidence=np.unique(edges,axis=0,return_counts=True)
    vol=np.einsum('ij,ij->i',t[:,0],np.cross(t[:,1],t[:,2])).sum()/6
    q=load(ROOT/(name+'-sketches.json'))
    assert np.all(incidence==2) and vol>0 and q['status']=='pass',name
    parts.append({'name':name,'quantity':count,'bounds_mm':np.r_[t.min((0,1)),t.max((0,1))].tolist(),'volume_mm3':vol,'watertight':True,'sketches':q['status']})
world=[]
for c in load(ROOT/'assembly-layout.json')['components']:
    m=np.array(c['transform']);t=meshes[Path(c['path']).stem]@m[:9].reshape(3,3)*m[12]+m[9:12]*1000
    world.append(t)
cx,cy,cz=0,250,626.4
view=[]
for w,h in [(520,480),(800,600)]:
    # f(p)>=0表示位于从镜头参考点到棋盘矩形的金字塔内。
    planes=[(np.array([0,0,1]),0),(np.array([0,0,-1]),cz),
        (np.array([1,0,-w/(2*cz)]),w/2),(np.array([-1,0,-w/(2*cz)]),w/2),
        (np.array([0,1,-h/(2*cz)]),h/2-cy),(np.array([0,-1,-h/(2*cz)]),h/2+cy)]
    hits=sum(intersect_surface(t,planes) for t in world)
    assert hits==0
    view.append({'board_rectangle_mm':[w,h],'frame_surface_intrusions':hits})
near=load(ROOT/'plate-evidence.json')['near_components'];near_hits=[]
for c in near:
    box=np.array(c['box_robot_mm']);planes=planes_box(box[:3],box[3:])
    for i,t in enumerate(world):
        if np.any(t.max((0,1))<=box[:3]+1e-5) or np.any(t.min((0,1))>=box[3:]-1e-5):continue
        hits=intersect_surface(t,planes)
        if hits:near_hits.append({'component':c['name'],'print_component_index':i,'surface_hits':hits})
assert not near_hits,near_hits
pcb=load(ROOT/'pcb-import-evidence.json');all_names={c['name'] for c in pcb['components']};pcb_hits=[]
# 横条底板与四根圆支柱分别用包络，避免将两柱之间的空气算成实体。
supports=[]
for y in (146,204):
    supports.append(([-73,y-5,650],[73,y+5,653]))
    for x in (-24.5,24.5):supports.append(([x-3,y-3,653],[x+3,y+3,665]))
for c in pcb['components']:
    if any(n.startswith(c['name']+'/') for n in all_names):continue
    a=np.array(c['box_mm']);lo=np.array([a[1]-24.4811506223,204.0616573533-a[3],665+a[2]])
    hi=np.array([a[4]-24.4811506223,204.0616573533-a[0],665+a[5]])
    for sl,sh in supports:
        if np.all(np.minimum(hi,sh)-np.maximum(lo,sl)>1e-5):pcb_hits.append(c['name'])
assert not pcb_hits,pcb_hits
old=load(ROOT.parent/'v1/parts-manifest.json')
old_volume=sum(p['quantity']*p['volume_mm3'] for p in old['parts'])
new_volume=sum(p['quantity']*p['volume_mm3'] for p in parts)
source=Path(pcb['source']);source_hash=hashlib.sha256(source.read_bytes()).hexdigest()
assert source_hash==pcb['sha256']
archive=load(ROOT.parent/'v1/original-file-verification.json')
arm=Path(load(ROOT/'plate-evidence.json')['original']);checks=[]
for a in archive:
    actual=hashlib.sha256((arm.parent/a['file']).read_bytes()).hexdigest()
    checks.append({'file':a['file'],'matches_original':actual==a['sha256']})
assert all(c['matches_original'] for c in checks)
result={'parts':parts,'physical_print_count':sum(qty.values()),'old_CAD_volume_mm3':old_volume,
        'new_STL_volume_mm3':new_volume,'volume_reduction_percent':100*(1-new_volume/old_volume),
        'view_cones':view,'neighbor_envelope_surface_intrusions':near_hits,
        'pcb_support_vs_all_component_envelope_overlaps':pcb_hits,
        'original_files_unchanged':checks,'user_pcb_sha256':source_hash,
        'scope':'静态几何和保守包络校核；不是打印配合、承载、长期蠕变、实物排线、光学标定或全关节扫掠验收。'}
(ROOT/'validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'parts':len(parts),'prints':sum(qty.values()),'volume_cm3':round(new_volume/1000,2),'reduction_percent':round(result['volume_reduction_percent'],2),'mesh':'pass','view_cones':'pass','neighbor_envelopes':'pass','pcb_support_envelopes':'pass','original_files':len(checks)},ensure_ascii=False))
