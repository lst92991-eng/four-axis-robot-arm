"""只读提取当前 SolidWorks 总装的三角面俯视投影，不改配合或保存模型。"""
import sys, os, site, json, hashlib, math
from pathlib import Path
RUNTIME = Path('C:/Users/lst/AppData/Local/Temp')
site.addsitedir(str(RUNTIME/'scara-cad-python'))
sys.path.extend(str(RUNTIME/'scara-cad-python'/p) for p in ('win32','win32/lib','Pythonwin'))
os.add_dll_directory(str(RUNTIME/'scara-cad-python/pywin32_system32'))
site.addsitedir(str(RUNTIME/'scara-workspace-python'))
import win32com.client
import numpy as np
from shapely.geometry import Polygon, mapping
from shapely.ops import unary_union

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/workspace-projection-2026-10-09'
OUT.mkdir(exist_ok=True,parents=True)
def get(obj,name,*args):
    attr=getattr(obj,name)
    return attr(*args) if callable(attr) and not hasattr(attr,'_oleobj_') else attr

def extract():
    ref=json.loads((ROOT/'research/scara-kinematics-geometry.json').read_text('utf-8'))
    sw=win32com.client.GetActiveObject('SldWorks.Application')
    doc=sw.GetOpenDocumentByName(ref['assembly'])
    if not doc: raise RuntimeError('原机械臂装配未打开')
    before=bool(get(doc,'GetSaveFlag'))
    shoulder=np.array(ref['joint_axes_xy_mm']['shoulder'])
    components=[]; skipped=[]; cache={}; axes={}
    for c in doc.GetComponents(False):
        part=get(c,'GetModelDoc2')
        if not part or get(part,'GetType')!=1: continue
        name=c.Name2
        try:
            transform=np.array(c.Transform2.ArrayData)
            rot=transform[:9].reshape(3,3); tr=transform[9:12]
            key=(get(part,'GetPathName'),c.ReferencedConfiguration)
            if key not in cache:
                triangles=[]; cylinders=[]
                for body in part.GetBodies2(0,False) or []:
                    for face in get(body,'GetFaces') or []:
                        tess=face.GetTessTriangles(True)
                        if tess: triangles.extend(tess)
                        surf=get(face,'GetSurface')
                        if get(surf,'IsCylinder'): cylinders.append(list(surf.CylinderParams))
                cache[key]=(np.array(triangles).reshape(-1,3,3),cylinders)
            local,cylinders=cache[key]
            world=(local@rot*transform[12]+tr)*1000
            xy=-(world[:,:,:2]-shoulder)
            polygons=[]
            for pts in xy:
                poly=Polygon(pts)
                if poly.area>1e-6: polygons.append(poly)
            shape=unary_union(polygons).simplify(0.03,preserve_topology=True)
            components.append({'name':name,'projection':mapping(shape),'zmax_mm':float(world[:,:,2].max()),'bounds_xyz_mm':[world.reshape(-1,3).min(axis=0).tolist(),world.reshape(-1,3).max(axis=0).tolist()]})
            if any(v in name for v in ('J1机械臂','第二机械臂','8mm夹紧法兰')):
                groups=[]
                for cp in cylinders:
                    center=(np.array(cp[:3])@rot*transform[12]+tr)*1000
                    direction=np.array(cp[3:6])@rot
                    if abs(direction[2])>0.999:
                        p=(-(center[:2]-shoulder)).round(4).tolist()
                        if p not in groups: groups.append(p)
                axes[name]=groups
        except Exception as e: skipped.append({'name':name,'error':str(e)})
    result={'assembly':ref['assembly'],'sha256':hashlib.sha256(Path(ref['assembly']).read_bytes()).hexdigest(),
        'save_flag_before':before,'save_flag_after':bool(get(doc,'GetSaveFlag')),
        'coordinate':'俯视：X=-(CAD_X-肩轴X)，Y=-(CAD_Y-肩轴Y)，Z同CAD；+Y为当前伸臂前方。毫米。',
        'method':'各实体表面三角形经组件Transform2转换，沿Z投影并逐组件取并集。三角化显示精度，不是制造用精确轮廓。',
        'limits':'理想腕轴工作区。未验证关节限位/碰撞/Z/工具TCP；整机实体扫掠区不等于腕轴可达区。',
        'axis_checks':axes,'components':components,'skipped':skipped}
    (OUT/'cad-projection-evidence.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='components'},ensure_ascii=False))
    print('projected components:',len(components))

if __name__=='__main__':extract()
