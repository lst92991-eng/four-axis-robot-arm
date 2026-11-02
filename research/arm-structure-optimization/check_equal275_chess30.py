"""30mm棋子对应两种格距的离线摆位初筛，保留同一肘型。"""
import json, math, site
from pathlib import Path
import numpy as np
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
from shapely.geometry import Polygon, mapping
from shapely.ops import unary_union
from shapely import contains_xy
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'research/arm-structure-optimization'
d=np.load(ROOT/'hardware/arm_equal_275/v1/analysis/workspace.npz')
angles,mask=d['angles'],d['connected']
a,b=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
x=-275*np.sin(a)-275*np.sin(a+b);y=275*np.cos(a)+275*np.cos(a+b)
cells=[]
for i in range(len(angles)-1):
    for j in range(len(angles)-1):
        if angles[j]<0 or not np.all(mask[i:i+2,j:j+2]):continue
        p=Polygon([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])])
        cells.append(p if p.is_valid else p.buffer(0))
field=unary_union(cells)
centers=np.array([(cx,cy) for cy in range(80,401,10) for cx in range(-350,351,10)])
results=[]
for pitch in [40,35]:
    pts=np.array([(cx,cy) for cy in np.arange(10)*pitch-4.5*pitch for cx in np.arange(9)*pitch-4*pitch])
    corners=np.array([[-4*pitch,-4.5*pitch],[4*pitch,-4.5*pitch],[4*pitch,4.5*pitch],[-4*pitch,4.5*pitch]])
    best=None
    for angle in range(0,180,5):
        r=math.radians(angle);rot=np.array([[math.cos(r),math.sin(r)],[-math.sin(r),math.cos(r)]])
        p=centers[:,None,:]+pts@rot
        counts=contains_xy(field,p[:,:,0],p[:,:,1]).sum(axis=1)
        for k in np.flatnonzero(counts==90):
            rect=Polygon(centers[k]+corners@rot)
            if not field.contains(rect):continue
            clearance=rect.distance(field.boundary)
            if best is None or clearance>best['field_boundary_margin_mm']:
                best={'center_mm':centers[k].tolist(),'rotation_ccw_deg':angle,
                      'corners_mm':(centers[k]+corners@rot).tolist(),
                      'field_boundary_margin_mm':clearance,'covered_intersections':90}
    results.append({'piece_diameter_mm':30,'pitch_mm':pitch,'intersection_span_mm':[8*pitch,9*pitch],
                    'piece_edge_gap_mm':pitch-30,'best_sampled_layout':best})
    print(json.dumps(results[-1],ensure_ascii=False),flush=True)
out={'results':results,'branch':'positive q2 only','scope':'275+275mm，当前Z与R姿态，4度CAD采样单元插值；摆位按10mm/5度初筛；边界距离不是实体机械净空；未验证抓手TCP、棋子高度及连续抬升路径。'}
(OUT/'equal275-chess30-layout.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
