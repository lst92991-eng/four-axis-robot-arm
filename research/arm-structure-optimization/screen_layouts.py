"""只用父聊天已有碰撞证据做基线摆位初筛，不把初筛当作新的CAD验收。"""
import json,site,math,time
from pathlib import Path
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
import numpy as np
from shapely.geometry import Polygon,mapping
from shapely.ops import unary_union
from shapely import contains_xy
ROOT=Path(__file__).resolve().parents[2];OUT=Path(__file__).resolve().parent
d=np.load(ROOT/'output/constrained-workspace-2026-10-09/cad-collision-scan.npz')
angles=d['angles'];g=d['connected'];a,b=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
x=-195*np.sin(a)-275*np.sin(a+b);y=195*np.cos(a)+275*np.cos(a+b)
cells=[]
for i in range(len(angles)-1):
    for j in range(len(angles)-1):
        if np.all(g[i:i+2,j:j+2]):
            p=Polygon([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])])
            if not p.is_valid:p=p.buffer(0)
            cells.append(p)
field=unary_union(cells)
pts=np.array([(u,v) for v in np.linspace(-160,160,9) for u in np.linspace(-180,180,10)])
centers=np.array([(xx,yy) for yy in range(-240,321,10) for xx in range(-280,281,10)])
best=[];maximum=0
for degree in range(0,180,5):
    aa=math.radians(degree);rot=np.array([[math.cos(aa),math.sin(aa)],[-math.sin(aa),math.cos(aa)]])
    world=centers[:,None,:]+pts@rot
    counts=contains_xy(field,world[:,:,0],world[:,:,1]).sum(axis=1)
    value=int(counts.max())
    if value>maximum:maximum=value;best=[]
    if value==maximum:
        for idx in np.flatnonzero(counts==value):best.append({'center_mm':centers[idx].tolist(),'rotation_deg':degree,'points_in_sampled_field':value})
result={'method':'父聊天4度构型图小格并集；摆位平移10mm、旋转5度初筛。绿色小格插值不是新的实体干涉检查。','grid_mm':[360,320],
    'max_points':maximum,'best_layouts':best,'searched_layouts':len(centers)*36}
(OUT/'baseline-layout-screen.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
(OUT/'baseline-field.geojson').write_text(json.dumps(mapping(field)),encoding='utf-8')
print(json.dumps({'max_points':maximum,'best_count':len(best),'examples':best[:12]},ensure_ascii=False))
