"""两个优先结构方案的同分辨率CAD工作区对比及单肘型摆位初筛。"""
import json,math,time,site
from pathlib import Path
import numpy as np
from compare_cad_variants import Factory,OUT,ROOT,get,DOC
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
from shapely.geometry import Polygon,mapping
from shapely.ops import unary_union
from shapely import contains_xy

def make_field(angles,mask,l1,l2,branch=None):
    a,b=np.meshgrid(np.deg2rad(angles),np.deg2rad(angles),indexing='ij')
    x=-l1*np.sin(a)-l2*np.sin(a+b);y=l1*np.cos(a)+l2*np.cos(a+b)
    cells=[]
    for i in range(len(angles)-1):
        for j in range(len(angles)-1):
            if branch=='positive' and angles[j]<0:continue
            if np.all(mask[i:i+2,j:j+2]):
                p=Polygon([(x[i,j],y[i,j]),(x[i+1,j],y[i+1,j]),(x[i+1,j+1],y[i+1,j+1]),(x[i,j+1],y[i,j+1])])
                cells.append(p if p.is_valid else p.buffer(0))
    return unary_union(cells)

def layout_search(field,label):
    pts=np.array([(x,y) for y in np.linspace(-160,160,9) for x in np.linspace(-180,180,10)])
    # 额外检查所有网格边的中点，共251个位置。
    mids=[(x,y) for y in range(-160,161,40) for x in range(-160,161,40)]
    mids +=[(x,y) for y in range(-140,141,40) for x in range(-180,181,40)]
    pts=np.vstack([pts,mids])
    centers=np.array([(x,y) for y in range(80,301,5) for x in range(-310,-79,5)])
    full=[];best_count=0;best=None
    for angle in range(20,86,2):
        a=math.radians(angle);rot=np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])
        p=centers[:,None,:]+pts@rot
        counts=contains_xy(field,p[:,:,0],p[:,:,1]).sum(axis=1)
        i=int(np.argmax(counts))
        if counts[i]>best_count:best_count=int(counts[i]);best={'center_mm':centers[i].tolist(),'angle_deg':angle}
        for i in np.flatnonzero(counts==len(pts)):
            # 对4个角围成的整个棋盘矩形计算到工作区边界的距离。
            rect=Polygon(centers[i]+np.array([[-180,-160],[180,-160],[180,160],[-180,160]])@rot)
            clearance=rect.distance(field.boundary) if field.contains(rect) else -1
            full.append({'center_mm':centers[i].tolist(),'angle_deg':angle,'sampled_field_boundary_clearance_mm':float(clearance)})
    full.sort(key=lambda x:x['sampled_field_boundary_clearance_mm'],reverse=True)
    result={'label':label,'branch':'q2 positive','checked_points_per_pose':len(pts),'best_count':best_count,'best_any':best,'full_layouts_count':len(full),'top_layouts':full[:20],
        'scope':'基于8度构型小格插值图的摆位筛选；尚须原始SolidWorks实体回验，边界距离不是机械净空。'}
    (OUT/f'{label}-single-branch-layout-screen.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('LAYOUT',json.dumps(result['top_layouts'][:2]),'max',best_count,flush=True)

def scan(scene):
    start=time.time();angles=np.arange(-176,177,8);mask=np.zeros((len(angles),len(angles)),bool)
    for i,a in enumerate(angles):
        for j,b in enumerate(angles):mask[i,j]=scene.pose(float(a),float(b))['free']
        if i%8==0:print(scene.label,'q1',a,'seconds',round(time.time()-start),'checks',scene.calls,flush=True)
    connected=np.zeros_like(mask);stack=[(len(angles)//2,len(angles)//2)]
    while stack:
        i,j=stack.pop()
        if i<0 or i>=len(angles) or connected[i,j] or not mask[i,j]:continue
        connected[i,j]=True;stack.extend([(i-1,j),(i+1,j),(i,(j-1)%len(angles)),(i,(j+1)%len(angles))])
    np.savez_compressed(OUT/f'{scene.label}-field.npz',angles=angles,good=mask,connected=connected,l1=scene.l1,l2=scene.l2)
    field=make_field(angles,connected,scene.l1,scene.l2)
    branch=make_field(angles,connected,scene.l1,scene.l2,'positive')
    for suffix,geom in [('field',field),('positive-branch',branch)]:
        (OUT/f'{scene.label}-{suffix}.geojson').write_text(json.dumps(mapping(geom)),encoding='utf-8')
    result={'label':scene.label,'step_deg':8,'grid_configurations':int(mask.size),'no_collision_samples':int(mask.sum()),'connected_samples':int(connected.sum()),
        'field_area_mm2':field.area,'positive_branch_area_mm2':branch.area,'body_checks':scene.calls,'elapsed_seconds':time.time()-start,'geometry':scene.details}
    (OUT/f'{scene.label}-field-summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('FIELD',scene.label,field.area,flush=True);layout_search(branch,scene.label)

if __name__=='__main__':
    import sys;sys.stdout.reconfigure(encoding='utf-8')
    base=np.load(ROOT/'output/constrained-workspace-2026-10-09/cad-collision-scan.npz')
    indices=np.arange(1,90,2);angles=base['angles'][indices];mask=base['connected'][np.ix_(indices,indices)]
    field=make_field(angles,mask,195,275)
    (OUT/'baseline-8deg-field.geojson').write_text(json.dumps(mapping(field)),encoding='utf-8')
    (OUT/'baseline-8deg-summary.json').write_text(json.dumps({'area_mm2':field.area,'step_deg':8,'source':'父聊天4度结果取8度共同格点'},ensure_ascii=False,indent=2),encoding='utf-8')
    f=Factory()
    for label,args in [('ratio_225_245',{'d1':30,'d2':-30}),('first_235_275',{'d1':40})]:scan(f.scene(label,**args))
    print('original save flag',f.before,get(DOC,'GetSaveFlag'),flush=True)
