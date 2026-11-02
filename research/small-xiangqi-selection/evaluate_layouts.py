"""只读既有CAD碰撞采样，离线筛选棋盘；不导入COM、不控制SolidWorks。"""
import json, math, hashlib, site
from pathlib import Path
import numpy as np
site.addsitedir('C:/Users/lst/AppData/Local/Temp/scara-workspace-python')
import shapely
from shapely.geometry import box, Polygon
from shapely.ops import unary_union

ROOT=Path(__file__).resolve().parents[2]
OUT=Path(__file__).resolve().parent
SOURCE=ROOT/'output/constrained-workspace-2026-10-09/cad-collision-scan.npz'
z=np.load(SOURCE);angles=z['angles'];free=z['connected']
cells=[box(angles[i],angles[j],angles[i+1],angles[j+1])
       for i in range(len(angles)-1) for j in range(len(angles)-1)
       if free[i:i+2,j:j+2].all()]
support=unary_union(cells)
# 额外选型条件：避开q2=0伸直附近±10°；这不是现有控制器的软限位。
branches=[support.intersection(box(-180,-180,180,-10)),support.intersection(box(-180,10,180,180))]

def ik(x,y):
    c=(x*x+y*y-195**2-275**2)/(2*195*275)
    valid=(c>=-1)&(c<=1);base=np.arccos(np.clip(c,-1,1))
    out=[]
    for sg in (-1,1):
        b=sg*base
        a=np.arctan2(y,x)-np.arctan2(275*np.sin(b),195+275*np.cos(b))-math.pi/2
        a=(np.rad2deg(a)+180)%360-180;b=np.rad2deg(b)
        out.append((a,b,valid))
    return out

def grid(p,theta):
    pts=np.array([( (c-4)*p,(r-4.5)*p) for r in range(10) for c in range(9)])
    t=math.radians(theta);rot=np.array([[math.cos(t),math.sin(t)],[-math.sin(t),math.cos(t)]])
    return pts@rot

def score_centers(p,theta,centers):
    pts=grid(p,theta)[None,:,:]+centers[:,None,:]
    scores=[];counts=[]
    for shape,(a,b,valid) in zip(branches,ik(pts[:,:,0],pts[:,:,1])):
        qs=shapely.points(a,b);ok=shapely.covers(shape,qs)&valid
        d=shapely.distance(qs,shape.boundary)
        scores.append(np.where(ok,d,-1).min(axis=1));counts.append(ok.sum(axis=1))
    scores=np.array(scores);counts=np.array(counts)
    best=scores.max(axis=0);branch=scores.argmax(axis=0)
    return best,branch,counts

def assess(p,theta,center,sign_index):
    pts=grid(p,theta)+center
    sols=ik(pts[:,0],pts[:,1]);rows=[]
    for i,(x,y) in enumerate(pts):
        ss=[]
        for shape,(a,b,valid) in zip(branches,sols):
            q=shapely.Point(a[i],b[i]);ok=bool(valid[i] and shape.covers(q))
            ss.append({'q1_deg':float(a[i]),'q2_deg':float(b[i]),'sample_supported':ok,
                       'angular_distance_to_support_boundary_deg':float(q.distance(shape.boundary)) if ok else None})
        rows.append({'row':i//9,'col':i%9,'x_mm':float(x),'y_mm':float(y),'branches':ss})
    shape=branches[sign_index];a,b,valid=sols[sign_index];qs=shapely.points(a,b)
    count=int((shapely.covers(shape,qs)&valid).sum())
    return {'pitch_mm':p,'grid_span_mm':[8*p,9*p],'theta_deg':theta,'center_xy_mm':list(center),
        'axis_bounds_mm':[pts.min(axis=0).tolist(),pts.max(axis=0).tolist()],
        'selected_elbow_branch':[-1,1][sign_index],
        'single_branch_supported_points':count,
        'min_angular_support_margin_deg':float(shapely.distance(qs,shape.boundary).min()) if count==90 else None,
        'radial_range_mm':[float(np.linalg.norm(pts,axis=1).min()),float(np.linalg.norm(pts,axis=1).max())],
        'q1_range_deg':[float(a.min()),float(a.max())],'q2_range_deg':[float(b.min()),float(b.max())],
        'points':rows}

def run():
    coarse=np.array([(x,y) for x in range(0,361,10) for y in range(0,391,10)])
    results=[]
    for p in [25,28,30,32,35,40]:
        best=None
        for theta in [0,30,45,60,90]:
            score,br,cnt=score_centers(p,theta,coarse)
            k=int(score.argmax());cand=(float(score[k]),theta,coarse[k],int(br[k]))
            if best is None or cand[0]>best[0]:best=cand
        m,theta,cen,br=best
        if m<0:
            results.append({'pitch_mm':p,'grid_span_mm':[8*p,9*p], 'found_90_point_single_branch_layout':False,
                'scope':'仅所列搜索范围与角度未找到，不是所有位置不可行的证明。'})
            print(p,'no 90-point single-branch layout found in this search',flush=True)
            continue
        refined=np.array([(x,y) for x in range(max(0,int(cen[0])-12),int(cen[0])+13,2)
            for y in range(max(0,int(cen[1])-12),int(cen[1])+13,2)])
        score,bb,_=score_centers(p,theta,refined);k=int(score.argmax())
        result=assess(p,theta,refined[k].tolist(),int(bb[k]))
        result['optimization_scope']='右侧及正前方有限网格搜索；中心10mm粗搜+2mm局部细搜，角度0/30/45/60/90；非全局最优证明。'
        results.append(result)
        print(p,result['theta_deg'],result['center_xy_mm'],result['single_branch_supported_points'],round(result['min_angular_support_margin_deg'],2),flush=True)
    # 易于量尺摆放的整值方案，与搜索结果分别保留。
    practical=[]
    for p in [28,30,32]:
        for center in ([290,50],[290,60],[300,40],[0,310]):
            theta=0 if center[0] else 90
            score,bb,_=score_centers(p,theta,np.array([center]))
            practical.append(assess(p,theta,center,int(bb[0])))
    output={'source':str(SOURCE),'source_sha256':hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
        'method':'每个交点解析IK；同一肘分支的IK须落入既有CAD四角均无碰撞采样单元。配置空间角度距边界是选型指标，不是实物毫米净空。另避开伸直±10度。',
        'limits':'无新增CAD碰撞调用；没有验证样本间连续碰撞、实际Z/TCP、工具/棋子/相机架、商品实测或真实抓取；选型初筛不能称整盘实机可抓。',
        'optimized':results,'practical':practical}
    (OUT/'layout-evaluation.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    print('PRACTICAL')
    for r in practical:print(r['pitch_mm'],r['center_xy_mm'],r['single_branch_supported_points'],r['min_angular_support_margin_deg'],flush=True)

if __name__=='__main__':run()
