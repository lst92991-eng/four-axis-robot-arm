"""将单肘型摆位初筛结果回到SolidWorks原始实体，按5mm采样检查网格连线。"""
import json,math,time
import numpy as np
from compare_cad_variants import Factory,OUT,get,DOC

def selected(layout):
    return layout['top_layouts'][0]

def run(scene,placement):
    center=np.array(placement['center_mm']);angle=placement['angle_deg'];a=math.radians(angle)
    rot=np.array([[math.cos(a),math.sin(a)],[-math.sin(a),math.cos(a)]])
    vertices=[(x,y) for y in range(-160,161,40) for x in range(-180,181,40)]
    route=set(vertices)
    for y in range(-160,161,40):route.update((x,y) for x in range(-180,181,5))
    for x in range(-180,181,40):route.update((x,y) for y in range(-160,161,5))
    records=[];start=time.time()
    for i,p in enumerate(sorted(route,key=lambda p:(p[1],p[0]))):
        xy=center+np.array(p)@rot;x,y=xy
        c=(x*x+y*y-scene.l1**2-scene.l2**2)/(2*scene.l1*scene.l2)
        if abs(c)>1:result={'free':False,'reason':'radial reach'}
        else:
            q2=math.acos(c);q1=math.atan2(y,x)-math.atan2(scene.l2*math.sin(q2),scene.l1+scene.l2*c)-math.pi/2
            q1=(math.degrees(q1)+180)%360-180;q2=math.degrees(q2)
            result={'q1':q1,'q2':q2,**scene.pose(q1,q2)}
        records.append({'board_mm':p,'robot_mm':xy.tolist(),'vertex':p in vertices,**result})
        if i%150==149:print(scene.label,'route points',i+1,'failures',sum(not r['free'] for r in records),'seconds',round(time.time()-start),flush=True)
    good=[r for r in records if r['free']]
    result={'label':scene.label,'links_mm':[scene.l1,scene.l2],'placement':placement,'branch':'q2 positive only',
        'route_sampling_mm':5,'sampled_points':len(records),'free_points':len(good),'vertex_points':90,
        'free_vertices':sum(r['free'] for r in records if r['vertex']),
        'q1_range_deg':[min(r['q1'] for r in good),max(r['q1'] for r in good)],'q2_range_deg':[min(r['q2'] for r in good),max(r['q2'] for r in good)],
        'elapsed_seconds':time.time()-start,'body_pair_checks':scene.calls,'points':records,
        'scope':'CAD临时实体、当前Z、同一正肘型、棋盘网格连线5mm离散检查；非连续碰撞认证，未加入棋子/吸盘/软管、动态挠度或硬件反馈。'}
    (OUT/f'{scene.label}-single-branch-native-validation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print('VERIFIED',scene.label,len(good),'/',len(records),flush=True)

if __name__=='__main__':
    import sys;sys.stdout.reconfigure(encoding='utf-8');f=Factory()
    for label,args in [('ratio_225_245',{'d1':30,'d2':-30}),('first_235_275',{'d1':40})]:
        layout=json.loads((OUT/f'{label}-single-branch-layout-screen.json').read_text('utf-8'))
        if not layout['top_layouts']:raise RuntimeError('No single-branch layout found: '+label)
        run(f.scene(label,**args),selected(layout))
    print('original save flag',f.before,get(DOC,'GetSaveFlag'),flush=True)
