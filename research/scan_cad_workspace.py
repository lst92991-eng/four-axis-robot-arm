"""SolidWorks临时实体碰撞扫描。当前Z、无工具，4度构型采样。"""
import sys,json,math,time,itertools
from pathlib import Path
import numpy as np
from probe_cad_collision import SW,DOC,COMPS,S,E,MATH,variant,get,body,moved,transform,clash,raw_copy_transform
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'output/constrained-workspace-2026-10-09'
AUD=json.loads((OUT/'constraint-audit.json').read_text('utf-8'))

def groups():
    g={}
    seeds={
      1:['J1机械臂-2','DLE42-L2BJ-长款50-2','42步进电机L60-2','42步进电机L60-3',
         'FF丝杆支撑座[FF10]-2','轴承座8mm内径-1','SKF_608-2ZZ-1','8x10x3mm轴套-1'],
      2:['第二机械臂-1','42步进电机L40-1','内径8mm法兰-1','61mm光轴-2','光轴固定环-1']}
    for group,names in seeds.items():
        for n in names:g[n]=group
    for c in AUD['components']:
        n=c['name']
        if n in g or c['doc_type'] is None or '/' in n:continue
        if not any(v in n for v in ['螺','M3','M4','M5','M6','M8','垫','单头六角铜柱']):g[n]=0
    # 紧固件按配合对象所属刚体归组，跨关节的轴承已显式分组。
    links={n:[] for n in COMPS}
    for m in AUD['documents'][0]['mates']:
        ns=set(e['component'] for e in m.get('entities',[]) if e['component'])
        for a,b in itertools.combinations(ns,2):
            if a in links and b in links:links[a].append(b);links[b].append(a)
    for _ in range(8):
        pending={}
        for n in links:
            if n in g or '/' in n:continue
            votes=[g[x] for x in links[n] if x in g]
            if votes:pending[n]=max(set(votes),key=votes.count)
        g.update(pending)
    # 无独立配合的阵列紧固件：依据本次读出的装配位置与对应母件归组。
    for c in AUD['components']:
        n=c['name']
        if n in g or c['doc_type'] is None or '/' in n:continue
        bb=np.array(c['box'])*1000;cy=(bb[1]+bb[4])/2;cz=(bb[2]+bb[5])/2
        if n.startswith('M4内六角x16mm-') and cy < -300:g[n]=2
        elif n.startswith('M3内六角x12mm-') and cy < -300:g[n]=1
        elif n.startswith('M3内六角小头x16mm-'):g[n]=1 if cz>1280 else 0
        elif n.startswith('M3x4mm紧固螺丝-'):g[n]=2 if cy < -300 else 1
        elif cy > -120:g[n]=0
    return g

def rotation(a):
    t=math.radians(a);c=math.cos(t);s=math.sin(t)
    return np.array([[c,s],[-s,c]])
def box_at(record,q1=0,q2=0):
    p=record['corners'].copy()
    if record['group']==2:p=(p-E[:2])@rotation(q2)+E[:2]
    if record['group']>0:p=(p-S[:2])@rotation(q1)+S[:2]
    return np.r_[p.min(axis=0),p.max(axis=0)]
def overlap(a,b):return np.all(a[:2]<b[2:]-1e-7) and np.all(b[:2]<a[2:]-1e-7)
def temp_at(record,q1=0,q2=0):
    cp=get(record['body'],'Copy')
    if record['group']==2 and q2:cp.ApplyTransform(transform(q2,E))
    if record['group']>0 and q1:cp.ApplyTransform(transform(q1,S))
    return cp

def scan():
    start=time.time();g=groups();records=[];unassigned=[]
    for c in AUD['components']:
        n=c['name']
        if c['doc_type'] is None or c['suppressed'] or '/' in n:continue
        if n not in g:unassigned.append(n);continue
        for k,b in enumerate(body(n)):
            bb=np.array(get(b,'GetBodyBox'))
            corners=np.array(list(itertools.product([bb[0],bb[3]],[bb[1],bb[4]])))
            records.append({'name':n,'body':b,'group':g[n],'box':bb,'corners':corners})
    (OUT/'rigid-groups.json').write_text(json.dumps({'groups':g,'unassigned':unassigned,'excluded_suppressed_parent':'装配体1^SCARA四轴机械臂总成-1','scope':'仅裸CAD刚体，末端R保持随第二臂的当前姿态；未加入吸盘软管。'},ensure_ascii=False,indent=2),encoding='utf-8')
    if unassigned:raise RuntimeError('unassigned: '+str(unassigned))
    pairs=[];baseline=[]
    for i,a in enumerate(records):
        for j,b in enumerate(records[:i]):
            if a['group']==b['group']:continue
            if min(a['box'][5],b['box'][5])-max(a['box'][2],b['box'][2])<=1e-7:continue
            if overlap(box_at(a),box_at(b)):
                hit=clash(temp_at(a),temp_at(b))
                if hit['bodies']:
                    baseline.append({'a':a['name'],'b':b['name'],'groups':[a['group'],b['group']],**hit})
                    continue
            pairs.append((i,j))
    (OUT/'baseline-interference.json').write_text(json.dumps(baseline,ensure_ascii=False,indent=2),encoding='utf-8')
    print('bodies',len(records),'z-overlap pairs',len(pairs),'baseline overlaps',len(baseline),flush=True)
    print(json.dumps(baseline,ensure_ascii=False),flush=True)
    # 先暂停在基准交叠审查；不可把所有已有干涉自动当成合法关节接触。
    if '--run' not in sys.argv:return
    allow_names={frozenset(['DLE42-L2BJ-长款50-2','8mm键槽法兰-1']),frozenset(['DLE42-L2BJ-长款50-2','方键条销-1']),frozenset(['61mm光轴-2','SKF_608-2ZZ-1']),frozenset(['61mm光轴-2','FF丝杆支撑座[FF10]-2']),frozenset(['61mm光轴-2','8x10x3mm轴套-1'])}
    unexpected=[b for b in baseline if frozenset([b['a'],b['b']]) not in allow_names]
    if unexpected:raise RuntimeError('须人工分析基准相交: '+str(unexpected))
    step=4
    angles=np.arange(-180,181,step)
    fixed=[box_at(a) for a in records]
    q1_pairs=[(i,j) for i,j in pairs if set([records[i]['group'],records[j]['group']])=={0,1}]
    q2_pairs=[(i,j) for i,j in pairs if set([records[i]['group'],records[j]['group']])=={1,2}]
    both_pairs=[(i,j) for i,j in pairs if set([records[i]['group'],records[j]['group']])=={0,2}]
    hits={};calls=0
    corners=np.array([a['corners'] for a in records]);group_arr=np.array([a['group'] for a in records])
    pair_arrays={}
    def check(ps,q1,q2):
        nonlocal calls
        key=id(ps)
        if key not in pair_arrays:pair_arrays[key]=np.array(ps,dtype=int)
        indices=pair_arrays[key];p=corners.copy()
        if q2:p[group_arr==2]=(p[group_arr==2]-E[:2])@rotation(q2)+E[:2]
        if q1:p[group_arr>0]=(p[group_arr>0]-S[:2])@rotation(q1)+S[:2]
        lo=p.min(axis=1);hi=p.max(axis=1)
        candidates=np.all(lo[indices[:,0]]<hi[indices[:,1]]-1e-7,axis=1)&np.all(lo[indices[:,1]]<hi[indices[:,0]]-1e-7,axis=1)
        bodies={};transforms={}
        for i,j in indices[candidates]:
            for k in (i,j):
                if k not in bodies:
                    grp=records[k]['group']
                    if grp==0 or (q1==0 and (grp==1 or q2==0)):
                        bodies[k]=records[k]['body']._oleobj_
                    else:
                        if grp not in transforms:
                            if grp==1:transforms[grp]=transform(q1,S)
                            else:
                                ra=np.eye(3);ra[:2,:2]=rotation(q1)
                                rb=np.eye(3);rb[:2,:2]=rotation(q2)
                                rot=rb@ra;tr=(E-E@rb-S)@ra+S
                                transforms[grp]=MATH.CreateTransform(variant([*rot.ravel(),*tr,1,0,0,0]))
                        bodies[k]=raw_copy_transform(records[k]['body'],transforms[grp])
            calls+=1
            if clash(bodies[i],bodies[j])['bodies']:
                key=records[i]['name']+' / '+records[j]['name'];hits[key]=hits.get(key,0)+1
                return False,key
        return True,None
    if '--z-remaining' in sys.argv:
        zproof=json.loads((OUT/'z-projection-proof.json').read_text('utf-8'))
        details=[]
        for case in zproof['unresolved']:
            a,b=case['q1'],case['q2'];ok,hit=check(both_pairs,a,b)
            details.append({**case,'current_collision_pair':hit})
        column=[]
        for record in records:
            if record['name']!='40x40x500-1':continue
            for face in get(record['body'],'GetFaces') or []:
                surface=get(face,'GetSurface');bb=list(get(face,'GetBox'))
                info={'z_box_mm':[bb[2]*1000,bb[5]*1000]}
                if get(surface,'IsPlane'):
                    values=list(surface.PlaneParams);info.update(kind='plane',params=values,vertical=abs(values[2])<1e-8)
                elif get(surface,'IsCylinder'):
                    values=list(surface.CylinderParams);info.update(kind='cylinder',params=values,vertical=abs(abs(values[5])-1)<1e-8)
                else:info.update(kind='other',vertical=False)
                column.append(info)
        nonvertical=[f for f in column if not f['vertical']]
        report={'cases':details,'column_faces':column,'nonvertical_faces':nonvertical,'z_shift_mm':[-223.19615686196482,104.30384313701269],
            'moving_component_z_intervals_mm':{c['name']:[c['box'][2]*1000,c['box'][5]*1000] for c in AUD['components'] if c['name'] in ['第二机械臂-1','42步进电机L40-1']}}
        (OUT/'z-remaining-evidence.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='column_faces'},ensure_ascii=False),flush=True)
        return
    if '--z-proof' in sys.argv:
        # 投影存在性验证：当前高度无碰撞的样本已经证明“至少一个Z可达”。
        # 若剩余样本均被随Z一起升降的支架挡住，则升降不能新增这些XY构型。
        cached=np.load(OUT/'cad-collision-scan.npz');angles=cached['angles']
        carriage={'L型支架底座-1','L型支架竖板-1','铝型材挤压角座-1','铝型材挤压角座-2'}
        carriage.update(['MGW12H滑块-1','T型丝杆螺母-1','8mm键槽法兰-1','方键条销-1'])
        carriage.update(n for n in g if n.startswith('M6内六角x22-'))
        carriage.update('M4内六角x16mm-'+str(i) for i in [15,19,20,21])
        carriage.update('M3内六角小头x16mm-'+str(i) for i in [5,6,7,8])
        carriage.update('M3内六角x12mm-'+str(i) for i in [5,6,7,8])
        def involves_carriage(pair):
            return any(records[k]['group']==0 and records[k]['name'] in carriage for k in pair)
        only_carriage=[p for p in both_pairs if involves_carriage(p)]
        results=[]
        for ai,a in enumerate(angles):
            if not cached['ok1'][ai]:continue
            for bi,b in enumerate(angles):
                if cached['good'][ai,bi]:continue
                free,hit=check(only_carriage,a,b)
                results.append({'q1':int(a),'q2':int(b),'blocked_by_z_carriage':not free,'pair':hit})
            if ai%10==0:print('Z-invariance q1',int(a),'tested',len(results),flush=True)
        unresolved=[p for p in results if not p['blocked_by_z_carriage']]
        summary={'tested_rejected_within_q1_component':len(results),'blocked_at_all_z_by_carriage':len(results)-len(unresolved),'unresolved':unresolved,
            'scope':'仅4度构型格点的XY投影存在性；不表示所有绿色点在所有Z均无碰撞。J1可达连通分量由随Z升降的M6螺钉接触屏障约±97.74度约束。',
            'all_z_projection_same_on_sample_grid':not unresolved,'evidence':results}
        (OUT/'z-projection-proof.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in summary.items() if k!='evidence'},ensure_ascii=False),flush=True)
        return
    if '--probe' in sys.argv:
        def pose(a,b):
            for ps in (q1_pairs,q2_pairs,both_pairs):
                good,hit=check(ps,a,b)
                if not good:return {'free':False,'pair':hit}
            return {'free':True}
        details={'q1_boundary':[],'poses':[],'board_points':[]}
        for sign in [-1,1]:
            lo,hi=90.0,105.0
            for _ in range(11):
                mid=(lo+hi)/2
                if check(q1_pairs,sign*mid,0)[0]:lo=mid
                else:hi=mid
            details['q1_boundary'].append({'sign':sign,'clear_deg':sign*lo,'collision_deg':sign*hi,'collision_pair':check(q1_pairs,sign*hi,0)[1]})
        for a,b in [(0,0),(0,150),(0,160),(0,170),(96,-140),(-96,140),(0,180)]:
            details['poses'].append({'q1':a,'q2':b,**pose(a,b)})
        for y in range(96,417,40):
            for x in range(-180,181,40):
                cs=(x*x+y*y-195**2-275**2)/(2*195*275);sol=[]
                for sign in (-1,1):
                    b=sign*math.acos(cs);a=math.atan2(y,x)-math.atan2(275*math.sin(b),195+275*math.cos(b))-math.pi/2
                    a=(math.degrees(a)+180)%360-180;b=math.degrees(b)
                    sol.append({'q1':a,'q2':b,**pose(a,b)})
                details['board_points'].append({'x':x,'y':y,'solutions':sol,'free':any(s['free'] for s in sol)})
        details['board_clear_count']=sum(p['free'] for p in details['board_points'])
        details['body_pair_checks']=calls
        (OUT/'boundary-and-board-probes.json').write_text(json.dumps(details,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in details.items() if k!='board_points'},ensure_ascii=False),flush=True)
        return
    ok1=np.array([check(q1_pairs,a,0)[0] for a in angles])
    ok2=np.array([check(q2_pairs,0,a)[0] for a in angles])
    print('independent collision-free: J1',angles[ok1].tolist(),'J2',angles[ok2].tolist(),flush=True)
    good=np.zeros((len(angles),len(angles)),bool)
    for ai,a in enumerate(angles):
        if ok1[ai]:
            for bi,b in enumerate(angles):
                if ok2[bi]:good[ai,bi]=check(both_pairs,a,b)[0]
        if ai%5==0:
            np.savez_compressed(OUT/'scan-progress.npz',angles=angles,good=good,rows_completed=ai+1)
            print('q1',int(a),'elapsed',round(time.time()-start),'calls',calls,flush=True)
    # 在构型空间取与当前伸直姿态相连的分量。相邻样本之间尚非连续碰撞证明。
    center=len(angles)//2;visited=np.zeros_like(good);stack=[(center,center)]
    while stack:
        i,j=stack.pop()
        if not (0<=i<len(angles) and 0<=j<len(angles)) or visited[i,j] or not good[i,j]:continue
        visited[i,j]=True;stack.extend([(i-1,j),(i+1,j),(i,(j-1)%len(angles)),(i,(j+1)%len(angles))])
    np.savez_compressed(OUT/'cad-collision-scan.npz',angles=angles,good=good,connected=visited,ok1=ok1,ok2=ok2)
    for record in records:
        if not np.allclose(get(record['body'],'GetBodyBox'),record['box'],atol=1e-10,rtol=0):
            raise RuntimeError('干涉检测改变了临时基准实体，扫描不能采信')
    summary={'angular_step_deg':step,'tested_configuration_grid':int(good.size),'no_collision_samples':int(good.sum()),'connected_samples':int(visited.sum()),'exact_body_pair_checks':calls,'elapsed_seconds':time.time()-start,'hits':hits,'excluded_baseline_pairs':baseline,'scope':'当前CAD Z高度，裸CAD，推导刚体分组，不执行装配求解器；4度采样，非连续碰撞认证。','save_after':get(DOC,'GetSaveFlag')}
    (OUT/'scan-summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(summary,ensure_ascii=False),flush=True)

if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');scan()
