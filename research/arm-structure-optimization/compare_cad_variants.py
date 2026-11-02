"""用原CAD的临时实体比较结构方案；所有输出只写本子聊天目录。"""
import sys,json,math,time,itertools
from pathlib import Path
import numpy as np
from temporary_geometry import ROOT,OUT,SW,DOC,COMPS,S,MATH,get,body,shift,resize_arm,variant,describe
sys.path.insert(0,str(ROOT/'research'))
from probe_cad_collision import clash,raw_copy_transform
AUD=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'))
GROUPS=json.loads((ROOT/'output/constrained-workspace-2026-10-09/rigid-groups.json').read_text('utf-8'))['groups']

def rz(deg):
    a=math.radians(deg);c,s=math.cos(a),math.sin(a)
    return np.array([[c,s,0],[-s,c,0],[0,0,1]])
def mt(rot,tr):return MATH.CreateTransform(variant([*rot.ravel(),*tr,1,0,0,0]))
def overlap(a,b):return np.all(a[:3]<b[3:]-1e-7) and np.all(b[:3]<a[3:]-1e-7)
def record(name,b,group):
    bb=np.array(get(b,'GetBodyBox'));corn=np.array(list(itertools.product(*zip(bb[:3],bb[3:]))))
    return {'name':name,'body':b,'group':group,'box':bb,'corners':corn}

class Factory:
    def __init__(self):
        self.before=get(DOC,'GetSaveFlag');self.records=[];self.excluded=set();self.initial=[]
        for c in AUD['components']:
            n=c['name']
            if c['doc_type'] is None or c['suppressed'] or '/' in n:continue
            for b in body(n):self.records.append(record(n,b,GROUPS[n]))
        allowed={frozenset([a,b]) for a,b in [('61mm光轴-2','FF丝杆支撑座[FF10]-2'),('61mm光轴-2','SKF_608-2ZZ-1'),
            ('61mm光轴-2','8x10x3mm轴套-1'),('DLE42-L2BJ-长款50-2','方键条销-1'),('DLE42-L2BJ-长款50-2','8mm键槽法兰-1')]}
        for i,a in enumerate(self.records):
            for j,b in enumerate(self.records[:i]):
                if a['group']==b['group'] or not overlap(a['box'],b['box']):continue
                result=clash(a['body'],b['body'])
                if result['bodies']:
                    if frozenset([a['name'],b['name']]) not in allowed:raise RuntimeError('unexpected baseline overlap')
                    self.excluded.add((i,j));self.initial.append([a['name'],b['name']])
        print('source bodies',len(self.records),'baseline exclusions',len(self.excluded),flush=True)

    def scene(self,label,d1=0,d2=0,forward=0,lower_bolts=0):
        results=[];details=[]
        for src in self.records:
            n=src['name'];b=src['body'];g=src['group'];dy=0;dz=0
            bb=src['box'];cy=S[1]*1000-(bb[1]+bb[4])*500
            if n=='J1机械臂-2' and d1:
                b=resize_arm(b,120,d1,[(-30,-15),(15,30)],bb[[2,5]]*1000)
                details.append({'part':n,'change_mm':d1,**describe(b)})
            elif n=='第二机械臂-1':
                if d2:b=resize_arm(b,330,d2,[(-30,-12.5),(12.5,30)],bb[[2,5]]*1000)
                dy+=d1;details.append({'part':n,'change_mm':d2,**describe(b)})
            elif g==1 and cy>140:dy+=d1
            elif g==2:dy+=d1+(d2 if cy>400 else 0)
            if forward:
                if n=='L型支架底座-1':
                    b=resize_arm(b,-20,forward,[(-30,30)],bb[[2,5]]*1000)
                    details.append({'part':n,'change_mm':forward,**describe(b)})
                if g>0 or n in ['8mm键槽法兰-1','方键条销-1'] or n in ['M3内六角小头x16mm-'+str(i) for i in range(5,9)]:dy+=forward
            if lower_bolts and n in ['M6内六角x22-1','M6内六角x22-5']:dz-=lower_bolts
            if dy or dz:b=shift(b,dy,dz)
            results.append(record(n,b,g))
        return Scene(label,results,self.excluded,195+d1,275+d2,forward,details)

class Scene:
    def __init__(self,label,records,excluded,l1,l2,forward,details):
        self.label=label;self.rs=records;self.l1=l1;self.l2=l2;self.details=details
        self.S=S+np.array([0,-forward/1000,0]);self.E=self.S+np.array([0,-l1/1000,0])
        self.corners=np.array([r['corners'] for r in records]);self.groups=np.array([r['group'] for r in records])
        self.calls=0;self.cache={};self.pairs={k:[] for k in ['first','second','base']}
        for i,a in enumerate(records):
            for j,b in enumerate(records[:i]):
                if a['group']==b['group'] or (i,j) in excluded:continue
                if min(a['box'][5],b['box'][5])-max(a['box'][2],b['box'][2])<=1e-7:continue
                typ={0,1} if {a['group'],b['group']}=={0,1} else {1,2} if {a['group'],b['group']}=={1,2} else {0,2}
                self.pairs['first' if typ=={0,1} else 'second' if typ=={1,2} else 'base'].append((i,j))
        self.pairs={k:np.array(v,dtype=int) for k,v in self.pairs.items()}

    def check_set(self,kind,a,b):
        # 第一臂-支架只依赖a；第二臂-第一臂只依赖b，消去共同刚体旋转。
        if kind=='first':b=0
        if kind=='second':a=0
        key=(kind,round(a,8),round(b,8))
        if key in self.cache:return self.cache[key]
        ra,rb=rz(a),rz(b);rots={0:np.eye(3),1:ra,2:rb@ra}
        trs={0:np.zeros(3),1:self.S-self.S@ra,2:(self.E-self.E@rb-self.S)@ra+self.S}
        p=self.corners.copy()
        for g in (1,2):p[self.groups==g]=p[self.groups==g]@rots[g]+trs[g]
        lo=p.min(axis=1);hi=p.max(axis=1);idx=self.pairs[kind]
        mask=np.all(lo[idx[:,0]]<hi[idx[:,1]]-1e-7,axis=1)&np.all(lo[idx[:,1]]<hi[idx[:,0]]-1e-7,axis=1)
        bodies={};transforms={};answer={'free':True}
        for i,j in idx[mask]:
            for k in (i,j):
                if k in bodies:continue
                g=self.rs[k]['group']
                if not g or (a==0 and (g==1 or b==0)):bodies[k]=self.rs[k]['body']._oleobj_
                else:
                    if g not in transforms:transforms[g]=mt(rots[g],trs[g])
                    bodies[k]=raw_copy_transform(self.rs[k]['body'],transforms[g])
            self.calls+=1
            if clash(bodies[i],bodies[j])['bodies']:
                answer={'free':False,'collision':[self.rs[i]['name'],self.rs[j]['name']]};break
        self.cache[key]=answer;return answer

    def pose(self,a,b):
        for kind in ['first','second','base']:
            answer=self.check_set(kind,a,b)
            if not answer['free']:return answer
        return {'free':True}

    def point(self,x,y):
        c=(x*x+y*y-self.l1**2-self.l2**2)/(2*self.l1*self.l2)
        if abs(c)>1:return {'x':x,'y':y,'free':False,'solutions':[],'reason':'radial reach'}
        solutions=[]
        for sign in [-1,1]:
            b=sign*math.acos(c);a=math.atan2(y,x)-math.atan2(self.l2*math.sin(b),self.l1+self.l2*c)-math.pi/2
            a=(math.degrees(a)+180)%360-180;b=math.degrees(b)
            solutions.append((a,b))
        results=[]
        for a,b in sorted(solutions,key=lambda s:abs(s[0])):
            result={'q1':a,'q2':b,**self.pose(a,b)};results.append(result)
            if result['free']:break
        return {'x':float(x),'y':float(y),'free':any(s['free'] for s in results),'solutions':results}

    def board(self,gap,critical_only=False):
        pts=[(-60,gap),(-20,gap),(0,gap),(20,gap),(60,gap),(-20,gap+40),(20,gap+40),(-180,gap+320),(180,gap+320)] if critical_only else [(x,y) for y in range(gap,gap+321,40) for x in range(-180,181,40)]
        start=time.time();results=[]
        for idx,(x,y) in enumerate(pts):
            results.append(self.point(x,y))
            if idx%10==9:print(self.label,'gap',gap,'points',idx+1,'free',sum(p['free'] for p in results),'seconds',round(time.time()-start),flush=True)
        result={'label':self.label,'links_mm':[self.l1,self.l2],'gap_mm':gap,'critical_only':critical_only,
            'tested_points':len(results),'free_points':sum(p['free'] for p in results),'points':results,'body_pair_checks':self.calls,'modified_geometry':self.details,
            'scope':'端点实体碰撞检验，保留父分析的5对连接交叠排除及抑制件/抓手缺失等边界；不等于连续轨迹或实机验收。'}
        name=f'{self.label}-gap-{gap}'+('-critical' if critical_only else '')+'.json'
        (OUT/name).write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print('RESULT',self.label,gap,result['free_points'],'/',len(results),flush=True)
        return result

if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8');f=Factory()
    if '--critical' in sys.argv:
        for args in [('baseline',{}),('bolt_recess_2mm',{'lower_bolts':2}),('ratio_215_255',{'d1':20,'d2':-20}),('ratio_225_245',{'d1':30,'d2':-30}),('first_235_275',{'d1':40}),('shoulder_forward_50',{'forward':50})]:
            sc=f.scene(args[0],**args[1]);sc.board(136 if args[0]=='first_235_275' else 100,True)
    else:
        for label,args,gap in [('ratio_215_255',{'d1':20,'d2':-20},100),('ratio_225_245',{'d1':30,'d2':-30},100),('first_235_275',{'d1':40},136)]:
            sc=f.scene(label,**args);sc.board(gap)
    (OUT/'source-preservation.json').write_text(json.dumps({'save_before':f.before,'save_after':get(DOC,'GetSaveFlag'),'baseline_connections':f.initial,'source_document':get(DOC,'GetPathName')},ensure_ascii=False,indent=2),encoding='utf-8')
