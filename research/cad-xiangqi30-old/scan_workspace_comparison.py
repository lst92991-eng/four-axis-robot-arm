"""新旧臂×两种螺丝头的同范围扫描；临时BRep运算，不修改或保存CAD。"""
import sys,json,math,itertools,ast,time,gc
from pathlib import Path
import numpy as np
import win32com.client,pythoncom
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'output/workspace-comparison-2026-10-13';OUT.mkdir(parents=True,exist_ok=True)
sys.path.insert(0,'C:/Users/lst/.codex/tools/solidworks-automation-mcp/scripts')
from sw_connect import get_com_member as get,open_document
SW=win32com.client.GetActiveObject('SldWorks.Application')
SW=win32com.client.dynamic.DumbDispatch(SW._oleobj_,'SldWorks.Application')
MATH=get(SW,'GetMathUtility');MATH._FlagAsMethod('CreateTransform')
MODELER=get(SW,'GetModeler');MODELER._FlagAsMethod('CreateBodyFromBox3')
S=np.array([.0005081230034509875,-.15521586575797696,0.])
GROUPS=json.loads((ROOT/'output/constrained-workspace-2026-10-09/rigid-groups.json').read_text('utf-8'))['groups']
RENAMED={r['new_name']:r['group'] for r in json.loads((ROOT/'hardware/arm_equal_275/v1/analysis/pattern-instance-mapping.json').read_text('utf-8'))}
ORIGINAL=json.loads((ROOT/'output/constrained-workspace-2026-10-09/constraint-audit.json').read_text('utf-8'))['components']
def variant(values):return win32com.client.VARIANT(pythoncom.VT_ARRAY|pythoncom.VT_R8,list(values))
def mt(rot,tr):return MATH.CreateTransform(variant([*rot.ravel(),*tr,1,0,0,0]))
_ids={}
def raw_copy_transform(b,t=None):
    raw=b._oleobj_ if hasattr(b,'_oleobj_') else b
    if not _ids:_ids.update({n:raw.GetIDsOfNames(n) for n in ('Copy','ApplyTransform')})
    cp=raw.InvokeTypes(_ids['Copy'],0,1,(9,0),())
    if t is not None:assert cp.InvokeTypes(_ids['ApplyTransform'],0,1,(11,0),((9,1),),t._oleobj_)
    return cp
def clash(a,b):
    args=[win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_VARIANT,None) for _ in range(3)]
    MODELER.CheckInterference(a,b,False,*args)
    return {'bodies':len(args[2].value) if args[2].value else 0}
tree=ast.parse((ROOT/'research/arm-structure-optimization/compare_cad_variants.py').read_text('utf-8'))
nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in ('rz','overlap','record','Scene')]
exec(compile(ast.Module(body=nodes,type_ignores=[]),'shared_collision_scene','exec'),globals())
allowed={frozenset(p) for p in [('61mm光轴-2','FF丝杆支撑座[FF10]-2'),('61mm光轴-2','SKF_608-2ZZ-1'),('61mm光轴-2','8x10x3mm轴套-1'),('DLE42-L2BJ-长款50-2','方键条销-1'),('DLE42-L2BJ-长款50-2','8mm键槽法兰-1')]}
def load_records(path,prefix,new):
    doc=open_document(SW,str(path),read_only=True,silent=True,raise_on_error=True)
    before=bool(get(doc,'GetSaveFlag'));rows=[];partnames=[];renamed=[]
    for c in get(doc,'GetComponents',False):
        name=get(c,'Name2').replace(prefix,'')
        # 两版统一不纳入同步轮/皮带子总成；不混用历史两种分析口径。
        if '/' in name or bool(get(c,'IsSuppressed')):continue
        p=get(c,'GetModelDoc2')
        if p is None:
            c.SetSuppression2(2);p=get(c,'GetModelDoc2')
        if p is None or get(p,'GetType')!=1:continue
        if name in GROUPS:group=GROUPS[name]
        else:
            # 原生阵列保存后会重新编号；按零件和实际位置核对，不能仅猜实例编号。
            bb=np.array(c.GetBox(False,False))*1000
            stem=Path(get(c,'GetPathName')).name.replace(prefix,'');choices=[]
            for oc in ORIGINAL:
                on=oc['name']
                if on not in GROUPS or Path(oc['path']).name!=stem:continue
                target=np.array(oc['box'])*1000;og=GROUPS[on]
                if new and (og==2 or (og==1 and (-155.215865757977-(target[1]+target[4])/2)>140)):target[[1,4]]-=80
                choices.append((float(np.max(np.abs(bb-target))),on,og))
            assert choices and min(choices)[0]<.02,('unmapped instance',name,choices)
            error,original,group=min(choices);renamed.append({'name':name,'original':original,'group':group,'bbox_error_mm':error})
        partnames.append(name)
        for b in p.GetBodies2(0,False) or []:
            cp=get(b,'Copy');assert cp.ApplyTransform(c.Transform2)
            rows.append(record(name,cp,group))
    assert len(rows)==204,(len(rows),len(partnames))
    (OUT/(('new' if new else 'old')+'-instance-mapping.json')).write_text(json.dumps(renamed,ensure_ascii=False,indent=2),'utf-8')
    return doc,before,rows,partnames
def lower_head(r):
    bb=r['box'];limit=float(bb[5]-.00218)
    lo=bb[:3]-.001;hi=bb[3:]+.001;hi[2]=limit
    # 官方API：底面中心、轴向、宽、长、高；高沿轴向从底面挤出。
    clip=MODELER.CreateBodyFromBox3(variant([(lo[0]+hi[0])/2,(lo[1]+hi[1])/2,lo[2],0,0,1,hi[0]-lo[0],hi[1]-lo[1],hi[2]-lo[2]]))
    b=get(r['body'],'Copy');b._FlagAsMethod('Operations2')
    err=win32com.client.VARIANT(pythoncom.VT_BYREF|pythoncom.VT_I4,0)
    result=b.Operations2(15901,clip,err);assert err.value==0 and len(result)==1
    cp=win32com.client.Dispatch(result[0]);out=record(r['name'],cp,r['group'])
    # GetBodyBox是近似包围盒，布尔运算后XY可能收紧；用实体极值验证切面和杆底。
    def extreme_z(body,direction):
        raw=body._oleobj_;dispid=raw.GetIDsOfNames('GetExtremePoint')
        result=raw.InvokeTypes(dispid,0,1,(11,0),((5,1),(5,1),(5,1),(16389,2),(16389,2),(16389,2)),0.,0.,direction)
        assert result[0],result
        return float(result[3])
    top=extreme_z(cp,1.);bottom=extreme_z(cp,-1.)
    assert abs(top-limit)<1e-8 and abs(bottom-extreme_z(r['body'],-1.))<1e-8,{'new_bbox':out['box'].tolist(),'old_bbox':bb.tolist(),'limit':limit,'top':top,'bottom':bottom}
    return out,{'component':r['name'],'old_top_cad_mm':float(bb[5]*1000),'new_top_cad_mm':limit*1000,'seat_cad_mm':float(bb[5]*1000-6),'head_height_mm':3.82,'unchanged_shaft_bottom_cad_mm':float(bb[2]*1000)}
def scan(label,rs,l1,head,metadata):
    start=time.time();changes=[]
    if head<6:
        changed=[]
        for r in rs:
            if r['name'] in ('M6内六角x22-1','M6内六角x22-5'):
                r,detail=lower_head(r);changes.append(detail)
            changed.append(r)
        rs=changed;assert len(changes)==2
    excluded=set();initial=[]
    for i,a in enumerate(rs):
        for j,b in enumerate(rs[:i]):
            if a['group']==b['group'] or not overlap(a['box'],b['box']):continue
            if clash(a['body'],b['body'])['bodies']:
                pair=[a['name'],b['name']];assert frozenset(pair) in allowed,('unexpected baseline interference',pair)
                excluded.add((i,j));initial.append(pair)
    scene=Scene(label,rs,excluded,l1,275,0,changes)
    angles=np.arange(-180,181,4);good=np.zeros((len(angles),len(angles)),bool)
    for i,a in enumerate(angles):
        for j,b in enumerate(angles):good[i,j]=scene.pose(float(a),float(b))['free']
        if i%15==0:print(label,'q1',int(a),'seconds',round(time.time()-start),'checks',scene.calls,flush=True)
        gc.collect()
    visited=np.zeros_like(good);stack=[(45,45)]
    while stack:
        i,j=stack.pop()
        if not 0<=i<len(angles) or visited[i,j] or not good[i,j]:continue
        visited[i,j]=True;stack.extend([(i-1,j),(i+1,j),(i,(j-1)%len(angles)),(i,(j+1)%len(angles))])
    points=[dict(row=r,col=c,**scene.point((c-4)*35,280+(r-4.5)*35)) for r in range(10) for c in range(9)]
    # 首次G1-基座干涉边界，二分细化到约0.001度；不是整轴机械软限位。
    boundaries=[]
    for sign in (-1,1):
        free=0.;blocked=None
        for a in range(1,181):
            if not scene.check_set('first',sign*a,0)['free']:blocked=float(a);free=float(a-1);break
        if blocked is not None:
            for _ in range(11):
                mid=(free+blocked)/2
                if scene.check_set('first',sign*mid,0)['free']:free=mid
                else:blocked=mid
            boundaries.append({'sign':sign,'free_deg':sign*free,'collision_deg':sign*blocked,'collision':scene.check_set('first',sign*blocked,0).get('collision')})
    np.savez_compressed(OUT/(label+'.npz'),angles=angles,good=good,connected=visited)
    report={'label':label,'links_mm':[l1,275],'head_height_mm':head,'body_count':len(rs),'angular_step_deg':4,'configurations':int(good.size),'free_configurations':int(good.sum()),'connected_configurations':int(visited.sum()),'body_checks':scene.calls,'elapsed_seconds':time.time()-start,'baseline_contacts_excluded':initial,'head_changes':changes,'first_axis_contact_boundary':boundaries,'board':{'center_world_mm':[0,280],'pitch_mm':35,'outline_mm':[315,350],'grid_mm':[280,315],'free_endpoint_count':sum(p['free'] for p in points),'total_points':90,'points':points},**metadata}
    (OUT/(label+'.json')).write_text(json.dumps(report,ensure_ascii=False,indent=2),'utf-8')
    print('DONE',label,'board',report['board']['free_endpoint_count'],'/90',flush=True)
    return report
def main():
    reports=[]
    for label,path,prefix,l1,isnew in [
        ('old',ROOT/'hardware/xiangqi30-old195275-2026-10-12/delivery-front-2026-10-13/SolidWorks/OLD_SCARA四轴机械臂总成.SLDASM','OLD_',195,False),
        ('new',ROOT/'hardware/xiangqi30-long275275-2026-10-13/SolidWorks/EQ275_SCARA四轴机械臂总成.SLDASM','EQ275_',275,True)]:
        doc,before,rs,names=load_records(path,prefix,isnew)
        print('Loaded',label,len(rs),'bodies',flush=True)
        metadata={'model':str(path),'part_names':names,'scope':'当前CAD装配Z高度，腕相对姿态不变；两版统一204实体，均不含传动子总成、线缆与完整电磁铁；5对轴/轴套/键的既有连接重叠单独排除。'}
        for head in (6,3.82):
            name=f'{label}_head{str(head).replace(".","p")}'
            if (OUT/(name+'.json')).exists():
                reports.append(json.loads((OUT/(name+'.json')).read_text('utf-8')));continue
            reports.append(scan(name,rs,l1,head,metadata))
        assert bool(get(doc,'GetSaveFlag'))==before,'Read-only scan changed save flag'
        del rs;gc.collect()
    (OUT/'comparison-summary.json').write_text(json.dumps({'variants':reports,'source_modified':False,'head_geometry_assumption':'保持承压面与杆部不变，将原CAD头部截低2.18mm；未取得实际低头螺丝完整轮廓。','limits':['4度采样与插值不证明连续无碰撞','腕轴中心可达，不是实体扫掠外包络','当前Z与R切片，不代表全部Z工作区','棋盘检验是精确端点干涉，不证明路径与抓取净空','传动子总成两版一致排除，实物使用前须连同电磁铁/布线复核']},ensure_ascii=False,indent=2),'utf-8')
if __name__=='__main__':sys.stdout.reconfigure(encoding='utf-8');main()
